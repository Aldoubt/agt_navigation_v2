#!/usr/bin/env python3
"""Replay one frozen rosbag through managed mapping and ingest its evidence."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Any

import rclpy
from agt_interfaces.action import ManageMappingSession
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from agt_offline_assets import AssetContractError, ingest_mapping_session
from agt_system_manager.offline_replay import prepare_offline_replay_plan


class ReplayMappingClient(Node):
    def __init__(self) -> None:
        super().__init__("agt_offline_replay_mapping_client")
        self._mapping = ActionClient(
            self, ManageMappingSession, "/agt/mapping/manage_session"
        )
        self._feedback_state = ""

    def _feedback(self, message: Any) -> None:
        feedback = message.feedback
        if feedback.state != self._feedback_state:
            self._feedback_state = feedback.state
            print(f"[{feedback.state}] {feedback.message}", flush=True)

    def _wait(self, future: Any, timeout_s: float, description: str) -> Any:
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise TimeoutError(f"{description} timed out")
        error = future.exception()
        if error is not None:
            raise error
        return future.result()

    def mapping_request(
        self,
        operation: int,
        *,
        map_id: str = "",
        session_id: str = "",
        arguments: dict[str, str] | None = None,
        timeout_s: float = 300.0,
    ) -> ManageMappingSession.Result:
        if not self._mapping.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(
                "未发现 /agt/mapping/manage_session，请先启动 agt_system_manager"
            )
        goal = ManageMappingSession.Goal()
        goal.operation = operation
        goal.map_id = map_id
        goal.session_id = session_id
        values = arguments or {}
        goal.argument_keys = list(values)
        goal.argument_values = [values[key] for key in values]
        goal.activate_after_commit = False
        goal.timeout_s = float(timeout_s)
        self._feedback_state = ""
        handle = self._wait(
            self._mapping.send_goal_async(goal, feedback_callback=self._feedback),
            10.0,
            "mapping-session goal",
        )
        if not handle.accepted:
            raise RuntimeError("mapping-session Action 拒绝了请求")
        wrapped = self._wait(
            handle.get_result_async(), timeout_s + 30.0, "mapping-session result"
        )
        result = wrapped.result
        if not result.success:
            raise RuntimeError(
                f"mapping-session failed [{result.error_code}]: {result.message}"
            )
        return result


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "将一份冻结 rosbag 通过受管 MappingSession 回放建图，并把 session-frame "
            "产物冻结进现有 PROCESSING map workspace"
        )
    )
    parser.add_argument("--workspace-manifest", required=True)
    parser.add_argument("--source-bag", required=True)
    parser.add_argument("--platform-profile", required=True)
    parser.add_argument("--user-config-path")
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--settle-seconds", type=float, default=2.0)
    parser.add_argument("--mapping-timeout", type=float, default=300.0)
    parser.add_argument("--playback-timeout", type=float, default=3600.0)
    return parser


def _terminate_process(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        except ProcessLookupError:
            return
        process.wait(timeout=5.0)


def _run(options: argparse.Namespace, client: ReplayMappingClient) -> int:
    if options.settle_seconds < 0.0:
        raise ValueError("settle-seconds must be non-negative")
    if options.mapping_timeout <= 0.0 or options.mapping_timeout > 300.0:
        raise ValueError("mapping-timeout must be in (0, 300]")
    if options.playback_timeout <= 0.0:
        raise ValueError("playback-timeout must be positive")

    plan = prepare_offline_replay_plan(
        options.workspace_manifest,
        source_bag=options.source_bag,
        platform_profile=options.platform_profile,
        playback_rate=options.rate,
        user_config_path=options.user_config_path,
    )
    print(
        json.dumps(
            {
                "phase": "PREFLIGHT_PASS",
                "map_id": plan.map_id,
                "source_bag": str(plan.source_bag),
                "source_bag_sha256": plan.source_bag_sha256,
                "topic_remaps": [
                    {"source": source, "canonical": canonical}
                    for source, canonical in plan.topic_remaps
                ],
            },
            ensure_ascii=False,
        ),
        flush=True,
    )

    started = None
    finalized = None
    finalize_attempted = False
    playback: subprocess.Popen | None = None
    replay_log = None
    try:
        started = client.mapping_request(
            ManageMappingSession.Goal.OP_START,
            map_id=plan.map_id,
            arguments=plan.start_arguments,
            timeout_s=options.mapping_timeout,
        )
        print(
            json.dumps(
                {
                    "phase": "MAPPING_READY",
                    "session_id": started.session_id,
                    "derived_bag": started.bag_directory,
                },
                ensure_ascii=False,
            ),
            flush=True,
        )

        plan.replay_log.parent.mkdir(parents=True, exist_ok=True)
        replay_log = open(plan.replay_log, "ab", buffering=0)
        playback = subprocess.Popen(
            list(plan.playback_command),
            stdin=subprocess.DEVNULL,
            stdout=replay_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        deadline = time.monotonic() + float(options.playback_timeout)
        while playback.poll() is None:
            if time.monotonic() >= deadline:
                raise TimeoutError("source bag playback exceeded playback-timeout")
            rclpy.spin_once(client, timeout_sec=0.2)
        if playback.returncode != 0:
            raise RuntimeError(
                f"source bag playback exited with {playback.returncode}; see {plan.replay_log}"
            )

        if options.settle_seconds:
            settle_deadline = time.monotonic() + float(options.settle_seconds)
            while time.monotonic() < settle_deadline:
                rclpy.spin_once(client, timeout_sec=min(0.2, settle_deadline - time.monotonic()))

        finalize_attempted = True
        finalized = client.mapping_request(
            ManageMappingSession.Goal.OP_FINALIZE_CAPTURE,
            session_id=started.session_id,
            timeout_s=options.mapping_timeout,
        )
        if finalized.state != "CANDIDATE_READY":
            raise RuntimeError(
                f"mapping session finalized in unexpected state: {finalized.state}"
            )

        ingest = ingest_mapping_session(
            plan.workspace_manifest,
            session_file=finalized.session_file,
            session_id=finalized.session_id,
            candidate_map_yaml=finalized.candidate_map_yaml,
            candidate_map_image=finalized.candidate_map_image,
            localization_pcd=finalized.localization_pcd,
            processing_record=finalized.processing_record,
            derived_bag_directory=finalized.bag_directory,
            source_bag_path=plan.source_bag,
        )
        print(
            json.dumps(
                {
                    "phase": "INGESTED_PROCESSING_EVIDENCE",
                    **ingest.to_dict(),
                    "next": "alignment/materialization required before READY promotion",
                },
                ensure_ascii=False,
            )
        )
        return 0
    except BaseException:
        _terminate_process(playback)
        # Failures before FINALIZE own no useful static-candidate state and are
        # cleaned automatically. Once FINALIZE begins, preserve the managed
        # session because CANDIDATE_BUILD_FAILED / BUILDING_STATIC_MAP are
        # explicitly retryable states and contain useful evidence.
        if started is not None and not finalize_attempted:
            try:
                client.mapping_request(
                    ManageMappingSession.Goal.OP_DISCARD,
                    session_id=started.session_id,
                    timeout_s=min(float(options.mapping_timeout), 120.0),
                )
            except Exception as cleanup_error:
                print(
                    f"warning: mapping-session cleanup failed: {cleanup_error}",
                    file=sys.stderr,
                )
        elif started is not None and finalize_attempted:
            print(
                "mapping-session was preserved after FINALIZE began; use "
                f"mapping_session_workflow.py status/finalize --session-id {started.session_id}",
                file=sys.stderr,
            )
        raise
    finally:
        _terminate_process(playback)
        if replay_log is not None:
            replay_log.close()


def main(argv=None) -> int:
    options = _parser().parse_args(argv)
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    client = ReplayMappingClient()
    try:
        return _run(options, client)
    except KeyboardInterrupt:
        print("offline replay interrupted", file=sys.stderr)
        return 130
    except (AssetContractError, OSError, RuntimeError, TimeoutError, ValueError) as exc:
        code = getattr(exc, "code", "offline_replay_failed")
        print(
            json.dumps(
                {"status": "ERROR", "code": code, "message": str(exc)},
                ensure_ascii=False,
            ),
            file=sys.stderr,
        )
        return 1
    finally:
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
