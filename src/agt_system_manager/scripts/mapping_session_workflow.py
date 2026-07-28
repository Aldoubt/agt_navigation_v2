#!/usr/bin/env python3
"""Operator CLI for the finite project-owned mapping-session workflow."""

import argparse
import signal
import subprocess
import sys
import threading
import time
from typing import Any

import rclpy
from agt_interfaces.action import ManageMappingSession
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


class MappingWorkflowClient(Node):
    def __init__(self) -> None:
        super().__init__("agt_mapping_session_workflow")
        self._client = ActionClient(
            self, ManageMappingSession, "/agt/mapping/manage_session"
        )
        self._feedback_state = ""

    def _feedback(self, message: Any) -> None:
        feedback = message.feedback
        if feedback.state != self._feedback_state:
            self._feedback_state = feedback.state
            print(f"[{feedback.state}] {feedback.message}", flush=True)

    def _wait(self, future: Any, timeout_s: float) -> Any:
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise TimeoutError("mapping-session Action timed out")
        error = future.exception()
        if error is not None:
            raise error
        return future.result()

    def request(
        self,
        operation: int,
        *,
        map_id: str = "",
        session_id: str = "",
        arguments: dict[str, str] | None = None,
        activate: bool = False,
        timeout_s: float = 120.0,
    ) -> ManageMappingSession.Result:
        if not self._client.wait_for_server(timeout_sec=5.0):
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
        goal.activate_after_commit = activate
        goal.timeout_s = timeout_s
        self._feedback_state = ""
        handle = self._wait(
            self._client.send_goal_async(goal, feedback_callback=self._feedback), 10.0
        )
        if not handle.accepted:
            raise RuntimeError("mapping-session Action 拒绝了请求")
        wrapped = self._wait(handle.get_result_async(), timeout_s + 20.0)
        return wrapped.result


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="统一建图采集、Ctrl+C 收口、候选编辑和版本提交"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    run = subparsers.add_parser("run", help="启动建图；按一次 Ctrl+C 自动保存并正常停止")
    run.add_argument("--map-id", required=True)
    run.add_argument("--timeout", type=float, default=120.0)
    run.add_argument("--platform-profile")
    run.add_argument("--user-config-path")
    run.add_argument("--can-interface")
    run.add_argument("--use-sim-time", action="store_true")
    run.add_argument("--no-sensor", action="store_true")
    run.add_argument("--start-chassis", action="store_true")
    run.add_argument("--start-chassis-monitor", action="store_true")
    run.add_argument("--no-rviz", action="store_true")
    run.add_argument("--start-qt", action="store_true")
    run.add_argument(
        "--qt-authoring",
        action="store_true",
        help="收口后依次打开受限候选编辑器、提交版本和 Qt 任务规划预览",
    )
    run.add_argument("--activate-after-edit", action="store_true")
    run.add_argument("--skip-task-preview", action="store_true")

    status = subparsers.add_parser("status", help="读取最新或指定会话状态")
    status.add_argument("--session-id", default="")
    status.add_argument("--timeout", type=float, default=10.0)

    finalize = subparsers.add_parser(
        "finalize", help="重试已停止会话的离线静态候选生成"
    )
    finalize.add_argument("--session-id", required=True)
    finalize.add_argument("--timeout", type=float, default=300.0)

    commit = subparsers.add_parser("commit", help="校验候选地图并生成新的 READY 版本")
    commit.add_argument("--session-id", required=True)
    commit.add_argument("--activate", action="store_true")
    commit.add_argument("--timeout", type=float, default=120.0)

    discard = subparsers.add_parser("discard", help="正常停止并回收测试会话")
    discard.add_argument("--session-id", required=True)
    discard.add_argument("--timeout", type=float, default=120.0)
    return parser


def _show_result(result: ManageMappingSession.Result) -> None:
    print(f"state: {result.state}")
    if result.session_id:
        print(f"session_id: {result.session_id}")
    if result.map_id:
        print(f"map_id: {result.map_id}")
    if result.map_version_id:
        print(f"map_version_id: {result.map_version_id}")
    for label, value in (
        ("candidate_map_yaml", result.candidate_map_yaml),
        ("candidate_map_image", result.candidate_map_image),
        ("localization_pcd", result.localization_pcd),
        ("processing_record", result.processing_record),
        ("bag_directory", result.bag_directory),
        ("registered_map_yaml", result.registered_map_yaml),
        ("tasks_directory", result.tasks_directory),
    ):
        if value:
            print(f"{label}: {value}")
    print(f"message: {result.message}")


def _require_success(result: ManageMappingSession.Result) -> None:
    if not result.success:
        raise RuntimeError(f"操作失败 [{result.error_code}]: {result.message}")


def _run_frontend(command: list[str], *, interrupt_is_success: bool = False) -> int:
    process = subprocess.Popen(command)
    try:
        return process.wait()
    except KeyboardInterrupt:
        process.send_signal(signal.SIGINT)
        return_code = process.wait()
        if interrupt_is_success:
            return 0
        return return_code or 130


def _run_qt_authoring(
    client: MappingWorkflowClient,
    finalized: ManageMappingSession.Result,
    options: argparse.Namespace,
) -> int:
    print("正在打开受限候选地图编辑器；保存只会更新当前候选。")
    return_code = _run_frontend(
        [
            "ros2",
            "run",
            "agt_ui_bridge",
            "start_ros_qt5_gui_app.sh",
            "--profile",
            "candidate",
            "--map",
            finalized.candidate_map_yaml,
            "--reset-config",
        ]
    )
    if return_code != 0:
        raise RuntimeError(
            f"候选地图编辑器异常退出（{return_code}），候选未提交，可稍后重试"
        )
    committed = client.request(
        ManageMappingSession.Goal.OP_COMMIT,
        session_id=finalized.session_id,
        activate=bool(options.activate_after_edit),
        timeout_s=options.timeout,
    )
    _require_success(committed)
    _show_result(committed)
    if options.skip_task_preview:
        return 0
    command = [
        "ros2",
        "launch",
        "agt_navigation",
        "waypoint_preview.launch.py",
        f"map:={committed.registered_map_yaml}",
        "start_rviz:=false",
    ]
    if options.platform_profile:
        command.append(f"platform_profile:={options.platform_profile}")
    print("正在打开 Qt 任务编排与规划预览；结束该阶段时按 Ctrl+C。")
    return _run_frontend(command, interrupt_is_success=True)


def _run_capture(client: MappingWorkflowClient, options: argparse.Namespace) -> int:
    arguments = {
        "start_sensor": "false" if options.no_sensor else "true",
        "start_chassis": "true" if options.start_chassis else "false",
        "start_chassis_monitor": "true" if options.start_chassis_monitor else "false",
        "start_rviz": "false" if options.no_rviz else "true",
        "start_mapping_gui": "true" if options.start_qt else "false",
        "use_sim_time": "true" if options.use_sim_time else "false",
    }
    for key, value in (
        ("platform_profile", options.platform_profile),
        ("user_config_path", options.user_config_path),
        ("can_interface", options.can_interface),
    ):
        if value:
            arguments[key] = value
    started = client.request(
        ManageMappingSession.Goal.OP_START,
        map_id=options.map_id,
        arguments=arguments,
        timeout_s=options.timeout,
    )
    _require_success(started)
    _show_result(started)
    print(
        "建图采集中；按一次 Ctrl+C 保存在线预览、正常收口 PCD/bag，并离线生成静态候选。",
        flush=True,
    )

    finalize_requested = threading.Event()
    signal_count = [0]

    def request_finalize(_signum, _frame) -> None:
        signal_count[0] += 1
        finalize_requested.set()
        if signal_count[0] == 1:
            print("\n收到 Ctrl+C，正在执行受管收口，请等待...", flush=True)
        else:
            print("\n收口仍在进行，不会强制终止写盘。", flush=True)

    signal.signal(signal.SIGINT, request_finalize)
    while not finalize_requested.is_set():
        rclpy.spin_once(client, timeout_sec=0.2)

    finalized = client.request(
        ManageMappingSession.Goal.OP_FINALIZE_CAPTURE,
        session_id=started.session_id,
        timeout_s=options.timeout,
    )
    _require_success(finalized)
    _show_result(finalized)
    print(
        "离线静态候选已通过质量门禁。只编辑 candidate_map_yaml/image；"
        "校验后使用 commit 生成新版本。"
    )
    signal.signal(signal.SIGINT, signal.default_int_handler)
    if options.qt_authoring:
        return _run_qt_authoring(client, finalized, options)
    return 0


def main(argv: list[str] | None = None) -> int:
    options = _parser().parse_args(argv)
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    client = MappingWorkflowClient()
    try:
        if options.command == "run":
            return _run_capture(client, options)
        operation = {
            "status": ManageMappingSession.Goal.OP_STATUS,
            "finalize": ManageMappingSession.Goal.OP_FINALIZE_CAPTURE,
            "commit": ManageMappingSession.Goal.OP_COMMIT,
            "discard": ManageMappingSession.Goal.OP_DISCARD,
        }[options.command]
        result = client.request(
            operation,
            session_id=options.session_id,
            activate=bool(getattr(options, "activate", False)),
            timeout_s=options.timeout,
        )
        _require_success(result)
        _show_result(result)
        return 0
    except (OSError, RuntimeError, TimeoutError, ValueError) as error:
        print(str(error), file=sys.stderr)
        return 1
    finally:
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
