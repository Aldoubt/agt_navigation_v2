#!/usr/bin/env python3

"""Automated V25-11B GlobalCorrectionManager integration acceptance."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time

import rclpy
from agt_interfaces.msg import LocalizationStatus
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


CORRECTION_MIN_INTERVAL_S = 1.0
CORRECTION_INTERVAL_MARGIN_S = 0.10


class LocalizationAcceptance(Node):
    def __init__(self) -> None:
        super().__init__("agt_v25_11b_localization_acceptance")
        self.report_path = Path(
            str(
                self.declare_parameter(
                    "report_path", "/tmp/agt_v25_11b_localization_result.json"
                ).value
            )
        )
        self.latest_status = None
        self.status_sequence = 0
        self.latest_decision = None
        self.decision_sequence = 0
        self.create_subscription(
            LocalizationStatus,
            "/agt/localization/status",
            self._on_status,
            20,
        )
        self.create_subscription(
            String,
            "/agt/localization/global_correction_status",
            self._on_decision,
            20,
        )
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

        self.submit_client = self.create_client(
            Trigger, "/agt/simulation/localization/submit_correction"
        )
        self.clear_client = self.create_client(
            Trigger, "/agt/simulation/localization/clear_faults"
        )
        self.set_parameters_client = self.create_client(
            SetParameters,
            "/agt_synthetic_localization_evidence/set_parameters",
        )

    def _on_status(self, message: LocalizationStatus) -> None:
        self.latest_status = message
        self.status_sequence += 1

    def _on_decision(self, message: String) -> None:
        try:
            self.latest_decision = json.loads(message.data)
        except json.JSONDecodeError:
            self.latest_decision = {"raw": message.data}
        self.decision_sequence += 1


def _spin_until(node: Node, predicate, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return bool(predicate())


def _wait_service(node: Node, client, timeout_s: float = 5.0) -> None:
    if not client.wait_for_service(timeout_sec=timeout_s):
        raise RuntimeError(f"service unavailable: {client.srv_name}")


def _call_trigger(node: Node, client, timeout_s: float = 5.0):
    _wait_service(node, client, timeout_s)
    future = client.call_async(Trigger.Request())
    if not _spin_until(node, future.done, timeout_s):
        raise RuntimeError(f"service timeout: {client.srv_name}")
    response = future.result()
    if response is None or not response.success:
        raise RuntimeError(
            f"service failed: {client.srv_name}: "
            f"{getattr(response, 'message', 'no response')}"
        )
    return response


def _set_remote_parameters(node: LocalizationAcceptance, values: dict) -> None:
    _wait_service(node, node.set_parameters_client)
    request = SetParameters.Request()
    request.parameters = [
        Parameter(name=name, value=value).to_parameter_msg()
        for name, value in values.items()
    ]
    future = node.set_parameters_client.call_async(request)
    if not _spin_until(node, future.done, 5.0):
        raise RuntimeError("remote parameter update timed out")
    response = future.result()
    if response is None or len(response.results) != len(values):
        raise RuntimeError("remote parameter update returned incomplete result")
    failures = [result.reason for result in response.results if not result.successful]
    if failures:
        raise RuntimeError("remote parameter update failed: " + "; ".join(failures))


def _wait_status_after(
    node: LocalizationAcceptance,
    sequence: int,
    predicate,
    timeout_s: float = 5.0,
):
    ok = _spin_until(
        node,
        lambda: node.status_sequence > sequence
        and node.latest_status is not None
        and predicate(node.latest_status),
        timeout_s,
    )
    if not ok:
        status = node.latest_status
        raise RuntimeError(
            "canonical localization status did not reach expected state; "
            f"latest={None if status is None else (status.state, status.localization_accepted, status.correction_generation, status.message)}"
        )
    return node.latest_status


def _decision_matches(decision, code: str, generation: int, accepted: bool) -> bool:
    return (
        isinstance(decision, dict)
        and decision.get("code") == code
        and decision.get("generation", -1) == generation
        and bool(decision.get("accepted")) is accepted
    )


def _wait_decision_after(
    node: LocalizationAcceptance,
    sequence: int,
    code: str,
    generation: int,
    accepted: bool,
    timeout_s: float = 5.0,
):
    ok = _spin_until(
        node,
        lambda: node.decision_sequence > sequence
        and _decision_matches(node.latest_decision, code, generation, accepted),
        timeout_s,
    )
    if not ok:
        raise RuntimeError(
            "global correction decision did not reach expected state; "
            f"latest={node.latest_decision} sequence={node.decision_sequence} "
            f"expected={(code, generation, accepted)}"
        )
    return node.latest_decision


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _wait_ros_correction_interval(
    node: LocalizationAcceptance,
    last_accepted_status: LocalizationStatus,
    min_interval_s: float = CORRECTION_MIN_INTERVAL_S,
    margin_s: float = CORRECTION_INTERVAL_MARGIN_S,
    wall_timeout_s: float = 5.0,
) -> None:
    """Wait until ROS time is safely beyond the last accepted evidence stamp.

    GlobalCorrectionCore rate-limits using observation.stamp_s, which comes from
    the synthetic evidence global_pose stamp. Waiting against ROS time rather
    than wall time keeps the smoke deterministic under use_sim_time.
    """
    accepted_stamp_ns = _stamp_ns(last_accepted_status.global_pose.header.stamp)
    if accepted_stamp_ns <= 0:
        raise RuntimeError("last accepted correction has invalid ROS timestamp")

    interval_ns = int((min_interval_s + margin_s) * 1_000_000_000)
    target_ns = accepted_stamp_ns + interval_ns
    wall_deadline = time.monotonic() + wall_timeout_s
    previous_ros_ns = node.get_clock().now().nanoseconds

    while rclpy.ok() and time.monotonic() < wall_deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        now_ros_ns = node.get_clock().now().nanoseconds
        if now_ros_ns < previous_ros_ns:
            raise RuntimeError(
                "ROS_TIME_ROLLBACK while waiting for correction minimum interval"
            )
        if now_ros_ns >= target_ns:
            return
        previous_ros_ns = now_ros_ns

    now_ros_ns = node.get_clock().now().nanoseconds
    raise RuntimeError(
        "ROS time did not advance past correction minimum interval; "
        f"now_ns={now_ros_ns} target_ns={target_ns}"
    )


def _submit_and_wait(
    node: LocalizationAcceptance,
    *,
    last_accepted_status: LocalizationStatus,
    status_predicate,
    decision_code: str,
    generation: int,
    accepted: bool,
    timeout_s: float = 5.0,
):
    _wait_ros_correction_interval(node, last_accepted_status)
    status_sequence = node.status_sequence
    decision_sequence = node.decision_sequence
    _call_trigger(node, node.submit_client)
    status = _wait_status_after(node, status_sequence, status_predicate, timeout_s)
    decision = _wait_decision_after(
        node,
        decision_sequence,
        decision_code,
        generation,
        accepted,
        timeout_s,
    )
    return status, decision


def _map_odom_xy(node: LocalizationAcceptance):
    if not node.tf.can_transform("map", "odom", Time()):
        return None
    transform = node.tf.lookup_transform("map", "odom", Time())
    return (
        float(transform.transform.translation.x),
        float(transform.transform.translation.y),
    )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationAcceptance()
    result = {
        "gate": "V25-11B",
        "status": "FAIL",
        "checks": {},
        "events": [],
    }
    exit_code = 1

    try:
        # The launch starts this smoke before the synthetic node's delayed initial
        # correction, so the first canonical TRACKING status must be generation 1+.
        initial_decision_sequence = node.decision_sequence
        initial = _wait_status_after(
            node,
            0,
            lambda status: status.state == LocalizationStatus.STATE_TRACKING
            and status.localization_accepted
            and status.pose_valid
            and status.correction_generation >= 1,
            8.0,
        )
        initial_generation = int(initial.correction_generation)
        _wait_decision_after(
            node, initial_decision_sequence, "CORRECTION_ACCEPTED", initial_generation, True
        )
        last_accepted = initial
        result["events"].append(
            {"event": "initial_correction", "generation": initial_generation}
        )
        result["checks"]["initial_tracking"] = True

        if not _spin_until(node, lambda: _map_odom_xy(node) is not None, 3.0):
            raise RuntimeError("map->odom TF not available after initial correction")
        initial_tf = _map_odom_xy(node)
        result["checks"]["map_odom_available"] = initial_tf is not None

        # TRACKING: +0.20 m is inside the 0.50 m envelope and must increment generation.
        _set_remote_parameters(node, {"translation_bias_x_m": 0.20})
        accepted_small, _ = _submit_and_wait(
            node,
            last_accepted_status=last_accepted,
            status_predicate=lambda status: status.state == LocalizationStatus.STATE_TRACKING
            and status.localization_accepted
            and status.correction_generation == initial_generation + 1,
            decision_code="CORRECTION_ACCEPTED",
            generation=initial_generation + 1,
            accepted=True,
        )
        last_accepted = accepted_small
        shifted_tf = _map_odom_xy(node)
        shift = math.hypot(
            shifted_tf[0] - initial_tf[0], shifted_tf[1] - initial_tf[1]
        )
        result["events"].append(
            {
                "event": "tracking_small_correction",
                "generation": int(accepted_small.correction_generation),
                "map_odom_shift_m": shift,
            }
        )
        result["checks"]["tracking_small_correction_accepted"] = 0.12 <= shift <= 0.30

        # TRACKING: +1.00 m relative to baseline is outside the 0.50 m envelope.
        _set_remote_parameters(node, {"translation_bias_x_m": 1.00})
        generation_before_reject = int(accepted_small.correction_generation)
        rejected, rejected_decision = _submit_and_wait(
            node,
            last_accepted_status=last_accepted,
            status_predicate=lambda status: status.state == LocalizationStatus.STATE_RECOVERING
            and not status.localization_accepted
            and status.correction_generation == generation_before_reject,
            decision_code="TRANSLATION_JUMP_REJECTED",
            generation=generation_before_reject,
            accepted=False,
        )
        decision_code = rejected_decision["code"]
        result["events"].append(
            {
                "event": "tracking_jump_rejected",
                "generation": int(rejected.correction_generation),
                "decision": decision_code,
            }
        )
        result["checks"]["tracking_jump_rejected"] = (
            decision_code == "TRANSLATION_JUMP_REJECTED"
        )
        result["checks"]["rejected_generation_frozen"] = (
            int(rejected.correction_generation) == generation_before_reject
        )

        # The same jump is inside the RECOVERING 2.0 m envelope and must be accepted.
        recovered, _ = _submit_and_wait(
            node,
            last_accepted_status=last_accepted,
            status_predicate=lambda status: status.state == LocalizationStatus.STATE_TRACKING
            and status.localization_accepted
            and status.correction_generation == generation_before_reject + 1,
            decision_code="CORRECTION_ACCEPTED",
            generation=generation_before_reject + 1,
            accepted=True,
        )
        last_accepted = recovered
        recovered_generation = int(recovered.correction_generation)
        result["events"].append(
            {"event": "recovering_correction_accepted", "generation": recovered_generation}
        )
        result["checks"]["recovering_envelope_accepted"] = True

        # Quality rejection is state-independent. Three consecutive failures must
        # escalate canonical localization to LOST without changing generation.
        _set_remote_parameters(node, {"fitness_score": 99.0})
        lost_status = None
        for attempt in range(1, 4):
            lost_status, _ = _submit_and_wait(
                node,
                last_accepted_status=last_accepted,
                status_predicate=lambda status: not status.localization_accepted
                and status.correction_generation == recovered_generation,
                decision_code="FITNESS_REJECTED",
                generation=recovered_generation,
                accepted=False,
            )
            result["events"].append(
                {
                    "event": "fitness_rejection",
                    "attempt": attempt,
                    "state": int(lost_status.state),
                    "generation": int(lost_status.correction_generation),
                }
            )
        result["checks"]["three_rejections_escalate_lost"] = (
            lost_status is not None
            and lost_status.state == LocalizationStatus.STATE_LOST
            and int(lost_status.correction_generation) == recovered_generation
        )

        # LOST may reanchor across a large displacement when quality is restored.
        _set_remote_parameters(
            node,
            {"fitness_score": 0.01, "translation_bias_x_m": 5.0},
        )
        reanchored, reanchored_decision = _submit_and_wait(
            node,
            last_accepted_status=last_accepted,
            status_predicate=lambda status: status.state == LocalizationStatus.STATE_TRACKING
            and status.localization_accepted
            and status.correction_generation == recovered_generation + 1,
            decision_code="REANCHOR_ACCEPTED",
            generation=recovered_generation + 1,
            accepted=True,
        )
        decision_code = reanchored_decision["code"]
        result["events"].append(
            {
                "event": "lost_reanchor",
                "generation": int(reanchored.correction_generation),
                "decision": decision_code,
            }
        )
        result["checks"]["lost_reanchor_accepted"] = (
            decision_code == "REANCHOR_ACCEPTED"
        )
        result["checks"]["ros_time_correction_spacing"] = True

        _call_trigger(node, node.clear_client)
        passed = all(bool(value) for value in result["checks"].values())
        result["status"] = "PASS" if passed else "FAIL"
        exit_code = 0 if passed else 1
    except Exception as error:  # noqa: BLE001 - acceptance must persist diagnostics
        result["error"] = str(error)
    finally:
        node.report_path.parent.mkdir(parents=True, exist_ok=True)
        node.report_path.write_text(
            json.dumps(result, indent=2) + "\n", encoding="utf-8"
        )
        print(json.dumps(result, indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
