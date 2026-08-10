#!/usr/bin/env python3

from __future__ import annotations

import json
import math
from pathlib import Path
import time

import rclpy
from agt_interfaces.msg import LocalizationStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


TRANSIENT_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

FAULT_CASES = {
    "map_identity_mismatch",
    "localization_lost",
    "planner_invalid",
    "controller_invalid",
}

EXPECTED_FAILURE_PREFIX = {
    "map_identity_mismatch": "LOCALIZATION_NOT_READY:",
    "localization_lost": "LOCALIZATION_GUARD_FAILED:",
    "planner_invalid": "PLANNER_",
    "controller_invalid": "CONTROLLER_",
}


class Acceptance(Node):
    def __init__(self) -> None:
        super().__init__("agt_v25_11d_failure_acceptance")
        self.fault_case = str(
            self.declare_parameter("fault_case", "map_identity_mismatch").value
        )
        if self.fault_case not in FAULT_CASES:
            raise RuntimeError(f"unsupported V25-11D fault_case: {self.fault_case}")
        self.timeout_s = float(self.declare_parameter("timeout_s", 35.0).value)
        default_report = f"/tmp/agt_v25_11d_{self.fault_case}_result.json"
        self.report_path = Path(
            str(self.declare_parameter("report_path", default_report).value)
        )

        self.route_state: dict | None = None
        self.fault_state: dict | None = None
        self.localization: LocalizationStatus | None = None
        self.first_truth: Odometry | None = None
        self.latest_truth: Odometry | None = None
        self.max_displacement_m = 0.0
        self.max_cmd_magnitude = 0.0
        self.route_succeeded_seen = False
        self.route_running_seen = False
        self.canonical_lost_seen = False

        self.create_subscription(
            String,
            "/agt/simulation/navigation/route_state",
            self._on_route_state,
            TRANSIENT_QOS,
        )
        self.create_subscription(
            String,
            "/agt/simulation/fault/state",
            self._on_fault_state,
            TRANSIENT_QOS,
        )
        self.create_subscription(
            LocalizationStatus, "/agt/localization/status", self._on_localization, 20
        )
        self.create_subscription(
            Odometry, "/simulation/bunker/ground_truth", self._on_truth, 20
        )
        self.create_subscription(Twist, "/agt/safety/cmd_vel", self._on_cmd, 20)

        self.planner_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.controller_client = ActionClient(self, FollowPath, "/follow_path")

    @staticmethod
    def _decode_state(msg: String) -> dict:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return {"state": "INVALID", "message": msg.data}
        return data if isinstance(data, dict) else {"state": "INVALID"}

    def _on_route_state(self, msg: String) -> None:
        data = self._decode_state(msg)
        self.route_state = data
        state = data.get("state")
        self.route_running_seen = self.route_running_seen or state == "RUNNING"
        self.route_succeeded_seen = self.route_succeeded_seen or state == "SUCCEEDED"

    def _on_fault_state(self, msg: String) -> None:
        self.fault_state = self._decode_state(msg)

    def _on_localization(self, msg: LocalizationStatus) -> None:
        self.localization = msg
        self.canonical_lost_seen = self.canonical_lost_seen or (
            msg.state == LocalizationStatus.STATE_LOST
        )

    def _on_truth(self, msg: Odometry) -> None:
        if self.first_truth is None:
            self.first_truth = msg
        self.latest_truth = msg
        if self.first_truth is not None:
            a = self.first_truth.pose.pose.position
            b = msg.pose.pose.position
            displacement = math.hypot(float(b.x - a.x), float(b.y - a.y))
            self.max_displacement_m = max(self.max_displacement_m, displacement)

    def _on_cmd(self, msg: Twist) -> None:
        self.max_cmd_magnitude = max(
            self.max_cmd_magnitude,
            math.hypot(float(msg.linear.x), float(msg.angular.z)),
        )

    def route_terminal(self) -> bool:
        return isinstance(self.route_state, dict) and self.route_state.get("state") in {
            "SUCCEEDED",
            "FAILED",
        }


def spin_until(node: Node, predicate, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return bool(predicate())


def spin_for(node: Node, duration_s: float) -> None:
    deadline = time.monotonic() + duration_s
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Acceptance()
    result = {
        "gate": "V25-11D",
        "fault_case": node.fault_case,
        "status": "FAIL",
        "checks": {},
        "metrics": {},
    }
    exit_code = 1

    try:
        infrastructure_ready = spin_until(
            node,
            lambda: node.route_state is not None
            and node.localization is not None
            and node.first_truth is not None,
            12.0,
        )
        result["checks"]["observer_inputs_available"] = infrastructure_ready
        result["checks"]["planner_action_server_available"] = (
            node.planner_client.wait_for_server(timeout_sec=5.0)
        )
        result["checks"]["controller_action_server_available"] = (
            node.controller_client.wait_for_server(timeout_sec=5.0)
        )

        terminal_received = spin_until(node, node.route_terminal, node.timeout_s)
        spin_for(node, 0.75)

        route = node.route_state or {}
        message = str(route.get("message", ""))
        expected_prefix = EXPECTED_FAILURE_PREFIX[node.fault_case]
        reached_segments = int(route.get("reached_segments", 0))
        total_segments = int(route.get("total_segments", -1))

        result["checks"]["route_terminal_received"] = terminal_received
        result["checks"]["route_failed"] = route.get("state") == "FAILED"
        result["checks"]["route_never_succeeded"] = not node.route_succeeded_seen
        result["checks"]["expected_failure_class"] = message.startswith(expected_prefix)
        result["checks"]["route_not_completed"] = (
            total_segments > 0 and reached_segments < total_segments
        )

        if node.fault_case == "localization_lost":
            result["checks"]["fault_injection_fired"] = bool(
                isinstance(node.fault_state, dict)
                and node.fault_state.get("fault_case") == "localization_lost"
                and node.fault_state.get("state") == "FIRED"
            )
            result["checks"]["canonical_lost_observed"] = node.canonical_lost_seen
            result["checks"]["route_started_before_fault"] = node.route_running_seen
            result["checks"]["navigation_command_seen_before_abort"] = (
                node.max_cmd_magnitude >= 0.05
            )
            result["checks"]["motion_bounded_after_abort"] = (
                node.max_displacement_m <= 2.50
            )
        else:
            result["checks"]["motion_blocked_before_execution"] = (
                node.max_displacement_m <= 0.50
            )

        if node.fault_case == "map_identity_mismatch":
            result["checks"]["map_identity_actually_mismatched"] = bool(
                node.localization is not None
                and route.get("map_id") == node.localization.map_id
                and route.get("map_hash") != node.localization.map_hash
            )

        result["metrics"]["route_state"] = route
        result["metrics"]["fault_state"] = node.fault_state
        result["metrics"]["max_displacement_m"] = node.max_displacement_m
        result["metrics"]["max_cmd_magnitude"] = node.max_cmd_magnitude
        result["metrics"]["canonical_lost_seen"] = node.canonical_lost_seen
        if node.localization is not None:
            result["metrics"]["latest_localization"] = {
                "state": int(node.localization.state),
                "localization_accepted": bool(node.localization.localization_accepted),
                "pose_valid": bool(node.localization.pose_valid),
                "correction_generation": int(node.localization.correction_generation),
                "map_id": node.localization.map_id,
                "map_hash": node.localization.map_hash,
            }

        passed = all(bool(value) for value in result["checks"].values())
        result["status"] = "PASS" if passed else "FAIL"
        exit_code = 0 if passed else 1
    except Exception as error:  # noqa: BLE001
        result["error"] = str(error)
    finally:
        node.report_path.parent.mkdir(parents=True, exist_ok=True)
        node.report_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(result, indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
