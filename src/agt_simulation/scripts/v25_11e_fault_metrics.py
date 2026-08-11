#!/usr/bin/env python3

from __future__ import annotations

import json
import math
from pathlib import Path
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


TRANSIENT_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

ACTIVE_FAULT_CASES = {
    "localization_lost",
    "lidar_dropout",
    "imu_dropout",
}


def _atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def _decode_json(text: str) -> dict[str, Any]:
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        return {"state": "INVALID", "message": text}
    return data if isinstance(data, dict) else {"state": "INVALID", "message": str(data)}


class FaultMetrics(Node):
    def __init__(self) -> None:
        super().__init__("agt_v25_11e_fault_metrics")
        self.fault_case = str(
            self.declare_parameter("fault_case", "localization_lost").value
        )
        if self.fault_case not in ACTIVE_FAULT_CASES:
            raise RuntimeError(
                f"unsupported V25-11E active fault_case={self.fault_case!r}; "
                f"expected one of {sorted(ACTIVE_FAULT_CASES)}"
            )

        default_path = f"/tmp/agt_v25_11e_{self.fault_case}_fault_metrics.json"
        self.metrics_path = Path(
            str(self.declare_parameter("metrics_path", default_path).value)
        )
        self.zero_cmd_threshold = float(
            self.declare_parameter("zero_cmd_threshold", 0.01).value
        )
        self.motion_cmd_threshold = float(
            self.declare_parameter("motion_cmd_threshold", 0.05).value
        )
        if self.zero_cmd_threshold < 0.0 or self.motion_cmd_threshold <= 0.0:
            raise RuntimeError("fault metric command thresholds must be non-negative")

        self.fault_state: dict[str, Any] | None = None
        self.route_state: dict[str, Any] | None = None
        self.latest_truth_xy: tuple[float, float] | None = None
        self.latest_truth_speed_mps = 0.0
        self.latest_cmd_norm = 0.0

        self.pre_fault_motion_seen = False
        self.first_nonzero_cmd_ros_ns: int | None = None
        self.fault_fired_ros_ns: int | None = None
        self.fault_position_xy: tuple[float, float] | None = None
        self.fault_cmd_norm: float | None = None
        self.fault_ground_truth_speed_mps: float | None = None
        self.first_zero_cmd_after_fault_ros_ns: int | None = None
        self.route_terminal_ros_ns: int | None = None
        self.max_post_fault_cmd_norm = 0.0
        self.max_post_fault_distance_m = 0.0
        self.max_post_fault_ground_truth_speed_mps = 0.0

        self.create_subscription(
            String,
            "/agt/simulation/fault/state",
            self._on_fault_state,
            TRANSIENT_QOS,
        )
        self.create_subscription(
            String,
            "/agt/simulation/navigation/route_state",
            self._on_route_state,
            TRANSIENT_QOS,
        )
        self.create_subscription(Twist, "/agt/safety/cmd_vel", self._on_cmd, 20)
        self.create_subscription(
            Odometry, "/simulation/bunker/ground_truth", self._on_truth, 20
        )
        self.write_timer = self.create_timer(0.20, self._write_metrics)
        self._write_metrics()

    def _ros_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _on_fault_state(self, msg: String) -> None:
        data = _decode_json(msg.data)
        if data.get("fault_case") != self.fault_case:
            return
        self.fault_state = data
        if data.get("state") != "FIRED" or self.fault_fired_ros_ns is not None:
            return

        payload_ros_ns = data.get("current_ros_ns")
        if isinstance(payload_ros_ns, int) and payload_ros_ns > 0:
            self.fault_fired_ros_ns = payload_ros_ns
        else:
            self.fault_fired_ros_ns = self._ros_ns()
        self.fault_position_xy = self.latest_truth_xy
        self.fault_cmd_norm = self.latest_cmd_norm
        self.fault_ground_truth_speed_mps = self.latest_truth_speed_mps
        self._write_metrics()

    def _on_route_state(self, msg: String) -> None:
        data = _decode_json(msg.data)
        self.route_state = data
        if (
            self.fault_fired_ros_ns is not None
            and data.get("state") in {"SUCCEEDED", "FAILED"}
            and self.route_terminal_ros_ns is None
        ):
            self.route_terminal_ros_ns = self._ros_ns()
            self._write_metrics()

    def _on_cmd(self, msg: Twist) -> None:
        self.latest_cmd_norm = math.hypot(float(msg.linear.x), float(msg.angular.z))
        now_ns = self._ros_ns()
        if self.fault_fired_ros_ns is None:
            if self.latest_cmd_norm >= self.motion_cmd_threshold:
                self.pre_fault_motion_seen = True
                if self.first_nonzero_cmd_ros_ns is None:
                    self.first_nonzero_cmd_ros_ns = now_ns
            return

        self.max_post_fault_cmd_norm = max(
            self.max_post_fault_cmd_norm, self.latest_cmd_norm
        )
        if (
            self.pre_fault_motion_seen
            and self.first_zero_cmd_after_fault_ros_ns is None
            and self.latest_cmd_norm <= self.zero_cmd_threshold
        ):
            self.first_zero_cmd_after_fault_ros_ns = now_ns
            self._write_metrics()

    def _on_truth(self, msg: Odometry) -> None:
        point = msg.pose.pose.position
        self.latest_truth_xy = (float(point.x), float(point.y))
        twist = msg.twist.twist
        self.latest_truth_speed_mps = math.hypot(
            float(twist.linear.x), float(twist.linear.y)
        )
        if self.fault_fired_ros_ns is None:
            return

        self.max_post_fault_ground_truth_speed_mps = max(
            self.max_post_fault_ground_truth_speed_mps,
            self.latest_truth_speed_mps,
        )
        if self.fault_position_xy is not None:
            distance = math.hypot(
                self.latest_truth_xy[0] - self.fault_position_xy[0],
                self.latest_truth_xy[1] - self.fault_position_xy[1],
            )
            self.max_post_fault_distance_m = max(
                self.max_post_fault_distance_m, distance
            )

    @staticmethod
    def _delta_ms(end_ns: int | None, start_ns: int | None) -> float | None:
        if end_ns is None or start_ns is None or end_ns < start_ns:
            return None
        return (end_ns - start_ns) / 1_000_000.0

    def _route_completion_ratio(self) -> float | None:
        if not isinstance(self.route_state, dict):
            return None
        total = int(self.route_state.get("total_segments", 0))
        reached = int(self.route_state.get("reached_segments", 0))
        if total <= 0:
            return None
        return max(0.0, min(1.0, reached / total))

    def payload(self) -> dict[str, Any]:
        fault_xy = None
        if self.fault_position_xy is not None:
            fault_xy = {"x": self.fault_position_xy[0], "y": self.fault_position_xy[1]}
        return {
            "schema": "agt_v25_11e_fault_metrics/v1",
            "gate": "V25-11E",
            "stage": "active_fault_comparison",
            "fault_case": self.fault_case,
            "ros_time_ns": self._ros_ns(),
            "fault_state": self.fault_state,
            "route_state": self.route_state,
            "checks": {
                "pre_fault_motion_seen": self.pre_fault_motion_seen,
                "fault_injection_seen": self.fault_fired_ros_ns is not None,
                "safety_zero_after_fault_seen": self.first_zero_cmd_after_fault_ros_ns
                is not None,
                "route_terminal_after_fault_seen": self.route_terminal_ros_ns is not None,
            },
            "metrics": {
                "first_nonzero_cmd_ros_ns": self.first_nonzero_cmd_ros_ns,
                "fault_fired_ros_ns": self.fault_fired_ros_ns,
                "first_zero_cmd_after_fault_ros_ns": self.first_zero_cmd_after_fault_ros_ns,
                "route_terminal_ros_ns": self.route_terminal_ros_ns,
                "fault_to_safety_zero_ms": self._delta_ms(
                    self.first_zero_cmd_after_fault_ros_ns, self.fault_fired_ros_ns
                ),
                "fault_to_route_terminal_ms": self._delta_ms(
                    self.route_terminal_ros_ns, self.fault_fired_ros_ns
                ),
                "safety_zero_to_route_terminal_ms": self._delta_ms(
                    self.route_terminal_ros_ns,
                    self.first_zero_cmd_after_fault_ros_ns,
                ),
                "fault_position_xy": fault_xy,
                "fault_cmd_norm": self.fault_cmd_norm,
                "fault_ground_truth_speed_mps": self.fault_ground_truth_speed_mps,
                "max_post_fault_cmd_norm": self.max_post_fault_cmd_norm,
                "max_post_fault_ground_truth_speed_mps": self.max_post_fault_ground_truth_speed_mps,
                "max_post_fault_distance_m": self.max_post_fault_distance_m,
                "route_completion_ratio": self._route_completion_ratio(),
            },
            "artifact": str(self.metrics_path),
        }

    def _write_metrics(self) -> None:
        _atomic_write_json(self.metrics_path, self.payload())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FaultMetrics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._write_metrics()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
