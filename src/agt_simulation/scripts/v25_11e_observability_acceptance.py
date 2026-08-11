#!/usr/bin/env python3

from __future__ import annotations

import json
from pathlib import Path
import time

import rclpy
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray


TRANSIENT_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class Acceptance(Node):
    def __init__(self) -> None:
        super().__init__("agt_v25_11e_observability_acceptance")
        self.timeout_s = float(self.declare_parameter("timeout_s", 90.0).value)
        self.summary_path = Path(
            str(
                self.declare_parameter(
                    "summary_path", "/tmp/agt_v25_11e_summary.json"
                ).value
            )
        )
        self.timeline_path = Path(
            str(
                self.declare_parameter(
                    "timeline_path", "/tmp/agt_v25_11e_timeline.jsonl"
                ).value
            )
        )
        self.report_path = Path(
            str(
                self.declare_parameter(
                    "report_path", "/tmp/agt_v25_11e_observability_result.json"
                ).value
            )
        )
        self.route_state = None
        self.marker_count = 0
        self.truth_pose_count = 0
        self.create_subscription(
            String,
            "/agt/simulation/navigation/route_state",
            self._on_route,
            TRANSIENT_QOS,
        )
        self.create_subscription(
            MarkerArray,
            "/agt/validation/status_markers",
            self._on_markers,
            TRANSIENT_QOS,
        )
        self.create_subscription(
            NavPath,
            "/agt/validation/ground_truth_path",
            self._on_truth_path,
            TRANSIENT_QOS,
        )

    def _on_route(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            data = {"state": "INVALID", "message": msg.data}
        self.route_state = data if isinstance(data, dict) else {"state": "INVALID"}

    def _on_markers(self, msg: MarkerArray) -> None:
        self.marker_count = max(self.marker_count, len(msg.markers))

    def _on_truth_path(self, msg: NavPath) -> None:
        self.truth_pose_count = max(self.truth_pose_count, len(msg.poses))

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
        "gate": "V25-11E",
        "stage": "observability_baseline",
        "status": "FAIL",
        "checks": {},
        "metrics": {},
    }
    exit_code = 1
    try:
        result["checks"]["markers_available"] = spin_until(
            node, lambda: node.marker_count >= 5, 12.0
        )
        result["checks"]["ground_truth_path_available"] = spin_until(
            node, lambda: node.truth_pose_count >= 10, 12.0
        )
        result["checks"]["route_terminal_received"] = spin_until(
            node, node.route_terminal, node.timeout_s
        )
        spin_for(node, 1.0)

        route = node.route_state or {}
        result["checks"]["happy_path_succeeded"] = route.get("state") == "SUCCEEDED"
        result["checks"]["summary_available"] = node.summary_path.is_file()
        result["checks"]["timeline_available"] = node.timeline_path.is_file()

        summary = {}
        if node.summary_path.is_file():
            summary = json.loads(node.summary_path.read_text(encoding="utf-8"))
        timeline_records = []
        if node.timeline_path.is_file():
            for line in node.timeline_path.read_text(encoding="utf-8").splitlines():
                if line.strip():
                    timeline_records.append(json.loads(line))

        metrics = summary.get("metrics", {}) if isinstance(summary, dict) else {}
        localization = summary.get("localization") if isinstance(summary, dict) else None
        summary_route = summary.get("route_state") if isinstance(summary, dict) else None
        events = {
            str(record.get("event"))
            for record in timeline_records
            if isinstance(record, dict)
        }

        result["checks"]["summary_schema_valid"] = (
            summary.get("schema") == "agt_v25_11e_summary/v1"
            and summary.get("gate") == "V25-11E"
        )
        result["checks"]["summary_route_succeeded"] = (
            isinstance(summary_route, dict)
            and summary_route.get("state") == "SUCCEEDED"
        )
        result["checks"]["summary_localization_tracking"] = bool(
            isinstance(localization, dict)
            and localization.get("state") == localization.get("tracking_constant")
            and localization.get("localization_accepted")
            and localization.get("pose_valid")
        )
        result["checks"]["motion_observed"] = (
            float(metrics.get("max_displacement_m", 0.0)) >= 0.5
            and float(metrics.get("max_cmd_norm", 0.0)) >= 0.05
            and metrics.get("first_nonzero_cmd_ros_ns") is not None
        )
        result["checks"]["key_timestamps_recorded"] = all(
            metrics.get(key) is not None
            for key in (
                "first_localization_tracking_ros_ns",
                "route_running_ros_ns",
                "first_nonzero_cmd_ros_ns",
                "route_terminal_ros_ns",
            )
        )
        result["checks"]["timeline_has_core_events"] = {
            "observer_started",
            "localization",
            "route_state",
            "sensor_health",
            "safety",
            "safety_cmd_motion",
        }.issubset(events)

        result["metrics"] = {
            "marker_count": node.marker_count,
            "ground_truth_pose_count": node.truth_pose_count,
            "timeline_record_count": len(timeline_records),
            "summary": summary,
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
