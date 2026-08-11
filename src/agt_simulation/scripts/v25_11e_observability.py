#!/usr/bin/env python3

from __future__ import annotations

import json
import math
from pathlib import Path
import traceback
from typing import Any

import rclpy
from agt_interfaces.msg import LocalizationStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker


TRANSIENT_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

SENSOR_SUMMARY_NAME = "agt_sensor_monitor/summary"
SAFETY_STATUS_NAME = "agt_safety/tracked_controller"


def _atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def _diagnostic_values(status) -> dict[str, str]:
    return {item.key: item.value for item in status.values}


class Observability(Node):
    def __init__(self) -> None:
        super().__init__("agt_v25_11e_observability")
        self.timeline_path = Path(
            str(
                self.declare_parameter(
                    "timeline_path", "/tmp/agt_v25_11e_timeline.jsonl"
                ).value
            )
        )
        self.summary_path = Path(
            str(
                self.declare_parameter(
                    "summary_path", "/tmp/agt_v25_11e_summary.json"
                ).value
            )
        )
        self.marker_frame = str(
            self.declare_parameter("marker_frame", "base_footprint").value
        )
        self.max_ground_truth_poses = int(
            self.declare_parameter("max_ground_truth_poses", 5000).value
        )
        if self.max_ground_truth_poses < 10:
            raise RuntimeError("max_ground_truth_poses must be >= 10")

        self.timeline_path.parent.mkdir(parents=True, exist_ok=True)
        self.timeline = self.timeline_path.open("w", encoding="utf-8", buffering=1)
        self.closed = False

        self.localization: LocalizationStatus | None = None
        self.route_state: dict[str, Any] | None = None
        self.sensor_summary: dict[str, Any] | None = None
        self.safety_status: dict[str, Any] | None = None
        self.latest_cmd = Twist()
        self.latest_cmd_norm = 0.0
        self.max_cmd_norm = 0.0
        self.first_truth: Odometry | None = None
        self.latest_truth: Odometry | None = None
        self.previous_truth_xy: tuple[float, float] | None = None
        self.total_distance_m = 0.0
        self.max_displacement_m = 0.0
        self.truth_path = NavPath()

        self.event_counts: dict[str, int] = {}
        self.last_event_value: dict[str, Any] = {}
        self.first_localization_tracking_ros_ns: int | None = None
        self.route_running_ros_ns: int | None = None
        self.first_nonzero_cmd_ros_ns: int | None = None
        self.route_terminal_ros_ns: int | None = None
        self.fatal_error: str | None = None

        # Five status rows are published as individual Marker messages instead of a
        # MarkerArray. This keeps the RViz contract simple and avoids the Humble
        # Python MarkerArray conversion path that failed during the first 11E run.
        self.marker_pub = self.create_publisher(
            Marker, "/agt/validation/status_markers", TRANSIENT_QOS
        )
        self.truth_path_pub = self.create_publisher(
            NavPath, "/agt/validation/ground_truth_path", TRANSIENT_QOS
        )

        self.create_subscription(
            LocalizationStatus, "/agt/localization/status", self._on_localization, 20
        )
        self.create_subscription(
            String,
            "/agt/simulation/navigation/route_state",
            self._on_route_state,
            TRANSIENT_QOS,
        )
        self.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diagnostics, 20
        )
        self.create_subscription(
            DiagnosticArray, "/agt/safety/status", self._on_safety, 20
        )
        self.create_subscription(Twist, "/agt/safety/cmd_vel", self._on_cmd, 20)
        self.create_subscription(
            Odometry, "/simulation/bunker/ground_truth", self._on_truth, 20
        )

        self.marker_timer = self.create_timer(0.20, self._publish_markers)
        self.summary_timer = self.create_timer(0.50, self._write_summary)
        self._record_event("observer_started", {"schema": "agt_v25_11e_timeline/v1"})
        self._write_summary()

    def _ros_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _record_event(self, event: str, payload: dict[str, Any]) -> None:
        record = {"ros_time_ns": self._ros_ns(), "event": event, **payload}
        self.timeline.write(json.dumps(record, separators=(",", ":")) + "\n")
        self.event_counts[event] = self.event_counts.get(event, 0) + 1

    def _record_change(self, event: str, value: Any, payload: dict[str, Any]) -> None:
        if self.last_event_value.get(event) == value:
            return
        self.last_event_value[event] = value
        self._record_event(event, payload)

    def _on_localization(self, msg: LocalizationStatus) -> None:
        self.localization = msg
        value = (
            int(msg.state),
            bool(msg.localization_accepted),
            bool(msg.pose_valid),
            int(msg.correction_generation),
            int(msg.error_code),
        )
        self._record_change(
            "localization",
            value,
            {
                "state": int(msg.state),
                "localization_accepted": bool(msg.localization_accepted),
                "pose_valid": bool(msg.pose_valid),
                "correction_generation": int(msg.correction_generation),
                "error_code": int(msg.error_code),
                "map_id": str(msg.map_id),
                "map_hash": str(msg.map_hash),
                "message": str(msg.message),
            },
        )
        if (
            self.first_localization_tracking_ros_ns is None
            and msg.state == LocalizationStatus.STATE_TRACKING
            and msg.localization_accepted
            and msg.pose_valid
        ):
            self.first_localization_tracking_ros_ns = self._ros_ns()

    def _on_route_state(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            data = {"state": "INVALID", "message": msg.data}
        if not isinstance(data, dict):
            data = {"state": "INVALID", "message": str(data)}
        self.route_state = data
        state = str(data.get("state", "UNKNOWN"))
        self._record_change("route_state", state, dict(data))
        now_ns = self._ros_ns()
        if state == "RUNNING" and self.route_running_ros_ns is None:
            self.route_running_ros_ns = now_ns
        if state in {"SUCCEEDED", "FAILED"} and self.route_terminal_ros_ns is None:
            self.route_terminal_ros_ns = now_ns
            self._write_summary()

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.name != SENSOR_SUMMARY_NAME:
                continue
            values = _diagnostic_values(status)
            ready = values.get("required_streams_healthy", "false").strip().lower() == "true"
            data = {
                "level": int(status.level),
                "message": str(status.message),
                "required_streams_healthy": ready,
            }
            self.sensor_summary = data
            self._record_change(
                "sensor_health",
                (data["level"], data["message"], ready),
                data,
            )

    def _on_safety(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.name != SAFETY_STATUS_NAME:
                continue
            values = _diagnostic_values(status)
            data = {
                "level": int(status.level),
                "reason": str(status.message),
                "navigation_ready": values.get("navigation_ready", "false").strip().lower()
                == "true",
                "localization_valid": values.get("localization_valid", "false")
                .strip()
                .lower()
                == "true",
                "sensor_input_ready": values.get("sensor_input_ready", "false")
                .strip()
                .lower()
                == "true",
            }
            self.safety_status = data
            self._record_change(
                "safety",
                (
                    data["level"],
                    data["reason"],
                    data["navigation_ready"],
                    data["localization_valid"],
                    data["sensor_input_ready"],
                ),
                data,
            )

    def _on_cmd(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.latest_cmd_norm = math.hypot(float(msg.linear.x), float(msg.angular.z))
        self.max_cmd_norm = max(self.max_cmd_norm, self.latest_cmd_norm)
        moving = self.latest_cmd_norm >= 0.05
        self._record_change(
            "safety_cmd_motion",
            moving,
            {
                "moving": moving,
                "linear_x": float(msg.linear.x),
                "angular_z": float(msg.angular.z),
                "command_norm": self.latest_cmd_norm,
            },
        )
        if moving and self.first_nonzero_cmd_ros_ns is None:
            self.first_nonzero_cmd_ros_ns = self._ros_ns()

    @staticmethod
    def _pose_from_odom(msg: Odometry) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp.sec = int(msg.header.stamp.sec)
        pose.header.stamp.nanosec = int(msg.header.stamp.nanosec)
        pose.header.frame_id = str(msg.header.frame_id or "odom")
        source = msg.pose.pose
        pose.pose.position.x = float(source.position.x)
        pose.pose.position.y = float(source.position.y)
        pose.pose.position.z = float(source.position.z)
        pose.pose.orientation.x = float(source.orientation.x)
        pose.pose.orientation.y = float(source.orientation.y)
        pose.pose.orientation.z = float(source.orientation.z)
        pose.pose.orientation.w = float(source.orientation.w)
        return pose

    def _on_truth(self, msg: Odometry) -> None:
        self.latest_truth = msg
        if self.first_truth is None:
            self.first_truth = msg

        point = msg.pose.pose.position
        xy = (float(point.x), float(point.y))
        if self.previous_truth_xy is not None:
            step = math.hypot(xy[0] - self.previous_truth_xy[0], xy[1] - self.previous_truth_xy[1])
            if math.isfinite(step):
                self.total_distance_m += step
        self.previous_truth_xy = xy

        if self.first_truth is not None:
            origin = self.first_truth.pose.pose.position
            displacement = math.hypot(float(point.x - origin.x), float(point.y - origin.y))
            self.max_displacement_m = max(self.max_displacement_m, displacement)

        pose = self._pose_from_odom(msg)
        if not self.truth_path.header.frame_id:
            self.truth_path.header.frame_id = pose.header.frame_id
        self.truth_path.header.stamp.sec = int(msg.header.stamp.sec)
        self.truth_path.header.stamp.nanosec = int(msg.header.stamp.nanosec)
        self.truth_path.poses.append(pose)
        if len(self.truth_path.poses) > self.max_ground_truth_poses:
            del self.truth_path.poses[: len(self.truth_path.poses) - self.max_ground_truth_poses]
        self.truth_path_pub.publish(self.truth_path)

    @staticmethod
    def _marker_color(level: str) -> tuple[float, float, float]:
        if level == "ok":
            return (0.1, 1.0, 0.1)
        if level == "warn":
            return (1.0, 0.8, 0.1)
        return (1.0, 0.2, 0.2)

    def _text_marker(self, marker_id: int, z: float, text: str, level: str) -> Marker:
        marker = Marker()
        now = self.get_clock().now().to_msg()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp.sec = int(now.sec)
        marker.header.stamp.nanosec = int(now.nanosec)
        marker.ns = "v25_11e_status"
        marker.id = int(marker_id)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.18
        red, green, blue = self._marker_color(level)
        marker.color.r = float(red)
        marker.color.g = float(green)
        marker.color.b = float(blue)
        marker.color.a = 1.0
        marker.text = str(text)
        # lifetime remains zero: RViz keeps each id until the next update.
        return marker

    def _publish_markers(self) -> None:
        localization_text = "LOC: waiting"
        localization_level = "warn"
        if self.localization is not None:
            tracking = (
                self.localization.state == LocalizationStatus.STATE_TRACKING
                and self.localization.localization_accepted
                and self.localization.pose_valid
            )
            localization_level = "ok" if tracking else "error"
            localization_text = (
                f"LOC: state={int(self.localization.state)} "
                f"gen={int(self.localization.correction_generation)} "
                f"accepted={bool(self.localization.localization_accepted)}"
            )

        route_text = "ROUTE: waiting"
        route_level = "warn"
        if self.route_state is not None:
            state = str(self.route_state.get("state", "UNKNOWN"))
            route_level = "ok" if state in {"RUNNING", "SUCCEEDED"} else (
                "error" if state == "FAILED" else "warn"
            )
            route_text = (
                f"ROUTE: {state} "
                f"{self.route_state.get('reached_segments', 0)}/"
                f"{self.route_state.get('total_segments', 0)}"
            )

        sensor_text = "SENSOR: waiting"
        sensor_level = "warn"
        if self.sensor_summary is not None:
            ready = bool(self.sensor_summary["required_streams_healthy"])
            sensor_level = "ok" if ready else "error"
            sensor_text = (
                f"SENSOR: ready={ready} level={self.sensor_summary['level']} "
                f"{self.sensor_summary['message']}"
            )

        safety_text = "SAFETY: waiting"
        safety_level = "warn"
        if self.safety_status is not None:
            ready = bool(self.safety_status["navigation_ready"])
            reason = str(self.safety_status["reason"])
            safety_level = "ok" if ready or reason == "navigation" else (
                "warn" if reason == "input_timeout" else "error"
            )
            safety_text = f"SAFETY: ready={ready} reason={reason}"

        motion_level = "ok" if self.latest_cmd_norm < 0.05 else "warn"
        motion_text = (
            f"CMD: v={float(self.latest_cmd.linear.x):.3f} "
            f"w={float(self.latest_cmd.angular.z):.3f} "
            f"distance={self.total_distance_m:.2f}m"
        )

        marker_specs = (
            (0, 1.20, localization_text, localization_level),
            (1, 1.42, route_text, route_level),
            (2, 1.64, sensor_text, sensor_level),
            (3, 1.86, safety_text, safety_level),
            (4, 2.08, motion_text, motion_level),
        )
        for marker_id, z, text, level in marker_specs:
            self.marker_pub.publish(self._text_marker(marker_id, z, text, level))

    def _summary(self) -> dict[str, Any]:
        localization = None
        if self.localization is not None:
            localization = {
                "state": int(self.localization.state),
                "tracking_constant": int(LocalizationStatus.STATE_TRACKING),
                "localization_accepted": bool(self.localization.localization_accepted),
                "pose_valid": bool(self.localization.pose_valid),
                "correction_generation": int(self.localization.correction_generation),
                "map_id": str(self.localization.map_id),
                "map_hash": str(self.localization.map_hash),
                "error_code": int(self.localization.error_code),
                "message": str(self.localization.message),
            }

        return {
            "schema": "agt_v25_11e_summary/v1",
            "gate": "V25-11E",
            "ros_time_ns": self._ros_ns(),
            "route_state": self.route_state,
            "localization": localization,
            "sensor_health": self.sensor_summary,
            "safety": self.safety_status,
            "fatal_error": self.fatal_error,
            "metrics": {
                "ground_truth_pose_count": len(self.truth_path.poses),
                "total_distance_m": self.total_distance_m,
                "max_displacement_m": self.max_displacement_m,
                "max_cmd_norm": self.max_cmd_norm,
                "latest_cmd_norm": self.latest_cmd_norm,
                "first_localization_tracking_ros_ns": self.first_localization_tracking_ros_ns,
                "route_running_ros_ns": self.route_running_ros_ns,
                "first_nonzero_cmd_ros_ns": self.first_nonzero_cmd_ros_ns,
                "route_terminal_ros_ns": self.route_terminal_ros_ns,
            },
            "event_counts": dict(sorted(self.event_counts.items())),
            "artifacts": {
                "timeline": str(self.timeline_path),
                "summary": str(self.summary_path),
            },
        }

    def _write_summary(self) -> None:
        _atomic_write_json(self.summary_path, self._summary())

    def record_fatal(self, error: BaseException) -> None:
        self.fatal_error = f"{type(error).__name__}: {error}"
        self._record_event(
            "observer_fatal",
            {
                "error": self.fatal_error,
                "traceback": traceback.format_exc(),
            },
        )
        self._write_summary()

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        self._record_event("observer_stopped", {})
        self._write_summary()
        self.timeline.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Observability()
    exit_code = 0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:  # noqa: BLE001 - persist runtime diagnostics
        exit_code = 1
        node.record_fatal(error)
        node.get_logger().error(node.fatal_error or "observer fatal error")
        traceback.print_exc()
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
