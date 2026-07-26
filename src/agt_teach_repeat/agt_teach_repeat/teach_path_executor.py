"""Fail-closed Nav2 FollowPath client for an explicitly enabled teach asset."""

from copy import deepcopy
import json
import math
from pathlib import Path
import threading
import time
from uuid import uuid4

from action_msgs.msg import GoalStatus
from agt_interfaces.msg import LocalizationStatus, TaskReadiness
from diagnostic_msgs.msg import DiagnosticArray
from nav2_msgs.action import FollowPath
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import rclpy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .path_io import (
    load_manifest,
    manifest_reference_path,
    resolve_asset,
    verify_manifest_bindings,
)
from .path_processing import nearest_segment_metrics, path_length
from .path_io import load_reference_path
from .ros_utils import latched_qos


class TeachPathExecutor(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__("agt_teach_path_executor", parameter_overrides=parameter_overrides)
        manifest_value = str(self.declare_parameter("manifest", "").value)
        self.execution_enabled = bool(self.declare_parameter("execution_enabled", False).value)
        self.auto_start = bool(self.declare_parameter("auto_start", True).value)
        self.follow_path_action = str(
            self.declare_parameter("follow_path_action", "/follow_path").value
        )
        self.controller_id = str(self.declare_parameter("controller_id", "FollowPath").value)
        self.maximum_linear_speed = float(
            self.declare_parameter("maximum_linear_speed_mps", 0.20).value
        )
        self.localization_timeout = float(
            self.declare_parameter("localization_status_timeout_s", 1.0).value
        )
        self.safety_timeout = float(self.declare_parameter("safety_status_timeout_s", 1.0).value)
        self.readiness_timeout = float(
            self.declare_parameter("task_readiness_timeout_s", 1.0).value
        )
        self.feedback_timeout = float(self.declare_parameter("feedback_timeout_s", 2.0).value)
        self.hard_lateral_error = float(self.declare_parameter("hard_lateral_error_m", 0.30).value)
        self.safety_status_name = str(
            self.declare_parameter(
                "safety_status_name", "agt_safety/tracked_controller"
            ).value
        )
        if not manifest_value:
            raise RuntimeError("manifest parameter is required")
        if not self.controller_id:
            raise RuntimeError("controller_id is required")
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (
                self.maximum_linear_speed,
                self.localization_timeout,
                self.safety_timeout,
                self.readiness_timeout,
                self.feedback_timeout,
                self.hard_lateral_error,
            )
        ):
            raise RuntimeError("execution limits and readiness timeouts must be positive")

        self.manifest_path, self.manifest = load_manifest(manifest_value)
        self.reference = load_reference_path(
            manifest_reference_path(self.manifest_path, self.manifest),
            expected_demo_id=self.manifest["demo_id"],
        )
        profile_document = _load_yaml(self.manifest["platform"]["profile"])
        profile_limit = float(profile_document["platform"]["limits"]["max_forward_velocity"])
        asset_limit = float(
            self.manifest["execution"].get(
                "maximum_linear_speed_mps",
                self.manifest["execution"].get("max_speed_mps", 0.20),
            )
        )
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (profile_limit, asset_limit)
        ):
            raise RuntimeError("profile and manifest speed limits must be positive")
        self.speed_limit = min(self.maximum_linear_speed, asset_limit, profile_limit)
        if self.speed_limit <= 0.0:
            raise RuntimeError("effective speed limit must be positive")
        self.reference_length = path_length(self.reference)
        self._asset_stats = self._snapshot_asset_stats()

        self._lock = threading.RLock()
        self._active = False
        self._start_requested = False
        self._auto_start_consumed = False
        self._child_goal = None
        self._pending_failure = ""
        self._pending_cancel = False
        self._run_id = ""
        self._feedback_at = float("-inf")
        self._distance_to_goal = self.reference_length
        self._localization_ready = False
        self._localization_at = float("-inf")
        self._safety_ready = False
        self._safety_at = float("-inf")
        self._task_ready = False
        self._task_at = float("-inf")
        self._emergency_stop = True
        self._validation_report = None
        self._validated_path = None
        self._current_error = None
        self._executed_path = NavPath()
        self._executed_path.header.frame_id = "map"

        group = ReentrantCallbackGroup()
        self._follow_path = ActionClient(
            self, FollowPath, self.follow_path_action, callback_group=group
        )
        self._status_publisher = self.create_publisher(
            String, "/agt/teach/execution_status", latched_qos()
        )
        self._executed_publisher = self.create_publisher(
            NavPath, "/agt/teach/executed_path", latched_qos()
        )
        self._error_publisher = self.create_publisher(
            String, "/agt/teach/current_error", latched_qos()
        )
        self._speed_publisher = self.create_publisher(SpeedLimit, "/speed_limit", 10)
        self.create_subscription(
            LocalizationStatus,
            "/agt/localization/status",
            self._localization_callback,
            10,
            callback_group=group,
        )
        self.create_subscription(
            DiagnosticArray,
            "/agt/safety/status",
            self._safety_callback,
            10,
            callback_group=group,
        )
        self.create_subscription(
            TaskReadiness,
            "/agt/system/task_readiness",
            self._task_callback,
            10,
            callback_group=group,
        )
        self.create_subscription(
            Bool,
            "/agt/safety/emergency_stop",
            self._estop_callback,
            10,
            callback_group=group,
        )
        self.create_subscription(
            String,
            "/agt/teach/validation_report",
            self._validation_callback,
            latched_qos(),
            callback_group=group,
        )
        self.create_subscription(
            NavPath,
            "/agt/teach/path_validated",
            self._validated_callback,
            latched_qos(),
            callback_group=group,
        )
        self.create_service(Trigger, "/agt/teach/start", self._start_service, callback_group=group)
        self.create_service(
            Trigger, "/agt/teach/cancel", self._cancel_service, callback_group=group
        )
        self.create_timer(0.1, self._watchdog, callback_group=group)
        self._publish_status("IDLE")

    @staticmethod
    def localization_status_is_ready(message):
        return (
            message.state == LocalizationStatus.STATE_TRACKING
            and message.pose_valid
            and message.localization_accepted
            and message.error_code == LocalizationStatus.ERROR_NONE
            and not message.status_stale
        )

    def _snapshot_asset_stats(self):
        paths = (
            self.manifest_path,
            manifest_reference_path(self.manifest_path, self.manifest),
            resolve_asset(self.manifest_path, self.manifest["map"]["map_yaml"]),
            resolve_asset(self.manifest_path, self.manifest["map"]["localization_pcd"]),
            resolve_asset(self.manifest_path, self.manifest["map"]["processing_record"]),
        )
        return {str(path): (path.stat().st_size, path.stat().st_mtime_ns) for path in paths}

    def _assets_unchanged(self):
        try:
            return self._snapshot_asset_stats() == self._asset_stats
        except OSError:
            return False

    def _localization_callback(self, message):
        map_matches = (
            str(message.map_id) == str(self.manifest["map"].get("map_id", ""))
            and str(message.map_hash)
            == str(self.manifest["map"].get("localization_pcd_sha256", ""))
        )
        ready = self.localization_status_is_ready(message) and map_matches
        with self._lock:
            self._localization_ready = ready
            self._localization_at = time.monotonic()
        pose = message.global_pose.pose.pose
        if message.global_pose.header.frame_id == "map" and message.pose_valid:
            self._record_actual_pose(message.global_pose.header, pose)
        if not ready:
            self._cancel_for_failure("LOCALIZATION_NOT_READY")

    def _record_actual_pose(self, header, pose):
        from geometry_msgs.msg import PoseStamped
        from .ros_utils import quaternion_yaw

        try:
            metrics = nearest_segment_metrics(
                self.reference,
                pose.position.x,
                pose.position.y,
                quaternion_yaw(pose.orientation),
            )
        except ValueError:
            self._cancel_for_failure("INVALID_LOCALIZATION_POSE")
            return
        self._current_error = metrics
        stamped = PoseStamped()
        stamped.header = header
        stamped.header.frame_id = "map"
        stamped.pose = pose
        with self._lock:
            if self._active:
                self._executed_path.header.stamp = header.stamp
                self._executed_path.poses.append(stamped)
                self._executed_publisher.publish(self._executed_path)
        error = String()
        error.data = json.dumps(metrics, sort_keys=True, separators=(",", ":"), allow_nan=False)
        self._error_publisher.publish(error)
        if self._active and abs(metrics["cross_track_error"]) > self.hard_lateral_error:
            self._cancel_for_failure("HARD_PATH_DEVIATION")

    def _safety_callback(self, message):
        ready = False
        for status in message.status:
            if status.name != self.safety_status_name:
                continue
            values = {item.key: item.value.lower() for item in status.values}
            ready = (
                values.get("motion_enabled") == "true"
                and values.get("estop_latched") == "false"
            )
            break
        with self._lock:
            self._safety_ready = ready
            self._safety_at = time.monotonic()
        if not ready:
            self._cancel_for_failure("SAFETY_NOT_READY")

    def _task_callback(self, message):
        ready = bool(message.ready) and str(message.map_id) == str(
            self.manifest["map"].get("map_id", "")
        )
        with self._lock:
            self._task_ready = ready
            self._task_at = time.monotonic()
        if not ready:
            self._cancel_for_failure("TASK_READINESS_NOT_READY")

    def _estop_callback(self, message):
        self._emergency_stop = bool(message.data)
        if self._emergency_stop:
            self._cancel_for_failure("EMERGENCY_STOP")

    def _validation_callback(self, message):
        try:
            self._validation_report = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            self._validation_report = None
        if self._active and not self._validation_ready():
            self._cancel_for_failure("PATH_VALIDATION_LOST")

    def _validated_callback(self, message):
        self._validated_path = deepcopy(message)
        if self._active and not message.poses:
            self._cancel_for_failure("VALIDATED_PATH_CLEARED")

    def _validation_ready(self):
        return (
            isinstance(self._validation_report, dict)
            and self._validation_report.get("demo_id") == self.manifest["demo_id"]
            and self._validation_report.get("valid") is True
            and self._validation_report.get("eligible_for_execution") is True
            and self._validated_path is not None
            and self._validated_path.header.frame_id == "map"
            and len(self._validated_path.poses) >= 2
        )

    def _gate_failure(self):
        now = time.monotonic()
        if not self.execution_enabled:
            return "EXECUTION_DISABLED"
        binding = verify_manifest_bindings(self.manifest_path, self.manifest)
        if not binding["valid"]:
            return "ASSET_BINDING_INVALID"
        if not self._assets_unchanged():
            return "ASSET_CHANGED"
        if not self._validation_ready():
            return "PATH_NOT_VALIDATED"
        if not self._localization_ready or now - self._localization_at > self.localization_timeout:
            return "LOCALIZATION_NOT_READY"
        if not self._safety_ready or now - self._safety_at > self.safety_timeout:
            return "SAFETY_NOT_READY"
        if self._emergency_stop:
            return "EMERGENCY_STOP"
        if not self._task_ready or now - self._task_at > self.readiness_timeout:
            return "TASK_READINESS_NOT_READY"
        if not self._follow_path.server_is_ready():
            return "FOLLOW_PATH_UNAVAILABLE"
        if self._active:
            return "TASK_ALREADY_ACTIVE"
        return ""

    def _start_service(self, _request, response):
        failure = self._gate_failure()
        if failure:
            response.success = False
            response.message = failure
            self._publish_status("REJECTED", failure)
            return response
        self._start_requested = True
        response.success = True
        response.message = "teach repeat start accepted"
        return response

    def _cancel_service(self, _request, response):
        if not self._active:
            response.success = False
            response.message = "no active teach repeat task"
            return response
        self._pending_cancel = True
        child = self._child_goal
        if child is not None:
            child.cancel_goal_async()
        response.success = True
        response.message = "Nav2 cancellation requested"
        return response

    def _watchdog(self):
        now = time.monotonic()
        with self._lock:
            active = self._active
        if active:
            if now - self._localization_at > self.localization_timeout:
                self._cancel_for_failure("LOCALIZATION_STATUS_STALE")
            elif now - self._safety_at > self.safety_timeout:
                self._cancel_for_failure("SAFETY_STATUS_STALE")
            elif now - self._task_at > self.readiness_timeout:
                self._cancel_for_failure("TASK_READINESS_STALE")
            elif now - self._feedback_at > self.feedback_timeout:
                self._cancel_for_failure("FOLLOW_PATH_FEEDBACK_STALE")
            elif not self._assets_unchanged():
                self._cancel_for_failure("ASSET_CHANGED")
            return
        should_auto_start = (
            self.execution_enabled
            and self.auto_start
            and not self._auto_start_consumed
        )
        if should_auto_start:
            failure = self._gate_failure()
            if not failure:
                self._auto_start_consumed = True
                self._start_requested = True
        if self._start_requested:
            self._start_requested = False
            self._send_goal()

    def _send_goal(self):
        failure = self._gate_failure()
        if failure:
            self._publish_status("REJECTED", failure)
            return
        self._run_id = time.strftime("run_%Y%m%d_%H%M%S", time.gmtime()) + "_" + uuid4().hex[:6]
        self._pending_failure = ""
        self._pending_cancel = False
        self._feedback_at = time.monotonic()
        self._distance_to_goal = self.reference_length
        self._executed_path = NavPath()
        self._executed_path.header.frame_id = "map"
        self._active = True
        speed = SpeedLimit()
        speed.percentage = False
        speed.speed_limit = self.speed_limit
        self._speed_publisher.publish(speed)
        goal = FollowPath.Goal()
        goal.path = deepcopy(self._validated_path)
        goal.controller_id = self.controller_id
        goal.goal_checker_id = ""
        self._publish_status("STARTING")
        future = self._follow_path.send_goal_async(goal, feedback_callback=self._feedback_callback)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        try:
            child = future.result()
        except Exception as exc:
            self._finish("FAILED", f"FOLLOW_PATH_SEND_FAILED:{exc}")
            return
        if not child.accepted:
            self._finish("FAILED", "FOLLOW_PATH_REJECTED")
            return
        self._child_goal = child
        if self._pending_failure or self._pending_cancel:
            child.cancel_goal_async()
        self._feedback_at = time.monotonic()
        self._publish_status("RUNNING")
        result = child.get_result_async()
        result.add_done_callback(self._result_callback)

    def _feedback_callback(self, message):
        distance = float(message.feedback.distance_to_goal)
        if math.isfinite(distance):
            self._distance_to_goal = max(0.0, distance)
        self._feedback_at = time.monotonic()
        self._publish_status("RUNNING")

    def _result_callback(self, future):
        try:
            status = future.result().status
        except Exception as exc:
            self._finish("FAILED", f"FOLLOW_PATH_RESULT_FAILED:{exc}")
            return
        if self._pending_failure:
            self._finish("FAILED", self._pending_failure)
        elif self._pending_cancel or status == GoalStatus.STATUS_CANCELED:
            self._finish("CANCELED", "USER_CANCELED")
        elif status == GoalStatus.STATUS_SUCCEEDED:
            self._distance_to_goal = 0.0
            self._finish("SUCCEEDED", "")
        else:
            self._finish("FAILED", f"FOLLOW_PATH_STATUS_{status}")

    def _cancel_for_failure(self, reason):
        with self._lock:
            if not self._active or self._pending_failure:
                return
            self._pending_failure = str(reason)
            child = self._child_goal
        self._publish_status("CANCELING", reason)
        if child is not None:
            child.cancel_goal_async()

    def _finish(self, state, reason):
        self._active = False
        self._child_goal = None
        self._clear_speed_limit()
        self._publish_status(state, reason)

    def _clear_speed_limit(self):
        speed = SpeedLimit()
        speed.percentage = False
        speed.speed_limit = 0.0
        self._speed_publisher.publish(speed)

    def _publish_status(self, state, reason=""):
        current = self._current_error or {}
        completed = max(0.0, self.reference_length - self._distance_to_goal)
        message = String()
        message.data = json.dumps(
            {
                "state": state,
                "demo_id": self.manifest["demo_id"],
                "run_id": self._run_id,
                "distance_completed_m": completed,
                "completion_ratio": (
                    min(1.0, completed / self.reference_length)
                    if self.reference_length
                    else 0.0
                ),
                "lateral_error_m": current.get("cross_track_error", 0.0),
                "heading_error_rad": current.get("heading_error", 0.0),
                "controller_id": self.controller_id,
                "speed_limit_mps": self.speed_limit,
                "failure_reason": reason,
            },
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )
        self._status_publisher.publish(message)


def _load_yaml(path):
    import yaml

    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def main(args=None):
    rclpy.init(args=args)
    node = TeachPathExecutor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node._clear_speed_limit()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
