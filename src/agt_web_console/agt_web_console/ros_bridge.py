"""ROS interface adapter used by the Web console runtime."""

import math
import threading
from typing import Any, Mapping

from agt_interfaces.action import ChangeSystemMode, Relocalize
from agt_interfaces.msg import LocalizationStatus, SystemHealth, TaskReadiness
from agt_interfaces.srv import SetLocalizationMode
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class RosConsoleBridge(Node):
    _PROFILE_MODES = {
        "sensor_only": ChangeSystemMode.Goal.MODE_SENSOR_ONLY,
        "mapping": ChangeSystemMode.Goal.MODE_MAPPING,
        "navigation": ChangeSystemMode.Goal.MODE_NAVIGATION,
        "qt_mapping": ChangeSystemMode.Goal.MODE_MAPPING,
        "qt_navigation": ChangeSystemMode.Goal.MODE_NAVIGATION,
        "localization_rviz": ChangeSystemMode.Goal.MODE_LOCALIZATION_DEBUG,
    }

    def __init__(self) -> None:
        super().__init__("agt_web_console_ros_bridge")
        group = ReentrantCallbackGroup()
        self._lock = threading.RLock()
        self._health = {"overall_state": "UNKNOWN", "components": []}
        self._readiness = {"ready": False, "blocker_codes": ["HEALTH_UNAVAILABLE"]}
        self._localization = LocalizationStatus()
        self._managed_processes: list[dict[str, Any]] = []
        self._status_listeners: list[Any] = []
        self._mode_action = ActionClient(self, ChangeSystemMode, "/agt/system/change_mode", callback_group=group)
        self._relocalize_action = ActionClient(self, Relocalize, "/agt/localization/relocalize", callback_group=group)
        self._localization_mode = self.create_client(SetLocalizationMode, "/agt/localization/set_mode", callback_group=group)
        self.create_subscription(SystemHealth, "/agt/system/health", self._health_callback, 10, callback_group=group)
        self.create_subscription(TaskReadiness, "/agt/system/task_readiness", self._readiness_callback, 10, callback_group=group)
        self.create_subscription(LocalizationStatus, "/agt/localization/status", self._localization_callback, 10, callback_group=group)
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def _health_callback(self, message: SystemHealth) -> None:
        with self._lock:
            self._health = {"overall_state": self._state_name(message.overall_state), "revision": message.revision, "blocker_codes": list(message.blocker_codes), "blocker_messages": list(message.blocker_messages), "warning_codes": list(message.warning_codes), "warning_messages": list(message.warning_messages), "components": [{"component_id": item.component_id, "display_name": item.display_name, "state": self._state_name(item.state), "required": item.required, "present": item.present, "observed_rate_hz": self._json_number(item.observed_rate_hz), "message_age_sec": self._json_number(item.message_age_sec), "detail": item.detail} for item in message.components]}
        self._notify_status()

    def _readiness_callback(self, message: TaskReadiness) -> None:
        with self._lock:
            self._readiness = {"ready": message.ready, "active_mode": message.active_mode, "map_id": message.map_id, "map_version_id": message.map_version_id, "localization_state": message.localization_state, "health_revision": message.health_revision, "blocker_codes": list(message.blocker_codes), "blocker_messages": list(message.blocker_messages), "warning_codes": list(message.warning_codes), "warning_messages": list(message.warning_messages)}
        self._notify_status()

    def _localization_callback(self, message: LocalizationStatus) -> None:
        with self._lock:
            self._localization = message
        self._notify_status()

    def add_status_listener(self, callback) -> None:
        with self._lock:
            self._status_listeners.append(callback)

    def _notify_status(self) -> None:
        with self._lock:
            listeners = list(self._status_listeners)
        event = {
            "type": "status",
            "health": self.health(),
            "task_readiness": self.readiness(),
            "localization": self.localization(),
            "mode": self.status(),
        }
        for callback in listeners:
            callback(event)

    @staticmethod
    def _state_name(value: int) -> str:
        return {0: "UNKNOWN", 1: "OK", 2: "WARN", 3: "ERROR"}.get(value, "UNKNOWN")

    @staticmethod
    def _json_number(value: Any) -> float | int | None:
        if isinstance(value, float) and not math.isfinite(value):
            return None
        return value

    def health(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._health)

    def readiness(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._readiness)

    def localization(self) -> dict[str, Any]:
        with self._lock:
            message = self._localization
            return {"state": int(message.state), "pose_valid": message.pose_valid, "localization_accepted": message.localization_accepted, "has_converged": message.has_converged, "ambiguous_result": message.ambiguous_result, "status_stale": message.status_stale, "error_code": int(message.error_code), "message": message.message, "backend": message.backend, "candidate_source": message.candidate_source, "candidate_id": message.candidate_id, "map_id": message.map_id, "map_hash": message.map_hash, "fitness_score": message.fitness_score, "overlap_ratio": message.overlap_ratio, "inlier_ratio": message.inlier_ratio, "ambiguity_score": message.ambiguity_score, "translation_innovation": message.translation_innovation, "yaw_innovation": message.yaw_innovation, "runtime_ms": message.runtime_ms, "tested_candidates": message.tested_candidates, "total_candidates": message.total_candidates, "consecutive_successes": message.consecutive_successes, "consecutive_failures": message.consecutive_failures}

    @staticmethod
    def _wait(future, timeout: float = 10.0):
        event = threading.Event()
        result = []
        future.add_done_callback(lambda completed: (result.append(completed), event.set()))
        if not event.wait(timeout):
            raise RuntimeError("ROS operation timed out")
        return result[0].result()

    def start(self, profile: str, arguments: dict[str, Any]) -> dict[str, Any]:
        if profile not in self._PROFILE_MODES:
            raise ValueError(f"unknown configured profile: {profile}")
        if not self._mode_action.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("system mode Action is unavailable")
        goal = ChangeSystemMode.Goal()
        goal.mode = self._PROFILE_MODES[profile]
        goal.profile = profile
        goal.argument_keys = [str(key) for key in arguments]
        goal.argument_values = [str(arguments[key]) for key in arguments]
        goal.wait_for_health = False
        goal.startup_timeout_s = 30.0
        handle = self._wait(self._mode_action.send_goal_async(goal), timeout=5.0)
        if not handle.accepted:
            raise RuntimeError("system mode Action rejected the profile")
        wrapped = self._wait(handle.get_result_async(), timeout=35.0)
        result = wrapped.result
        if not result.success:
            raise RuntimeError(result.message)
        with self._lock:
            self._managed_processes = [
                {"pid": pid, "profile": result.profile, "log_path": path}
                for pid, path in zip(result.process_ids, result.log_paths)
            ]
        return {"success": result.success, "active_mode": result.active_mode, "profile": result.profile, "process_ids": list(result.process_ids), "log_paths": list(result.log_paths), "message": result.message}

    def stop_all(self) -> list[dict[str, Any]]:
        if not self._mode_action.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("system mode Action is unavailable")
        goal = ChangeSystemMode.Goal()
        goal.mode = ChangeSystemMode.Goal.MODE_IDLE
        handle = self._wait(self._mode_action.send_goal_async(goal), timeout=5.0)
        if not handle.accepted:
            raise RuntimeError("system mode stop was rejected")
        wrapped = self._wait(handle.get_result_async(), timeout=35.0)
        if not wrapped.result.success:
            raise RuntimeError(wrapped.result.message)
        with self._lock:
            self._managed_processes = []
        return [{"pid": pid, "profile": wrapped.result.profile} for pid in wrapped.result.process_ids]

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {"active_mode": self._readiness.get("active_mode", "UNKNOWN"), "processes": list(self._managed_processes)}

    def set_mode(self, mode: str) -> dict[str, Any]:
        if not self._localization_mode.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("localization mode service is unavailable")
        request = SetLocalizationMode.Request()
        request.mode = {"MANUAL_ONLY": 0, "AUTO_ON_START": 1, "AUTO_RECOVERY": 2}[mode]
        response = self._wait(self._localization_mode.call_async(request), timeout=5.0)
        if not response.success:
            raise RuntimeError(response.message)
        return {"mode": mode, "attempts": response.attempts, "message": response.message}

    def relocalize(self, values: Mapping[str, Any] | None = None) -> dict[str, Any]:
        values = dict(values or {})
        allowed = {"mode", "max_candidates", "timeout_s", "use_last_valid_pose", "use_configured_candidates", "use_external_coarse_pose", "publish_debug"}
        unknown = set(values) - allowed
        if unknown:
            raise ValueError(f"unsupported relocalization fields: {sorted(unknown)}")
        if not self._relocalize_action.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("relocalization Action is unavailable")
        goal = Relocalize.Goal()
        modes = {
            "AUTO_SEARCH": Relocalize.Goal.MODE_AUTO_SEARCH,
            "LOCAL_CANDIDATES": Relocalize.Goal.MODE_LOCAL_CANDIDATES,
            "EXTERNAL_COARSE_POSE": Relocalize.Goal.MODE_EXTERNAL_COARSE_POSE,
        }
        mode_name = str(values.get("mode", "AUTO_SEARCH"))
        if mode_name not in modes:
            raise ValueError("unsupported relocalization Action mode")
        goal.mode = modes[mode_name]
        goal.max_candidates = min(max(int(values.get("max_candidates", 128)), 1), 1024)
        goal.timeout_s = min(max(float(values.get("timeout_s", 60.0)), 0.1), 300.0)
        goal.use_last_valid_pose = bool(values.get("use_last_valid_pose", True))
        goal.use_configured_candidates = bool(values.get("use_configured_candidates", True))
        goal.use_external_coarse_pose = bool(values.get("use_external_coarse_pose", True))
        goal.publish_debug = bool(values.get("publish_debug", False))
        handle = self._wait(self._relocalize_action.send_goal_async(goal), timeout=5.0)
        if not handle.accepted:
            raise RuntimeError("relocalization Action rejected the request")
        wrapped = self._wait(handle.get_result_async(), timeout=goal.timeout_s + 10.0)
        result = wrapped.result
        status = result.final_status
        return {
            "success": result.success,
            "error_code": int(status.error_code),
            "failure_reason": result.failure_reason,
            "final_status": {
                "state": int(status.state),
                "pose_valid": status.pose_valid,
                "localization_accepted": status.localization_accepted,
                "has_converged": status.has_converged,
                "ambiguous_result": status.ambiguous_result,
                "status_stale": status.status_stale,
                "map_id": status.map_id,
                "map_hash": status.map_hash,
                "fitness_score": status.fitness_score,
                "overlap_ratio": status.overlap_ratio,
                "inlier_ratio": status.inlier_ratio,
                "ambiguity_score": status.ambiguity_score,
                "translation_innovation": status.translation_innovation,
                "yaw_innovation": status.yaw_innovation,
                "runtime_ms": status.runtime_ms,
                "tested_candidates": status.tested_candidates,
                "total_candidates": status.total_candidates,
                "candidate_source": status.candidate_source,
                "candidate_id": status.candidate_id,
                "message": status.message,
            },
        }

    def close(self) -> None:
        self._executor.shutdown()
        self._thread.join(timeout=2.0)
