"""ROS interface adapter used by the Web console runtime."""

import math
import json
from pathlib import Path
import threading
import time
from typing import Any, Mapping

from agt_interfaces.action import ChangeSystemMode, ManageMappingSession, Relocalize
from agt_interfaces.msg import LocalizationStatus, SystemHealth, TaskReadiness
from diagnostic_msgs.msg import DiagnosticArray
from agt_interfaces.srv import SetLocalizationMode
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import BatteryState, PointCloud2
from std_msgs.msg import Bool


_LATCHED_MAP_QOS = QoSProfile(depth=1)
_LATCHED_MAP_QOS.reliability = ReliabilityPolicy.RELIABLE
_LATCHED_MAP_QOS.durability = DurabilityPolicy.TRANSIENT_LOCAL


class RosConsoleBridge(Node):
    _PROFILE_MODES = {
        "sensor_only": ChangeSystemMode.Goal.MODE_SENSOR_ONLY,
        "mapping": ChangeSystemMode.Goal.MODE_MAPPING,
        "navigation": ChangeSystemMode.Goal.MODE_NAVIGATION,
        "qt_mapping": ChangeSystemMode.Goal.MODE_MAPPING,
        "qt_navigation": ChangeSystemMode.Goal.MODE_NAVIGATION,
        "localization_rviz": ChangeSystemMode.Goal.MODE_LOCALIZATION_DEBUG,
    }

    def __init__(self, runtime_dir: str | Path = "runtime", can_interface: str = "can0") -> None:
        super().__init__("agt_web_console_ros_bridge")
        group = ReentrantCallbackGroup()
        self._lock = threading.RLock()
        self._health = {"overall_state": "UNKNOWN", "components": []}
        self._readiness = {"ready": False, "blocker_codes": ["HEALTH_UNAVAILABLE"]}
        self._localization = LocalizationStatus()
        self._mapping_map: dict[str, Any] = {"available": False, "message": "尚未收到二维建图地图"}
        self._mapping_pointcloud: dict[str, Any] = {
            "available": False,
            "message": "尚未收到注册点云",
        }
        self._robot_pose: dict[str, Any] = {
            "available": False,
            "frame_id": "",
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "age_sec": None,
        }
        self._pointcloud_voxels: dict[tuple[int, int, int], tuple[float, float, float]] = {}
        self._pointcloud_voxel_size = 0.10
        self._pointcloud_max_voxels = 50000
        self._can_interface = str(can_interface).strip() or "can0"
        self._chassis = {
            "available": False,
            "connected": False,
            "diagnostics": [],
            "battery_voltage": None,
            "battery_percentage": None,
            "last_status_age_sec": None,
        }
        self._chassis_last_status = None
        self._runtime_dir = Path(runtime_dir).expanduser().resolve()
        self._status_path = self._runtime_dir / "logs" / "system_manager" / "process_status.json"
        self._managed_processes: list[dict[str, Any]] = []
        self._active_mode = "IDLE"
        self._status_listeners: list[Any] = []
        self._mode_action = ActionClient(self, ChangeSystemMode, "/agt/system/change_mode", callback_group=group)
        self._mapping_session_action = ActionClient(
            self,
            ManageMappingSession,
            "/agt/mapping/manage_session",
            callback_group=group,
        )
        self._relocalize_action = ActionClient(self, Relocalize, "/agt/localization/relocalize", callback_group=group)
        self._localization_mode = self.create_client(SetLocalizationMode, "/agt/localization/set_mode", callback_group=group)
        self.create_subscription(SystemHealth, "/agt/system/health", self._health_callback, 10, callback_group=group)
        self.create_subscription(TaskReadiness, "/agt/system/task_readiness", self._readiness_callback, 10, callback_group=group)
        self.create_subscription(LocalizationStatus, "/agt/localization/status", self._localization_callback, 10, callback_group=group)
        self.create_subscription(OccupancyGrid, "/agt/map/mapping_occupancy", self._mapping_map_callback, _LATCHED_MAP_QOS, callback_group=group)
        self.create_subscription(PointCloud2, "/agt/mapping/registered_points", self._mapping_pointcloud_callback, 2, callback_group=group)
        self.create_subscription(Odometry, "/agt/mapping/odometry", self._mapping_odometry_callback, 10, callback_group=group)
        self.create_subscription(DiagnosticArray, "/agt/chassis/status", self._chassis_diagnostic_callback, 10, callback_group=group)
        self.create_subscription(Bool, "/agt/chassis/connected", self._chassis_connected_callback, 10, callback_group=group)
        self.create_subscription(BatteryState, "/battery", self._battery_callback, 10, callback_group=group)
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
            self._active_mode = str(message.active_mode or self._active_mode).upper()
            # A replay can publish immediately after the managed chain starts,
            # before the first readiness message reports MAPPING. Do not clear
            # that valid preview when the readiness transition catches up.
            if self._active_mode != "MAPPING":
                self._clear_mapping_previews_locked()
        self._notify_status()

    def _localization_callback(self, message: LocalizationStatus) -> None:
        with self._lock:
            self._localization = message
        self._notify_status()

    def _mapping_map_callback(self, message: OccupancyGrid) -> None:
        with self._lock:
            if not self._mapping_active_locked():
                return
        width = int(message.info.width)
        height = int(message.info.height)
        if width <= 0 or height <= 0:
            return
        raw = list(message.data)
        max_cells = 160000
        stride = max(1, int(math.ceil(math.sqrt((width * height) / max_cells))))
        sampled_width = (width + stride - 1) // stride
        sampled_height = (height + stride - 1) // stride
        sampled = []
        for output_y in range(sampled_height):
            source_y = output_y * stride
            for output_x in range(sampled_width):
                source_x = output_x * stride
                values = []
                for cell_y in range(source_y, min(source_y + stride, height)):
                    start = cell_y * width + source_x
                    values.extend(raw[start:min(start + stride, cell_y * width + width)])
                if 100 in values:
                    sampled.append(100)
                elif -1 in values:
                    sampled.append(-1)
                else:
                    sampled.append(max(values) if values else -1)
        with self._lock:
            if not self._mapping_active_locked():
                return
            self._mapping_map = {
                "available": True,
                "frame_id": message.header.frame_id,
                "width": sampled_width,
                "height": sampled_height,
                "resolution": float(message.info.resolution) * stride,
                "origin": {"x": float(message.info.origin.position.x), "y": float(message.info.origin.position.y)},
                "data": sampled,
                "downsample_factor": stride,
            }

    def _mapping_pointcloud_callback(self, message: PointCloud2) -> None:
        try:
            points = point_cloud2.read_points(
                message,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
            with self._lock:
                if not self._mapping_active_locked():
                    return
                for x, y, z in points:
                    x = float(x)
                    y = float(y)
                    z = float(z)
                    if not all(math.isfinite(value) for value in (x, y, z)):
                        continue
                    key = (
                        math.floor(x / self._pointcloud_voxel_size),
                        math.floor(y / self._pointcloud_voxel_size),
                        math.floor(z / self._pointcloud_voxel_size),
                    )
                    self._pointcloud_voxels[key] = (x, y, z)
                if len(self._pointcloud_voxels) > self._pointcloud_max_voxels:
                    keep = int(self._pointcloud_max_voxels * 0.85)
                    for key in list(self._pointcloud_voxels)[: len(self._pointcloud_voxels) - keep]:
                        del self._pointcloud_voxels[key]
                values = list(self._pointcloud_voxels.values())
                self._mapping_pointcloud = {
                    "available": bool(values),
                    "frame_id": message.header.frame_id,
                    "point_count": len(values),
                    "voxel_size": self._pointcloud_voxel_size,
                    "points": values,
                }
        except (AttributeError, TypeError, ValueError, RuntimeError) as error:
            self.get_logger().warning(f"point cloud preview skipped: {error}")

    def _mapping_odometry_callback(self, message: Odometry) -> None:
        with self._lock:
            if not self._mapping_active_locked():
                return
            orientation = message.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
            )
            self._robot_pose = {
                "available": True,
                "frame_id": message.header.frame_id,
                "x": float(message.pose.pose.position.x),
                "y": float(message.pose.pose.position.y),
                "yaw": float(yaw),
                "age_sec": 0.0,
            }

    def _mapping_active_locked(self) -> bool:
        readiness_mode = str(self._readiness.get("active_mode", "")).upper()
        return self._active_mode == "MAPPING" or readiness_mode == "MAPPING"

    def _clear_mapping_previews_locked(self) -> None:
        self._pointcloud_voxels.clear()
        self._robot_pose = {
            "available": False,
            "frame_id": "",
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "age_sec": None,
        }
        self._mapping_map = {"available": False, "message": "未启动建图链，二维栅格地图预览为空"}
        self._mapping_pointcloud = {"available": False, "message": "未启动建图链，点云地图预览为空"}

    def _chassis_diagnostic_callback(self, message: DiagnosticArray) -> None:
        diagnostics = []
        for status in message.status:
            diagnostics.append(
                {
                    "name": status.name,
                    "level": int(status.level),
                    "message": status.message,
                    "values": {item.key: item.value for item in status.values},
                }
            )
        with self._lock:
            self._chassis["available"] = True
            self._chassis["diagnostics"] = diagnostics
            self._chassis_last_status = time.monotonic()

    def _chassis_connected_callback(self, message: Bool) -> None:
        with self._lock:
            self._chassis["available"] = True
            self._chassis["connected"] = bool(message.data)

    def _battery_callback(self, message: BatteryState) -> None:
        with self._lock:
            self._chassis["battery_voltage"] = self._json_number(float(message.voltage))
            self._chassis["battery_percentage"] = self._json_number(float(message.percentage))

    def _can_interface_status(self) -> dict[str, Any]:
        interface_path = Path("/sys/class/net") / self._can_interface
        if not interface_path.is_dir():
            return {
                "interface": self._can_interface,
                "present": False,
                "operstate": "missing",
            }
        try:
            operstate = (interface_path / "operstate").read_text(encoding="ascii").strip()
        except OSError:
            operstate = "unknown"
        return {
            "interface": self._can_interface,
            "present": True,
            "operstate": operstate,
        }

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

    def mapping_status(self) -> dict[str, Any]:
        with self._lock:
            if not self._mapping_active_locked():
                return {"available": False, "active_mode": self._active_mode, "message": "未启动建图链，二维栅格地图预览为空"}
            status = dict(self._mapping_map)
            status["active_mode"] = self._active_mode
            status["robot_pose"] = dict(self._robot_pose)
            return status

    def mapping_pointcloud_status(self) -> dict[str, Any]:
        with self._lock:
            if not self._mapping_active_locked():
                return {"available": False, "active_mode": self._active_mode, "message": "未启动建图链，点云地图预览为空"}
            status = dict(self._mapping_pointcloud)
            status["active_mode"] = self._active_mode
            status["robot_pose"] = dict(self._robot_pose)
            return status

    def chassis_status(self) -> dict[str, Any]:
        with self._lock:
            status = dict(self._chassis)
            status["diagnostics"] = list(self._chassis["diagnostics"])
            last_status = self._chassis_last_status
        status["last_status_age_sec"] = None if last_status is None else max(0.0, time.monotonic() - last_status)
        status["can"] = self._can_interface_status()
        return status

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
        if not self._mode_action.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(
                "系统管理器未运行：未发现 /agt/system/change_mode Action server；"
                "请先启动 agt_system_manager，再检查 ros2 action info /agt/system/change_mode"
            )
        goal = ChangeSystemMode.Goal()
        goal.mode = self._PROFILE_MODES[profile]
        goal.profile = profile
        goal.argument_keys = [str(key) for key in arguments]
        goal.argument_values = [str(arguments[key]) for key in arguments]
        # Sensor-only startup is the hardware probe. Report failed driver
        # initialization instead of treating a launch parent as success.
        goal.wait_for_health = profile == "sensor_only"
        goal.startup_timeout_s = 20.0 if profile == "sensor_only" else 30.0
        handle = self._wait(self._mode_action.send_goal_async(goal), timeout=5.0)
        if not handle.accepted:
            raise RuntimeError("system mode Action rejected the profile")
        wrapped = self._wait(handle.get_result_async(), timeout=25.0 if profile == "sensor_only" else 35.0)
        result = wrapped.result
        if not result.success:
            log_hint = f"；日志：{result.log_paths[0]}" if result.log_paths else ""
            raise RuntimeError(f"{result.message}{log_hint}")
        with self._lock:
            self._active_mode = str(result.active_mode or "UNKNOWN").upper()
            if self._active_mode != "MAPPING":
                self._clear_mapping_previews_locked()
            self._managed_processes = [
                {"pid": pid, "profile": Path(path).stem or result.profile, "log_path": path}
                for pid, path in zip(result.process_ids, result.log_paths)
            ]
        return {"success": result.success, "active_mode": result.active_mode, "profile": result.profile, "process_ids": list(result.process_ids), "log_paths": list(result.log_paths), "message": result.message}

    def stop_all(self) -> list[dict[str, Any]]:
        if not self._mode_action.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(
                "系统管理器未运行：未发现 /agt/system/change_mode Action server；"
                "请先启动 agt_system_manager，再检查 ros2 action info /agt/system/change_mode"
            )
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
            self._active_mode = "IDLE"
            self._clear_mapping_previews_locked()
        return [{"pid": pid, "profile": wrapped.result.profile} for pid in wrapped.result.process_ids]

    def status(self) -> dict[str, Any]:
        with self._lock:
            active_mode = self._readiness.get("active_mode") or self._active_mode or "UNKNOWN"
            processes = list(self._managed_processes)
        try:
            with open(self._status_path, "r", encoding="utf-8") as stream:
                manager_status = json.load(stream)
            if isinstance(manager_status, list):
                processes = manager_status
        except (OSError, TypeError, ValueError, json.JSONDecodeError):
            pass
        return {"active_mode": active_mode, "processes": processes}

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

    def manage_mapping_session(
        self,
        operation: str,
        *,
        map_id: str = "",
        session_id: str = "",
        arguments: Mapping[str, Any] | None = None,
        activate: bool = False,
        timeout_s: float = 120.0,
    ) -> dict[str, Any]:
        operations = {
            "status": ManageMappingSession.Goal.OP_STATUS,
            "start": ManageMappingSession.Goal.OP_START,
            "finalize": ManageMappingSession.Goal.OP_FINALIZE_CAPTURE,
            "commit": ManageMappingSession.Goal.OP_COMMIT,
            "discard": ManageMappingSession.Goal.OP_DISCARD,
        }
        if operation not in operations:
            raise ValueError(f"unsupported mapping-session operation: {operation}")
        if not self._mapping_session_action.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(
                "系统管理器未运行：未发现 /agt/mapping/manage_session Action server"
            )
        goal = ManageMappingSession.Goal()
        goal.operation = operations[operation]
        goal.map_id = str(map_id)
        goal.session_id = str(session_id)
        values = {str(key): str(value) for key, value in (arguments or {}).items()}
        goal.argument_keys = list(values)
        goal.argument_values = [values[key] for key in values]
        goal.activate_after_commit = bool(activate)
        goal.timeout_s = min(max(float(timeout_s), 0.1), 300.0)
        handle = self._wait(
            self._mapping_session_action.send_goal_async(goal), timeout=10.0
        )
        if not handle.accepted:
            raise RuntimeError("mapping-session Action rejected the request")
        wrapped = self._wait(handle.get_result_async(), timeout=goal.timeout_s + 20.0)
        result = wrapped.result
        if not result.success:
            if (
                operation == "status"
                and result.error_code == ManageMappingSession.Result.ERROR_NOT_FOUND
            ):
                return {"success": False, "available": False, "state": "IDLE", "message": result.message}
            raise RuntimeError(f"{result.message}（错误码 {result.error_code}）")
        response = {
            "success": True,
            "available": result.state != "DISCARDED",
            "state": result.state,
            "session_id": result.session_id,
            "map_id": result.map_id,
            "map_name": result.map_id,
            "map_version_id": result.map_version_id,
            "version_id": result.map_version_id,
            "session_file": result.session_file,
            "candidate_map_yaml": result.candidate_map_yaml,
            "candidate_map_image": result.candidate_map_image,
            "localization_pcd": result.localization_pcd,
            "processing_record": result.processing_record,
            "bag_directory": result.bag_directory,
            "registered_map_yaml": result.registered_map_yaml,
            "tasks_directory": result.tasks_directory,
            "pgm_ready": bool(result.candidate_map_yaml and Path(result.candidate_map_yaml).is_file()),
            "pcd_ready": bool(result.processing_record and Path(result.processing_record).is_file()),
            "message": result.message,
        }
        if result.session_file:
            response["root"] = str(Path(result.session_file).resolve().parent)
        with self._lock:
            if result.state == "MAPPING":
                self._active_mode = "MAPPING"
            elif result.state in {"CANDIDATE_READY", "REGISTERED", "DISCARDED"}:
                self._active_mode = "IDLE"
                self._clear_mapping_previews_locked()
        return response
