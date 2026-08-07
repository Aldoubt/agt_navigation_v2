"""ROS interface adapter used by the Web console runtime."""

import json
import math
from pathlib import Path
import threading
import time
from typing import Any, Mapping

from agt_interfaces.action import (
    ChangeSystemMode,
    ExecuteMission,
    ManageMappingSession,
    Relocalize,
)
from agt_interfaces.msg import (
    BagSessionSummary,
    ExperimentSummary,
    LocalizationStatus,
    MapVersionSummary,
    MissionStatus,
    RobotState,
    SystemHealth,
    TaskReadiness,
)
from diagnostic_msgs.msg import DiagnosticArray
from agt_interfaces.srv import (
    ListBagSessions,
    ListExperiments,
    ListMapVersions,
    ManageBagSession,
    ManageMapVersion,
    SetLocalizationMode,
    SetMissionRunState,
)
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
        self._robot_state = {
            "revision": 0,
            "system_mode": "UNKNOWN",
            "message": "尚未收到统一机器人状态",
        }
        self._mission_status = self._mission_summary(MissionStatus())
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
        self._managed_processes: list[dict[str, Any]] = []
        self._active_mode = "IDLE"
        self._status_listeners: list[Any] = []
        self._mission_goal_handle = None
        self._mode_action = ActionClient(self, ChangeSystemMode, "/agt/system/change_mode", callback_group=group)
        self._mapping_session_action = ActionClient(
            self,
            ManageMappingSession,
            "/agt/mapping/manage_session",
            callback_group=group,
        )
        self._relocalize_action = ActionClient(self, Relocalize, "/agt/localization/relocalize", callback_group=group)
        self._mission_action = ActionClient(
            self, ExecuteMission, "/agt/missions/execute", callback_group=group
        )
        self._localization_mode = self.create_client(SetLocalizationMode, "/agt/localization/set_mode", callback_group=group)
        self._map_list = self.create_client(
            ListMapVersions, "/agt/maps/list", callback_group=group
        )
        self._map_manage = self.create_client(
            ManageMapVersion, "/agt/maps/manage", callback_group=group
        )
        self._bag_list = self.create_client(
            ListBagSessions, "/agt/data/bags/list", callback_group=group
        )
        self._bag_manage = self.create_client(
            ManageBagSession, "/agt/data/bags/manage", callback_group=group
        )
        self._experiment_list = self.create_client(
            ListExperiments, "/agt/data/experiments/list", callback_group=group
        )
        self._mission_run_state = self.create_client(
            SetMissionRunState, "/agt/missions/set_run_state", callback_group=group
        )
        self.create_subscription(
            RobotState,
            "/agt/system/robot_state",
            self._robot_state_callback,
            _LATCHED_MAP_QOS,
            callback_group=group,
        )
        self.create_subscription(
            MissionStatus,
            "/agt/missions/status",
            self._mission_callback,
            _LATCHED_MAP_QOS,
            callback_group=group,
        )
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

    def _robot_state_callback(self, message: RobotState) -> None:
        summary = self._robot_state_summary(message)
        with self._lock:
            self._robot_state = summary
            self._active_mode = str(summary["system_mode"])
            if self._active_mode != "MAPPING":
                self._clear_mapping_previews_locked()
        self._notify_status()

    def _mission_callback(self, message: MissionStatus) -> None:
        with self._lock:
            self._mission_status = self._mission_summary(message)
        self._notify_status()

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
            "robot_state": self.robot_state(),
            "mission": self.mission_status(),
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

    @staticmethod
    def _map_summary(message: MapVersionSummary) -> dict[str, Any]:
        states = {
            MapVersionSummary.STATE_UNKNOWN: "UNKNOWN",
            MapVersionSummary.STATE_DRAFT: "DRAFT",
            MapVersionSummary.STATE_PROCESSING: "PROCESSING",
            MapVersionSummary.STATE_READY: "READY",
            MapVersionSummary.STATE_INVALID: "INVALID",
            MapVersionSummary.STATE_ARCHIVED: "ARCHIVED",
            MapVersionSummary.STATE_DELETED: "DELETED",
        }
        assets = {
            "map": message.navigation_yaml,
            "global_map_pcd": message.localization_pcd,
            "global_map_processing_record": message.processing_record,
            "tasks_directory": message.tasks_directory,
        }
        return {
            "map_id": message.map_id,
            "version_id": message.map_version_id,
            "map_version_id": message.map_version_id,
            "parent_version_id": message.parent_map_version_id,
            "state": states.get(int(message.state), "UNKNOWN"),
            "active": int(message.active),
            "pinned": int(message.pinned),
            "deleted": int(message.deleted),
            "valid": bool(message.valid),
            "map_hash": message.map_hash,
            "manifest_sha256": message.manifest_sha256,
            "assets": assets,
            "storage_bytes": int(message.storage_bytes),
            "created_at": message.created_at,
            "errors": list(message.validation_errors),
            "warnings": list(message.validation_warnings),
        }

    @staticmethod
    def _bag_summary(message: BagSessionSummary) -> dict[str, Any]:
        states = {
            BagSessionSummary.STATE_UNKNOWN: "UNKNOWN",
            BagSessionSummary.STATE_IDLE: "IDLE",
            BagSessionSummary.STATE_RECORDING: "RECORDING",
            BagSessionSummary.STATE_PLAYING: "PLAYING",
            BagSessionSummary.STATE_COMPLETED: "COMPLETED",
            BagSessionSummary.STATE_INTERRUPTED: "INTERRUPTED",
            BagSessionSummary.STATE_ERROR: "ERROR",
        }
        state = states.get(int(message.state), "UNKNOWN")
        return {
            "state": state,
            "bag_id": message.bag_id,
            "experiment_id": message.experiment_id,
            "profile_id": message.profile_id,
            "playback_profile": message.profile_id,
            "relative_uri": message.relative_uri,
            "complete": bool(message.complete),
            "simulation": bool(message.simulation),
            "playback_rate": float(message.playback_rate),
            "rate": float(message.playback_rate),
            "storage_bytes": int(message.storage_bytes),
            "started_at": message.started_at,
            "updated_at": message.updated_at,
            "message": message.message,
            "process_id": int(message.process_id),
            "pid": int(message.process_id),
            "message_count": int(message.message_count),
            "storage_identifier": message.storage_identifier,
            "mapping_input_ready": bool(message.mapping_input_ready),
            "contains_mapping_outputs": bool(message.contains_mapping_outputs),
            "contains_navigation_outputs": bool(message.contains_navigation_outputs),
            "playing": state == "PLAYING",
            "recording": state == "RECORDING",
        }

    @staticmethod
    def _experiment_summary(message: ExperimentSummary) -> dict[str, Any]:
        states = {
            ExperimentSummary.STATE_UNKNOWN: "UNKNOWN",
            ExperimentSummary.STATE_CREATED: "CREATED",
            ExperimentSummary.STATE_RUNNING: "RUNNING",
            ExperimentSummary.STATE_COMPLETED: "COMPLETED",
            ExperimentSummary.STATE_INTERRUPTED: "INTERRUPTED",
            ExperimentSummary.STATE_INVALID: "INVALID",
        }
        return {
            "experiment_id": message.experiment_id,
            "title": message.title,
            "state": states.get(int(message.state), "UNKNOWN"),
            "created_at": message.created_at,
            "start_time": message.start_time,
            "end_time": message.end_time,
            "platform_profile": message.platform_profile,
            "active_map": {
                "map_id": message.map_id,
                "map_version_id": message.map_version_id,
                "manifest_sha256": message.map_hash,
            },
            "launch_profile": message.launch_profile,
            "launch_arguments": {
                "mission_id": message.mission_id,
                "mission_version": message.mission_version,
                "mission_sha256": message.mission_sha256,
            },
            "result_status": message.result_status,
            "config_snapshot_count": int(message.config_snapshot_count),
            "message": message.message,
        }

    @staticmethod
    def _mission_summary(message: MissionStatus) -> dict[str, Any]:
        states = {
            MissionStatus.STATE_IDLE: "IDLE",
            MissionStatus.STATE_VALIDATING: "VALIDATING",
            MissionStatus.STATE_RUNNING: "RUNNING",
            MissionStatus.STATE_WAITING_DURATION: "WAITING_DURATION",
            MissionStatus.STATE_WAITING_EVENT: "WAITING_EVENT",
            MissionStatus.STATE_PAUSING: "PAUSING",
            MissionStatus.STATE_PAUSED: "PAUSED",
            MissionStatus.STATE_RESUMING: "RESUMING",
            MissionStatus.STATE_CANCELING: "CANCELING",
            MissionStatus.STATE_SUCCEEDED: "SUCCEEDED",
            MissionStatus.STATE_FAILED: "FAILED",
            MissionStatus.STATE_CANCELED: "CANCELED",
            MissionStatus.STATE_INTERRUPTED: "INTERRUPTED",
        }
        step_types = {
            MissionStatus.STEP_UNKNOWN: "UNKNOWN",
            MissionStatus.STEP_WAYPOINT_TASK: "WAYPOINT_TASK",
            MissionStatus.STEP_WAIT_DURATION: "WAIT_DURATION",
            MissionStatus.STEP_WAIT_EVENT: "WAIT_EVENT",
        }
        return {
            "state": states.get(int(message.state), "IDLE"),
            "mission_id": message.mission_id,
            "mission_version": message.mission_version,
            "content_sha256": message.content_sha256,
            "map_id": message.map_id,
            "map_version_id": message.map_version_id,
            "map_manifest_sha256": message.map_manifest_sha256,
            "current_step_index": int(message.current_step_index),
            "total_steps": int(message.total_steps),
            "current_step_id": message.current_step_id,
            "current_step_type": step_types.get(int(message.current_step_type), "UNKNOWN"),
            "current_waypoint": int(message.current_waypoint),
            "total_waypoints": int(message.total_waypoints),
            "step_elapsed_s": float(message.step_elapsed_s),
            "step_remaining_s": float(message.step_remaining_s),
            "error_code": int(message.error_code),
            "blocker_codes": list(message.blocker_codes),
            "blocker_messages": list(message.blocker_messages),
            "message": message.message,
        }

    def _robot_state_summary(self, message: RobotState) -> dict[str, Any]:
        modes = {
            RobotState.MODE_UNKNOWN: "UNKNOWN",
            RobotState.MODE_IDLE: "IDLE",
            RobotState.MODE_SENSOR_ONLY: "SENSOR_ONLY",
            RobotState.MODE_MAPPING: "MAPPING",
            RobotState.MODE_LOCALIZATION_DEBUG: "LOCALIZATION_DEBUG",
            RobotState.MODE_NAVIGATION: "NAVIGATION",
            RobotState.MODE_ERROR: "ERROR",
        }
        nav2_states = {
            RobotState.NAV2_UNKNOWN: "UNKNOWN",
            RobotState.NAV2_INACTIVE: "INACTIVE",
            RobotState.NAV2_ACTIVE: "ACTIVE",
            RobotState.NAV2_ERROR: "ERROR",
        }
        return {
            "revision": int(message.revision),
            "system_mode": modes.get(int(message.system_mode), "UNKNOWN"),
            "active_profile": message.active_profile,
            "managed_process_count": int(message.managed_process_count),
            "running_process_count": int(message.running_process_count),
            "system_health_known": bool(message.system_health_known),
            "system_health_freshness_s": self._json_number(message.system_health_freshness_s),
            "task_readiness_known": bool(message.task_readiness_known),
            "task_readiness_freshness_s": self._json_number(message.task_readiness_freshness_s),
            "active_map_known": bool(message.active_map_known),
            "active_map_freshness_s": self._json_number(message.active_map_freshness_s),
            "active_map": self._map_summary(message.active_map),
            "localization_status_known": bool(message.localization_status_known),
            "localization_freshness_s": self._json_number(message.localization_freshness_s),
            "mission_status_known": bool(message.mission_status_known),
            "mission_freshness_s": self._json_number(message.mission_freshness_s),
            "mission": self._mission_summary(message.mission),
            "nav2_state": nav2_states.get(int(message.nav2_state), "UNKNOWN"),
            "nav2_freshness_s": self._json_number(message.nav2_freshness_s),
            "safety_status_known": bool(message.safety_status_known),
            "safety_motion_enabled": bool(message.safety_motion_enabled),
            "emergency_stop": bool(message.emergency_stop),
            "estop_latched": bool(message.estop_latched),
            "navigation_ready": bool(message.navigation_ready),
            "safety_freshness_s": self._json_number(message.safety_freshness_s),
            "chassis_status_known": bool(message.chassis_status_known),
            "chassis_connected": bool(message.chassis_connected),
            "chassis_control_mode": int(message.chassis_control_mode),
            "chassis_status_freshness_s": self._json_number(message.chassis_status_freshness_s),
            "chassis_odometry_freshness_s": self._json_number(message.chassis_odometry_freshness_s),
            "bag_status_known": bool(message.bag_status_known),
            "bag_freshness_s": self._json_number(message.bag_freshness_s),
            "bag_session": self._bag_summary(message.bag_session),
            "error_code": int(message.error_code),
            "blocker_codes": list(message.blocker_codes),
            "blocker_messages": list(message.blocker_messages),
            "message": message.message,
        }

    def robot_state(self) -> dict[str, Any]:
        with self._lock:
            result = dict(self._robot_state)
            if "active_map" in result:
                result["active_map"] = dict(result["active_map"])
            if "mission" in result:
                result["mission"] = dict(result["mission"])
            if "bag_session" in result:
                result["bag_session"] = dict(result["bag_session"])
            return result

    def mission_status(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._mission_status)

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

    def _call_service(self, client, request, name: str, timeout: float = 10.0):
        if not client.wait_for_service(timeout_sec=min(timeout, 2.0)):
            raise RuntimeError(f"{name} service is unavailable")
        return self._wait(client.call_async(request), timeout=timeout)

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
            robot_state = dict(self._robot_state)
            active_mode = (
                robot_state.get("system_mode")
                or self._readiness.get("active_mode")
                or self._active_mode
                or "UNKNOWN"
            )
            processes = list(self._managed_processes)
        return {
            "active_mode": active_mode,
            "active_profile": str(robot_state.get("active_profile", "")),
            "managed_process_count": int(
                robot_state.get("managed_process_count", len(processes))
            ),
            "running_process_count": int(
                robot_state.get("running_process_count", len(processes))
            ),
            "processes": processes,
        }

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

    def maps(self, *, map_id: str = "", state: str = "") -> list[dict[str, Any]]:
        state_values = {
            "": MapVersionSummary.STATE_UNKNOWN,
            "UNKNOWN": MapVersionSummary.STATE_UNKNOWN,
            "DRAFT": MapVersionSummary.STATE_DRAFT,
            "PROCESSING": MapVersionSummary.STATE_PROCESSING,
            "READY": MapVersionSummary.STATE_READY,
            "INVALID": MapVersionSummary.STATE_INVALID,
            "ARCHIVED": MapVersionSummary.STATE_ARCHIVED,
            "DELETED": MapVersionSummary.STATE_DELETED,
        }
        state_name = str(state or "").upper()
        if state_name not in state_values:
            raise ValueError(f"unsupported map state: {state}")
        request = ListMapVersions.Request()
        request.map_id = str(map_id or "")
        request.state = state_values[state_name]
        request.include_deleted = state_name == "DELETED"
        response = self._call_service(self._map_list, request, "map list")
        if not response.success:
            raise RuntimeError(response.message)
        return [self._map_summary(item) for item in response.versions]

    def _manage_map(
        self,
        operation: int,
        version_id: str = "",
        *,
        confirm_destructive: bool = False,
        values: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        request = ManageMapVersion.Request()
        request.operation = int(operation)
        request.map_version_id = str(version_id)
        request.confirm_destructive = bool(confirm_destructive)
        supplied = dict(values or {})
        request.map_id = str(supplied.get("map_id", ""))
        request.candidate_map_yaml = str(
            supplied.get("map_yaml", supplied.get("candidate_map_yaml", ""))
        )
        request.localization_pcd = str(
            supplied.get("pcd", supplied.get("localization_pcd", ""))
        )
        request.processing_record = str(supplied.get("processing_record", ""))
        request.platform_profile = str(supplied.get("platform_profile", ""))
        request.parent_map_version_id = str(
            supplied.get("parent_map_version_id", "")
        )
        response = self._call_service(self._map_manage, request, "map manage")
        if not response.success:
            raise RuntimeError(response.message)
        return self._map_summary(response.version)

    def validate_map(self, version_id: str) -> dict[str, Any]:
        result = self._manage_map(ManageMapVersion.Request.OP_VALIDATE, version_id)
        return {
            "valid": result["valid"],
            "map_id": result["map_id"],
            "map_version_id": result["version_id"],
            "errors": result["errors"],
            "warnings": result["warnings"],
            "map_hash": result["map_hash"],
            "manifest_sha256": result["manifest_sha256"],
            "assets": result["assets"],
        }

    def activate_map(self, version_id: str) -> dict[str, Any]:
        result = self._manage_map(ManageMapVersion.Request.OP_ACTIVATE, version_id)
        return {**result, "activated": bool(result["active"])}

    def map_action(self, version_id: str, action: str) -> dict[str, Any]:
        operations = {
            "pin": ManageMapVersion.Request.OP_PIN,
            "unpin": ManageMapVersion.Request.OP_UNPIN,
            "archive": ManageMapVersion.Request.OP_ARCHIVE,
            "delete": ManageMapVersion.Request.OP_SOFT_DELETE,
            "purge": ManageMapVersion.Request.OP_PURGE,
        }
        if action not in operations:
            raise ValueError(f"unsupported map action: {action}")
        result = self._manage_map(
            operations[action],
            version_id,
            confirm_destructive=action in {"delete", "purge"},
        )
        return {**result, "action": action}

    def import_map(self, values: Mapping[str, Any]) -> dict[str, Any]:
        allowed = {
            "map_id",
            "map_yaml",
            "pcd",
            "processing_record",
            "platform_profile",
            "parent_map_version_id",
        }
        unknown = set(values) - allowed
        required = {"map_id", "map_yaml", "pcd", "processing_record"}
        if unknown or not required.issubset(values):
            raise ValueError(
                "map import requires map_id, map_yaml, pcd and processing_record; "
                f"unknown={sorted(unknown)}"
            )
        result = self._manage_map(
            ManageMapVersion.Request.OP_IMPORT_CANDIDATE, values=values
        )
        return {
            "valid": result["valid"],
            "map_id": result["map_id"],
            "map_version_id": result["version_id"],
            "errors": result["errors"],
            "warnings": result["warnings"],
        }

    def experiments(self, *, state: str = "") -> list[dict[str, Any]]:
        state_values = {
            "": ExperimentSummary.STATE_UNKNOWN,
            "UNKNOWN": ExperimentSummary.STATE_UNKNOWN,
            "CREATED": ExperimentSummary.STATE_CREATED,
            "RUNNING": ExperimentSummary.STATE_RUNNING,
            "COMPLETED": ExperimentSummary.STATE_COMPLETED,
            "INTERRUPTED": ExperimentSummary.STATE_INTERRUPTED,
            "INVALID": ExperimentSummary.STATE_INVALID,
        }
        state_name = str(state or "").upper()
        if state_name not in state_values:
            raise ValueError(f"unsupported experiment state: {state}")
        request = ListExperiments.Request()
        request.state = state_values[state_name]
        response = self._call_service(
            self._experiment_list, request, "experiment list"
        )
        if not response.success:
            raise RuntimeError(response.message)
        return [self._experiment_summary(item) for item in response.experiments]

    def _manage_bag(
        self, operation: int, values: Mapping[str, Any] | None = None
    ) -> dict[str, Any]:
        supplied = dict(values or {})
        request = ManageBagSession.Request()
        request.operation = int(operation)
        request.bag_id = str(supplied.get("bag_id", ""))
        request.experiment_id = str(supplied.get("experiment_id", ""))
        request.experiment_title = str(
            supplied.get("title", supplied.get("experiment_title", ""))
        )
        request.objective = str(supplied.get("objective", ""))
        request.hypothesis = str(supplied.get("hypothesis", ""))
        request.tags_json = json.dumps(
            supplied.get("tags", []), ensure_ascii=False, allow_nan=False
        )
        request.operator_note = str(supplied.get("operator_note", ""))
        request.profile_id = str(
            supplied.get("profile_id", supplied.get("profile", ""))
        )
        request.playback_rate = float(
            supplied.get("playback_rate", supplied.get("rate", 1.0)) or 1.0
        )
        active_map = supplied.get("active_map", {})
        if not isinstance(active_map, Mapping):
            raise ValueError("active_map must be an object")
        request.map_id = str(supplied.get("map_id", active_map.get("map_id", "")))
        request.map_version_id = str(
            supplied.get("map_version_id", active_map.get("map_version_id", ""))
        )
        request.map_sha256 = str(
            supplied.get(
                "map_sha256",
                active_map.get("manifest_sha256", active_map.get("map_hash", "")),
            )
        )
        launch_arguments = supplied.get("launch_arguments", {})
        if not isinstance(launch_arguments, Mapping):
            raise ValueError("launch_arguments must be an object")
        request.mission_id = str(
            supplied.get("mission_id", launch_arguments.get("mission_id", ""))
        )
        request.mission_version = str(
            supplied.get("mission_version", launch_arguments.get("mission_version", ""))
        )
        request.mission_sha256 = str(
            supplied.get("mission_sha256", launch_arguments.get("mission_sha256", ""))
        )
        request.platform_profile = str(supplied.get("platform_profile", ""))
        request.calibration_profile = str(
            supplied.get(
                "calibration_profile", launch_arguments.get("calibration_profile", "")
            )
        )
        request.nav2_profile = str(supplied.get("nav2_profile", ""))
        request.launch_profile = str(supplied.get("launch_profile", ""))
        request.start_experiment = bool(supplied.get("start_experiment", False))
        request.event_type = str(supplied.get("event_type", ""))
        request.metadata_json = str(supplied.get("metadata_json", ""))
        request.result_status = str(supplied.get("result_status", ""))
        request.reason = str(supplied.get("reason", ""))
        response = self._call_service(self._bag_manage, request, "bag manage")
        if not response.success:
            raise RuntimeError(response.message)
        return self._bag_summary(response.session)

    def bags(self) -> dict[str, Any]:
        request = ListBagSessions.Request()
        response = self._call_service(self._bag_list, request, "bag list")
        if not response.success:
            raise RuntimeError(response.message)
        sessions = [self._bag_summary(item) for item in response.sessions]
        current = self._manage_bag(ManageBagSession.Request.OP_STATUS)
        return {
            "bags": [item for item in sessions if item["complete"]],
            "playback": current
            if current["state"] in {"PLAYING", "ERROR"}
            else {"playing": False, "state": current["state"]},
            "recording": current
            if current["state"] == "RECORDING"
            else {"recording": False, "state": current["state"]},
        }

    def bag_action(
        self, action: str, values: Mapping[str, Any] | None = None
    ) -> dict[str, Any]:
        if action == "play":
            return self._manage_bag(
                ManageBagSession.Request.OP_START_PLAYBACK, values
            )
        if action == "stop":
            return self._manage_bag(ManageBagSession.Request.OP_STOP_PLAYBACK, values)
        raise ValueError(f"unsupported bag action: {action}")

    def create_experiment(self, values: Mapping[str, Any]) -> dict[str, Any]:
        result = self._manage_bag(
            ManageBagSession.Request.OP_CREATE_EXPERIMENT,
            {**dict(values), "start_experiment": False},
        )
        return {
            "experiment_id": result["experiment_id"],
            "state": "CREATED",
            "message": result["message"],
        }

    def experiment_action(
        self,
        experiment_id: str,
        action: str,
        values: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        operations = {
            "start": ManageBagSession.Request.OP_START_EXPERIMENT,
            "event": ManageBagSession.Request.OP_ADD_EXPERIMENT_EVENT,
            "finalize": ManageBagSession.Request.OP_COMPLETE_EXPERIMENT,
            "invalid": ManageBagSession.Request.OP_MARK_EXPERIMENT_INVALID,
            "start_bag": ManageBagSession.Request.OP_START_RECORDING,
            "stop_bag": ManageBagSession.Request.OP_STOP_RECORDING,
        }
        if action not in operations:
            raise ValueError(f"unsupported experiment action: {action}")
        supplied = {**dict(values or {}), "experiment_id": experiment_id}
        if action == "event":
            supplied["event_type"] = str(supplied.pop("type", "operator_event"))
            supplied["metadata_json"] = json.dumps(
                supplied.pop("data", {}), ensure_ascii=False, allow_nan=False
            )
        result = self._manage_bag(operations[action], supplied)
        states = {
            "start": "RUNNING",
            "event": "RUNNING",
            "finalize": str(supplied.get("result_status", "COMPLETED")),
            "invalid": "INVALID",
            "start_bag": "RECORDING",
            "stop_bag": "RUNNING",
        }
        return {**result, "experiment_id": experiment_id, "state": states[action]}

    def execute_mission(self, values: Mapping[str, Any]) -> dict[str, Any]:
        allowed = {"mission_id", "mission_version", "expected_content_sha256"}
        unknown = set(values) - allowed
        if unknown:
            raise ValueError(f"unsupported mission fields: {sorted(unknown)}")
        mission_id = str(values.get("mission_id", "")).strip()
        mission_version = str(values.get("mission_version", "")).strip()
        if not mission_id or not mission_version:
            raise ValueError("mission_id and mission_version are required")
        if not self._mission_action.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("mission Action is unavailable")
        goal = ExecuteMission.Goal()
        goal.mission_id = mission_id
        goal.mission_version = mission_version
        goal.expected_content_sha256 = str(
            values.get("expected_content_sha256", "")
        )

        def feedback_callback(feedback) -> None:
            self._mission_callback(feedback.feedback.status)

        handle = self._wait(
            self._mission_action.send_goal_async(goal, feedback_callback=feedback_callback),
            timeout=5.0,
        )
        if not handle.accepted:
            raise RuntimeError("mission Action rejected the request")
        with self._lock:
            self._mission_goal_handle = handle
        handle.get_result_async().add_done_callback(self._mission_result_callback)
        return {
            "accepted": True,
            "mission_id": mission_id,
            "mission_version": mission_version,
        }

    def _mission_result_callback(self, future) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
            summary = self._mission_summary(result.final_status)
            summary.update(
                {
                    "success": bool(result.success),
                    "audit_log_uri": result.audit_log_uri,
                    "message": result.message,
                }
            )
            with self._lock:
                self._mission_status = summary
                self._mission_goal_handle = None
        except Exception as exc:
            self.get_logger().error(f"mission result failed: {exc}")
            with self._lock:
                self._mission_goal_handle = None
        self._notify_status()

    def set_mission_run_state(self, mission_id: str, command: str) -> dict[str, Any]:
        commands = {
            "pause": SetMissionRunState.Request.COMMAND_PAUSE,
            "resume": SetMissionRunState.Request.COMMAND_RESUME,
        }
        if command not in commands:
            raise ValueError(f"unsupported mission run-state command: {command}")
        request = SetMissionRunState.Request()
        request.command = commands[command]
        request.mission_id = str(mission_id)
        response = self._call_service(
            self._mission_run_state, request, "mission run-state"
        )
        if not response.success:
            raise RuntimeError(response.message)
        return self._mission_summary(response.status)

    def cancel_mission(self, mission_id: str) -> dict[str, Any]:
        with self._lock:
            handle = self._mission_goal_handle
            active_id = str(self._mission_status.get("mission_id", ""))
        if handle is None or (mission_id and active_id and mission_id != active_id):
            raise RuntimeError("no matching active mission")
        response = self._wait(handle.cancel_goal_async(), timeout=5.0)
        if not response.goals_canceling:
            raise RuntimeError("mission cancellation was not accepted")
        return {"accepted": True, "mission_id": active_id, "state": "CANCELING"}

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
            "pgm_ready": bool(result.candidate_map_yaml),
            "pcd_ready": bool(result.processing_record),
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
