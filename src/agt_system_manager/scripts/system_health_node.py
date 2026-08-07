#!/usr/bin/env python3

"""ROS adapter for the pure health evaluator and task-readiness gate."""

import time
from pathlib import Path
import shutil
import threading

import rclpy
from ament_index_python.packages import get_package_share_directory
import yaml
from agt_interfaces.msg import ComponentHealth as ComponentHealthMsg
from agt_interfaces.msg import (
    LocalizationStatus, MapVersionSummary, SystemHealth, TaskReadiness,
)
from agt_interfaces.srv import EvaluateTaskReadiness, GetSystemHealth
from diagnostic_msgs.msg import DiagnosticArray
from livox_ros_driver2.msg import CustomMsg
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from agt_system_manager.health import HealthEvaluator, TopicObservation
from agt_system_manager.readiness import ReadinessInputs, evaluate_task_readiness


_TOPIC_TYPES = {
    "/agt/sensors/lidar/custom": (CustomMsg, "livox_ros_driver2/msg/CustomMsg"),
    "/agt/sensors/imu/data": (Imu, "sensor_msgs/msg/Imu"),
    "/agt/mapping/odometry": (Odometry, "nav_msgs/msg/Odometry"),
    "/agt/mapping/registered_points": (PointCloud2, "sensor_msgs/msg/PointCloud2"),
    "/agt/map/mapping_occupancy": (OccupancyGrid, "nav_msgs/msg/OccupancyGrid"),
    "/agt/chassis/connected": (Bool, "std_msgs/msg/Bool"),
    "/agt/chassis/odometry": (Odometry, "nav_msgs/msg/Odometry"),
    "/agt/chassis/status": (DiagnosticArray, "diagnostic_msgs/msg/DiagnosticArray"),
    "/agt/safety/status": (DiagnosticArray, "diagnostic_msgs/msg/DiagnosticArray"),
    "/agt/safety/emergency_stop": (Bool, "std_msgs/msg/Bool"),
    "/agt/localization/status": (LocalizationStatus, "agt_interfaces/msg/LocalizationStatus"),
    "/agt/maps/active": (MapVersionSummary, "agt_interfaces/msg/MapVersionSummary"),
    "/agt/map/global_occupancy": (OccupancyGrid, "nav_msgs/msg/OccupancyGrid"),
    "/global_costmap/costmap": (OccupancyGrid, "nav_msgs/msg/OccupancyGrid"),
}

_LATCHED_MAP_QOS = QoSProfile(depth=1)
_LATCHED_MAP_QOS.reliability = ReliabilityPolicy.RELIABLE
_LATCHED_MAP_QOS.durability = DurabilityPolicy.TRANSIENT_LOCAL
_NAV2_LIFECYCLE_NODES = (
    "map_server",
    "planner_server",
    "smoother_server",
    "controller_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "collision_monitor",
)


def _topic_is_fresh(stats, topic: str, timeout: float, now: float) -> bool:
    observation = stats.get(topic)
    return bool(
        observation
        and int(observation.get("count", 0)) > 0
        and now - float(observation.get("last_seen", float("-inf"))) <= timeout
    )


def _safety_gate_from_status(message: DiagnosticArray) -> tuple[bool, bool, bool, bool]:
    for status in message.status:
        if status.name != "agt_safety/tracked_controller":
            continue
        values = {item.key: item.value.strip().lower() for item in status.values}
        required = {
            "motion_enabled",
            "emergency_stop",
            "estop_latched",
            "navigation_ready",
        }
        if not required.issubset(values):
            return True, False, True, False
        motion_enabled = values.get("motion_enabled") == "true"
        emergency_stop = any(
            values[key] != "false" for key in ("emergency_stop", "estop_latched")
        )
        navigation_ready = (
            values.get("navigation_ready") == "true"
            and motion_enabled
            and not emergency_stop
        )
        return True, motion_enabled, emergency_stop, navigation_ready
    return False, False, True, False


def _active_map_gate(message: MapVersionSummary) -> tuple[str, str, str, bool, bool, bool]:
    ready = bool(
        message.active
        and message.state == MapVersionSummary.STATE_READY
        and message.valid
        and message.map_hash
    )
    return (
        str(message.map_id) if message.active else "",
        str(message.map_version_id) if message.active else "",
        str(message.map_hash) if message.active else "",
        ready,
        ready and bool(message.navigation_yaml),
        ready and bool(message.localization_pcd) and bool(message.processing_record),
    )


class SystemHealthNode(Node):
    def __init__(self) -> None:
        super().__init__("agt_system_manager")
        default_contract = str(Path(get_package_share_directory("agt_system_manager")) / "config" / "health_contracts.yaml")
        contract_path = str(self.declare_parameter("health_contract", default_contract).value)
        with open(contract_path, "r", encoding="utf-8") as stream:
            contract = yaml.safe_load(stream)
        self._evaluator = HealthEvaluator(contract)
        self._mode = str(self.declare_parameter("active_mode", "IDLE").value).upper()
        self._runtime_dir = Path(str(self.declare_parameter("runtime_dir", "runtime").value)).expanduser()
        # Kept only so older launch files do not fail parameter declaration.
        # Active map identity itself comes exclusively from /agt/maps/active.
        self.declare_parameter("active_map_pointer", "")
        self._map_id = ""
        self._map_version_id = ""
        self._active_map_hash = ""
        self._map_ready = False
        self._navigation_map_valid = False
        self._localization_pcd_valid = False
        self._active_map_seen = float("-inf")
        # The waypoint Action remains the authoritative per-task validator. This
        # flag represents the shared runtime prerequisites until a future task
        # validator publishes a task-specific decision.
        self._task_valid = bool(self.declare_parameter("task_valid", True).value)
        self._localization_status_timeout = float(
            self.declare_parameter("localization_status_timeout", 10.0).value
        )
        self._safety_status_timeout = float(
            self.declare_parameter("safety_status_timeout", 1.0).value
        )
        self._chassis_status_timeout = float(
            self.declare_parameter("chassis_status_timeout", 1.0).value
        )
        self._active_map_timeout = float(
            self.declare_parameter("active_map_timeout", 3.0).value
        )
        if min(
            self._localization_status_timeout,
            self._safety_status_timeout,
            self._chassis_status_timeout,
            self._active_map_timeout,
        ) <= 0.0:
            raise ValueError("readiness status timeouts must be positive")
        self._min_free_space_bytes = int(float(self.declare_parameter("min_free_space_gb", 1.0).value) * 1024**3)
        self._stats_lock = threading.Lock()
        self._topic_callback_group = ReentrantCallbackGroup()
        self._control_callback_group = MutuallyExclusiveCallbackGroup()
        self._stats: dict[str, dict[str, object]] = {}
        self._conditions: dict[str, object] = {
            "disk.free_space_ok": True,
            "web.process_alive": False,
            "frontend.process_alive": False,
            "rosbag.process_alive": False,
        }
        self._frames: set[str] = set()
        self._nodes: set[str] = set()
        self._lifecycle_states: dict[str, str] = {}
        self._lifecycle_futures = {}
        self._lifecycle_clients = {
            node: self.create_client(GetState, f"/{node}/get_state", callback_group=self._control_callback_group)
            for node in _NAV2_LIFECYCLE_NODES
        }
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._localization = LocalizationStatus()
        self._chassis_connected = False
        # The safety controller is the authoritative owner of the estop latch.
        # Missing the optional input topic must not fabricate an active estop.
        self._emergency_stop = True
        self._safety_status_seen = False
        self._safety_allows_navigation = False
        self._health = None
        self._publisher = self.create_publisher(SystemHealth, "/agt/system/health", 10)
        self._readiness_publisher = self.create_publisher(TaskReadiness, "/agt/system/task_readiness", 10)
        self.create_service(GetSystemHealth, "/agt/system/get_health", self._get_health, callback_group=self._control_callback_group)
        self.create_service(EvaluateTaskReadiness, "/agt/system/evaluate_task_readiness", self._evaluate_readiness, callback_group=self._control_callback_group)
        for topic, (message_type, type_name) in _TOPIC_TYPES.items():
            self.create_subscription(
                message_type,
                topic,
                lambda message, topic=topic, type_name=type_name: self._topic_callback(topic, type_name, message),
                _LATCHED_MAP_QOS if topic == "/agt/map/mapping_occupancy" else 10,
                callback_group=self._topic_callback_group,
            )
        self.create_timer(self._evaluator.period_sec, self._tick, callback_group=self._control_callback_group)
        self._tick()

    def _topic_callback(self, topic: str, type_name: str, message) -> None:
        now = time.monotonic()
        with self._stats_lock:
            stat = self._stats.setdefault(topic, {"count": 0, "first_seen": now, "last_seen": now, "message_type": type_name})
            stat["count"] = int(stat["count"]) + 1
            stat["last_seen"] = now
            if topic == "/agt/localization/status":
                self._localization = message
            elif topic == "/agt/chassis/connected":
                self._chassis_connected = bool(message.data)
            elif topic == "/agt/maps/active":
                (
                    self._map_id,
                    self._map_version_id,
                    self._active_map_hash,
                    self._map_ready,
                    self._navigation_map_valid,
                    self._localization_pcd_valid,
                ) = _active_map_gate(message)
                self._active_map_seen = now
            elif topic == "/agt/safety/status":
                (
                    self._safety_status_seen,
                    motion_enabled,
                    self._emergency_stop,
                    self._safety_allows_navigation,
                ) = _safety_gate_from_status(message)
                self._conditions["safety.motion_enabled"] = motion_enabled
                self._conditions["safety.emergency_stop_clear"] = (
                    self._safety_status_seen and not self._emergency_stop
                )
                self._conditions["safety.navigation_ready"] = (
                    self._safety_status_seen and self._safety_allows_navigation
                )

    def _refresh_graph(self) -> None:
        self._nodes = {
            name.lstrip("/")
            for name, _namespace in self.get_node_names_and_namespaces()
        }
        # Lifecycle states are populated by an optional lifecycle adapter. An
        # unknown state remains a health failure for required lifecycle nodes.

    def _refresh_lifecycle(self) -> None:
        for node, client in self._lifecycle_clients.items():
            if node in self._lifecycle_futures and not self._lifecycle_futures[node].done():
                continue
            if not client.service_is_ready():
                self._lifecycle_states.pop(node, None)
                continue
            future = client.call_async(GetState.Request())
            self._lifecycle_futures[node] = future
            future.add_done_callback(lambda result, node=node: self._lifecycle_done(node, result))

    def _lifecycle_done(self, node: str, future) -> None:
        try:
            self._lifecycle_states[node] = str(future.result().current_state.label).lower()
        except Exception:
            self._lifecycle_states.pop(node, None)

    def _refresh_tf(self) -> None:
        frames = set()
        for target, source in (("map", "odom"), ("odom", "base_footprint"), ("base_link", "lidar_link")):
            try:
                self._tf_buffer.lookup_transform(target, source, Time())
                frames.add(f"{target}->{source}")
            except Exception:
                pass
        self._frames = frames

    def _tick(self) -> None:
        self._mode = str(self.get_parameter("active_mode").value).upper()
        self._refresh_graph()
        self._refresh_lifecycle()
        self._refresh_tf()
        try:
            self._conditions["disk.free_space_ok"] = shutil.disk_usage(self._runtime_dir).free >= self._min_free_space_bytes
        except OSError:
            self._conditions["disk.free_space_ok"] = False
        with self._stats_lock:
            observations = {key: TopicObservation(**value) for key, value in self._stats.items()}
        self._health = self._evaluator.evaluate(
            self._mode,
            observations,
            now=time.monotonic(),
            frames=self._frames,
            nodes=self._nodes,
            lifecycle_states=self._lifecycle_states,
            conditions=self._conditions,
        )
        message = self._to_message(self._health)
        self._publisher.publish(message)
        self._readiness_publisher.publish(self._readiness_message(self._evaluate()))

    @staticmethod
    def _state_value(message_type, state: str) -> int:
        return int(getattr(message_type, f"STATE_{state}", message_type.STATE_UNKNOWN))

    def _to_message(self, snapshot):
        message = SystemHealth()
        message.header.stamp = self.get_clock().now().to_msg()
        message.overall_state = self._state_value(SystemHealth, snapshot.overall_state)
        message.revision = snapshot.revision
        message.blocker_codes = snapshot.blocker_codes
        message.blocker_messages = snapshot.blocker_messages
        message.warning_codes = snapshot.warning_codes
        message.warning_messages = snapshot.warning_messages
        for item in snapshot.components:
            component = ComponentHealthMsg()
            component.header = message.header
            component.component_id = item.component_id
            component.display_name = item.display_name
            component.state = self._state_value(ComponentHealthMsg, item.state)
            component.required = item.required
            component.present = item.present
            component.observed_rate_hz = item.observed_rate_hz
            component.message_age_sec = item.message_age_sec
            component.message_count = item.message_count
            component.missing_topics = item.missing_topics
            component.missing_frames = item.missing_frames
            component.missing_nodes = item.missing_nodes
            component.lifecycle_failures = item.lifecycle_failures
            component.condition_failures = item.condition_failures
            component.warnings = item.warnings
            component.errors = item.errors
            component.detail = item.detail
            message.components.append(component)
        return message

    def _evaluate(self):
        now = time.monotonic()
        with self._stats_lock:
            localization = self._localization
            localization_fresh = _topic_is_fresh(
                self._stats,
                "/agt/localization/status",
                self._localization_status_timeout,
                now,
            )
            safety_fresh = _topic_is_fresh(
                self._stats,
                "/agt/safety/status",
                self._safety_status_timeout,
                now,
            )
            chassis_fresh = _topic_is_fresh(
                self._stats,
                "/agt/chassis/connected",
                self._chassis_status_timeout,
                now,
            )
            emergency_stop = (
                self._emergency_stop
                if self._safety_status_seen and safety_fresh
                else True
            )
            chassis_connected = self._chassis_connected and chassis_fresh
            safety_allows_navigation = self._safety_allows_navigation and safety_fresh
            active_map_fresh = now - self._active_map_seen <= self._active_map_timeout
            map_id = self._map_id if active_map_fresh else ""
            map_version_id = self._map_version_id if active_map_fresh else ""
            active_map_hash = self._active_map_hash if active_map_fresh else ""
            map_ready = self._map_ready and active_map_fresh
            navigation_map_valid = self._navigation_map_valid and active_map_fresh
            localization_pcd_valid = self._localization_pcd_valid and active_map_fresh
        return evaluate_task_readiness(
            ReadinessInputs(
                active_mode=self._mode,
                map_id=map_id,
                map_version_id=map_version_id,
                map_ready=map_ready,
                navigation_map_valid=navigation_map_valid,
                localization_pcd_valid=localization_pcd_valid,
                active_map_hash=active_map_hash,
                localization_map_id=localization.map_id,
                localization_map_hash=localization.map_hash,
                localization_state=self._localization_state(localization.state),
                pose_valid=localization.pose_valid,
                localization_accepted=localization.localization_accepted,
                status_stale=localization.status_stale or not localization_fresh,
                emergency_stop=emergency_stop,
                chassis_connected=chassis_connected,
                safety_allows_navigation=safety_allows_navigation,
                nav2_active=all(
                    self._lifecycle_states.get(node) == "active"
                    for node in _NAV2_LIFECYCLE_NODES
                ),
                tf_chain_fresh={"map->odom", "odom->base_footprint", "base_link->lidar_link"}.issubset(self._frames),
                task_valid=self._task_valid,
                health_revision=self._health.revision if self._health else 0,
            )
        )

    @staticmethod
    def _localization_state(value: int) -> str:
        return {
            LocalizationStatus.STATE_UNINITIALIZED: "UNINITIALIZED",
            LocalizationStatus.STATE_SEARCHING: "SEARCHING",
            LocalizationStatus.STATE_VERIFYING: "VERIFYING",
            LocalizationStatus.STATE_TRACKING: "TRACKING",
            LocalizationStatus.STATE_DEGRADED: "DEGRADED",
            LocalizationStatus.STATE_RECOVERING: "RECOVERING",
            LocalizationStatus.STATE_LOST: "LOST",
            LocalizationStatus.STATE_ERROR: "ERROR",
        }.get(value, "UNKNOWN")

    def _readiness_message(self, result):
        message = TaskReadiness()
        message.header.stamp = self.get_clock().now().to_msg()
        message.ready = result.ready
        message.active_mode = result.active_mode
        message.map_id = result.map_id
        message.map_version_id = result.map_version_id
        message.localization_state = result.localization_state
        message.health_revision = result.health_revision
        message.blocker_codes = result.blocker_codes
        message.blocker_messages = result.blocker_messages
        message.warning_codes = result.warning_codes
        message.warning_messages = result.warning_messages
        return message

    def _get_health(self, request, response):
        del request
        if self._health is None:
            self._tick()
        response.health = self._to_message(self._health)
        return response

    def _evaluate_readiness(self, request, response):
        del request
        response.readiness = self._readiness_message(self._evaluate())
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SystemHealthNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
