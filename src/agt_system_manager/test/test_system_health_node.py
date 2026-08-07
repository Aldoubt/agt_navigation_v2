import importlib.util
from pathlib import Path
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from agt_interfaces.msg import LocalizationStatus, MapVersionSummary
from agt_interfaces.srv import EvaluateTaskReadiness
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import GetState
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "system_health_node.py"
SPEC = importlib.util.spec_from_file_location("system_health_node", SCRIPT)
HEALTH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(HEALTH)


def _status(values, name="agt_safety/tracked_controller"):
    status = DiagnosticStatus(name=name)
    status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
    message = DiagnosticArray()
    message.status = [status]
    return message


def test_safety_gate_accepts_controller_clear_and_ready_status():
    assert HEALTH._safety_gate_from_status(
        _status(
            {
                "motion_enabled": "true",
                "estop_latched": "false",
                "emergency_stop": "false",
                "navigation_ready": "true",
            }
        )
    ) == (True, True, False, True)


def test_safety_gate_rejects_missing_or_foreign_status():
    assert HEALTH._safety_gate_from_status(_status({"motion_enabled": "true"})) == (
        True,
        False,
        True,
        False,
    )
    assert HEALTH._safety_gate_from_status(
        _status({"motion_enabled": "true"}, name="other/node")
    ) == (False, False, True, False)


def test_topic_freshness_expires_cached_readiness_inputs():
    stats = {"/status": {"count": 1, "last_seen": 10.0}}
    assert HEALTH._topic_is_fresh(stats, "/status", 1.0, 11.0)
    assert not HEALTH._topic_is_fresh(stats, "/status", 1.0, 11.001)


def test_sensor_diagnostics_ignore_unrelated_shared_bus_messages():
    sensor = _status({"healthy": "true"}, name="agt_sensor_monitor/lidar")
    unrelated = _status({"healthy": "false"}, name="agt_livox_self_filter")
    assert HEALTH._sensor_health_updates(sensor) == {"lidar": True}
    assert HEALTH._sensor_health_updates(unrelated) == {}


def test_sensor_diagnostic_cache_expires_each_stream_independently():
    values = {"lidar": True, "filtered_lidar": True, "imu": True}
    seen = {"lidar": 10.0, "filtered_lidar": 10.0, "imu": 10.0}
    states, ready = HEALTH._sensor_health_snapshot(values, seen, 2.0, 11.9)
    assert states == {"lidar": True, "filtered_lidar": True, "imu": True}
    assert ready

    seen["imu"] = 8.0
    states, ready = HEALTH._sensor_health_snapshot(values, seen, 2.0, 11.9)
    assert states["lidar"] is True
    assert states["filtered_lidar"] is True
    assert states["imu"] is False
    assert ready is False


def test_active_map_gate_uses_typed_manager_context_only():
    message = MapVersionSummary()
    message.active = True
    message.state = MapVersionSummary.STATE_READY
    message.valid = True
    message.map_id = "map_a"
    message.map_version_id = "version_a"
    message.map_hash = "sha256:" + "1" * 64
    message.navigation_yaml = "/managed/navigation/map.yaml"
    message.localization_pcd = "/managed/pointcloud/map.pcd"
    message.processing_record = "/managed/pointcloud/map.processing.yaml"
    assert HEALTH._active_map_gate(message) == (
        "map_a", "version_a", message.map_hash, True, True, True
    )
    message.active = False
    assert HEALTH._active_map_gate(message) == ("", "", "", False, False, False)


class _ReadinessInputs(Node):
    """ROS-only prerequisites for exercising the real SystemHealthNode gate."""

    _LIFECYCLE_NODES = (
        "map_server", "planner_server", "smoother_server", "controller_server",
        "behavior_server", "bt_navigator", "waypoint_follower", "collision_monitor",
    )

    def __init__(self):
        super().__init__("startup_ordering_readiness_inputs")
        self._localization_publisher = self.create_publisher(
            LocalizationStatus, "/agt/localization/status", 10
        )
        self._localization_enabled = False
        self._map_publisher = self.create_publisher(MapVersionSummary, "/agt/maps/active", 10)
        self._chassis_publisher = self.create_publisher(Bool, "/agt/chassis/connected", 10)
        self._safety_publisher = self.create_publisher(DiagnosticArray, "/agt/safety/status", 10)
        self._diagnostics_publisher = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._tf = StaticTransformBroadcaster(self)
        for name in self._LIFECYCLE_NODES:
            self.create_service(GetState, f"/{name}/get_state", self._active_state)
        self.create_timer(0.05, self._publish_prerequisites)
        self._publish_static_tf()

    @staticmethod
    def _active_state(_request, response):
        response.current_state.id = 3
        response.current_state.label = "active"
        return response

    def enable_valid_localization(self):
        self._localization_enabled = True

    def _publish_static_tf(self):
        transforms = []
        for parent, child in (
            ("map", "odom"), ("odom", "base_footprint"), ("base_link", "lidar_link")
        ):
            transform = TransformStamped()
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        self._tf.sendTransform(transforms)

    def _publish_prerequisites(self):
        active_map = MapVersionSummary()
        active_map.active = True
        active_map.valid = True
        active_map.state = MapVersionSummary.STATE_READY
        active_map.map_id = "startup_map"
        active_map.map_version_id = "v1"
        active_map.map_hash = "sha256:" + "b" * 64
        active_map.navigation_yaml = "managed/navigation.yaml"
        active_map.localization_pcd = "managed/map.pcd"
        active_map.processing_record = "managed/map.processing.yaml"
        self._map_publisher.publish(active_map)

        connected = Bool()
        connected.data = True
        self._chassis_publisher.publish(connected)

        safety = DiagnosticArray()
        safety.status = [_status(
            {
                "motion_enabled": "true",
                "estop_latched": "false",
                "emergency_stop": "false",
                "navigation_ready": "true",
            }
        ).status[0]]
        self._safety_publisher.publish(safety)

        diagnostics = DiagnosticArray()
        diagnostics.status = [
            _status({"healthy": "true"}, name=f"agt_sensor_monitor/{stream}").status[0]
            for stream in ("lidar", "filtered_lidar", "imu")
        ]
        self._diagnostics_publisher.publish(diagnostics)

        if self._localization_enabled:
            localization = LocalizationStatus()
            localization.state = LocalizationStatus.STATE_TRACKING
            localization.pose_valid = True
            localization.localization_accepted = True
            localization.status_stale = False
            localization.map_id = "startup_map"
            localization.map_hash = "sha256:" + "b" * 64
            self._localization_publisher.publish(localization)


def _spin_until(executor, future, timeout=8.0):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    assert future.done(), "bounded startup-ordering ROS future timed out"
    return future.result()


def test_system_health_startup_ordering_recovers_relocalization_readiness():
    rclpy.init()
    inputs = _ReadinessInputs()
    health = HEALTH.SystemHealthNode()
    health.set_parameters([Parameter("active_mode", Parameter.Type.STRING, "NAVIGATION")])
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(inputs)
    executor.add_node(health)
    client = health.create_client(EvaluateTaskReadiness, "/agt/system/evaluate_task_readiness")
    try:
        assert client.wait_for_service(timeout_sec=5.0)
        request = EvaluateTaskReadiness.Request()
        request.gate_profile = EvaluateTaskReadiness.Request.PROFILE_RELOCALIZATION
        first = _spin_until(executor, client.call_async(request))
        assert not first.readiness.ready
        assert "LOCALIZATION_MAP_MISMATCH" in first.readiness.blocker_codes

        inputs.enable_valid_localization()
        deadline = time.monotonic() + 8.0
        recovered = None
        while time.monotonic() < deadline:
            recovered = _spin_until(executor, client.call_async(request))
            if recovered.readiness.ready:
                break
        assert recovered is not None and recovered.readiness.ready
        assert recovered.readiness.map_id == "startup_map"
        assert recovered.readiness.map_version_id == "v1"
    finally:
        executor.shutdown()
        health.destroy_node()
        inputs.destroy_node()
        rclpy.shutdown()
