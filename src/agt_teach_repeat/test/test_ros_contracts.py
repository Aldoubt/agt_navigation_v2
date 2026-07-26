import json
from pathlib import Path
import secrets
import time

from agt_interfaces.msg import LocalizationStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid, Path as NavPath
import pytest
import rclpy
from rclpy.parameter import Parameter

from agt_teach_repeat.path_io import (
    atomic_write_json,
    atomic_write_yaml,
    sha256_file,
    write_reference_paths,
)
from agt_teach_repeat.path_types import PathPose
from agt_teach_repeat.repeatability_evaluator import RepeatabilityEvaluator
from agt_teach_repeat.teach_path_executor import TeachPathExecutor
from agt_teach_repeat.teach_path_publisher import TeachPathPublisher
from agt_teach_repeat.teach_path_validator import TeachPathValidator
from std_msgs.msg import String


ROOT = Path(__file__).resolve().parents[3]
PROFILE = ROOT / "profiles/platforms/bunker.yaml"


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    # Keep synthetic contract topics off any live robot or preview graph.
    rclpy.init(domain_id=20 + secrets.randbelow(180))
    yield
    rclpy.shutdown()


def make_asset(tmp_path):
    processed = tmp_path / "processed"
    audit = tmp_path / "audit"
    audit.mkdir()
    poses = (
        PathPose(1, 2.0, 2.0, frame_id="map"),
        PathPose(2, 4.0, 2.0, frame_id="map"),
    )
    reference = write_reference_paths(processed, "route_01", poses)["yaml"]
    control = processed / "task_control_points.json"
    atomic_write_json(
        control,
        {
            "schema_version": 1,
            "name": "route_01",
            "points": [
                {"name": "P000", "x": 2.0, "y": 2.0, "theta": 0.0},
                {"name": "P001", "x": 4.0, "y": 2.0, "theta": 0.0},
            ],
        },
    )
    image = tmp_path / "map.pgm"
    image.write_bytes(b"P5\n1 1\n255\n\xfe")
    map_yaml = tmp_path / "map.yaml"
    map_yaml.write_text("image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n", encoding="utf-8")
    pcd = tmp_path / "map.pcd"
    pcd.write_bytes(b"pcd")
    record = tmp_path / "processing.yaml"
    record.write_text("state: ready\nmap_file: map.pcd\n", encoding="utf-8")
    manifest = {
        "schema_version": 1,
        "demo_id": "route_01",
        "source": {
            "bag_path": "bag",
            "bag_sha256": "sha256:" + "0" * 64,
            "odometry_topic": "/agt/mapping/odometry",
        },
        "map": {
            "map_id": "map_01",
            "map_yaml": str(map_yaml),
            "map_yaml_sha256": sha256_file(map_yaml),
            "localization_pcd": str(pcd),
            "localization_pcd_sha256": sha256_file(pcd),
            "processing_record": str(record),
            "processing_record_sha256": sha256_file(record),
        },
        "platform": {"profile": str(PROFILE)},
        "frames": {
            "source_frame": "odom",
            "execution_frame": "map",
            "map_from_teach_odom": {"x": 0, "y": 0, "z": 0, "yaw": 0},
        },
        "processing": {},
        "execution": {"controller_id": "FollowPath", "maximum_linear_speed_mps": 0.2},
        "assets": {
            "reference_path": "processed/reference_path.yaml",
            "reference_path_sha256": sha256_file(reference),
            "task_control_points": "processed/task_control_points.json",
            "task_control_points_sha256": sha256_file(control),
            "processing_report": "processed/processing_report.json",
        },
    }
    path = tmp_path / "manifest.yaml"
    atomic_write_yaml(path, manifest)
    return path


def parameters(manifest, **values):
    return [Parameter("manifest", value=str(manifest))] + [
        Parameter(name, value=value) for name, value in values.items()
    ]


def costmap(obstacle=False):
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.info.width = 80
    message.info.height = 50
    message.info.resolution = 0.1
    message.info.origin.orientation.w = 1.0
    message.data = [0] * (message.info.width * message.info.height)
    if obstacle:
        message.data[20 * message.info.width + 30] = 100
    return message


def path(frame="map"):
    from geometry_msgs.msg import PoseStamped

    message = NavPath()
    message.header.frame_id = frame
    for x in (2.0, 4.0):
        stamped = PoseStamped()
        stamped.header.frame_id = frame
        stamped.pose.position.x = x
        stamped.pose.position.y = 2.0
        stamped.pose.orientation.w = 1.0
        message.poses.append(stamped)
    return message


def test_reference_publisher_is_transient_local(tmp_path):
    node = TeachPathPublisher(parameter_overrides=parameters(make_asset(tmp_path)))
    try:
        qos = node.get_publishers_info_by_topic("/agt/teach/reference_path")[0].qos_profile
        assert qos.durability == rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        annotation_qos = node.get_publishers_info_by_topic(
            "/agt/teach/route_annotations"
        )[0].qos_profile
        assert annotation_qos.durability == rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        markers = node._annotation_markers(node.get_clock().now().to_msg())
        assert any(marker.text == "START" for marker in markers.markers)
    finally:
        node.destroy_node()


def test_validator_clears_path_for_wrong_frame_and_collision(tmp_path):
    node = TeachPathValidator(parameter_overrides=parameters(make_asset(tmp_path)))
    try:
        node._costmap_callback(costmap())
        node._path_callback(path("odom"))
        assert node.last_validated_path.poses == []
        assert node.last_report["error_codes"] == ["validator_input_error"]
        node._path_callback(path())
        node._costmap_callback(costmap(obstacle=True))
        assert node.last_report["valid"] is False
        assert node.last_validated_path.poses == []
    finally:
        node.destroy_node()


def test_executor_rejects_missing_readiness_and_never_publishes_velocity(tmp_path):
    node = TeachPathExecutor(
        parameter_overrides=parameters(
            make_asset(tmp_path), execution_enabled=True, auto_start=False
        )
    )
    try:
        assert node._gate_failure() == "PATH_NOT_VALIDATED"
        node._validation_report = {
            "demo_id": "route_01",
            "valid": True,
            "eligible_for_execution": True,
        }
        node._validated_path = path()
        assert node._gate_failure() == "LOCALIZATION_NOT_READY"
        node._localization_ready = True
        node._localization_at = time.monotonic()
        assert node._gate_failure() == "SAFETY_NOT_READY"
        topics = dict(node.get_topic_names_and_types())
        assert "/cmd_vel" not in topics
        assert not any(name.startswith("/agt/teach") and "cmd_vel" in name for name in topics)
    finally:
        node.destroy_node()


def test_executor_clears_speed_limit_on_terminal_state(tmp_path):
    node = TeachPathExecutor(
        parameter_overrides=parameters(
            make_asset(tmp_path), execution_enabled=True, auto_start=False
        )
    )

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    try:
        publisher = Publisher()
        node._speed_publisher = publisher
        node._finish("CANCELED", "USER_CANCELED")
        assert len(publisher.messages) == 1
        assert publisher.messages[0].percentage is False
        assert publisher.messages[0].speed_limit == 0.0
    finally:
        node.destroy_node()


def test_localization_readiness_and_runtime_loss_cancel_child(tmp_path):
    node = TeachPathExecutor(
        parameter_overrides=parameters(
            make_asset(tmp_path), execution_enabled=True, auto_start=False
        )
    )

    class Child:
        def __init__(self):
            self.canceled = False

        def cancel_goal_async(self):
            self.canceled = True

    try:
        message = LocalizationStatus()
        assert TeachPathExecutor.localization_status_is_ready(message) is False
        message.state = LocalizationStatus.STATE_TRACKING
        message.pose_valid = True
        message.localization_accepted = True
        message.error_code = LocalizationStatus.ERROR_NONE
        assert TeachPathExecutor.localization_status_is_ready(message) is True
        child = Child()
        node._active = True
        node._child_goal = child
        node._cancel_for_failure("LOCALIZATION_NOT_READY")
        assert child.canceled is True
        assert node._pending_failure == "LOCALIZATION_NOT_READY"
    finally:
        node.destroy_node()


def test_accepted_tracking_validation_status_does_not_cancel_child(tmp_path):
    manifest_path = make_asset(tmp_path)
    node = TeachPathExecutor(
        parameter_overrides=parameters(manifest_path, execution_enabled=True, auto_start=False)
    )

    class Child:
        def __init__(self):
            self.canceled = False

        def cancel_goal_async(self):
            self.canceled = True

    try:
        child = Child()
        node._active = True
        node._child_goal = child
        message = LocalizationStatus()
        message.state = LocalizationStatus.STATE_TRACKING
        message.pose_valid = True
        message.localization_accepted = True
        message.error_code = LocalizationStatus.ERROR_NONE
        message.map_id = "map_01"
        message.map_hash = node.manifest["map"]["localization_pcd_sha256"]

        node._localization_callback(message)

        assert node._localization_ready is True
        assert child.canceled is False
        assert node._pending_failure == ""
    finally:
        node.destroy_node()


def test_evaluator_records_machine_safety_loss_and_empty_failed_run(tmp_path):
    manifest = make_asset(tmp_path)
    node = RepeatabilityEvaluator(parameter_overrides=parameters(manifest))
    try:
        start = String()
        start.data = json.dumps({"state": "STARTING", "run_id": "run_01"})
        node._execution_callback(start)

        safety = DiagnosticArray()
        status = DiagnosticStatus()
        status.name = "agt_safety/tracked_controller"
        status.values = [
            KeyValue(key="motion_enabled", value="true"),
            KeyValue(key="estop_latched", value="false"),
        ]
        safety.status = [status]
        node._safety_callback(safety)
        status.values[0].value = "false"
        node._safety_callback(safety)

        terminal = String()
        terminal.data = json.dumps(
            {
                "state": "FAILED",
                "run_id": "run_01",
                "failure_reason": "SAFETY_NOT_READY",
            }
        )
        node._execution_callback(terminal)

        metrics = json.loads(
            (tmp_path / "runs/run_01/metrics.json").read_text(encoding="utf-8")
        )
        assert metrics["trajectory_metrics_available"] is False
        assert metrics["safety_readiness_loss_count"] == 1
        assert (tmp_path / "runs/run_01/safety_samples.csv").is_file()
        report = (tmp_path / "runs/run_01/report.md").read_text(encoding="utf-8")
        assert "not available" in report
    finally:
        node.destroy_node()


def test_preview_and_dense_path_contracts_are_static_and_motion_free():
    preview = (
        ROOT / "src/agt_teach_repeat/launch/teach_preview.launch.py"
    ).read_text(encoding="utf-8")
    executor = (
        ROOT / "src/agt_teach_repeat/agt_teach_repeat/teach_path_executor.py"
    ).read_text(encoding="utf-8")
    assert "controller_server" not in preview
    assert "agt_chassis" not in preview
    assert "motion_enable" not in preview
    assert "FollowWaypoints" not in executor
    assert "FollowPath" in executor


def test_preview_rviz_uses_latched_topics_and_snap_safe_launcher():
    preview = (
        ROOT / "src/agt_teach_repeat/launch/teach_preview.launch.py"
    ).read_text(encoding="utf-8")
    config = (
        ROOT / "src/agt_teach_repeat/rviz/teach_preview.rviz"
    ).read_text(encoding="utf-8")
    launcher = (
        ROOT / "src/agt_teach_repeat/scripts/start_teach_preview_rviz.sh"
    ).read_text(encoding="utf-8")

    assert 'executable="start_teach_preview_rviz.sh"' in preview
    assert '"rviz_config"' in preview
    for topic in (
        "/agt/map/global_occupancy",
        "/agt/teach/reference_path",
        "/agt/teach/route_annotations",
        "/agt/teach/collision_poses",
        "/agt/teach/footprint_markers",
        "/agt/teach/corridor_markers",
    ):
        assert topic in config
    assert config.count("Durability Policy: Transient Local") >= 6
    assert "SNAP SNAP_ARCH" in launcher
    assert "GDK_PIXBUF_MODULEDIR" in launcher
    assert 'exec ros2 run rviz2 rviz2 "$@"' in launcher
