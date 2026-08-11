from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_observer_collects_validation_authorities_and_publishes_visual_outputs():
    observer = read("scripts/v25_11e_observability.py")
    compile(observer, "v25_11e_observability.py", "exec")
    for token in (
        '"/agt/localization/status"',
        '"/agt/simulation/navigation/route_state"',
        '"/diagnostics"',
        '"/agt/safety/status"',
        '"/agt/safety/cmd_vel"',
        '"/simulation/bunker/ground_truth"',
        'Marker, "/agt/validation/status_markers"',
        'NavPath, "/agt/validation/ground_truth_path"',
        '"/tmp/agt_v25_11e_timeline.jsonl"',
        '"/tmp/agt_v25_11e_summary.json"',
        '"agt_v25_11e_summary/v1"',
        '"first_nonzero_cmd_ros_ns"',
        '"route_terminal_ros_ns"',
        '"fatal_error"',
        '"observer_fatal"',
    ):
        assert token in observer
    # Comments may document the rejected MarkerArray design. The executable
    # contract only forbids importing, constructing, or publishing MarkerArray.
    assert "from visualization_msgs.msg import MarkerArray" not in observer
    assert "MarkerArray()" not in observer
    assert "create_publisher(MarkerArray" not in observer


def test_observer_records_state_changes_not_only_final_snapshot():
    observer = read("scripts/v25_11e_observability.py")
    for token in (
        '"observer_started"',
        '"localization"',
        '"route_state"',
        '"sensor_health"',
        '"safety"',
        '"safety_cmd_motion"',
        "_record_change",
        "event_counts",
    ):
        assert token in observer


def test_observer_uses_conservative_humble_message_construction():
    observer = read("scripts/v25_11e_observability.py")
    for token in (
        "pose.header.stamp.sec = int(msg.header.stamp.sec)",
        "pose.header.stamp.nanosec = int(msg.header.stamp.nanosec)",
        "pose.pose.position.x = float(source.position.x)",
        "self.marker_pub.publish(self._text_marker",
        "lifetime remains zero",
    ):
        assert token in observer
    assert "pose.header = msg.header" not in observer
    assert "pose.pose = msg.pose.pose" not in observer
    assert "markers.markers =" not in observer


def test_observability_launch_keeps_navigation_acceptance_and_single_rviz_owner():
    launch = read("launch/gazebo_observability_validation.launch.py")
    compile(launch, "gazebo_observability_validation.launch.py", "exec")
    for token in (
        "gazebo_navigation_validation.launch.py",
        '"use_rviz": "false"',
        '"run_acceptance": run_navigation_acceptance',
        'executable="v25_11e_observability.py"',
        'executable="v25_11e_observability_acceptance.py"',
        'name="agt_v25_11e_observability_rviz"',
        "gazebo_observability_validation.rviz",
        'DeclareLaunchArgument("run_navigation_acceptance"',
        'DeclareLaunchArgument("run_observability_acceptance"',
        "TimerAction(period=0.25, actions=[observer])",
        "TimerAction(period=0.75, actions=[acceptance])",
    ):
        assert token in launch
    assert launch.index("navigation,") < launch.index(
        "TimerAction(period=0.25, actions=[observer])"
    )
    assert launch.index("TimerAction(period=0.25, actions=[observer])") < launch.index(
        "TimerAction(period=0.75, actions=[acceptance])"
    )


def test_observability_acceptance_requires_motion_timeline_and_happy_path():
    acceptance = read("scripts/v25_11e_observability_acceptance.py")
    compile(acceptance, "v25_11e_observability_acceptance.py", "exec")
    for token in (
        'Marker,',
        'self.marker_ids: set[int] = set()',
        '"markers_available"',
        '"ground_truth_path_available"',
        '"happy_path_succeeded"',
        '"summary_schema_valid"',
        '"observer_runtime_healthy"',
        '"summary_route_succeeded"',
        '"summary_localization_tracking"',
        '"motion_observed"',
        '"key_timestamps_recorded"',
        '"timeline_has_core_events"',
        '"/tmp/agt_v25_11e_observability_result.json"',
    ):
        assert token in acceptance
    assert "from visualization_msgs.msg import MarkerArray" not in acceptance
    assert "MarkerArray()" not in acceptance
    assert "create_subscription(MarkerArray" not in acceptance


def test_observability_rviz_exposes_route_truth_and_status_markers():
    rviz = yaml.safe_load(read("rviz/gazebo_observability_validation.rviz"))
    assert rviz["Visualization Manager"]["Global Options"]["Fixed Frame"] == "map"
    text = read("rviz/gazebo_observability_validation.rviz")
    for topic in (
        "/map",
        "/agt/sensors/lidar/scan",
        "/agt/navigation/runtime_path",
        "/agt/validation/ground_truth_path",
        "/agt/validation/status_markers",
    ):
        assert topic in text
    assert "rviz_default_plugins/Marker\n" in text
    assert "rviz_default_plugins/MarkerArray" not in text
