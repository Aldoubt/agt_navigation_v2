from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def _read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_simulation_package_uses_ros_gz_not_gazebo_classic():
    package = _read("package.xml")
    launch = _read("launch/gazebo_system_validation.launch.py")
    model = _read("models/bunker_sim/model.sdf")

    assert "ros_gz_bridge" in package
    assert "ros_gz_sim" in package
    assert "gazebo_ros_pkgs" not in package
    assert "libgazebo_ros" not in model
    assert "ros_gz_bridge" in launch
    assert "ros_gz_sim" in launch


def test_bunker_proxy_is_explicitly_software_only_and_has_drive_sensors():
    config = _read("models/bunker_sim/model.config")
    model = _read("models/bunker_sim/model.sdf")

    assert "SOFTWARE_ONLY" in config
    assert "SOFTWARE_ONLY" in model
    assert "DiffDrive" in model
    assert "navigation_lidar" in model
    assert "navigation_imu" in model
    assert "/model/bunker_sim/cmd_vel" in model
    assert "/model/bunker_sim/odometry" in model
    assert "/model/bunker_sim/ground_truth" in model
    assert "OdometryPublisher" in model


def test_model_avoids_relative_to_gui_serialization_warning_and_keeps_wheel_axes():
    model = _read("models/bunker_sim/model.sdf")
    assert "relative_to=" not in model
    assert model.count("<pose/>") == 4
    assert model.count('xyz expressed_in="__model__">0 1 0</xyz>') == 4
    for pose in (
        "0.34 0.40 0.14 -1.57079632679 0 0",
        "-0.34 0.40 0.14 -1.57079632679 0 0",
        "0.34 -0.40 0.14 -1.57079632679 0 0",
        "-0.34 -0.40 0.14 -1.57079632679 0 0",
    ):
        assert f"<pose>{pose}</pose>" in model


def test_lidar_has_visible_mount_and_launch_tf_matches_model_offset():
    model = _read("models/bunker_sim/model.sdf")
    launch = _read("launch/gazebo_system_validation.launch.py")
    assert "lidar_mount_visual" in model
    assert "<pose>0.10 0 0.535 0 0 0</pose>" in model
    assert '"--x", "0.10", "--y", "0", "--z", "0.195"' in launch


def test_world_contains_rows_headland_obstacle_and_required_sensor_systems():
    world = _read("worlds/agri_validation.sdf")
    assert "crop_row_north_outer" in world
    assert "crop_row_south_outer" in world
    assert "fixed_obstacle" in world
    assert "headland_marker" in world
    assert "model://bunker_sim" in world
    assert "libignition-gazebo-sensors-system.so" in world
    assert "libignition-gazebo-imu-system.so" in world
    assert "ignition::gazebo::systems::Imu" in world


def test_simulation_odom_adapter_never_claims_map_odom():
    adapter = _read("scripts/gazebo_odom_adapter.py")
    assert '"/agt/mapping/odometry"' in adapter
    assert "base_footprint" in adapter
    assert "map -> odom" in adapter
    assert 'child_frame_id = self.base_frame' in adapter
    assert 'frame_id = "map"' not in adapter


def test_rviz_config_is_valid_and_reserves_planning_and_mapping_topics():
    rviz = yaml.safe_load(_read("rviz/gazebo_system_validation.rviz"))
    assert rviz["Visualization Manager"]["Global Options"]["Fixed Frame"] == "odom"
    text = _read("rviz/gazebo_system_validation.rviz")
    assert "/agt/mapping/odometry" in text
    assert "/agt/sensors/lidar/scan" in text
    assert "/agt/mapping/registered_points" in text
    assert "/agt/map/global_occupancy" in text
    assert "/plan" in text
    assert "/agt/navigation/runtime_path" in text
    assert "Mapping Registered Points (enable with mapping frontend)" in text
    assert "Global Occupancy (enable with map stack)" in text


def test_launch_bridges_clock_drive_odom_ground_truth_lidar_and_imu_and_delays_rviz():
    launch = _read("launch/gazebo_system_validation.launch.py")
    for token in (
        "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        "/model/bunker_sim/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
        "/model/bunker_sim/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        "/model/bunker_sim/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        "/simulation/bunker/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        "/simulation/bunker/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
    ):
        assert token in launch
    assert "/agt/safety/cmd_vel" in launch
    assert "/simulation/bunker/odometry" in launch
    assert "/simulation/bunker/ground_truth" in launch
    assert "TimerAction" in launch
    assert "period=1.5" in launch


def test_baseline_acceptance_requires_physics_truth_not_only_wheel_odometry():
    acceptance = _read("scripts/v25_11a_baseline_acceptance.py")
    assert '"/simulation/bunker/ground_truth"' in acceptance
    assert '"ground_truth_ge_5_hz"' in acceptance
    assert '"wheel_odom_reports_motion"' in acceptance
    assert '"physics_model_moves"' in acceptance
    assert "ground_truth_displacement_m" in acceptance
    assert "odom_truth_displacement_error_m" in acceptance


def test_keyboard_teleop_is_tty_deadman_tool_on_post_safety_topic():
    teleop = _read("scripts/keyboard_teleop.py")
    assert "SOFTWARE_ONLY" in teleop
    assert '"/agt/safety/cmd_vel"' in teleop
    assert "deadman_timeout_s" in teleop
    assert "sys.stdin.isatty()" in teleop
    assert "requires an interactive TTY" in teleop
    assert "node.stop()" in teleop
    assert "node.publisher.publish(Twist())" in teleop
    assert '"w": (1.0, 0.0)' in teleop
    assert '"s": (-1.0, 0.0)' in teleop
    assert '"a": (0.0, 1.0)' in teleop
    assert '"d": (0.0, -1.0)' in teleop


def test_symlink_install_entrypoints_are_materialized_executable():
    cmake = _read("CMakeLists.txt")
    for token in (
        "AGT_SIMULATION_GENERATED_SCRIPT_DIR",
        "FILE_PERMISSIONS",
        "OWNER_EXECUTE",
        "GROUP_EXECUTE",
        "WORLD_EXECUTE",
        "keyboard_teleop.py",
        "v25_11a_baseline_acceptance.py",
    ):
        assert token in cmake
