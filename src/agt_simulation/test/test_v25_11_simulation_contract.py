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


def test_world_contains_rows_headland_and_obstacle_fixture():
    world = _read("worlds/agri_validation.sdf")
    assert "crop_row_north_outer" in world
    assert "crop_row_south_outer" in world
    assert "fixed_obstacle" in world
    assert "headland_marker" in world
    assert "model://bunker_sim" in world


def test_simulation_odom_adapter_never_claims_map_odom():
    adapter = _read("scripts/gazebo_odom_adapter.py")
    assert '"/agt/mapping/odometry"' in adapter
    assert "base_footprint" in adapter
    assert "map -> odom" in adapter
    assert 'child_frame_id = self.base_frame' in adapter
    assert 'frame_id = "map"' not in adapter


def test_rviz_config_is_valid_and_reserves_planning_visualization_topics():
    rviz = yaml.safe_load(_read("rviz/gazebo_system_validation.rviz"))
    assert rviz["Visualization Manager"]["Global Options"]["Fixed Frame"] == "odom"
    text = _read("rviz/gazebo_system_validation.rviz")
    assert "/agt/sensors/lidar/scan" in text
    assert "/plan" in text
    assert "/agt/navigation/runtime_path" in text


def test_launch_bridges_clock_drive_odom_lidar_and_imu():
    launch = _read("launch/gazebo_system_validation.launch.py")
    for token in (
        "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        "/model/bunker_sim/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
        "/model/bunker_sim/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        "/simulation/bunker/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        "/simulation/bunker/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
    ):
        assert token in launch
    assert "/agt/safety/cmd_vel" in launch
    assert "/simulation/bunker/odometry" in launch


def test_baseline_acceptance_exercises_command_rates_tf_and_no_map_odom():
    smoke = _read("scripts/v25_11a_baseline_acceptance.py")
    assert '"/agt/safety/cmd_vel"' in smoke
    assert '"/agt/mapping/odometry"' in smoke
    assert '"/agt/sensors/lidar/scan"' in smoke
    assert '"/agt/sensors/imu/data"' in smoke
    assert 'not node.tf.can_transform("map", "odom", Time())' in smoke
    assert 'displacement >= 0.05' in smoke
    assert 'node.rate("scan") >= 5.0' in smoke
    assert 'node.rate("imu") >= 50.0' in smoke


def test_symlink_install_entrypoints_are_materialized_executable():
    cmake = _read("CMakeLists.txt")
    assert "AGT_SIMULATION_GENERATED_SCRIPT_DIR" in cmake
    assert "FILE_PERMISSIONS" in cmake
    assert "OWNER_EXECUTE" in cmake
    for script in (
        "gazebo_odom_adapter.py",
        "gazebo_sensor_adapter.py",
        "v25_11a_baseline_acceptance.py",
    ):
        assert script in cmake
