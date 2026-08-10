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


def test_v25_11b_truth_adapter_only_publishes_sparse_evidence_not_tf_or_canonical():
    adapter = _read("scripts/synthetic_localization_evidence.py")
    assert "SOFTWARE_ONLY" in adapter
    assert '"/simulation/bunker/ground_truth"' in adapter
    assert '"/agt/localization/evidence_status"' in adapter
    assert 'message.correction_generation = 0' in adapter
    assert "TransformBroadcaster" not in adapter
    assert '"/agt/localization/status"' not in adapter
    assert '"/agt/simulation/localization/submit_correction"' in adapter
    assert '"/agt/simulation/localization/publish_recovering"' in adapter
    assert '"/agt/simulation/localization/publish_lost"' in adapter
    assert '"translation_bias_x_m"' in adapter
    assert '"yaw_bias_deg"' in adapter
    assert '"fitness_score"' in adapter


def test_v25_11b_config_reuses_v25_10_correction_envelopes_and_map_identity():
    config = yaml.safe_load(_read("config/v25_11_localization.yaml"))
    manager = config["global_correction_manager"]["ros__parameters"]
    evidence = config["agt_synthetic_localization_evidence"]["ros__parameters"]
    assert manager["use_sim_time"] is True
    assert evidence["use_sim_time"] is True
    assert manager["map_id"] == evidence["map_id"] == "v25_11_gazebo"
    assert manager["map_hash"] == evidence["map_hash"]
    assert manager["tracking_max_translation_m"] == 0.50
    assert manager["tracking_max_yaw_rad"] == 0.20
    assert manager["recovering_max_translation_m"] == 2.0
    assert manager["recovering_max_yaw_rad"] == 0.70
    assert manager["correction_rejections_to_lost"] == 3
    assert manager["allow_lost_reanchor"] is True


def test_v25_11b_launch_keeps_global_correction_manager_as_map_odom_authority():
    launch = _read("launch/gazebo_localization_validation.launch.py")
    rviz = yaml.safe_load(_read("rviz/gazebo_localization_validation.rviz"))
    assert 'package="agt_localization"' in launch
    assert 'executable="global_correction_manager"' in launch
    assert 'executable="synthetic_localization_evidence.py"' in launch
    assert 'executable="v25_11b_localization_acceptance.py"' in launch
    assert 'DeclareLaunchArgument("run_acceptance"' in launch
    assert "period=0.5" in launch
    assert "period=3.0" in launch
    assert rviz["Visualization Manager"]["Global Options"]["Fixed Frame"] == "map"


def test_v25_11b_acceptance_freezes_generation_on_reject_and_tests_reanchor():
    acceptance = _read("scripts/v25_11b_localization_acceptance.py")
    for token in (
        "tracking_small_correction_accepted",
        "TRANSLATION_JUMP_REJECTED",
        "rejected_generation_frozen",
        "recovering_envelope_accepted",
        "three_rejections_escalate_lost",
        "REANCHOR_ACCEPTED",
        "lost_reanchor_accepted",
        "/tmp/agt_v25_11b_localization_result.json",
    ):
        assert token in acceptance


def test_simulation_package_declares_v25_11b_runtime_dependencies():
    package = _read("package.xml")
    for dependency in (
        "agt_interfaces",
        "agt_localization",
        "rcl_interfaces",
        "std_msgs",
        "std_srvs",
    ):
        assert f"<exec_depend>{dependency}</exec_depend>" in package


def test_symlink_install_entrypoints_are_materialized_executable():
    cmake = _read("CMakeLists.txt")
    for token in (
        "AGT_SIMULATION_GENERATED_SCRIPT_DIR",
        "FILE_PERMISSIONS",
        "OWNER_EXECUTE",
        "GROUP_EXECUTE",
        "WORLD_EXECUTE",
        "keyboard_teleop.py",
        "synthetic_localization_evidence.py",
        "v25_11a_baseline_acceptance.py",
        "v25_11b_localization_acceptance.py",
    ):
        assert token in cmake
