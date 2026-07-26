from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]


def read(relative_path):
    return (REPOSITORY_ROOT / relative_path).read_text(encoding="utf-8")


def test_mapping_mode_offers_non_executing_gui_but_keeps_it_off_by_default():
    source = read("src/agt_bringup/launch/mapping_mode.launch.py")
    assert '"start_gui",\n                default_value="false"' in source
    assert '"ros_qt5_gui.launch.py"' in source
    assert '"profile": "mapping"' in source
    assert '"source_map_topic": "/agt/map/mapping_occupancy"' in source
    assert '"map_frame_id": "odom"' in source
    assert 'executable="map_saver_server"' in source
    assert 'name="agt_mapping_map_saver"' in source
    assert 'name="agt_mapping_map_saver_lifecycle"' in source
    assert '"free_thresh_default": 0.196' in source
    assert 'free_thresh_default:=0.196' in read(
        "src/agt_bringup/launch/save_mapping_result.launch.py"
    )
    assert 'occupied_thresh_default:=0.65' in read(
        "src/agt_bringup/launch/save_mapping_result.launch.py"
    )
    assert "refusing to overwrite existing map output" in read(
        "src/agt_bringup/launch/save_mapping_result.launch.py"
    )
    assert 'DeclareLaunchArgument(\n                "mapping_output_dir"' in source
    assert '"start_chassis",\n                default_value="false"' in source
    assert '"start_chassis_monitor", default_value="false"' in source
    assert '"start_octomap_projection",' in source
    assert 'DeclareLaunchArgument("octomap_input_rate_hz", default_value="0.2")' in source
    assert 'DeclareLaunchArgument("octomap_cloud_max_points", default_value="8000")' in source
    assert "refusing to overwrite existing PCD output directory" in source


def test_navigation_mode_starts_navigation_gui_and_defaults_optional_features_off():
    source = read("src/agt_bringup/launch/navigation_system.launch.py")
    assert '"profile": "navigation"' in source
    assert '"source_map_topic": "/agt/map/global_occupancy"' in source
    assert '"map_frame_id": "map"' in source
    assert 'DeclareLaunchArgument("start_semantic_map_server", default_value="false")' in source
    assert 'DeclareLaunchArgument("start_coverage_planning", default_value="false")' in source
    assert '"global_map_processing_record"' in source
    assert 'DeclareLaunchArgument("map_id", default_value="")' in source
    assert '"map_version_id"' in read("src/agt_bringup/launch/system.launch.py")
    assert "OpaqueFunction(function=validate_navigation_arguments)" in source
    nav_source = read("src/agt_navigation/launch/navigation.launch.py")
    assert 'DeclareLaunchArgument("autostart", default_value="false")' in nav_source
    assert 'DeclareLaunchArgument("enable_localization_gate", default_value="true")' in nav_source
    assert '"auto_relocalize_on_start"' in source
    assert 'executable="automatic_relocalization.py"' in source
    assert '"start_chassis",\n                default_value="false"' in source
    assert 'DeclareLaunchArgument("start_chassis_monitor", default_value="false")' in source
    assert 'auto_relocalize_on_start": LaunchConfiguration(' in read(
        "src/agt_bringup/launch/system.launch.py"
    )


def test_system_passes_gui_to_both_modes_and_keeps_optional_features_off():
    source = read("src/agt_bringup/launch/system.launch.py")
    assert source.count('"start_gui": LaunchConfiguration("start_gui")') == 1
    assert '"start_gui": LaunchConfiguration("start_mapping_gui")' in source
    assert '"start_mapping_gui",\n                default_value="false"' in source
    assert 'DeclareLaunchArgument("start_semantic_map_server", default_value="false")' in source
    assert 'DeclareLaunchArgument("start_coverage_planning", default_value="false")' in source
    assert 'DeclareLaunchArgument("global_map_processing_record", default_value="")' in source
    assert '"global_map_processing_record": LaunchConfiguration(' in source
    assert '"user_config_path": LaunchConfiguration("user_config_path")' in source
    assert 'DeclareLaunchArgument(\n                "start_chassis",\n                default_value="false"' in source
    assert '"start_chassis_monitor",\n                default_value="false"' in source
    assert 'DeclareLaunchArgument("chassis_backend", default_value="bunker_can")' in source
    assert 'DeclareLaunchArgument("can_interface", default_value="can0")' in source


def test_nav_velocity_passes_collision_monitor_and_safety():
    nav_launch = read("src/agt_navigation/launch/navigation.launch.py")
    nav_config = read("src/agt_navigation/config/nav2_bunker.yaml")
    safety = read("src/agt_safety/scripts/tracked_safety_controller.py")
    guard_config = read("src/agt_chassis/config/bunker.yaml")
    chassis_launch = read("src/agt_chassis/launch/bunker.launch.py")

    assert '("cmd_vel", "/agt/navigation/cmd_vel_raw")' in nav_launch
    assert "cmd_vel_in_topic: /agt/navigation/cmd_vel_raw" in nav_config
    assert "cmd_vel_out_topic: /agt/navigation/cmd_vel" in nav_config
    assert 'Twist, "/agt/navigation/cmd_vel"' in safety
    assert 'Twist, "/agt/cmd_vel_manual"' in safety
    assert "input_topic: /agt/safety/cmd_vel" in guard_config
    assert "output_topic: /agt/chassis/cmd_vel" in guard_config
    assert '("/cmd_vel", LaunchConfiguration("command_topic"))' in chassis_launch
    assert '"operation_mode",\n                default_value="control"' in chassis_launch
    assert 'LaunchConfiguration("operation_mode")' in chassis_launch
    assert 'LaunchConfiguration("command_topic")' in chassis_launch
    assert '"command_topic": "/agt/chassis/monitor_cmd_vel"' in read(
        "src/agt_bringup/launch/mapping_mode.launch.py"
    )


def test_bunker_driver_does_not_publish_duplicate_odom_tf_by_default():
    chassis_launch = read("src/agt_chassis/launch/bunker.launch.py")
    mapping_launch = read("src/agt_mapping/launch/fast_livo2_mapping.launch.py")
    assert 'DeclareLaunchArgument("publish_driver_odom_tf", default_value="false")' in chassis_launch
    assert '"common.publish_tf": False' in mapping_launch


def test_bunker_fast_livo_chain_starts_custommsg_self_filter_for_bag_replay():
    mapping = read("src/agt_mapping/launch/fast_livo2_mapping.launch.py")
    sensor = read("src/agt_sensor_adapters/launch/lidar_self_filter.launch.py")
    fast_livo_config = read("src/agt_mapping/config/mid360_lio_only.yaml")
    assert "lidar_self_filter.launch.py" in mapping
    assert "custom_filtered" in fast_livo_config
    assert 'DeclareLaunchArgument("start_lidar_self_filter", default_value="true")' in mapping
    assert 'UnlessCondition(LaunchConfiguration("start_lidar_self_filter"))' in mapping
    assert 'DeclareLaunchArgument("platform_profile"' in sensor
    assert '"filter_params_file"' in sensor
    assert '"filter_params_file": LaunchConfiguration("lidar_self_filter_params_file")' in mapping
    assert '"fail_open_on_tf_error", default_value="false"' in sensor
    assert '"zero_point_epsilon", default_value="0.000001"' in sensor


def test_mapping_pcd_output_is_fail_closed_when_reused():
    source = read("src/agt_mapping/launch/fast_livo2_mapping.launch.py")
    assert "OpaqueFunction(function=validate_pcd_output)" in source
    assert "refusing to overwrite existing PCD output directory" in source
