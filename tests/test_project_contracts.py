import ast
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_NAME = re.compile(r"^agt_[a-z0-9_]+$")
FRAME_NAME = re.compile(r"^[a-z][a-z0-9_]*$")
TOPIC_NAME = re.compile(r"^/agt/[a-z0-9_/]+$")


def test_all_packages_follow_the_naming_contract():
    manifests = sorted((ROOT / "src").glob("agt_*/package.xml"))
    assert len(manifests) == 20
    for manifest in manifests:
        name = ET.parse(manifest).getroot().findtext("name")
        assert PACKAGE_NAME.fullmatch(name), name
        assert manifest.parent.name == name


def test_python_launch_files_parse():
    for launch_file in (ROOT / "src").glob("agt_*/launch/*.launch.py"):
        ast.parse(launch_file.read_text(encoding="utf-8"), filename=str(launch_file))


def test_mid360_profile_uses_only_the_canonical_calibration_file():
    profile = yaml.safe_load(
        (ROOT / "profiles/sensors/mid360.yaml").read_text(encoding="utf-8")
    )
    lidar = profile["sensors"]["lidar"]
    assert lidar["extrinsics"] == {
        "parent_frame": "base_link",
        "calibration_file": "agt_description/config/mk_mini_mid360.yaml",
    }
    assert FRAME_NAME.fullmatch(lidar["frame_id"])
    assert FRAME_NAME.fullmatch(lidar["driver_frame_id"])
    assert TOPIC_NAME.fullmatch(lidar["points_topic"])
    assert TOPIC_NAME.fullmatch(lidar["native_topic"])
    assert lidar["native_message_type"] == "livox_ros_driver2/msg/CustomMsg"


def test_calibration_has_all_required_fields_and_radian_values():
    calibration = yaml.safe_load(
        (ROOT / "src/agt_description/config/mk_mini_mid360.yaml").read_text(
            encoding="utf-8"
        )
    )["/**"]["ros__parameters"]
    required = {
        "calibration_verified",
        "base_length",
        "base_width",
        "base_height",
        "base_link_z",
        "lidar_x",
        "lidar_y",
        "lidar_z",
        "lidar_roll",
        "lidar_pitch",
        "lidar_yaw",
    }
    assert set(calibration) == required
    for angle in ("lidar_roll", "lidar_pitch", "lidar_yaw"):
        assert -3.141592653589793 <= calibration[angle] <= 3.141592653589793


def test_fast_livo_patch_disables_the_native_tf_by_parameter():
    patch = (
        ROOT / "src/agt_mapping/patches/fast_livo2_publish_tf.patch"
    ).read_text(encoding="utf-8")
    assert '"common.publish_tf"' in patch
    assert "if (publish_tf)" in patch
    cmake_patch = (
        ROOT / "src/agt_mapping/patches/fast_livo2_cmake_portability.patch"
    ).read_text(encoding="utf-8")
    assert "vikit_common::vikit_common" in cmake_patch
    assert "vikit_ros::vikit_ros" in cmake_patch
    added_lines = [
        line for line in cmake_patch.splitlines()
        if line.startswith("+") and not line.startswith("+++")
    ]
    assert all("../../install" not in line for line in added_lines)
    lidar_cloud_patch = (
        ROOT / "src/agt_mapping/patches/fast_livo2_lidar_frame_cloud.patch"
    ).read_text(encoding="utf-8")
    assert '"/cloud_registered_lidar"' in lidar_cloud_patch
    assert 'frame_id = "lidar_link"' in lidar_cloud_patch


def test_fast_livo_is_vendored_at_the_selected_pinned_commit():
    vendor = ROOT / "third_party/fast_livo2_ros2"
    assert (vendor / "package.xml").exists()
    assert (vendor / "LICENSE").exists()
    provenance = (ROOT / "third_party/README.md").read_text(encoding="utf-8")
    assert "https://github.com/Aldoubt/FASTLIVO2_ROS2.git" in provenance
    assert "a713004f0ba0624c8fb80d85c7047fe62523c6fb" in provenance
    manifest = ET.parse(vendor / "package.xml").getroot()
    assert manifest.findtext("license") == "GPL-2.0-only"


def test_vendored_fast_livo_publishes_lidar_frame_cloud():
    source = (
        ROOT / "third_party/fast_livo2_ros2/src/LIVMapper.cpp"
    ).read_text(encoding="utf-8")
    assert '"/cloud_registered_lidar"' in source
    assert 'message.header.frame_id = "lidar_link"' in source
    assert "message.header.stamp = odomAftMapped.header.stamp" in source


def test_vikit_is_a_clean_fixed_vendor_with_generic_eigen_abi():
    vendor = ROOT / "third_party/rpg_vikit_ros2_fisheye"
    assert (vendor / "vikit_common/package.xml").exists()
    assert (vendor / "vikit_ros/package.xml").exists()
    assert not (vendor / "vikit_py").exists()
    assert not (vendor / "ername").exists()
    assert not (vendor / ".git").exists()
    provenance = (ROOT / "third_party/README.md").read_text(encoding="utf-8")
    assert "fee3d50ae2af472fb27eb62b4526dd4b32ede8ef" in provenance
    flags = (vendor / "vikit_common/CMakeLists.txt").read_text(encoding="utf-8")
    assert "-march=x86-64 -mtune=native" in flags
    assert "-march=native" not in flags
    dependencies = (ROOT / "nav_dependencies.repos").read_text(encoding="utf-8")
    assert "rpg_vikit_ros2_fisheye" not in dependencies


def test_fast_livo_adapter_is_installed_as_an_executable():
    adapter = ROOT / "src/agt_mapping/scripts/fast_livo2_adapter.py"
    assert adapter.stat().st_mode & 0o111


def test_teach_repeat_package_and_recording_contract_are_present():
    package = ROOT / "src/agt_teach_repeat"
    assert (package / "package.xml").is_file()
    assert (package / "launch/teach_extract.launch.py").is_file()
    assert (package / "launch/teach_preview.launch.py").is_file()
    assert (package / "launch/repeat_test.launch.py").is_file()
    profiles = yaml.safe_load(
        (ROOT / "src/agt_experiment_manager/config/bag_profiles.yaml").read_text(
            encoding="utf-8"
        )
    )["profiles"]
    teach_topics = profiles["teach_repeat"]["topics"]
    assert teach_topics
    assert all(topic.startswith("/") for topic in teach_topics)
    for topic in (
        "/agt/teach/reference_path",
        "/agt/teach/path_validated",
        "/agt/teach/executed_path",
        "/agt/teach/execution_status",
        "/agt/teach/metrics",
    ):
        assert topic in teach_topics


def test_teach_repeat_reuses_canonical_geometry_and_cannot_edit_maps():
    package = ROOT / "src/agt_teach_repeat/agt_teach_repeat"
    source = "\n".join(
        path.read_text(encoding="utf-8") for path in sorted(package.glob("*.py"))
    )
    assert "load_platform_profile" in source
    assert "agt_coverage_planning.path_validator" in source
    for bunker_dimension in ("0.5915", "0.4690", "1.023", "0.778"):
        assert bunker_dimension not in source
    corridor = (package / "corridor_auditor.py").read_text(encoding="utf-8")
    assert "OccupancyGrid" in corridor
    assert "create_publisher(OccupancyGrid" not in corridor
    assert "write_bytes" not in corridor
    core = (package / "corridor_audit.py").read_text(encoding="utf-8")
    assert '"eligible_for_automatic_map_edit": False' in core
    assert "write_p5" not in source
    assert "imwrite" not in source
    assert "agt_teach_repeat" not in "\n".join(
        path.read_text(encoding="utf-8", errors="ignore")
        for path in (ROOT / "third_party/fast_livo2_ros2").rglob("*")
        if path.is_file()
    )


def test_lidar_only_fast_livo_still_has_required_camera_model():
    camera = yaml.safe_load(
        (
            ROOT / "src/agt_mapping/config/camera_disabled_placeholder.yaml"
        ).read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]["camera"]
    assert camera["model"] == "Pinhole"
    assert camera["width"] > 0
    assert camera["height"] > 0


def test_mid360_fast_livo_uses_native_message_path():
    config = yaml.safe_load(
        (ROOT / "src/agt_mapping/config/mid360_lio_only.yaml").read_text()
    )["/**"]["ros__parameters"]
    assert config["common"]["lid_topic"] == "/agt/sensors/lidar/custom_filtered"
    assert config["preprocess"]["lidar_type"] == 1


def test_mid360_network_config_keeps_extrinsics_in_description_only():
    config = json.loads(
        (ROOT / "src/agt_sensor_adapters/config/mid360_network.json").read_text(
            encoding="utf-8"
        )
    )
    device = config["lidar_configs"][0]
    assert device["ip"]
    assert set(device["extrinsic_parameter"].values()) == {0, 0.0}


def test_vendored_livox_driver_provenance_and_license_exist():
    driver = ROOT / "third_party/livox_ros_driver2"
    assert (driver / "package.xml").exists()
    assert (driver / "LICENSE.txt").exists()
    provenance = (ROOT / "third_party/README.md").read_text(encoding="utf-8")
    assert "115c7beeaea02593957af46ccbecc263bc5cf12f" in provenance


def test_relocalization_uses_base_to_lidar_extrinsics_before_publishing_map_odom():
    source = (ROOT / "src/agt_localization/src/relocalization_node.cpp").read_text(
        encoding="utf-8"
    )
    assert 'declare_parameter<std::string>("base_frame", "base_link")' in source
    assert "candidateToPose(candidate) * base_from_tracking" in source
    assert "attempt.result.estimated_pose * tracking_from_base" in source
    assert "best.result.estimated_pose * tracking_from_odom" in source
    assert '"/agt/localization/status"' in source
    assert (ROOT / "third_party/relocalization_core/LICENSE").exists()
    assert (ROOT / "third_party/ndt_omp_ros2/LICENSE").exists()


def test_qt5_map_editor_uses_v2_map_interfaces():
    gui = ROOT / "third_party/ros_qt5_gui_app"
    assert (gui / "CMakeLists.txt").exists()
    assert (gui / "LICENSE").read_text(encoding="utf-8").startswith(
        "                    GNU GENERAL PUBLIC LICENSE"
    )
    provenance = (ROOT / "third_party/README.md").read_text(encoding="utf-8")
    assert "https://github.com/Aldoubt/Ros_Qt5_Gui_App.git" in provenance
    assert "agt-navigation-v2" in provenance
    assert "1597fbbcb25c2c069e51627ab6994470706384fc" in provenance
    config = json.loads(
        (ROOT / "src/agt_ui_bridge/config/ros_qt5_gui_app.json").read_text(
            encoding="utf-8"
        )
    )
    topics = {item["display_name"]: item["topic"] for item in config["display_config"]}
    assert topics["kOccupancyMap"] == "/agt/map/global_occupancy"
    assert topics["kRobotPose"] == "/agt/mapping/odometry"
    assert topics["kSetRobotSpeed"] == "/agt/cmd_vel_manual"
    assert config["key_value"]["BaseFrameId"] == "base_footprint"
    rclcomm = (gui / "src/channel/ros2/rclcomm.cpp").read_text(encoding="utf-8")
    task_widget = (gui / "src/app/widgets/nav_goal_table_view.cpp").read_text(
        encoding="utf-8"
    )
    task_library = (gui / "src/app/widgets/task_library_dock.cpp").read_text(
        encoding="utf-8"
    )
    map_loader = (gui / "src/basic/map/occupancy_map.h").read_text(
        encoding="utf-8"
    )
    assert '"/agt/navigation/execute_waypoint_task"' in rclcomm
    assert "waypoint_task_client_->async_send_goal" in rclcomm
    assert 'GET_CONFIG_VALUE("EnableTaskExecution", "false")' in rclcomm
    mainwindow = (gui / "src/app/mainwindow.cpp").read_text(encoding="utf-8")
    assert 'GET_CONFIG_VALUE("EnableTaskExecution", "false")' in mainwindow
    scene_manager = (
        gui / "src/app/display/manager/scene_manager.cpp"
    ).read_text(encoding="utf-8")
    view_manager = (
        gui / "src/app/display/manager/view_manager.cpp"
    ).read_text(encoding="utf-8")
    assert "nav_goal_position_pending_" in scene_manager
    assert "std::atan2(dy, dx)" in scene_manager
    assert "timer->start(100)" in scene_manager
    assert "RefreshPointChoices()" in task_widget
    assert "const int row = ActiveRow()" in task_widget
    assert "AddSelectedTopologyPoint" in task_library
    assert "RefreshTopologyPointChoices" in task_library
    assert "waypoint.yaw = task_group::normalizeYaw(match->theta)" in task_library
    assert "TaskLibraryDock::UpdateTopologyMap" in mainwindow
    assert "emit signalTopologyMapUpdate(topology_map_)" in scene_manager
    assert "void ViewManager::ZoomAt" in view_manager
    assert "SetRobotFocus(false)" in view_manager
    assert 'GET_CONFIG_VALUE("UiLanguage", "zh_CN")' in mainwindow
    assert 'GET_CONFIG_VALUE("UseNativeWindowFrame", "true")' in mainwindow
    assert 'GET_CONFIG_VALUE("EnableCostmapDisplay", "false")' in rclcomm
    assert "QueueTaskChain(request)" in rclcomm
    assert '"/agt/navigation/waypoint_preview_request"' in rclcomm
    assert "signalPreviewTaskChain" in task_widget
    assert "absoluteDifference" not in task_widget
    assert "task_chain_.points.clear()" in task_widget
    assert "catch (const YAML::Exception &e)" in map_loader
    bridge = (ROOT / "src/agt_ui_bridge/scripts/map_io_bridge.py").read_text(
        encoding="utf-8"
    )
    editor = (ROOT / "src/agt_ui_bridge/scripts/map_editor_qt5.py").read_text(
        encoding="utf-8"
    )
    assert '"/agt/map/load"' in bridge
    assert '"/agt/map/save"' in bridge
    assert '"/agt/map/edited"' in editor
    assert '"/initialpose"' in editor
    assert '"/goal_pose"' in editor
    assert "jie_map_msgs" not in bridge + editor
