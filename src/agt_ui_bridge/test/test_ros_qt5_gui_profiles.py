import json
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_profile(name):
    path = PACKAGE_ROOT / "config" / f"ros_qt5_gui_{name}.json"
    return json.loads(path.read_text(encoding="utf-8"))


def topics(config):
    return {item["display_name"]: item["topic"] for item in config["display_config"]}


def visibility(config):
    return {
        item["display_name"]: item["visible"] for item in config["display_config"]
    }


def task_library_keys():
    return {
        "TaskLibraryEnabled": "true",
        "TaskMaximumPoints": "200",
        "TaskMaximumLoops": "10",
        "TaskUnknownCellPolicy": "reject",
        "TaskAutosaveEnabled": "true",
        "TaskAutosaveIntervalS": "30",
        "TaskBackupCount": "5",
    }


def shell_defaults():
    return {
        "UiThemeId": "agt-light",
        "UiLayoutId": "control-center-v1",
        "UiDensity": "comfortable",
        "ShowAdvancedDiagnostics": "false",
    }


def assert_common_business_capabilities(config):
    keys = config["key_value"]
    assert shell_defaults().items() <= keys.items()
    for name in (
        "EnableLegacyWaypointExecution",
        "EnableMissionExecution",
        "EnableSystemModeControl",
        "EnableDebugGoalPose",
        "EnableMappingSessionControl",
        "EnableRelocalization",
        "EnableMapManager",
        "EnableBagManager",
        "ShowOverviewPage",
        "ShowPlatformPage",
        "ShowMappingPage",
        "ShowTeachTuningPage",
        "ShowNavigationMissionPage",
        "ShowMapTaskPage",
        "ShowDiagnosticsPage",
    ):
        assert keys[name] in {"true", "false"}


def test_mapping_profile_contract():
    config = load_profile("mapping")
    assert topics(config)["kOccupancyMap"] == "/agt/map/mapping_occupancy"
    assert topics(config)["kRobotPose"] == "/agt/mapping/odometry"
    assert topics(config)["kSetRobotSpeed"] == "/agt/cmd_vel_manual"
    assert topics(config)["kGlobalPath"] == "/plan"
    assert topics(config)["kLocalPath"] == "/local_plan"
    assert visibility(config)["kLocalCostMap"] is False
    assert visibility(config)["kGlobalCostMap"] is False
    assert {
        "BaseFrameId": "base_footprint",
        "FixedFrameId": "odom",
        "EnableTaskExecution": "false",
        "EnableCostmapDisplay": "false",
        "EnableOfflinePlanningPreview": "false",
        "EnableBaseMapEditing": "false",
        "EnableBaseMapSaveAs": "false",
        "EnableMapOpen": "false",
        "EnableLegacyTopologyTasks": "false",
        **{**task_library_keys(), "TaskLibraryEnabled": "false"},
    }.items() <= config["key_value"].items()
    assert config["key_value"]["ShowMappingPage"] == "true"
    assert config["key_value"]["ShowNavigationMissionPage"] == "false"
    assert config["key_value"]["EnableMappingSessionControl"] == "true"
    assert config["key_value"]["EnableRelocalization"] == "false"
    assert config["key_value"]["EnableMapManager"] == "false"


def test_candidate_profile_edits_only_the_selected_candidate_in_place():
    config = load_profile("candidate")
    assert topics(config)["kOccupancyMap"] == "/agt/map/edited"
    assert config["key_value"]["EnableTaskExecution"] == "false"
    assert config["key_value"]["EnableOfflinePlanningPreview"] == "false"
    assert config["key_value"]["EnableManualControl"] == "false"
    assert config["key_value"]["EnableBaseMapEditing"] == "true"
    assert config["key_value"]["EnableBaseMapSaveAs"] == "false"
    assert config["key_value"]["EnableMapOpen"] == "false"
    assert config["key_value"]["TaskLibraryEnabled"] == "false"
    assert config["key_value"]["ShowMapTaskPage"] == "true"
    assert config["key_value"]["ShowNavigationMissionPage"] == "false"


def test_navigation_profile_contract():
    config = load_profile("navigation")
    profile_topics = topics(config)
    assert profile_topics["kOccupancyMap"] == "/agt/map/global_occupancy"
    assert profile_topics["GoalPose"] == "/goal_pose"
    assert profile_topics["kSetRelocPose"] == "/initialpose"
    assert profile_topics["kSetRobotSpeed"] == "/agt/cmd_vel_manual"
    assert visibility(config)["kLocalCostMap"] is False
    assert visibility(config)["kGlobalCostMap"] is False
    assert visibility(config)["GoalPose"] is False
    assert {
        "BaseFrameId": "base_footprint",
        "FixedFrameId": "map",
        "EnableTaskExecution": "true",
        "EnableCostmapDisplay": "false",
        "EnableOfflinePlanningPreview": "true",
        "EnableBaseMapEditing": "false",
        "EnableLegacyTopologyTasks": "false",
        **task_library_keys(),
    }.items() <= config["key_value"].items()
    assert config["key_value"]["EnableMissionExecution"] == "true"
    assert config["key_value"]["EnableLegacyWaypointExecution"] == "false"
    assert config["key_value"]["EnableDebugGoalPose"] == "false"
    assert config["key_value"]["ShowNavigationMissionPage"] == "true"
    assert config["key_value"]["EnableMappingSessionControl"] == "false"
    assert config["key_value"]["EnableRelocalization"] == "true"
    assert config["key_value"]["EnableMapManager"] == "true"
    assert config["key_value"]["EnableBagManager"] == "false"


def test_offline_profile_is_preview_only():
    config = load_profile("offline")
    assert topics(config)["kOccupancyMap"] == "/agt/map/global_occupancy"
    assert topics(config)["kGlobalPath"] == "/plan"
    assert config["key_value"]["EnableTaskExecution"] == "false"
    assert config["key_value"]["EnableOfflinePlanningPreview"] == "true"
    assert config["key_value"]["EnableBaseMapEditing"] == "false"
    assert config["key_value"]["EnableLegacyTopologyTasks"] == "false"
    assert config["key_value"]["EnableMissionExecution"] == "false"
    assert config["key_value"]["EnableLegacyWaypointExecution"] == "false"
    assert config["key_value"]["ShowNavigationMissionPage"] == "false"
    assert config["key_value"]["ShowMapTaskPage"] == "true"
    assert config["key_value"]["EnableMapManager"] == "false"
    assert config["key_value"]["EnableBagManager"] == "false"
    assert task_library_keys().items() <= config["key_value"].items()


def test_teach_profile_is_read_only_and_latched():
    config = load_profile("teach")
    assert topics(config)["kOccupancyMap"] == "/agt/map/global_occupancy"
    assert topics(config)["kGlobalPath"] == "/agt/teach/reference_path"
    assert (
        topics(config)["kTeachRouteAnnotations"]
        == "/agt/teach/route_annotations"
    )
    assert visibility(config)["kTeachRouteAnnotations"] is True
    assert config["key_value"]["GlobalPathTransientLocal"] == "true"
    assert config["key_value"]["AutoFitTeachRoute"] == "true"
    assert config["key_value"]["EnableTaskExecution"] == "false"
    assert config["key_value"]["EnableOfflinePlanningPreview"] == "false"
    assert config["key_value"]["EnableManualControl"] == "false"
    assert config["key_value"]["ShowDashboard"] == "false"
    assert config["key_value"]["ShowSettingsOnStartup"] == "false"
    assert config["key_value"]["TaskLibraryEnabled"] == "false"
    assert config["key_value"]["EnableBaseMapEditing"] == "false"
    assert config["key_value"]["EnableLegacyTopologyTasks"] == "false"
    assert config["key_value"]["EnableMissionExecution"] == "false"
    assert config["key_value"]["EnableLegacyWaypointExecution"] == "false"
    assert config["key_value"]["ShowTeachTuningPage"] == "true"
    assert config["key_value"]["ShowNavigationMissionPage"] == "false"
    assert config["key_value"]["EnableBagManager"] == "true"


def test_profiles_have_isolated_runtime_configs():
    script = (PACKAGE_ROOT / "scripts/start_ros_qt5_gui_app.sh").read_text(
        encoding="utf-8"
    )
    assert 'RUNTIME_DIR="${RUNTIME_ROOT}/${PROFILE}"' in script
    assert "ros_qt5_gui_${PROFILE}.json" in script
    forbidden_home = "/".join(["", "home", "yangxuan"])
    assert forbidden_home not in script
    assert "LEGACY_BUILD_DIR" not in script


def test_gui_never_targets_chassis_command_topic():
    for profile in (
        load_profile("mapping"),
        load_profile("candidate"),
        load_profile("navigation"),
        load_profile("offline"),
        load_profile("teach"),
    ):
        assert "/agt/chassis/cmd_vel" not in topics(profile).values()


def test_all_profiles_define_replaceable_shell_and_business_capabilities():
    for name in ("mapping", "candidate", "navigation", "offline", "teach"):
        assert_common_business_capabilities(load_profile(name))


def test_theme_and_layout_do_not_change_profile_capabilities(tmp_path):
    from agt_ui_bridge.qt_runtime import prepare_runtime_config

    template_path = PACKAGE_ROOT / "config" / "ros_qt5_gui_offline.json"
    runtime = load_profile("offline")
    runtime["key_value"].update(
        {
            "UiThemeId": "agt-dark",
            "UiLayoutId": "legacy",
            "EnableTaskExecution": "true",
            "EnableMissionExecution": "true",
            "EnableManualControl": "true",
            "EnableBaseMapEditing": "true",
        }
    )
    runtime_path = tmp_path / "config.json"
    runtime_path.write_text(json.dumps(runtime), encoding="utf-8")

    prepare_runtime_config(runtime_path, template_path)
    prepared = json.loads(runtime_path.read_text(encoding="utf-8"))["key_value"]

    assert prepared["UiThemeId"] == "agt-dark"
    assert prepared["UiLayoutId"] == "legacy"
    assert prepared["EnableTaskExecution"] == "false"
    assert prepared["EnableMissionExecution"] == "false"
    assert prepared["EnableManualControl"] == "false"
    assert prepared["EnableBaseMapEditing"] == "false"


def test_ready_map_profiles_never_enable_base_raster_writes():
    for name in ("navigation", "offline", "teach"):
        keys = load_profile(name)["key_value"]
        assert keys["EnableBaseMapEditing"] == "false"
        assert keys["EnableBaseMapSaveAs"] == "false"


def test_candidate_cannot_escape_managed_map_asset():
    keys = load_profile("candidate")["key_value"]
    assert keys["EnableMapOpen"] == "false"
    assert keys["EnableBaseMapSaveAs"] == "false"
    assert keys["EnableMapManager"] == "false"


def test_qt_ros_channel_uses_project_business_boundaries():
    channel = (
        PACKAGE_ROOT.parent.parent
        / "third_party"
        / "ros_qt5_gui_app"
        / "src"
        / "channel"
        / "ros2"
        / "rclcomm.cpp"
    ).read_text(encoding="utf-8")
    for endpoint in (
        '"/agt/system/robot_state"',
        '"/agt/missions/execute"',
        '"/agt/mapping/manage_session"',
        '"/agt/localization/relocalize"',
        '"/agt/maps/list"',
        '"/agt/maps/manage"',
        '"/agt/data/bags/list"',
        '"/agt/data/bags/manage"',
    ):
        assert endpoint in channel
    assert (
        'SET_DEFAULT_TOPIC_NAME(MSG_ID_SET_ROBOT_SPEED, "/agt/cmd_vel_manual")'
        in channel
    )
    assert 'SET_DEFAULT_TOPIC_NAME(MSG_ID_SET_ROBOT_SPEED, "/cmd_vel")' not in channel
    assert '"/follow_waypoints"' not in channel
    assert '"/navigate_to_pose"' not in channel
