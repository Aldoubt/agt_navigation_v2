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


def test_mapping_profile_contract():
    config = load_profile("mapping")
    assert topics(config)["kOccupancyMap"] == "/agt/map/mapping_occupancy"
    assert topics(config)["kRobotPose"] == "/agt/mapping/odometry"
    assert topics(config)["kSetRobotSpeed"] == "/agt/cmd_vel_manual"
    assert topics(config)["kGlobalPath"] == "/plan"
    assert topics(config)["kLocalPath"] == "/local_plan"
    assert visibility(config)["kLocalCostMap"] is False
    assert visibility(config)["kGlobalCostMap"] is False
    assert config["key_value"] == {
        "BaseFrameId": "base_footprint",
        "FixedFrameId": "odom",
        "EnableTaskExecution": "false",
        "EnableCostmapDisplay": "false",
        "EnableOfflinePlanningPreview": "false",
        "EnableBaseMapEditing": "false",
        "EnableBaseMapSaveAs": "false",
        "EnableMapOpen": "false",
        "EnableLegacyTopologyTasks": "false",
        "UiLanguage": "zh_CN",
        "UseNativeWindowFrame": "true",
        **{**task_library_keys(), "TaskLibraryEnabled": "false"},
    }


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


def test_navigation_profile_contract():
    config = load_profile("navigation")
    profile_topics = topics(config)
    assert profile_topics["kOccupancyMap"] == "/agt/map/global_occupancy"
    assert profile_topics["GoalPose"] == "/goal_pose"
    assert profile_topics["kSetRelocPose"] == "/initialpose"
    assert profile_topics["kSetRobotSpeed"] == "/agt/cmd_vel_manual"
    assert visibility(config)["kLocalCostMap"] is False
    assert visibility(config)["kGlobalCostMap"] is False
    assert config["key_value"] == {
        "BaseFrameId": "base_footprint",
        "FixedFrameId": "map",
        "EnableTaskExecution": "true",
        "EnableCostmapDisplay": "false",
        "EnableOfflinePlanningPreview": "true",
        "EnableBaseMapEditing": "false",
        "EnableLegacyTopologyTasks": "false",
        "UiLanguage": "zh_CN",
        "UseNativeWindowFrame": "true",
        **task_library_keys(),
    }


def test_offline_profile_is_preview_only():
    config = load_profile("offline")
    assert topics(config)["kOccupancyMap"] == "/agt/map/global_occupancy"
    assert topics(config)["kGlobalPath"] == "/plan"
    assert config["key_value"]["EnableTaskExecution"] == "false"
    assert config["key_value"]["EnableOfflinePlanningPreview"] == "true"
    assert config["key_value"]["EnableBaseMapEditing"] == "false"
    assert config["key_value"]["EnableLegacyTopologyTasks"] == "false"
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
