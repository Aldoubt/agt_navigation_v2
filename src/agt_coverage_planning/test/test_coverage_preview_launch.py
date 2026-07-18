import importlib.util
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / "launch/coverage_preview.launch.py"
RVIZ_FILE = PACKAGE_ROOT / "rviz/coverage_preview.rviz"
PLANNING_LAUNCH_FILE = PACKAGE_ROOT / "launch/coverage_planning.launch.py"
PROFILE = PACKAGE_ROOT.parents[1] / "profiles/platforms/greenhouse_ackermann.yaml"


def _planning_launch_module():
    spec = importlib.util.spec_from_file_location(
        "coverage_planning_launch", PLANNING_LAUNCH_FILE
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_preview_launch_is_offline_and_fail_closed():
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    compile(source, str(LAUNCH_FILE), "exec")
    assert '"plan_on_start": "true"' in source
    assert '"execution_enabled": "false"' in source
    assert 'package="nav2_map_server"' in source
    assert 'package="rviz2"' in source
    assert 'executable="coverage_time_simulator.py"' in source
    assert 'executable="coverage_preview_auditor.py"' in source
    assert '"path_topic": "/agt/coverage/path_repaired"' in source
    assert '"path_topic": "/agt/coverage/path_preview"' in source
    assert 'package="nav2_planner"' in source
    assert 'executable="coverage_approach_preview.py"' in source
    assert '"auto_repair": "true"' in source
    assert "nav2_controller" not in source
    assert "agt_chassis" not in source
    assert "agt_safety" not in source
    assert "bt_navigator" not in source
    assert "localization" not in source


def test_preview_rviz_contains_map_and_coverage_layers():
    config = yaml.safe_load(RVIZ_FILE.read_text(encoding="utf-8"))
    displays = config["Visualization Manager"]["Displays"]
    topics = {
        display.get("Topic", {}).get("Value")
        for display in displays
        if isinstance(display.get("Topic"), dict)
    }

    assert config["Visualization Manager"]["Global Options"]["Fixed Frame"] == "map"
    assert "/agt/map/global_occupancy" in topics
    assert "/agt/map/keepout_mask" in topics
    assert "/agt/coverage/path_preview" in topics
    assert "/agt/coverage/path_reconstructed" in topics
    assert "/agt/coverage/path_repaired" in topics
    assert "/agt/coverage/path_approach_preview" in topics
    assert "/agt/coverage/swaths" in topics


def test_coverage_servers_start_with_canonical_ackermann_geometry():
    module = _planning_launch_module()

    parameters = module._coverage_server_parameters(PROFILE)

    assert parameters == {"robot_width": 0.6, "min_turning_radius": 1.5}


def test_planner_only_preview_uses_canonical_ackermann_geometry():
    source = LAUNCH_FILE.read_text(encoding="utf-8")
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config/coverage_preview_nav2.yaml").read_text(
            encoding="utf-8"
        )
    )

    assert 'footprint = str(platform["footprint"])' in source
    assert 'radius = float(platform["min_turning_radius"])' in source
    planner = config["planner_server"]["ros__parameters"]
    assert planner["planner_plugins"] == ["CoverageRepairHybrid"]
    assert planner["CoverageRepairHybrid"]["plugin"].endswith("SmacPlannerHybrid")
    costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
    assert costmap["robot_base_frame"] == "map"
    assert costmap["footprint_padding"] == 0.0
