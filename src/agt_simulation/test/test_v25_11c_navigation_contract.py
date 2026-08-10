from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_route_contract_and_map_identity():
    route = yaml.safe_load(read("routes/v25_11c_map_route.yaml"))
    assert route["schema"] == "agt_route/v1"
    assert route["map_id"] == "v25_11_gazebo"
    assert route["map_hash"] == "software-only:agri-validation-v1"
    assert route["frame_id"] == "map"
    assert route["points"][0] == {"x": -7.0, "y": 0.0, "yaw": 0.0}
    assert route["points"][-1] == {"x": -2.0, "y": 0.0, "yaw": 0.0}


def test_nav2_contract_uses_canonical_frames_topics_and_plugins():
    config = yaml.safe_load(read("config/v25_11c_nav2.yaml"))
    planner = config["planner_server"]["ros__parameters"]
    controller = config["controller_server"]["ros__parameters"]
    global_map = config["global_costmap"]["global_costmap"]["ros__parameters"]
    local_map = config["local_costmap"]["local_costmap"]["ros__parameters"]
    assert planner["GridBased"]["plugin"] == "nav2_navfn_planner/NavfnPlanner"
    assert controller["FollowPath"]["plugin"] == "dwb_core::DWBLocalPlanner"
    assert controller["odom_topic"] == "/agt/mapping/odometry"
    assert global_map["global_frame"] == "map"
    assert "static_layer" in global_map["plugins"]
    assert local_map["global_frame"] == "odom"
    assert local_map["obstacle_layer"]["scan"]["topic"] == "/agt/sensors/lidar/scan"


def test_launch_reuses_global_correction_manager_path_without_amcl():
    launch = read("launch/gazebo_navigation_validation.launch.py")
    compile(launch, "gazebo_navigation_validation.launch.py", "exec")
    assert "gazebo_localization_validation.launch.py" in launch
    assert '"run_acceptance": "false"' in launch
    assert 'package="nav2_map_server"' in launch
    assert 'package="nav2_planner"' in launch
    assert 'package="nav2_controller"' in launch
    assert '("cmd_vel", "/agt/safety/cmd_vel")' in launch
    assert "nav2_amcl" not in launch
    assert "SOFTWARE_ONLY" in launch


def test_runner_is_fail_closed_on_localization_map_identity():
    runner = read("scripts/v25_11c_route_runner.py")
    compile(runner, "v25_11c_route_runner.py", "exec")
    for token in (
        "ComputePathToPose",
        "FollowPath",
        '"/agt/navigation/runtime_path"',
        '"/agt/localization/status"',
        "status.map_id == self.map_id",
        "status.map_hash == self.map_hash",
        'self.state = "SUCCEEDED"',
        'self.state = "FAILED"',
    ):
        assert token in runner


def test_acceptance_requires_physics_motion_and_goal_reach():
    acceptance = read("scripts/v25_11c_navigation_acceptance.py")
    compile(acceptance, "v25_11c_navigation_acceptance.py", "exec")
    for token in (
        '"map_has_occupied_and_free_space"',
        '"runtime_path_on_free_map_cells"',
        '"route_map_identity_matches_localization"',
        '"route_execution_succeeded"',
        '"physics_model_moves"',
        '"navigation_cmd_published"',
        '"goal_reached"',
        '"/tmp/agt_v25_11c_navigation_result.json"',
    ):
        assert token in acceptance
