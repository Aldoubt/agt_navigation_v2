from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_wrong_map_route_differs_only_by_validation_identity_contract():
    normal = yaml.safe_load(read("routes/v25_11c_map_route.yaml"))
    wrong = yaml.safe_load(read("routes/v25_11d_wrong_map_route.yaml"))
    assert wrong["schema"] == "agt_route/v1"
    assert wrong["map_id"] == normal["map_id"] == "v25_11_gazebo"
    assert wrong["map_hash"] != normal["map_hash"]
    assert wrong["frame_id"] == normal["frame_id"] == "map"
    assert wrong["points"] == normal["points"]
    assert wrong["source"] == "V25-11D_FAULT_INJECTION"


def test_route_runner_continuously_guards_canonical_localization():
    runner = read("scripts/v25_11c_route_runner.py")
    compile(runner, "v25_11c_route_runner.py", "exec")
    for token in (
        "LOCALIZATION_NOT_READY:",
        "LOCALIZATION_GUARD_FAILED:",
        "PLANNER_GOAL_REJECTED",
        "PLANNER_RESULT_FAILED:",
        "CONTROLLER_GOAL_REJECTED",
        "CONTROLLER_RESULT_FAILED:",
        "cancel_goal_async",
        "require_localization=True",
    ):
        assert token in runner


def test_navigation_launch_exposes_only_testable_runner_controls():
    launch = read("launch/gazebo_navigation_validation.launch.py")
    compile(launch, "gazebo_navigation_validation.launch.py", "exec")
    for token in (
        'DeclareLaunchArgument("planner_id"',
        'DeclareLaunchArgument("controller_id"',
        'DeclareLaunchArgument("runner_server_timeout_s"',
        'DeclareLaunchArgument("runner_segment_timeout_s"',
        '"planner_id": ParameterValue(planner_id, value_type=str)',
        '"controller_id": ParameterValue(controller_id, value_type=str)',
    ):
        assert token in launch


def test_failure_launch_supports_deterministic_matrix_without_second_localizer():
    launch = read("launch/gazebo_failure_validation.launch.py")
    compile(launch, "gazebo_failure_validation.launch.py", "exec")
    for token in (
        "map_identity_mismatch",
        "localization_lost",
        "planner_invalid",
        "controller_invalid",
        "v25_11d_wrong_map_route.yaml",
        "__v25_11d_missing_planner__",
        "__v25_11d_missing_controller__",
        "gazebo_navigation_validation.launch.py",
        "v25_11d_fault_injector.py",
        "v25_11d_failure_acceptance.py",
        'trigger_delay_s = float(trigger_delay_text)',
        '"trigger_delay_s": trigger_delay_s',
    ):
        assert token in launch
    assert "ParameterValue(\n                            trigger_delay_s" not in launch
    assert "amcl" not in launch.lower()


def test_fault_injector_uses_existing_localization_evidence_service():
    injector = read("scripts/v25_11d_fault_injector.py")
    compile(injector, "v25_11d_fault_injector.py", "exec")
    for token in (
        '"/agt/simulation/localization/publish_lost"',
        '"/agt/simulation/fault/state"',
        'self.state = "FIRED"',
        '"gate": "V25-11D"',
    ):
        assert token in injector


def test_failure_acceptance_rejects_false_positive_success():
    acceptance = read("scripts/v25_11d_failure_acceptance.py")
    compile(acceptance, "v25_11d_failure_acceptance.py", "exec")
    for token in (
        '"route_failed"',
        '"route_never_succeeded"',
        '"expected_failure_class"',
        '"route_not_completed"',
        '"motion_blocked_before_execution"',
        '"fault_injection_fired"',
        '"canonical_lost_observed"',
        '"navigation_command_seen_before_abort"',
        '"motion_bounded_after_abort"',
        '"map_identity_actually_mismatched"',
        '"planner_action_server_available"',
        '"controller_action_server_available"',
    ):
        assert token in acceptance


def test_cmake_installs_and_tests_v25_11d_entrypoints():
    cmake = read("CMakeLists.txt")
    for token in (
        "FILE_PERMISSIONS",
        "OWNER_EXECUTE",
        "v25_11d_fault_injector.py",
        "v25_11d_failure_acceptance.py",
        "test_v25_11_simulation_contract",
        "test_v25_11c_navigation_contract",
        "test_v25_11d_failure_contract",
    ):
        assert token in cmake
