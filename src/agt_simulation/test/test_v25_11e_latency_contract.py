from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_route_runner_consumes_safety_authority_and_fails_fast_on_sensor_guard():
    runner = read("scripts/v25_11c_route_runner.py")
    compile(runner, "v25_11c_route_runner.py", "exec")
    for token in (
        "from diagnostic_msgs.msg import DiagnosticArray",
        '"/agt/safety/status"',
        'SAFETY_STATUS_NAME = "agt_safety/tracked_controller"',
        'SAFETY_SENSOR_FAILURE = "sensor_input_unhealthy"',
        "_on_safety_status",
        "_safety_sensor_guard_failed",
        "require_safety: bool = False",
        "require_safety=True",
        '"CONTROLLER_SAFETY_GUARD_FAILED: "',
        "cancel_goal_async",
    ):
        assert token in runner

    # Startup/transient reasons such as input_timeout must not fail a route. The
    # fast abort is deliberately scoped to the sensor-health authority decision.
    assert 'return self.latest_safety_reason == SAFETY_SENSOR_FAILURE' in runner


def test_v25_11d_sensor_failure_class_remains_controller_compatible():
    acceptance = read("scripts/v25_11d_failure_acceptance.py")
    compile(acceptance, "v25_11d_failure_acceptance.py", "exec")
    assert '"lidar_dropout": "CONTROLLER_"' in acceptance
    assert '"imu_dropout": "CONTROLLER_"' in acceptance
    assert "CONTROLLER_SAFETY_GUARD_FAILED:" in read(
        "scripts/v25_11c_route_runner.py"
    )


def test_comparison_matrix_enforces_tightened_response_bounds():
    reporter = read("scripts/v25_11e_compare_reports.py")
    compile(reporter, "v25_11e_compare_reports.py", "exec")
    for token in (
        "MAX_FAULT_TO_SAFETY_ZERO_MS = 750.0",
        "MAX_FAULT_TO_ROUTE_TERMINAL_MS = 1000.0",
        "MAX_POST_FAULT_DISTANCE_M = 0.75",
        '"all_safety_zero_bounded"',
        '"all_route_terminal_bounded"',
        '"all_post_fault_distance_bounded"',
        '"agt_v25_11e_comparison_matrix/v2"',
        '"safety_response_latency_gate"',
        '"response_checks"',
    ):
        assert token in reporter
