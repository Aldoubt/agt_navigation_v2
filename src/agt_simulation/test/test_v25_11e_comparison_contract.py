from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_fault_metrics_observer_tracks_fault_zero_terminal_and_distance():
    script = read("scripts/v25_11e_fault_metrics.py")
    compile(script, "v25_11e_fault_metrics.py", "exec")
    for token in (
        '"localization_lost"',
        '"lidar_dropout"',
        '"imu_dropout"',
        '"/agt/simulation/fault/state"',
        '"/agt/simulation/navigation/route_state"',
        '"/agt/safety/cmd_vel"',
        '"/simulation/bunker/ground_truth"',
        '"agt_v25_11e_fault_metrics/v1"',
        '"fault_to_safety_zero_ms"',
        '"fault_to_route_terminal_ms"',
        '"max_post_fault_distance_m"',
        '"max_post_fault_cmd_norm"',
        '"route_completion_ratio"',
        'data.get("current_ros_ns")',
        "self.first_zero_cmd_after_fault_ros_ns = self.fault_fired_ros_ns",
    ):
        assert token in script


def test_comparison_acceptance_requires_original_11d_gate_and_response_metrics():
    script = read("scripts/v25_11e_comparison_acceptance.py")
    compile(script, "v25_11e_comparison_acceptance.py", "exec")
    for token in (
        '"v25_11d_result_available"',
        "node.v25_11d_result_path.is_file",
        '"v25_11d_gate_passed"',
        'v25_11d.get("status") == "PASS"',
        '"fault_metrics_schema_valid"',
        '"pre_fault_motion_observed"',
        '"fault_injection_observed"',
        '"safety_zero_observed"',
        '"route_terminal_after_fault"',
        '"timing_order_valid"',
        '"fault_to_zero_bounded"',
        '"post_fault_motion_bounded"',
        '"observability_runtime_healthy"',
        '"observability_core_events_present"',
        '"fault_to_safety_zero_ms"',
        '"fault_to_route_terminal_ms"',
        '"max_post_fault_distance_m"',
    ):
        assert token in script
    assert "fault_ns <= zero_ns <= terminal_ns" not in script
    assert "fault_ns <= zero_ns" in script
    assert "fault_ns <= terminal_ns" in script


def test_comparison_launch_reuses_v25_11d_failure_stack_without_topology_copy():
    launch = read("launch/gazebo_observability_failure_comparison.launch.py")
    compile(launch, "gazebo_observability_failure_comparison.launch.py", "exec")
    for token in (
        "gazebo_failure_validation.launch.py",
        '"fault_case": fault_case',
        '"use_rviz": "false"',
        '"trigger_delay_s": trigger_delay_s',
        'executable="v25_11e_observability.py"',
        'executable="v25_11e_fault_metrics.py"',
        'executable="v25_11e_comparison_acceptance.py"',
        "TimerAction(period=0.25, actions=[observer, fault_metrics])",
        "TimerAction(period=0.75, actions=[comparison_acceptance])",
        'default_value="localization_lost"',
        "Path(artifact).unlink(missing_ok=True)",
        "v25_11d_result_path",
    ):
        assert token in launch
    assert "gazebo_navigation_validation.launch.py" not in launch
    assert "v25_11d_fault_injector.py" not in launch


def test_matrix_reporter_requires_all_three_active_fault_reports():
    script = read("scripts/v25_11e_compare_reports.py")
    compile(script, "v25_11e_compare_reports.py", "exec")
    for token in (
        '"localization_lost"',
        '"lidar_dropout"',
        '"imu_dropout"',
        '"agt_v25_11e_comparison_matrix/v1"',
        '"all_fault_reports_present"',
        '"all_fault_reports_passed"',
        '"fault_to_safety_zero_ms"',
        '"fault_to_route_terminal_ms"',
        '"max_post_fault_distance_m"',
        '"route_completion_ratio"',
    ):
        assert token in script
