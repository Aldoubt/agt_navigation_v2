import math
from agt_route_benchmark.contracts import PathPoint
from agt_route_benchmark.metrics import compute_path_metrics, compute_mission_metrics


def _circular_arc(radius, step_deg, count, direction="F"):
    step = math.radians(step_deg)
    return [
        PathPoint(
            radius * math.sin(i * step),
            radius * (1.0 - math.cos(i * step)),
            i * step,
            direction,
            "TURN",
            "",
        )
        for i in range(count + 1)
    ]


def test_length_reverse_and_curvature_metrics():
    pts = [
        PathPoint(0.0, 0.0, 0.0, "F", "P2P", ""),
        PathPoint(1.0, 0.0, 0.0, "F", "P2P", ""),
        PathPoint(2.0, 0.0, math.pi / 4, "R", "TURN", ""),
    ]
    m = compute_path_metrics(pts)
    assert abs(m["path_length_m"] - 2.0) < 1e-9
    assert m["reverse_segment_count"] == 1
    assert abs(m["reverse_distance_m"] - 1.0) < 1e-9
    assert m["max_abs_curvature_1pm"] == 0.0
    assert m["direction_transition_count"] == 1
    assert m["ambiguous_direction_transition_count"] == 1
    assert m["direction_transition_feasible"] is False


def test_circular_arc_curvature_uses_chord_invariant_relation():
    for step_deg in (5, 10):
        metrics = compute_path_metrics(_circular_arc(1.5, step_deg, 1), curvature_limit_1pm=1.0 / 1.5)
        assert metrics["max_abs_curvature_1pm"] <= 1.0 / 1.5
    metrics = compute_path_metrics(_circular_arc(1.5, 20, 1), curvature_limit_1pm=1.0 / 1.5)
    assert abs(metrics["max_abs_curvature_1pm"] - 1.0 / 1.5) < 1e-12


def test_curvature_diagnostics_identify_worst_segment():
    metrics = compute_path_metrics(_circular_arc(1.5, 10, 2), curvature_limit_1pm=1.0 / 1.5)
    assert metrics["worst_curvature_segment_index"] == 0
    assert metrics["worst_curvature_direction"] == "F"
    assert abs(metrics["worst_curvature_delta_yaw_rad"] - math.radians(10)) < 1e-12
    assert metrics["worst_curvature_chord_m"] > 0.0
    assert metrics["worst_curvature_ratio_to_limit"] == 1.0


def test_small_radius_and_sharp_corner_fail_the_frozen_limit():
    assert compute_path_metrics(_circular_arc(1.4, 10, 1))["max_abs_curvature_1pm"] > 1.0 / 1.5
    corner = [
        PathPoint(0.0, 0.0, 0.0, "F", "TURN", ""),
        PathPoint(0.1, 0.0, math.pi / 2.0, "F", "TURN", ""),
    ]
    assert compute_path_metrics(corner)["max_abs_curvature_1pm"] > 1.0 / 1.5


def test_reverse_arc_and_forward_reverse_cusp_do_not_add_artificial_curvature():
    reverse = compute_path_metrics(_circular_arc(1.5, 10, 2, direction="R"), curvature_limit_1pm=1.0 / 1.5)
    assert reverse["max_abs_curvature_1pm"] <= 1.0 / 1.5
    cusp = compute_path_metrics([
        PathPoint(0.0, 0.0, 0.0, "F", "TURN", ""),
        PathPoint(1.0, 0.0, 0.0, "F", "TURN", ""),
        PathPoint(1.0, 0.0, 0.0, "R", "TURN", ""),
        PathPoint(0.0, 0.0, 0.0, "R", "TURN", ""),
    ])
    assert cusp["max_abs_curvature_1pm"] == 0.0
    assert cusp["worst_curvature_direction"] == "F"


def test_true_rotate_in_place_is_diagnosed_as_curvature_failure():
    metrics = compute_path_metrics([
        PathPoint(0.0, 0.0, 0.0, "F", "TURN", ""),
        PathPoint(0.0, 0.0, math.pi / 2.0, "F", "TURN", ""),
    ])
    assert metrics["max_abs_curvature_1pm"] == math.inf
    assert metrics["worst_curvature_chord_m"] == 0.0


def test_direction_transition_is_separate_event_and_ambiguous_transition_fails_closed():
    metrics = compute_path_metrics([
        PathPoint(0.0, 0.0, 0.0, "F", "TURN", ""),
        PathPoint(1.0, 0.0, 0.0, "R", "TURN", ""),
        PathPoint(1.1, 0.0, 0.0, "F", "TURN", ""),
    ], curvature_limit_1pm=1.0 / 1.5)
    assert metrics["max_abs_curvature_1pm"] == 0.0
    assert metrics["direction_transition_count"] == 2
    assert metrics["valid_cusp_count"] == 0
    assert metrics["ambiguous_direction_transition_count"] == 2
    assert metrics["direction_transition_feasible"] is False
    assert metrics["direction_transition_status"] == "AMBIGUOUS_DIRECTION_TRANSITION"
    assert "ambiguous_direction_transition" in metrics["validation_error_codes"]


def test_explicit_continuous_cusp_is_valid_but_not_curvature():
    metrics = compute_path_metrics([
        PathPoint(0.0, 0.0, 0.0, "F", "TURN", ""),
        PathPoint(1.0, 0.0, 0.0, "F", "TURN", ""),
        PathPoint(1.0, 0.0, 0.0, "R", "TURN", ""),
        PathPoint(0.0, 0.0, 0.0, "R", "TURN", ""),
    ], curvature_limit_1pm=1.0 / 1.5)
    assert metrics["direction_transition_count"] == 1
    assert metrics["valid_cusp_count"] == 1
    assert metrics["ambiguous_direction_transition_count"] == 0
    assert metrics["direction_transition_feasible"] is True
    assert metrics["direction_transition_status"] == "VALID_CUSP"
    assert metrics["max_abs_curvature_1pm"] == 0.0


def test_reverse_segment_count_means_contiguous_reverse_maneuvers():
    pts = [
        PathPoint(0, 0, 0, "F", "P2P", ""),
        PathPoint(1, 0, 0, "R", "TURN", ""),
        PathPoint(2, 0, 0, "R", "TURN", ""),
        PathPoint(3, 0, 0, "R", "TURN", ""),
        PathPoint(4, 0, 0, "F", "P2P", ""),
        PathPoint(5, 0, 0, "R", "TURN", ""),
        PathPoint(6, 0, 0, "R", "TURN", ""),
    ]
    m = compute_path_metrics(pts)
    assert m["reverse_segment_count"] == 2
    assert m["reverse_distance_m"] == 5.0


def test_mission_metrics_keep_unreachable_tasks_visible():
    m = compute_mission_metrics(
        required_semantic_ids=["row_1", "row_2", "row_3"],
        reachable_semantic_ids=["row_1", "row_3"],
        visited_semantic_ids=["row_1", "row_3"],
        path_points=[
            PathPoint(0, 0, 0, "F", "SWATH", "row_1"),
            PathPoint(1, 0, 0, "F", "CONNECTION", ""),
            PathPoint(2, 0, 0, "F", "SWATH", "row_3"),
        ],
    )
    assert m["required_task_coverage_ratio"] == 2 / 3
    assert m["reachable_task_coverage_ratio"] == 1.0
    assert m["required_but_unreachable"] == ["row_2"]
    assert m["deadhead_distance_m"] == 1.0


def test_mission_metrics_use_independent_reference_reachable_set():
    m = compute_mission_metrics(
        required_semantic_ids=["row_1", "row_2", "row_3"],
        reachable_semantic_ids=["row_1", "row_3"],
        visited_semantic_ids=["row_1"],
        path_points=[PathPoint(0, 0, 0, "F", "SWATH", "row_1")],
    )
    assert m["reachable_task_coverage_ratio"] == 0.5
    assert m["missed_reachable"] == ["row_3"]
