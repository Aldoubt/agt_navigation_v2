import math
from agt_route_benchmark.contracts import PathPoint
from agt_route_benchmark.metrics import compute_path_metrics, compute_mission_metrics


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
    assert abs(m["max_abs_curvature_1pm"] - math.pi / 4) < 1e-9


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
