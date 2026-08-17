from pathlib import Path
import pytest

from agt_route_benchmark.manual_waypoint_io import load_manual_waypoint_plan


def test_manual_waypoint_plan_freezes_best_p2p_and_target_rows(tmp_path: Path):
    path = tmp_path / "manual_waypoints.yaml"
    path.write_text(
        "schema_version: '1.0'\n"
        "frame_id: map\n"
        "p2p_planner: hybrid_astar\n"
        "waypoints:\n"
        "  - {x: 0.0, y: 0.0, yaw: 0.0, semantic_ref: row_01}\n"
        "  - {x: 5.0, y: 0.0, yaw: 0.0, semantic_ref: row_01}\n"
        "  - {x: 5.0, y: 2.0, yaw: 3.14159, semantic_ref: row_02}\n",
        encoding="utf-8",
    )
    plan = load_manual_waypoint_plan(path)
    assert plan.frame_id == "map"
    assert plan.p2p_planner == "hybrid_astar"
    assert plan.waypoints == ((0.0, 0.0, 0.0), (5.0, 0.0, 0.0), (5.0, 2.0, 3.14159))
    assert plan.target_semantic_ids == ("row_01", "row_02")


def test_manual_waypoint_plan_rejects_non_p2p_planner(tmp_path: Path):
    path = tmp_path / "manual_waypoints.yaml"
    path.write_text(
        "schema_version: '1.0'\nframe_id: map\np2p_planner: fields2cover\n"
        "waypoints:\n  - {x: 0, y: 0, yaw: 0}\n  - {x: 1, y: 0, yaw: 0}\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="p2p_planner"):
        load_manual_waypoint_plan(path)


def test_manual_waypoint_plan_requires_two_finite_waypoints(tmp_path: Path):
    path = tmp_path / "manual_waypoints.yaml"
    path.write_text(
        "schema_version: '1.0'\nframe_id: map\np2p_planner: astar\n"
        "waypoints:\n  - {x: 0, y: 0, yaw: 0}\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="at least two"):
        load_manual_waypoint_plan(path)
