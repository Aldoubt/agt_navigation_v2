import pytest
from agt_route_benchmark.contracts import PathPoint, P2P_PLANNERS, SCENARIO_IDS


def test_frozen_matrix_names_are_stable():
    assert P2P_PLANNERS == ("astar", "theta_star", "hybrid_astar", "state_lattice")
    assert SCENARIO_IDS == (
        "S01_straight_row",
        "S02_90deg_entry",
        "S03_headland_uturn",
        "S04_narrow_headland",
        "S05_blocked_row",
        "S06_full_mission",
    )


def test_path_point_rejects_bad_direction():
    with pytest.raises(ValueError, match="direction"):
        PathPoint(0.0, 0.0, 0.0, "SIDEWAYS", "P2P", "")


def test_path_point_rejects_nonfinite_coordinates():
    with pytest.raises(ValueError, match="finite"):
        PathPoint(float("nan"), 0.0, 0.0, "F", "P2P", "")
