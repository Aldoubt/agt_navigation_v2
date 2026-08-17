import pytest
from agt_route_benchmark.batch import (
    MatrixCell,
    canonical_expected_cells,
    planner_outcome_counts,
    validate_batch_completeness,
    validate_canonical_batch_completeness,
)


def test_formal_batch_rejects_missing_required_cell():
    cells = [MatrixCell("S01_straight_row", "astar", "OK")]
    with pytest.raises(ValueError, match="missing"):
        validate_batch_completeness(cells, [("S01_straight_row", "astar"), ("S01_straight_row", "theta_star")], formal=True)


def test_development_batch_allows_skipped_dependency():
    cells = [
        MatrixCell("S01_straight_row", "astar", "OK"),
        MatrixCell("S01_straight_row", "theta_star", "SKIPPED_DEPENDENCY"),
    ]
    validate_batch_completeness(
        cells,
        [("S01_straight_row", "astar"), ("S01_straight_row", "theta_star")],
        formal=False,
    )


def test_canonical_matrix_is_20_p2p_plus_3_mission_cells():
    expected = canonical_expected_cells()
    assert len(expected) == 23
    assert ("S01_straight_row", "astar") in expected
    assert ("S05_blocked_row", "state_lattice") in expected
    assert ("S06_full_mission", "manual_waypoints_best_p2p") in expected
    assert ("S06_full_mission", "fields2cover") in expected
    assert ("S06_full_mission", "ours") in expected
    assert ("S01_straight_row", "ours") not in expected
    assert ("S06_full_mission", "astar") not in expected


def test_canonical_formal_matrix_rejects_one_missing_cell():
    expected = canonical_expected_cells()
    cells = [MatrixCell(s, p, "OK") for s, p in expected[:-1]]
    with pytest.raises(ValueError, match="missing"):
        validate_canonical_batch_completeness(cells, formal=True)


def test_canonical_formal_matrix_accepts_exact_valid_cells():
    cells = [MatrixCell(s, p, "OK") for s, p in canonical_expected_cells()]
    validate_canonical_batch_completeness(cells, formal=True)


def test_planner_outcome_counts_excludes_invalid_scenario_from_denominator():
    counts = planner_outcome_counts(
        [
            MatrixCell("S01_straight_row", "astar", "OK"),
            MatrixCell("S02_90deg_entry", "astar", "NO_PATH"),
            MatrixCell("S03_headland_uturn", "astar", "INVALID_SCENARIO"),
        ]
    )

    assert counts == {
        "planner_evaluated_count": 2,
        "planner_success_count": 1,
        "planner_failure_count": 1,
        "invalid_scenario_count": 1,
    }
