import pytest
from agt_route_benchmark.batch import MatrixCell, validate_batch_completeness


def test_formal_batch_rejects_missing_required_cell():
    cells = [MatrixCell("S01_straight_row", "astar", "OK")]
    with pytest.raises(ValueError, match="missing"):
        validate_batch_completeness(cells, ["S01_straight_row"], ["astar", "theta_star"], formal=True)


def test_development_batch_allows_skipped_dependency():
    cells = [
        MatrixCell("S01_straight_row", "astar", "OK"),
        MatrixCell("S01_straight_row", "theta_star", "SKIPPED_DEPENDENCY"),
    ]
    validate_batch_completeness(cells, ["S01_straight_row"], ["astar", "theta_star"], formal=False)
