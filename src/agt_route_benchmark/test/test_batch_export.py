import json
from pathlib import Path
from agt_route_benchmark.batch import MatrixCell, write_comparison_summary


def test_batch_summary_exports_csv_and_json(tmp_path: Path):
    cells = [
        MatrixCell("S01_straight_row", "astar", "OK", {"path_length_m": 2.0}),
        MatrixCell("S01_straight_row", "theta_star", "NO_PATH", {"path_length_m": None}),
    ]
    csv_path, json_path = write_comparison_summary(cells, tmp_path)
    assert csv_path.exists()
    payload = json.loads(json_path.read_text())
    assert payload[0]["planner_id"] == "astar"
    assert "path_length_m" in csv_path.read_text().splitlines()[0]
