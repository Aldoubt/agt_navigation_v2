import json
from pathlib import Path

from agt_route_benchmark.result_selection import load_site_snapshot_identity, select_result_run


def _run(path: Path, *, formal: bool, snapshot_sha: str) -> Path:
    path.mkdir(parents=True)
    (path / "planner_report.json").write_text('{"error_code":"OK"}', encoding="utf-8")
    (path / "metrics.json").write_text('{"success":true}', encoding="utf-8")
    (path / "experiment_manifest.json").write_text(
        json.dumps({"formal": formal, "metadata": {"site_snapshot_sha256": snapshot_sha}}),
        encoding="utf-8",
    )
    return path


def test_load_site_snapshot_identity(tmp_path: Path):
    path = tmp_path / "snapshot.json"
    path.write_text(json.dumps({"snapshot_sha256": "a" * 64}), encoding="utf-8")
    assert load_site_snapshot_identity(path) == "a" * 64


def test_formal_selection_ignores_development_and_old_snapshot_runs(tmp_path: Path):
    base = tmp_path / "greenhouse_01" / "S01_straight_row"
    _run(base / "astar-z-development", formal=False, snapshot_sha="b" * 64)
    _run(base / "astar-y-old", formal=True, snapshot_sha="a" * 64)
    expected = _run(base / "astar-x-current", formal=True, snapshot_sha="b" * 64)
    selected = select_result_run(
        tmp_path,
        "greenhouse_01",
        "S01_straight_row",
        "astar",
        formal=True,
        site_snapshot_sha256="b" * 64,
    )
    assert selected == expected


def test_formal_selection_returns_none_without_matching_revision(tmp_path: Path):
    base = tmp_path / "greenhouse_01" / "S01_straight_row"
    _run(base / "astar-run", formal=True, snapshot_sha="a" * 64)
    assert select_result_run(
        tmp_path,
        "greenhouse_01",
        "S01_straight_row",
        "astar",
        formal=True,
        site_snapshot_sha256="b" * 64,
    ) is None
