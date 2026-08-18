import json
from pathlib import Path

import pytest

from agt_route_benchmark.matrix_plan import build_execution_plan


def _snapshot(path: Path, sha: str = "a" * 64) -> Path:
    path.write_text(
        json.dumps({
            "snapshot_sha256": sha,
            "curation_gate": "ACCEPTED_REPLAY_CLEAN",
            "assets": {"curation_manifest": {"sha256": "c" * 64}},
            "acceptance": {
                "map_reliability_accepted": True,
                "semantic_correctness_accepted": True,
                "platform_geometry_accepted": True,
            },
        }),
        encoding="utf-8",
    )
    return path


def _formal_result(run: Path, snapshot_sha: str = "a" * 64, *, formal: bool = True) -> None:
    run.mkdir(parents=True)
    (run / "planner_report.json").write_text("{}", encoding="utf-8")
    (run / "metrics.json").write_text("{}", encoding="utf-8")
    (run / "experiment_manifest.json").write_text(
        json.dumps({"formal": formal, "metadata": {"site_snapshot_sha256": snapshot_sha}}),
        encoding="utf-8",
    )


def _plan(tmp_path: Path, **updates):
    kwargs = dict(
        result_root=tmp_path / "results",
        site_id="greenhouse_01",
        scenario_dir=tmp_path / "scenarios",
        map_yaml=tmp_path / "map.yaml",
        platform_profile=tmp_path / "mk_mini.yaml",
        semantic_map=tmp_path / "semantic.geojson",
        manual_waypoints=tmp_path / "manual.yaml",
        ours_route_csv=tmp_path / "route.csv",
        lattice_filepath=tmp_path / "lattice.json",
        site_snapshot=tmp_path / "site_snapshot.json",
        formal=True,
    )
    kwargs.update(updates)
    return build_execution_plan(**kwargs)


def test_execution_plan_contains_exact_23_canonical_cells(tmp_path: Path):
    plan = _plan(tmp_path)
    assert len(plan) == 23
    assert sum(cell.runtime_kind == "nav2_p2p" for cell in plan) == 20
    assert sum(cell.runtime_kind == "manual_waypoints" for cell in plan) == 1
    assert sum(cell.runtime_kind == "fields2cover" for cell in plan) == 1
    assert sum(cell.runtime_kind == "v25_route_asset" for cell in plan) == 1


def test_only_state_lattice_requires_lattice_file(tmp_path: Path):
    plan = _plan(tmp_path, lattice_filepath=None)
    blocked_lattice = [cell for cell in plan if "lattice_filepath" in cell.missing_inputs]
    assert len(blocked_lattice) == 5
    assert {cell.planner_id for cell in blocked_lattice} == {"state_lattice"}


def test_matching_formal_result_cell_is_marked_complete(tmp_path: Path):
    snapshot = _snapshot(tmp_path / "site_snapshot.json")
    run = tmp_path / "results" / "greenhouse_01" / "S01_straight_row" / "astar-deadbeef00-run_001"
    _formal_result(run)
    plan = _plan(tmp_path, site_snapshot=snapshot)
    cell = next(cell for cell in plan if cell.scenario_id == "S01_straight_row" and cell.planner_id == "astar")
    assert cell.status == "COMPLETE"
    assert cell.result_dir is not None


def test_development_result_cannot_satisfy_formal_matrix(tmp_path: Path):
    snapshot = _snapshot(tmp_path / "site_snapshot.json")
    run = tmp_path / "results" / "greenhouse_01" / "S01_straight_row" / "astar-deadbeef00-dev"
    _formal_result(run, formal=False)
    plan = _plan(tmp_path, site_snapshot=snapshot)
    cell = next(cell for cell in plan if cell.scenario_id == "S01_straight_row" and cell.planner_id == "astar")
    assert cell.status != "COMPLETE"


def test_old_snapshot_result_cannot_satisfy_new_formal_matrix(tmp_path: Path):
    snapshot = _snapshot(tmp_path / "site_snapshot.json", "b" * 64)
    run = tmp_path / "results" / "greenhouse_01" / "S01_straight_row" / "astar-deadbeef00-old"
    _formal_result(run, "a" * 64, formal=True)
    plan = _plan(tmp_path, site_snapshot=snapshot)
    cell = next(cell for cell in plan if cell.scenario_id == "S01_straight_row" and cell.planner_id == "astar")
    assert cell.status != "COMPLETE"


def test_formal_matrix_rejects_legacy_snapshot_without_curation_gate(tmp_path: Path):
    snapshot = tmp_path / "legacy_snapshot.json"
    snapshot.write_text(json.dumps({"snapshot_sha256": "a" * 64}), encoding="utf-8")
    with pytest.raises(ValueError, match="formal curation"):
        _plan(tmp_path, site_snapshot=snapshot)
