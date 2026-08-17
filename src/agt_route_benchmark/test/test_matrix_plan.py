from pathlib import Path

from agt_route_benchmark.matrix_plan import build_execution_plan


def test_execution_plan_contains_exact_23_canonical_cells(tmp_path: Path):
    plan = build_execution_plan(
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
    assert len(plan) == 23
    assert sum(cell.runtime_kind == "nav2_p2p" for cell in plan) == 20
    assert sum(cell.runtime_kind == "manual_waypoints" for cell in plan) == 1
    assert sum(cell.runtime_kind == "fields2cover" for cell in plan) == 1
    assert sum(cell.runtime_kind == "v25_route_asset" for cell in plan) == 1


def test_only_state_lattice_requires_lattice_file(tmp_path: Path):
    plan = build_execution_plan(
        result_root=tmp_path / "results",
        site_id="greenhouse_01",
        scenario_dir=tmp_path / "scenarios",
        map_yaml=tmp_path / "map.yaml",
        platform_profile=tmp_path / "mk_mini.yaml",
        semantic_map=tmp_path / "semantic.geojson",
        manual_waypoints=tmp_path / "manual.yaml",
        ours_route_csv=tmp_path / "route.csv",
        lattice_filepath=None,
        site_snapshot=tmp_path / "site_snapshot.json",
        formal=True,
    )
    blocked_lattice = [cell for cell in plan if "lattice_filepath" in cell.missing_inputs]
    assert len(blocked_lattice) == 5
    assert {cell.planner_id for cell in blocked_lattice} == {"state_lattice"}


def test_existing_result_cell_is_marked_complete(tmp_path: Path):
    run = tmp_path / "results" / "greenhouse_01" / "S01_straight_row" / "astar-deadbeef00-run_001"
    run.mkdir(parents=True)
    (run / "planner_report.json").write_text("{}", encoding="utf-8")
    (run / "metrics.json").write_text("{}", encoding="utf-8")
    plan = build_execution_plan(
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
    cell = next(cell for cell in plan if cell.scenario_id == "S01_straight_row" and cell.planner_id == "astar")
    assert cell.status == "COMPLETE"
    assert cell.result_dir is not None
