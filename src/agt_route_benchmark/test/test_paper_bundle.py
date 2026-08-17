from __future__ import annotations

import csv
import json
from pathlib import Path

from agt_route_benchmark.paper_bundle import build_paper_bundle


def _write_run(
    root: Path,
    *,
    scenario: str,
    planner: str,
    success: bool = True,
    error_code: str = "OK",
    collision_free=None,
    kinematic_feasible=None,
    execution_feasible=None,
    path_points=None,
):
    run = root / scenario / planner
    run.mkdir(parents=True)
    (run / "experiment_manifest.json").write_text(
        json.dumps({
            "schema_version": "1.0",
            "site_id": "synthetic_greenhouse_v1",
            "scenario_id": scenario,
            "planner_id": planner,
            "formal": False,
            "development_fixture": True,
            "run_id": "diagnostic_001",
            "metadata": {},
        }),
        encoding="utf-8",
    )
    (run / "planner_report.json").write_text(
        json.dumps({
            "planner_id": planner,
            "success": success,
            "error_code": error_code,
            "planning_time_s": 0.01,
            "metadata": {},
        }),
        encoding="utf-8",
    )
    metrics = {
        "success": success,
        "error_code": error_code,
        "planning_time_s": 0.01,
    }
    if collision_free is not None:
        metrics["collision_free"] = collision_free
    if kinematic_feasible is not None:
        metrics["kinematic_feasible"] = kinematic_feasible
    if execution_feasible is not None:
        metrics["execution_feasible"] = execution_feasible
    (run / "metrics.json").write_text(json.dumps(metrics), encoding="utf-8")
    if success:
        points = path_points or [(1.0, 1.0, 0.0), (2.0, 2.0, 0.5)]
        with (run / "path.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["index", "x_m", "y_m", "yaw_rad", "direction", "segment_type", "semantic_ref"])
            for index, (x, y, yaw) in enumerate(points):
                writer.writerow([index, x, y, yaw, "F", "P2P", ""])
    return run


def test_claims_distinguish_geometric_success_from_kinematic_infeasibility(tmp_path: Path):
    results = tmp_path / "results"
    _write_run(
        results,
        scenario="S02_90deg_entry",
        planner="astar",
        success=True,
        collision_free=True,
        kinematic_feasible=False,
        execution_feasible=False,
    )
    _write_run(
        results,
        scenario="S02_90deg_entry",
        planner="hybrid_astar",
        success=True,
        collision_free=True,
        kinematic_feasible=True,
        execution_feasible=True,
    )

    bundle = build_paper_bundle(results, tmp_path / "paper")
    claims = bundle.claims_markdown

    assert "A* returned a collision-free geometric path" in claims
    assert "classified it as kinematically infeasible" in claims
    assert "Hybrid A* passed the frozen path-level execution-feasibility checks" in claims
    assert "real-vehicle tracking validation" in claims
    assert "outperforms" not in claims.lower()
    assert "superior" not in claims.lower()


def test_claims_refuse_planner_conclusion_for_invalid_scenario(tmp_path: Path):
    results = tmp_path / "results"
    _write_run(
        results,
        scenario="S02_90deg_entry",
        planner="astar",
        success=False,
        error_code="INVALID_SCENARIO",
    )

    bundle = build_paper_bundle(results, tmp_path / "paper")
    assert "No planner conclusion is drawn" in bundle.claims_markdown
    assert "invalid scenario input" in bundle.claims_markdown


def test_claims_state_when_scenario_does_not_separate_planners(tmp_path: Path):
    results = tmp_path / "results"
    for planner in ("astar", "theta_star", "hybrid_astar"):
        _write_run(
            results,
            scenario="S03_headland_uturn",
            planner=planner,
            success=True,
            collision_free=True,
            kinematic_feasible=True,
            execution_feasible=True,
        )

    bundle = build_paper_bundle(results, tmp_path / "paper")
    assert "did not separate the evaluated planners in path-level feasibility" in bundle.claims_markdown
    assert "cannot support a planner-capability difference claim" in bundle.claims_markdown


def test_bundle_writes_reproducible_tables_figures_and_source_manifest(tmp_path: Path):
    results = tmp_path / "results"
    for scenario in ("S02_90deg_entry", "S03_headland_uturn"):
        for planner in ("astar", "theta_star", "hybrid_astar"):
            _write_run(
                results,
                scenario=scenario,
                planner=planner,
                success=True,
                collision_free=True,
                kinematic_feasible=(planner == "hybrid_astar" or scenario == "S03_headland_uturn"),
                execution_feasible=(planner == "hybrid_astar" or scenario == "S03_headland_uturn"),
            )

    out = tmp_path / "paper"
    bundle = build_paper_bundle(results, out)

    for filename in (
        "comparison.csv",
        "comparison.json",
        "claims.md",
        "figure_manifest.json",
        "D2_S02_planner_comparison.svg",
        "D2_S02_planner_comparison.pdf",
        "D2_S02_planner_comparison.png",
        "D3_S03_planner_comparison.svg",
        "D3_S03_planner_comparison.pdf",
        "D3_S03_planner_comparison.png",
        "D4_feasibility_matrix.svg",
        "D4_feasibility_matrix.pdf",
        "D4_feasibility_matrix.png",
    ):
        assert (out / filename).is_file(), filename
        assert (out / filename).stat().st_size > 0

    rows = json.loads((out / "comparison.json").read_text())
    assert len(rows) == 6
    manifest = json.loads((out / "figure_manifest.json").read_text())
    assert manifest["schema_version"] == "1.0"
    assert set(manifest["figures"]) == {
        "D2_S02_planner_comparison",
        "D3_S03_planner_comparison",
        "D4_feasibility_matrix",
    }
    assert all(manifest["figures"][name]["source_runs"] for name in manifest["figures"])
    assert bundle.output_dir == out
