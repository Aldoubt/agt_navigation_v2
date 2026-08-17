from __future__ import annotations

import csv
import hashlib
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
            "scenario_request": {
                "start": {"x_m": 0.5, "y_m": 0.5, "yaw_rad": 0.0},
                "goal": {"x_m": 2.5, "y_m": 2.5, "yaw_rad": 1.0},
            },
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
        "path_length_m": 3.2,
        "max_abs_curvature_1pm": 0.8,
        "required_max_curvature_1pm": 2.0 / 3.0,
        "start_pose_deviation_m": 0.70710678,
        "goal_pose_deviation_m": 0.70710678,
        "goal_heading_deviation_rad": 0.5,
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
    assert "A* cannot solve" not in claims
    assert "always slower" not in claims


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

    semantic_map = tmp_path / "semantic.geojson"
    semantic_map.write_text(
        '{"type":"FeatureCollection","frame_id":"map","features":[]}',
        encoding="utf-8",
    )
    semantic_sha = hashlib.sha256(semantic_map.read_bytes()).hexdigest()

    out = tmp_path / "paper"
    bundle = build_paper_bundle(results, out, semantic_map=semantic_map)

    for filename in (
        "comparison.csv",
        "comparison.json",
        "claims.md",
        "figure_manifest.json",
        "D1_synthetic_problem.svg",
        "D1_synthetic_problem.pdf",
        "D1_synthetic_problem.png",
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
    assert all(len(row["metrics_sha256"]) == 64 for row in rows)
    assert all(row["run_id"] == "diagnostic_001" for row in rows)
    assert all(row["formal"] is False for row in rows)
    assert all(len(row["path_sha256"]) == 64 for row in rows)

    manifest = json.loads((out / "figure_manifest.json").read_text())
    assert manifest["schema_version"] == "1.0"
    assert manifest["source_files_sha256"]["external_semantic/semantic.geojson"] == semantic_sha
    assert set(manifest["figures"]) == {
        "D1_synthetic_problem",
        "D2_S02_planner_comparison",
        "D3_S03_planner_comparison",
        "D4_feasibility_matrix",
    }
    assert all(manifest["figures"][name]["source_runs"] for name in manifest["figures"])
    for figure_id in ("D2_S02_planner_comparison", "D3_S03_planner_comparison"):
        assert manifest["figures"][figure_id]["pose_annotations"] == {
            "requested_pose_source": "experiment_manifest.scenario_request",
            "returned_endpoint_source": "path.csv",
        }
    assert bundle.output_dir == out
