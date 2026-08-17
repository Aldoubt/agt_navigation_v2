import json
from pathlib import Path
from agt_route_benchmark.contracts import ExperimentSpec, PlannerResult, ScenarioSpec, PathPoint
from agt_route_benchmark.experiment import ExperimentRunner
from agt_route_benchmark.adapters.base import PlannerAdapter


class FakeAdapter(PlannerAdapter):
    def plan(self, spec):
        return PlannerResult(
            planner_id=spec.planner_id,
            success=True,
            error_code="OK",
            path=(PathPoint(0, 0, 0, "F", "P2P", ""), PathPoint(1, 0, 0, "F", "P2P", "")),
            planning_time_s=0.01,
        )


def test_experiment_runner_emits_stable_artifacts(tmp_path: Path):
    scenario = ScenarioSpec("S01_straight_row", "p2p", False, (0, 0, 0), (1, 0, 0), (), {})
    spec = ExperimentSpec("greenhouse_01", "astar", scenario, formal=True, metadata={"site_snapshot_sha256": "a" * 64})
    out = ExperimentRunner(tmp_path).run(spec, FakeAdapter())
    assert (out / "experiment_manifest.json").exists()
    assert (out / "metrics.json").exists()
    assert (out / "path.csv").exists()
    manifest = json.loads((out / "experiment_manifest.json").read_text())
    assert manifest["planner_id"] == "astar"


def test_experiment_runner_merges_offline_validation_metrics(tmp_path: Path):
    scenario = ScenarioSpec("S01_straight_row", "p2p", False, (0, 0, 0), (1, 0, 0), (), {})
    spec = ExperimentSpec("greenhouse_01", "astar", scenario, formal=True, metadata={"site_snapshot_sha256": "a" * 64})

    def evaluator(points):
        assert len(points) == 2
        return {
            "execution_feasible": False,
            "collision_free": False,
            "kinematic_feasible": True,
            "min_clearance_m": 0.08,
            "footprint_collision_count": 2,
            "validation_error_codes": ["footprint_collision"],
        }

    out = ExperimentRunner(tmp_path).run(spec, FakeAdapter(), path_evaluator=evaluator)
    metrics = json.loads((out / "metrics.json").read_text())
    assert metrics["execution_feasible"] is False
    assert metrics["collision_free"] is False
    assert metrics["kinematic_feasible"] is True
    assert metrics["min_clearance_m"] == 0.08
    assert metrics["footprint_collision_count"] == 2


class FailingAdapter(PlannerAdapter):
    def plan(self, spec):
        return PlannerResult(spec.planner_id, False, "NO_PATH", (), 0.02)


def test_failed_run_still_emits_report_without_path_csv(tmp_path: Path):
    scenario = ScenarioSpec("S01_straight_row", "p2p", False, (0, 0, 0), (1, 0, 0), (), {})
    spec = ExperimentSpec("greenhouse_01", "astar", scenario, formal=True, metadata={"site_snapshot_sha256": "a" * 64})
    out = ExperimentRunner(tmp_path).run(spec, FailingAdapter())
    assert (out / "planner_report.json").exists()
    assert not (out / "path.csv").exists()


def test_formal_rerun_does_not_overwrite_existing_result(tmp_path: Path):
    scenario = ScenarioSpec("S01_straight_row", "p2p", False, (0, 0, 0), (1, 0, 0), (), {}, ())
    spec = ExperimentSpec("greenhouse_01", "astar", scenario, formal=True, metadata={"site_snapshot_sha256": "a" * 64})
    runner = ExperimentRunner(tmp_path)
    runner.run(spec, FakeAdapter())
    try:
        runner.run(spec, FakeAdapter())
    except FileExistsError as exc:
        assert "formal result already exists" in str(exc)
    else:
        raise AssertionError("formal rerun silently overwrote experiment")


def test_formal_run_requires_bound_site_snapshot_identity(tmp_path: Path):
    scenario = ScenarioSpec("S01_straight_row", "p2p", False, (0, 0, 0), (1, 0, 0), (), {})
    spec = ExperimentSpec("greenhouse_01", "astar", scenario, formal=True, metadata={})
    with __import__("pytest").raises(ValueError, match="site_snapshot_sha256"):
        ExperimentRunner(tmp_path).run(spec, FakeAdapter())
