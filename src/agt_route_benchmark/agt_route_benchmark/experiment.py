from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
import hashlib

from .adapters.base import PlannerAdapter
from .contracts import ExperimentSpec
from .metrics import compute_mission_metrics, compute_path_metrics
from .path_io import write_json_atomic, write_path_csv, write_path_geojson


class ExperimentRunner:
    def __init__(self, result_root: Path | str):
        self.result_root = Path(result_root)

    def _run_dir(self, spec: ExperimentSpec) -> Path:
        key = f"{spec.site_id}|{spec.scenario.scenario_id}|{spec.planner_id}|{spec.platform_profile}"
        suffix = hashlib.sha256(key.encode()).hexdigest()[:10]
        safe_run_id = spec.run_id.replace("/", "_").replace("\\", "_")
        return self.result_root / spec.site_id / spec.scenario.scenario_id / f"{spec.planner_id}-{suffix}-{safe_run_id}"

    def run(self, spec: ExperimentSpec, adapter: PlannerAdapter) -> Path:
        if spec.formal:
            snapshot_id = str(spec.metadata.get("site_snapshot_sha256", ""))
            if len(snapshot_id) != 64 or any(c not in "0123456789abcdef" for c in snapshot_id):
                raise ValueError("formal experiment requires 64-hex site_snapshot_sha256")
        out = self._run_dir(spec)
        if spec.formal and out.exists():
            raise FileExistsError(f"formal result already exists: {out}")
        out.mkdir(parents=True, exist_ok=True)
        result = adapter.plan(spec)
        manifest = {
            "schema_version": "1.0",
            "site_id": spec.site_id,
            "scenario_id": spec.scenario.scenario_id,
            "experiment_level": spec.scenario.level,
            "planner_id": spec.planner_id,
            "platform_profile": spec.platform_profile,
            "formal": spec.formal,
            "development_fixture": spec.scenario.development_fixture,
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "run_id": spec.run_id,
            "metadata": dict(spec.metadata),
        }
        write_json_atomic(manifest, out / "experiment_manifest.json")
        write_json_atomic(
            {
                "planner_id": result.planner_id,
                "success": result.success,
                "error_code": result.error_code,
                "planning_time_s": result.planning_time_s,
                "reachable_semantic_ids": list(result.reachable_semantic_ids),
                "visited_semantic_ids": list(result.visited_semantic_ids),
                "metadata": dict(result.metadata),
            },
            out / "planner_report.json",
        )
        metrics = {"success": result.success, "error_code": result.error_code, "planning_time_s": result.planning_time_s}
        if result.success:
            write_path_csv(result.path, out / "path.csv")
            write_path_geojson(result.path, out / "path.geojson")
            metrics.update(compute_path_metrics(result.path))
            if spec.scenario.level == "mission":
                metrics.update(
                    compute_mission_metrics(
                        required_semantic_ids=spec.scenario.required_semantic_ids,
                        reachable_semantic_ids=(
                            spec.scenario.reference_reachable_semantic_ids
                            if spec.scenario.reference_reachable_semantic_ids is not None
                            else result.reachable_semantic_ids
                        ),
                        visited_semantic_ids=result.visited_semantic_ids,
                        path_points=result.path,
                    )
                )
        write_json_atomic(metrics, out / "metrics.json")
        return out
