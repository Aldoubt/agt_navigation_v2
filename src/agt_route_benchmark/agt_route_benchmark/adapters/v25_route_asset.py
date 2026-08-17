from __future__ import annotations

from pathlib import Path
import time

from .base import PlannerAdapter
from ..contracts import ExperimentSpec, PathPoint, PlannerResult


def _segment_type(segment_id: str, semantic_ref: str) -> str:
    token = segment_id.lower()
    if semantic_ref == "<connector>" or "connector" in token:
        return "CONNECTION"
    if "turn" in token:
        return "TURN"
    if "access" in token or "service" in token:
        return "ACCESS"
    return "SWATH"


class V25RouteAssetAdapter(PlannerAdapter):
    """Normalize a V2.5 canonical Route Asset CSV for the Paper I benchmark.

    The adapter intentionally does not regenerate, smooth, or otherwise change route
    geometry. V2.5/12g remains responsible for maximum-feasible coverage and vehicle-
    feasible connector production; the benchmark independently evaluates the exact
    produced path and exports it using the shared Paper I contract.
    """

    def __init__(self, route_csv: Path | str):
        self.route_csv = Path(route_csv).expanduser().resolve()

    def plan(self, spec: ExperimentSpec) -> PlannerResult:
        started = time.perf_counter()
        try:
            from agt_offline_assets.route_asset import load_route_csv
        except ImportError as exc:
            raise RuntimeError("V2.5 route bridge requires agt_offline_assets.route_asset") from exc

        samples = load_route_csv(self.route_csv)
        points = tuple(
            PathPoint(
                float(sample.x),
                float(sample.y),
                float(sample.yaw),
                str(sample.direction),
                _segment_type(str(sample.segment_id), str(sample.semantic_ref)),
                "" if str(sample.semantic_ref).startswith("<") else str(sample.semantic_ref),
            )
            for sample in samples
        )
        visited = tuple(
            dict.fromkeys(
                point.semantic_ref
                for point in points
                if point.semantic_ref
            )
        )
        reachable = (
            tuple(spec.scenario.reference_reachable_semantic_ids)
            if spec.scenario.reference_reachable_semantic_ids is not None
            else visited
        )
        return PlannerResult(
            planner_id=spec.planner_id,
            success=True,
            error_code="OK",
            path=points,
            planning_time_s=time.perf_counter() - started,
            reachable_semantic_ids=reachable,
            visited_semantic_ids=visited,
            metadata={
                "source_route_csv": str(self.route_csv),
                "source_backend": "agt_navigation_v2_route_asset",
                "geometry_modified_by_benchmark": False,
            },
        )
