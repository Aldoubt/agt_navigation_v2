from __future__ import annotations

from collections.abc import Callable, Sequence
import time

from .base import PlannerAdapter
from ..contracts import ExperimentSpec, PathPoint, PlannerResult

CoverageCall = Callable[[ExperimentSpec], tuple[Sequence[dict], float]]


class Fields2CoverAdapter(PlannerAdapter):
    """Normalize existing agt_coverage_planning PathComponents semantics."""

    def __init__(self, coverage_call: CoverageCall | None = None):
        self._coverage_call = coverage_call

    def normalize(self, *, planner_id: str, planning_time_s: float, components: Sequence[dict]) -> PlannerResult:
        points: list[PathPoint] = []
        visited: list[str] = []
        for component in components:
            segment_type = str(component.get("segment_type", "UNKNOWN"))
            semantic_ref = str(component.get("semantic_ref", ""))
            raw_points = component.get("points") or []
            if not raw_points:
                continue
            if segment_type == "SWATH" and semantic_ref and semantic_ref not in visited:
                visited.append(semantic_ref)
            normalized = [PathPoint(float(p[0]), float(p[1]), float(p[2]), "F", segment_type, semantic_ref) for p in raw_points]
            if points and normalized and points[-1].x_m == normalized[0].x_m and points[-1].y_m == normalized[0].y_m and points[-1].yaw_rad == normalized[0].yaw_rad:
                normalized = normalized[1:]
            points.extend(normalized)
        if not points:
            return PlannerResult(planner_id, False, "EMPTY_COVERAGE_PATH", (), planning_time_s)
        return PlannerResult(
            planner_id,
            True,
            "OK",
            tuple(points),
            planning_time_s,
            reachable_semantic_ids=tuple(visited),
            visited_semantic_ids=tuple(visited),
            metadata={"source": "agt_coverage_planning/path_components"},
        )

    def plan(self, spec: ExperimentSpec) -> PlannerResult:
        started = time.perf_counter()
        if self._coverage_call is None:
            return PlannerResult("fields2cover", False, "SKIPPED_DEPENDENCY", (), time.perf_counter() - started)
        try:
            components, planning_time_s = self._coverage_call(spec)
        except Exception as exc:
            return PlannerResult("fields2cover", False, "COVERAGE_CALL_FAILED", (), time.perf_counter() - started, metadata={"detail": str(exc)})
        return self.normalize(planner_id="fields2cover", planning_time_s=float(planning_time_s), components=components)
