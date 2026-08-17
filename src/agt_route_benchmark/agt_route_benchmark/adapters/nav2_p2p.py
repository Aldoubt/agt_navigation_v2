from __future__ import annotations

from collections.abc import Callable, Sequence
import math
import time

from .base import PlannerAdapter
from ..contracts import ExperimentSpec, P2P_PLANNERS, PathPoint, PlannerResult

NAV2_PLUGIN_BY_PLANNER = {
    "astar": "GridBased",
    "theta_star": "ThetaStar",
    "hybrid_astar": "GridBasedHybrid",
    "state_lattice": "GridBasedLattice",
}

PlannerCall = Callable[[str, tuple[float, float, float], tuple[float, float, float]], Sequence[PathPoint]]


def _infer_missing_directions(points: tuple[PathPoint, ...]) -> tuple[PathPoint, ...]:
    if len(points) < 2:
        return points
    inferred: list[str] = []
    for a, b in zip(points, points[1:]):
        dx = b.x_m - a.x_m
        dy = b.y_m - a.y_m
        if math.hypot(dx, dy) <= 1e-9:
            inferred.append("UNKNOWN")
            continue
        dot = math.cos(a.yaw_rad) * dx + math.sin(a.yaw_rad) * dy
        inferred.append("F" if dot >= 0.0 else "R")
    inferred.append(inferred[-1])
    return tuple(
        PathPoint(p.x_m, p.y_m, p.yaw_rad, inferred[i] if p.direction == "UNKNOWN" else p.direction, p.segment_type, p.semantic_ref)
        for i, p in enumerate(points)
    )


class Nav2P2PAdapter(PlannerAdapter):
    """Thin benchmark adapter around a runtime Nav2 planner caller.

    The pure-Python core only owns baseline identity and normalization. A ROS2 node
    injects `planner_call` so unit tests do not depend on Nav2.
    """

    def __init__(self, planner_id: str, planner_call: PlannerCall | None = None):
        if planner_id not in P2P_PLANNERS:
            raise ValueError(f"unsupported P2P planner {planner_id!r}")
        self.planner_id = planner_id
        self.plugin_id = NAV2_PLUGIN_BY_PLANNER[planner_id]
        self._planner_call = planner_call

    def plan(self, spec: ExperimentSpec) -> PlannerResult:
        started = time.perf_counter()
        if spec.scenario.level != "p2p" or spec.scenario.start is None or spec.scenario.goal is None:
            return PlannerResult(self.planner_id, False, "INVALID_P2P_SCENARIO", (), time.perf_counter() - started)
        if self._planner_call is None:
            return PlannerResult(
                self.planner_id,
                False,
                "SKIPPED_DEPENDENCY",
                (),
                time.perf_counter() - started,
                metadata={"nav2_plugin_id": self.plugin_id},
            )
        try:
            points = _infer_missing_directions(tuple(self._planner_call(self.plugin_id, spec.scenario.start, spec.scenario.goal)))
        except Exception as exc:
            return PlannerResult(
                self.planner_id,
                False,
                "PLANNER_CALL_FAILED",
                (),
                time.perf_counter() - started,
                metadata={"nav2_plugin_id": self.plugin_id, "detail": str(exc)},
            )
        if not points:
            return PlannerResult(self.planner_id, False, "NO_PATH", (), time.perf_counter() - started, metadata={"nav2_plugin_id": self.plugin_id})
        return PlannerResult(self.planner_id, True, "OK", points, time.perf_counter() - started, metadata={"nav2_plugin_id": self.plugin_id})
