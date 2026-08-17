from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping
import time

from .base import PlannerAdapter
from ..contracts import ExperimentSpec, PathPoint, PlannerResult

Edge = tuple[str, float, str]
Pose = tuple[float, float, float]


@dataclass(frozen=True)
class AgriculturalGraph:
    tasks: tuple[str, ...]
    edges: Mapping[str, tuple[Edge, ...]]
    reachable: tuple[str, ...]
    node_poses: Mapping[str, Pose] = field(default_factory=dict)


class ProposedAdapter(PlannerAdapter):
    """Interpretable maximum-feasible task ordering on a legal semantic graph.

    This core deliberately does not perform collision checking or low-level Ackermann
    connection generation. It selects/ordering reachable task nodes through legal edges;
    a runtime connector may later replace the simple node-pose polyline.
    """

    def __init__(self, graph: AgriculturalGraph):
        self.graph = graph

    def _best_order(self, required: tuple[str, ...]) -> tuple[str, ...]:
        reachable = tuple(node for node in required if node in set(self.graph.reachable))
        if not reachable:
            return ()
        target = set(reachable)
        best: tuple[float, tuple[str, ...]] | None = None

        def search(path: tuple[str, ...], cost: float) -> None:
            nonlocal best
            if set(path) == target:
                candidate = (cost, path)
                if best is None or candidate < best:
                    best = candidate
                return
            current = path[-1]
            for nxt, edge_cost, _semantic in self.graph.edges.get(current, ()):
                if nxt in target and nxt not in path:
                    search(path + (nxt,), cost + float(edge_cost))

        for start in reachable:
            search((start,), 0.0)
        if best is not None:
            return best[1]
        longest: tuple[float, tuple[str, ...]] | None = None

        def search_partial(path: tuple[str, ...], cost: float) -> None:
            nonlocal longest
            score = (-len(path), cost, path)
            if longest is None or score < longest:
                longest = score
            current = path[-1]
            for nxt, edge_cost, _semantic in self.graph.edges.get(current, ()):
                if nxt in target and nxt not in path:
                    search_partial(path + (nxt,), cost + float(edge_cost))

        for start in reachable:
            search_partial((start,), 0.0)
        return longest[2] if longest else ()

    def plan(self, spec: ExperimentSpec) -> PlannerResult:
        started = time.perf_counter()
        required = spec.scenario.required_semantic_ids
        order = self._best_order(required)
        if not order:
            return PlannerResult(spec.planner_id, False, "NO_REACHABLE_TASK", (), time.perf_counter() - started, tuple(self.graph.reachable), ())
        points: list[PathPoint] = []
        for i, node in enumerate(order):
            if node not in self.graph.node_poses:
                return PlannerResult(spec.planner_id, False, "MISSING_NODE_POSE", (), time.perf_counter() - started, tuple(self.graph.reachable), ())
            if i:
                prev = order[i - 1]
                edge = next((e for e in self.graph.edges.get(prev, ()) if e[0] == node), None)
                if edge is None:
                    return PlannerResult(spec.planner_id, False, "ILLEGAL_TRANSITION", (), time.perf_counter() - started, tuple(self.graph.reachable), tuple(order[:i]))
                semantic = edge[2]
                if semantic not in self.graph.node_poses:
                    return PlannerResult(spec.planner_id, False, "MISSING_TRANSITION_POSE", (), time.perf_counter() - started, tuple(self.graph.reachable), tuple(order[:i]))
                x, y, yaw = self.graph.node_poses[semantic]
                points.append(PathPoint(x, y, yaw, "F", "CONNECTION", semantic))
            x, y, yaw = self.graph.node_poses[node]
            points.append(PathPoint(x, y, yaw, "F", "SWATH", node))
        return PlannerResult(spec.planner_id, True, "OK", tuple(points), time.perf_counter() - started, tuple(self.graph.reachable), tuple(order), {"route_order": list(order)})
