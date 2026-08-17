from __future__ import annotations

import time
from typing import Sequence

from .base import PlannerAdapter
from ..contracts import ExperimentSpec, PlannerResult, ScenarioSpec


class ManualWaypointAdapter(PlannerAdapter):
    def __init__(
        self,
        waypoints: Sequence[tuple[float, float, float]],
        p2p_adapter: PlannerAdapter,
        *,
        visited_semantic_ids: Sequence[str] = (),
    ):
        if len(waypoints) < 2:
            raise ValueError("manual waypoint baseline requires at least two waypoints")
        self.waypoints = tuple((float(x), float(y), float(yaw)) for x, y, yaw in waypoints)
        self.p2p_adapter = p2p_adapter
        self.visited_semantic_ids = tuple(dict.fromkeys(str(value) for value in visited_semantic_ids if str(value)))

    def plan(self, spec: ExperimentSpec) -> PlannerResult:
        started = time.perf_counter()
        merged = []
        planning_time = 0.0
        for index, (start, goal) in enumerate(zip(self.waypoints, self.waypoints[1:])):
            segment = ScenarioSpec(
                scenario_id="S01_straight_row",
                level="p2p",
                development_fixture=spec.scenario.development_fixture,
                start=start,
                goal=goal,
                required_semantic_ids=(),
                metadata={"manual_segment_index": index},
            )
            segment_spec = ExperimentSpec(
                site_id=spec.site_id,
                planner_id=getattr(self.p2p_adapter, "planner_id", spec.planner_id),
                scenario=segment,
                formal=spec.formal,
                platform_profile=spec.platform_profile,
                metadata=spec.metadata,
            )
            result = self.p2p_adapter.plan(segment_spec)
            planning_time += result.planning_time_s
            if not result.success:
                return PlannerResult(
                    spec.planner_id,
                    False,
                    f"MANUAL_SEGMENT_{index}_{result.error_code}",
                    (),
                    time.perf_counter() - started,
                    metadata={
                        "manual_waypoints": [list(point) for point in self.waypoints],
                        "manual_target_semantic_ids": list(self.visited_semantic_ids),
                        "failed_segment": index,
                    },
                )
            points = list(result.path)
            if merged and points and merged[-1] == points[0]:
                points = points[1:]
            merged.extend(points)

        reachable = (
            tuple(spec.scenario.reference_reachable_semantic_ids)
            if spec.scenario.reference_reachable_semantic_ids is not None
            else self.visited_semantic_ids
        )
        return PlannerResult(
            spec.planner_id,
            True,
            "OK",
            tuple(merged),
            planning_time,
            reachable_semantic_ids=reachable,
            visited_semantic_ids=self.visited_semantic_ids,
            metadata={
                "manual_waypoints": [list(point) for point in self.waypoints],
                "manual_target_semantic_ids": list(self.visited_semantic_ids),
                "p2p_planner_id": getattr(self.p2p_adapter, "planner_id", "unknown"),
            },
        )
