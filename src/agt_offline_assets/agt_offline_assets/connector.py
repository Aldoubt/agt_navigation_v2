"""Offline connector planner backend boundary for Route generation."""

from dataclasses import dataclass
import math
from typing import Any, Mapping

from .contracts import AssetContractError


@dataclass(frozen=True)
class ConnectorRequest:
    start_pose: tuple[float, float, float]
    goal_pose: tuple[float, float, float]
    path_resolution_m: float
    min_turning_radius_m: float
    allow_reverse: bool


@dataclass(frozen=True)
class ConnectorResult:
    samples: tuple[tuple[float, float], ...]
    backend: str
    success: bool
    failure_reason: str = ""


class ConnectorPlannerBackend:
    name = ""

    def plan(self, request: ConnectorRequest, context: Mapping[str, Any] | None = None) -> ConnectorResult:
        raise NotImplementedError


class StraightConnectorBackend(ConnectorPlannerBackend):
    name = "straight"

    def plan(self, request: ConnectorRequest, context: Mapping[str, Any] | None = None) -> ConnectorResult:
        if request.path_resolution_m <= 0.0 or not math.isfinite(request.path_resolution_m):
            return ConnectorResult((), self.name, False, "path resolution must be positive")
        start = request.start_pose[:2]
        goal = request.goal_pose[:2]
        distance = math.dist(start, goal)
        if distance <= 1.0e-9:
            return ConnectorResult((tuple(start),), self.name, True)
        count = max(1, int(math.ceil(distance / request.path_resolution_m)))
        samples = tuple(
            (
                start[0] + (goal[0] - start[0]) * index / count,
                start[1] + (goal[1] - start[1]) * index / count,
            )
            for index in range(count + 1)
        )
        return ConnectorResult(samples, self.name, True)


def create_connector_backend(name: str) -> ConnectorPlannerBackend:
    if str(name).strip().lower() == "straight":
        return StraightConnectorBackend()
    raise AssetContractError(
        "connector_backend_unknown", f"unsupported connector backend: {name}"
    )
