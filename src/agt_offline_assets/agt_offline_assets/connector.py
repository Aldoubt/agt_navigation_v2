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
class ConnectorSample:
    x: float
    y: float
    yaw: float
    direction: str


@dataclass(frozen=True)
class ConnectorResult:
    samples: tuple[ConnectorSample, ...]
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
            return ConnectorResult(
                (ConnectorSample(start[0], start[1], request.start_pose[2], "F"),),
                self.name,
                True,
            )
        count = max(1, int(math.ceil(distance / request.path_resolution_m)))
        yaw = math.atan2(goal[1] - start[1], goal[0] - start[0])
        samples = tuple(
            ConnectorSample(
                start[0] + (goal[0] - start[0]) * index / count,
                start[1] + (goal[1] - start[1]) * index / count,
                yaw,
                "F",
            )
            for index in range(count + 1)
        )
        return ConnectorResult(samples, self.name, True)


def create_connector_backend(name: str) -> ConnectorPlannerBackend:
    if str(name).strip().lower() == "straight":
        return StraightConnectorBackend()
    if str(name).strip().lower() == "reeds_shepp":
        from .reeds_shepp import ReedsSheppConnectorBackend

        return ReedsSheppConnectorBackend()
    raise AssetContractError(
        "connector_backend_unknown", f"unsupported connector backend: {name}"
    )
