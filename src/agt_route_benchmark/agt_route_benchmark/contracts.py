from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Mapping

P2P_PLANNERS = ("astar", "theta_star", "hybrid_astar", "state_lattice")
MISSION_PLANNERS = ("manual_waypoints_best_p2p", "fields2cover", "ours")
FORMAL_PLANNERS = ("astar", "theta_star", "hybrid_astar", "state_lattice", "fields2cover", "ours")
SCENARIO_IDS = (
    "S01_straight_row",
    "S02_90deg_entry",
    "S03_headland_uturn",
    "S04_narrow_headland",
    "S05_blocked_row",
    "S06_full_mission",
)
DIRECTIONS = ("F", "R", "UNKNOWN")
SEGMENT_TYPES = ("P2P", "SWATH", "CONNECTION", "ACCESS", "TURN", "UNKNOWN")

Pose2D = tuple[float, float, float]


def validate_matrix_name(name: str, allowed: tuple[str, ...]) -> str:
    if name not in allowed:
        raise ValueError(f"unsupported matrix name {name!r}; allowed={allowed}")
    return name


@dataclass(frozen=True)
class PathPoint:
    x_m: float
    y_m: float
    yaw_rad: float
    direction: str = "UNKNOWN"
    segment_type: str = "UNKNOWN"
    semantic_ref: str = ""

    def __post_init__(self) -> None:
        if not all(math.isfinite(float(v)) for v in (self.x_m, self.y_m, self.yaw_rad)):
            raise ValueError("path point coordinates/yaw must be finite")
        if self.direction not in DIRECTIONS:
            raise ValueError(f"invalid direction {self.direction!r}")
        if self.segment_type not in SEGMENT_TYPES:
            raise ValueError(f"invalid segment_type {self.segment_type!r}")


@dataclass(frozen=True)
class ScenarioSpec:
    scenario_id: str
    level: str
    development_fixture: bool
    start: Pose2D | None
    goal: Pose2D | None
    required_semantic_ids: tuple[str, ...]
    metadata: Mapping[str, Any] = field(default_factory=dict)
    reference_reachable_semantic_ids: tuple[str, ...] | None = None

    def __post_init__(self) -> None:
        validate_matrix_name(self.scenario_id, SCENARIO_IDS)
        if self.level not in ("p2p", "mission"):
            raise ValueError("scenario level must be 'p2p' or 'mission'")


@dataclass(frozen=True)
class ExperimentSpec:
    site_id: str
    planner_id: str
    scenario: ScenarioSpec
    formal: bool = False
    platform_profile: str = "profiles/platforms/mk_mini.yaml"
    metadata: Mapping[str, Any] = field(default_factory=dict)
    run_id: str = "run_001"


@dataclass(frozen=True)
class PlannerResult:
    planner_id: str
    success: bool
    error_code: str
    path: tuple[PathPoint, ...]
    planning_time_s: float
    reachable_semantic_ids: tuple[str, ...] = ()
    visited_semantic_ids: tuple[str, ...] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)
