from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import yaml

from .contracts import P2P_PLANNERS


@dataclass(frozen=True)
class ManualWaypointPlan:
    frame_id: str
    p2p_planner: str
    waypoints: tuple[tuple[float, float, float], ...]
    target_semantic_ids: tuple[str, ...]


def load_manual_waypoint_plan(path: Path | str) -> ManualWaypointPlan:
    source = Path(path).expanduser().resolve()
    data = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("manual waypoint plan must be a YAML mapping")
    if str(data.get("schema_version", "")) != "1.0":
        raise ValueError("manual waypoint plan schema_version must be '1.0'")
    frame_id = str(data.get("frame_id", ""))
    if frame_id != "map":
        raise ValueError("manual waypoint plan frame_id must be map")
    p2p_planner = str(data.get("p2p_planner", ""))
    if p2p_planner not in P2P_PLANNERS:
        raise ValueError(f"manual waypoint p2p_planner must be one of {P2P_PLANNERS}")
    raw_waypoints = data.get("waypoints")
    if not isinstance(raw_waypoints, list) or len(raw_waypoints) < 2:
        raise ValueError("manual waypoint plan requires at least two waypoints")

    waypoints: list[tuple[float, float, float]] = []
    semantic_ids: list[str] = []
    for index, raw in enumerate(raw_waypoints):
        if not isinstance(raw, dict):
            raise ValueError(f"waypoints[{index}] must be a mapping")
        missing = [key for key in ("x", "y", "yaw") if key not in raw]
        if missing:
            raise ValueError(f"waypoints[{index}] missing {missing}")
        pose = (float(raw["x"]), float(raw["y"]), float(raw["yaw"]))
        if not all(math.isfinite(value) for value in pose):
            raise ValueError(f"waypoints[{index}] must contain finite x/y/yaw")
        waypoints.append(pose)
        semantic_ref = str(raw.get("semantic_ref", "")).strip()
        if semantic_ref:
            semantic_ids.append(semantic_ref)

    return ManualWaypointPlan(
        frame_id=frame_id,
        p2p_planner=p2p_planner,
        waypoints=tuple(waypoints),
        target_semantic_ids=tuple(dict.fromkeys(semantic_ids)),
    )
