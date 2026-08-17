from __future__ import annotations

from pathlib import Path
from typing import Any
import math
import yaml

from .contracts import ScenarioSpec


def _pose(value: Any, key: str) -> tuple[float, float, float]:
    if not isinstance(value, dict):
        raise ValueError(f"{key} must be a mapping with x/y/yaw")
    missing = [name for name in ("x", "y", "yaw") if name not in value]
    if missing:
        raise ValueError(f"{key} missing {missing}")
    pose = (float(value["x"]), float(value["y"]), float(value["yaw"]))
    if not all(math.isfinite(v) for v in pose):
        raise ValueError(f"{key} must contain finite values")
    return pose


def load_scenario(path: Path | str, formal: bool = False) -> ScenarioSpec:
    path = Path(path)
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("scenario YAML must contain a mapping")
    for key in ("id", "level"):
        if key not in data:
            raise ValueError(f"scenario missing {key}")
    development_fixture = bool(data.get("development_fixture", False))
    if formal and development_fixture:
        raise ValueError("formal mode rejects development_fixture scenarios")
    level = str(data["level"])
    start = goal = None
    required: tuple[str, ...] = ()
    if level == "p2p":
        if "start" not in data:
            raise ValueError("p2p scenario missing start")
        if "goal" not in data:
            raise ValueError("p2p scenario missing goal")
        start = _pose(data["start"], "start")
        goal = _pose(data["goal"], "goal")
    elif level == "mission":
        mission = data.get("mission")
        if not isinstance(mission, dict) or not mission.get("required_semantic_ids"):
            raise ValueError("mission scenario missing required_semantic_ids")
        required = tuple(str(v) for v in mission["required_semantic_ids"])
    else:
        raise ValueError("scenario level must be p2p or mission")
    metadata = {k: v for k, v in data.items() if k not in {"id", "level", "development_fixture", "start", "goal", "mission"}}
    return ScenarioSpec(str(data["id"]), level, development_fixture, start, goal, required, metadata)
