from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import yaml


@dataclass(frozen=True)
class PlatformProfile:
    name: str
    kinematics: str
    wheel_base_m: float
    min_turning_radius_m: float
    navigation_footprint: tuple[tuple[float, float], ...]
    preview_planning_enabled: bool
    execution_ready: bool


def load_platform_profile(path: Path | str) -> PlatformProfile:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not isinstance(data.get("platform"), dict):
        raise ValueError("platform profile must contain platform mapping")
    platform = data["platform"]
    name = str(platform.get("name", ""))
    kinematics = str(platform.get("kinematics", ""))
    if kinematics != "ackermann":
        raise ValueError(f"frozen Paper I benchmark requires ackermann kinematics, got {kinematics!r}")
    geometry = platform.get("geometry")
    if not isinstance(geometry, dict):
        raise ValueError("platform.geometry missing")
    wheel_base = float(geometry.get("wheel_base"))
    min_radius = float(geometry.get("min_turning_radius"))
    if not math.isfinite(wheel_base) or wheel_base <= 0:
        raise ValueError("wheel_base must be positive and finite")
    if not math.isfinite(min_radius) or min_radius <= 0:
        raise ValueError("min_turning_radius must be positive and finite")
    footprint_raw = geometry.get("navigation_footprint")
    if not isinstance(footprint_raw, list) or len(footprint_raw) < 3:
        raise ValueError("navigation_footprint must contain at least three vertices")
    footprint: list[tuple[float, float]] = []
    for vertex in footprint_raw:
        if not isinstance(vertex, (list, tuple)) or len(vertex) != 2:
            raise ValueError("navigation_footprint vertex must be [x, y]")
        x, y = float(vertex[0]), float(vertex[1])
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("navigation_footprint must contain finite values")
        footprint.append((x, y))
    acceptance = platform.get("route_acceptance") or {}
    return PlatformProfile(
        name=name,
        kinematics=kinematics,
        wheel_base_m=wheel_base,
        min_turning_radius_m=min_radius,
        navigation_footprint=tuple(footprint),
        preview_planning_enabled=bool(acceptance.get("preview_planning_enabled", False)),
        execution_ready=bool(acceptance.get("enabled", False)),
    )
