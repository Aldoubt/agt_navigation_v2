from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping

import numpy as np
from shapely.geometry import Polygon, box

from .contracts import ScenarioSpec
from .map_io import Nav2Map, nav2_map_occupancy_data
from .profile import PlatformProfile


@dataclass(frozen=True)
class PreflightResult:
    valid: bool
    error_codes: tuple[str, ...]
    metadata: Mapping[str, object]


def _inside_map(nav_map: Nav2Map, x: float, y: float) -> bool:
    xmin, xmax, ymin, ymax = nav_map.extent
    return xmin <= x < xmax and ymin <= y < ymax


def _cell_index(nav_map: Nav2Map, x: float, y: float) -> tuple[int, int]:
    return (
        math.floor((x - nav_map.origin[0]) / nav_map.resolution_m),
        math.floor((y - nav_map.origin[1]) / nav_map.resolution_m),
    )


def _occupancy_array(nav_map: Nav2Map) -> np.ndarray:
    height, width = nav_map.image.shape[:2]
    return np.asarray(nav2_map_occupancy_data(nav_map), dtype=np.int16).reshape(height, width)


def _footprint_polygon(
    pose: tuple[float, float, float],
    footprint: tuple[tuple[float, float], ...],
) -> Polygon:
    x, y, yaw = pose
    c = math.cos(yaw)
    s = math.sin(yaw)
    vertices = [
        (x + c * fx - s * fy, y + s * fx + c * fy)
        for fx, fy in footprint
    ]
    polygon = Polygon(vertices)
    if not polygon.is_valid or polygon.area <= 0.0:
        raise ValueError("navigation footprint must form a valid non-zero polygon")
    return polygon


def _footprint_collides(
    nav_map: Nav2Map,
    occupancy: np.ndarray,
    pose: tuple[float, float, float],
    footprint: tuple[tuple[float, float], ...],
) -> bool:
    polygon = _footprint_polygon(pose, footprint)
    xmin, xmax, ymin, ymax = nav_map.extent
    pminx, pminy, pmaxx, pmaxy = polygon.bounds
    if pminx < xmin or pmaxx > xmax or pminy < ymin or pmaxy > ymax:
        return True

    height, width = occupancy.shape
    ix0 = max(0, math.floor((pminx - nav_map.origin[0]) / nav_map.resolution_m))
    iy0 = max(0, math.floor((pminy - nav_map.origin[1]) / nav_map.resolution_m))
    ix1 = min(width - 1, math.floor((pmaxx - nav_map.origin[0]) / nav_map.resolution_m))
    iy1 = min(height - 1, math.floor((pmaxy - nav_map.origin[1]) / nav_map.resolution_m))

    for iy in range(iy0, iy1 + 1):
        for ix in range(ix0, ix1 + 1):
            if occupancy[iy, ix] == 0:
                continue
            cell = box(
                nav_map.origin[0] + ix * nav_map.resolution_m,
                nav_map.origin[1] + iy * nav_map.resolution_m,
                nav_map.origin[0] + (ix + 1) * nav_map.resolution_m,
                nav_map.origin[1] + (iy + 1) * nav_map.resolution_m,
            )
            if polygon.intersects(cell):
                return True
    return False


def evaluate_p2p_preflight(
    scenario: ScenarioSpec,
    nav_map: Nav2Map,
    profile: PlatformProfile,
) -> PreflightResult:
    if scenario.level != "p2p" or scenario.start is None or scenario.goal is None:
        raise ValueError("P2P preflight requires a p2p scenario with start and goal")

    occupancy = _occupancy_array(nav_map)
    errors: list[str] = []
    metadata: dict[str, object] = {}

    for label, pose in (("START", scenario.start), ("GOAL", scenario.goal)):
        x, y, _ = pose
        key = label.lower()
        if not _inside_map(nav_map, x, y):
            errors.append(f"{label}_OUT_OF_MAP")
            metadata[f"{key}_cell"] = None
            continue

        ix, iy = _cell_index(nav_map, x, y)
        metadata[f"{key}_cell"] = [ix, iy]
        occupancy_value = int(occupancy[iy, ix])
        metadata[f"{key}_occupancy"] = occupancy_value
        if occupancy_value != 0:
            errors.append(f"{label}_OCCUPIED")
            continue

        if _footprint_collides(nav_map, occupancy, pose, profile.navigation_footprint):
            errors.append(f"{label}_FOOTPRINT_COLLISION")

    return PreflightResult(
        valid=not errors,
        error_codes=tuple(errors),
        metadata=metadata,
    )
