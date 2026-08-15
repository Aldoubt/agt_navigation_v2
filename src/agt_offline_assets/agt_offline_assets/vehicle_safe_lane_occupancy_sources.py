"""Audit why Navigation Grid OCCUPIED cells block vehicle-safe aisle lanes.

This layer is diagnostic only.  It does not alter the Navigation Map, Aisle
Graph, vehicle profile, lane acceptance, or R6/R7 admission.

It consumes the frozen Navigation Map plus derivation sidecars written by
``write_navigation_map_derivation`` and attributes occupied pose-cell hits to
mutually-exclusive source classes:

* RAW_OBSTACLE_DIRECT
* GEOMETRY_DIRECT
* PADDING_ONLY
* UNEXPLAINED_OCCUPIED

``GEOMETRY_DIRECT`` uses the frozen slope/step threshold arrays.  Existing
pre-2026-08-15 derivations do not materialize point_count/ground_valid sidecars,
so this attribution is a conservative threshold-source audit rather than a bit-
exact reconstruction of ``geometry_bad``.  The output records that limitation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from .agricultural_aisle_graph import AislePrimitive, AgriculturalAisleGraph
from .forward_connector import ForwardConnectorSample
from .forward_connector_navigation_gate import (
    _cell_index,
    _preview_local_footprint,
    _transform_polygon,
)
from .navigation_grid import NavigationGridEvidence
from .navigation_map_derivation import FREE, OCCUPIED, UNKNOWN
from .turn_zones import _points_inside_polygon
from .vehicle_profile import CanonicalVehicleProfile
from .vehicle_safe_lane import _normalize, _offset_candidates, _resample_polyline


VEHICLE_SAFE_LANE_OCCUPANCY_SOURCE_SCHEMA = (
    "agt_vehicle_safe_lane_occupancy_source_audit/v1"
)


@dataclass(frozen=True)
class VehicleSafeLaneOccupancySourceConfig:
    sample_spacing_m: float = 0.10
    lateral_search_step_m: float = 0.05
    maximum_lateral_shift_m: float = 0.50
    preview_footprint_padding_m: float = 0.05

    def validate(self) -> None:
        for name in (
            "sample_spacing_m",
            "lateral_search_step_m",
            "maximum_lateral_shift_m",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")
        if (
            not math.isfinite(self.preview_footprint_padding_m)
            or self.preview_footprint_padding_m < 0.0
        ):
            raise ValueError("preview_footprint_padding_m must be finite and >= 0")


@dataclass(frozen=True)
class NavigationOccupancySourceMasks:
    raw_obstacle_direct: np.ndarray
    geometry_direct: np.ndarray
    padding_only: np.ndarray
    unexplained_occupied: np.ndarray
    slope_threshold_exceeded: np.ndarray
    step_threshold_exceeded: np.ndarray
    source_exactness: str
    derivation_config: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class VehicleSafeLaneOccupancySourceAisle:
    aisle_id: str
    classification: str
    total_station_count: int
    free_candidate_station_count: int
    blocked_candidate_station_count: int
    occupied_pose_cell_hits: int
    raw_obstacle_direct_hits: int
    geometry_direct_hits: int
    padding_only_hits: int
    unexplained_occupied_hits: int
    slope_threshold_hits: int
    step_threshold_hits: int
    mean_selected_free_fraction: float
    mean_selected_occupied_fraction: float
    mean_selected_unknown_fraction: float
    reason: str

    @property
    def raw_obstacle_fraction(self) -> float:
        return _fraction(self.raw_obstacle_direct_hits, self.occupied_pose_cell_hits)

    @property
    def geometry_fraction(self) -> float:
        return _fraction(self.geometry_direct_hits, self.occupied_pose_cell_hits)

    @property
    def padding_fraction(self) -> float:
        return _fraction(self.padding_only_hits, self.occupied_pose_cell_hits)

    @property
    def unexplained_fraction(self) -> float:
        return _fraction(self.unexplained_occupied_hits, self.occupied_pose_cell_hits)


@dataclass(frozen=True)
class VehicleSafeLaneOccupancySourcePlan:
    frame_id: str
    platform_id: str
    platform_profile_sha256: str
    source_exactness: str
    aisles: tuple[VehicleSafeLaneOccupancySourceAisle, ...]
    global_actual_occupied_cells: int
    global_raw_obstacle_direct_cells: int
    global_geometry_direct_cells: int
    global_padding_only_cells: int
    global_unexplained_occupied_cells: int
    source: Mapping[str, Any] = field(default_factory=dict)
    schema: str = VEHICLE_SAFE_LANE_OCCUPANCY_SOURCE_SCHEMA
    status: str = "DIAGNOSTIC_ONLY"


def _fraction(value: int, total: int) -> float:
    return 0.0 if total <= 0 else float(value / total)


def _require_scipy():
    try:
        from scipy import ndimage
    except ImportError as exc:
        raise RuntimeError("occupancy source audit requires scipy") from exc
    return ndimage


def _resolve_navigation_dir(path: str | Path) -> Path:
    input_path = Path(path).expanduser().resolve()
    return input_path if input_path.is_dir() else input_path.parent


def load_navigation_occupancy_source_masks(
    navigation: NavigationGridEvidence,
    navigation_asset: str | Path,
) -> NavigationOccupancySourceMasks:
    """Load frozen derivation sidecars and classify actual OCCUPIED cells."""
    asset_dir = _resolve_navigation_dir(navigation_asset)
    derivation_path = asset_dir / "derivation.yaml"
    required = {
        "obstacle_count": asset_dir / "obstacle_count.npy",
        "slope_deg": asset_dir / "slope_deg.npy",
        "step_m": asset_dir / "step_m.npy",
    }
    missing = [str(path.name) for path in required.values() if not path.is_file()]
    if not derivation_path.is_file():
        missing.insert(0, "derivation.yaml")
    if missing:
        raise FileNotFoundError(
            "occupancy source audit requires frozen Navigation Map derivation "
            f"sidecars in {asset_dir}: missing {', '.join(missing)}"
        )

    payload = yaml.safe_load(derivation_path.read_text(encoding="utf-8")) or {}
    config = payload.get("config") or {}
    if not isinstance(config, Mapping):
        raise ValueError("derivation.yaml config must be a mapping")

    obstacle_count = np.load(required["obstacle_count"])
    slope_deg = np.load(required["slope_deg"])
    step_m = np.load(required["step_m"])
    shape = navigation.occupancy.shape
    for name, array in (
        ("obstacle_count", obstacle_count),
        ("slope_deg", slope_deg),
        ("step_m", step_m),
    ):
        if array.shape != shape:
            raise ValueError(
                f"{name}.npy shape {array.shape} does not match Navigation Grid {shape}"
            )

    minimum_obstacle_points = int(config.get("minimum_obstacle_points", 2))
    maximum_slope_deg = float(config.get("maximum_slope_deg", 18.0))
    maximum_step_m = float(config.get("maximum_step_m", 0.12))
    obstacle_padding_m = float(config.get("obstacle_padding_m", 0.0))

    raw_obstacle = obstacle_count >= minimum_obstacle_points
    slope_bad = np.isfinite(slope_deg) & (slope_deg > maximum_slope_deg)
    step_bad = np.isfinite(step_m) & (step_m > maximum_step_m)

    point_count_path = asset_dir / "point_count.npy"
    ground_valid_path = asset_dir / "ground_valid.npy"
    exact = point_count_path.is_file() and ground_valid_path.is_file()
    if exact:
        point_count = np.load(point_count_path)
        ground_valid = np.load(ground_valid_path).astype(bool)
        if point_count.shape != shape or ground_valid.shape != shape:
            raise ValueError("point_count/ground_valid sidecar shape mismatch")
        geometry_source = (
            ground_valid
            & (point_count > 0)
            & (slope_bad | step_bad)
        )
        source_exactness = "EXACT_GEOMETRY_BAD_INPUTS_AVAILABLE"
    else:
        # Older frozen derivations do not contain the two arrays needed to
        # reproduce geometry_bad bit-for-bit.  Intersecting threshold exceedance
        # with actual OCCUPIED later keeps the audit conservative.
        geometry_source = slope_bad | step_bad
        source_exactness = "APPROX_GEOMETRY_THRESHOLD_SOURCE_NO_POINT_COUNT_GROUND_VALID"

    direct_source = raw_obstacle | geometry_source
    padding_cells = int(math.ceil(obstacle_padding_m / navigation.resolution_m))
    if padding_cells > 0:
        ndimage = _require_scipy()
        padded_source = ndimage.maximum_filter(
            direct_source.astype(np.uint8),
            size=2 * padding_cells + 1,
            mode="constant",
        ).astype(bool)
    else:
        padded_source = direct_source.copy()

    actual_occupied = navigation.occupancy == OCCUPIED
    raw_direct = actual_occupied & raw_obstacle
    geometry_direct = actual_occupied & ~raw_obstacle & geometry_source
    padding_only = actual_occupied & ~direct_source & padded_source
    unexplained = actual_occupied & ~direct_source & ~padded_source

    return NavigationOccupancySourceMasks(
        raw_obstacle_direct=raw_direct,
        geometry_direct=geometry_direct,
        padding_only=padding_only,
        unexplained_occupied=unexplained,
        slope_threshold_exceeded=slope_bad,
        step_threshold_exceeded=step_bad,
        source_exactness=source_exactness,
        derivation_config=dict(config),
    )


def _pose_mask(
    x: float,
    y: float,
    yaw: float,
    navigation: NavigationGridEvidence,
    local_footprint,
) -> tuple[np.ndarray, float]:
    occupancy = np.asarray(navigation.occupancy, dtype=np.uint8)
    mask = np.zeros(occupancy.shape, dtype=bool)
    sample = ForwardConnectorSample(x=float(x), y=float(y), z=0.0, yaw=float(yaw))
    polygon = _transform_polygon(local_footprint, sample)
    poly = np.asarray(polygon, dtype=np.float64)
    px0, py0 = np.min(poly, axis=0)
    px1, py1 = np.max(poly, axis=0)
    min_x, min_y, max_x, max_y = navigation.bounds_m()
    coverage = 1.0
    if px0 < min_x or py0 < min_y or px1 > max_x or py1 > max_y:
        coverage = 0.0

    resolution = float(navigation.resolution_m)
    col0 = max(0, int(math.floor((float(px0) - navigation.origin_x_m) / resolution)))
    col1 = min(
        navigation.width - 1,
        int(math.floor((float(px1) - navigation.origin_x_m) / resolution)),
    )
    row0 = max(0, int(math.floor((float(py0) - navigation.origin_y_m) / resolution)))
    row1 = min(
        navigation.height - 1,
        int(math.floor((float(py1) - navigation.origin_y_m) / resolution)),
    )
    if row0 > row1 or col0 > col1:
        return mask, 0.0

    rows, cols = np.indices((row1 - row0 + 1, col1 - col0 + 1), dtype=np.float64)
    xx = navigation.origin_x_m + (cols + col0 + 0.5) * resolution
    yy = navigation.origin_y_m + (rows + row0 + 0.5) * resolution
    inside = _points_inside_polygon(xx, yy, polygon)
    mask[row0 : row1 + 1, col0 : col1 + 1] |= inside
    return mask, coverage


def _candidate_key(
    mask: np.ndarray,
    coverage: float,
    navigation: NavigationGridEvidence,
    offset: float,
) -> tuple[float, ...]:
    cells = navigation.occupancy[mask]
    if cells.size <= 0:
        return (1.0, 1.0, 1.0, 1.0, abs(float(offset)))
    free = float(np.count_nonzero(cells == FREE) / cells.size)
    occupied = float(np.count_nonzero(cells == OCCUPIED) / cells.size)
    unknown = float(np.count_nonzero(cells == UNKNOWN) / cells.size)
    out = max(0.0, 1.0 - float(coverage))
    non_free = min(1.0, occupied + unknown + out)
    return (non_free, occupied, unknown, out, abs(float(offset)), -free)


def _fully_free(mask: np.ndarray, coverage: float, navigation: NavigationGridEvidence) -> bool:
    cells = navigation.occupancy[mask]
    return bool(
        coverage >= 1.0 - 1.0e-9
        and cells.size > 0
        and np.all(cells == FREE)
    )


def _classify_source(
    raw: int,
    geometry: int,
    padding: int,
    unexplained: int,
    occupied: int,
) -> tuple[str, str]:
    if occupied <= 0:
        return "NO_OCCUPIED_POSE_CELL_HITS", "selected candidates contain no OCCUPIED pose-cell hits"
    values = {
        "RAW_OBSTACLE_DIRECT_DOMINANT": raw,
        "GEOMETRY_DIRECT_DOMINANT": geometry,
        "PADDING_ONLY_DOMINANT": padding,
        "UNEXPLAINED_OCCUPIED_DOMINANT": unexplained,
    }
    classification = max(values, key=lambda key: values[key])
    return classification, (
        "dominant occupied pose-cell source among best bounded lateral candidates: "
        f"raw={raw}, geometry={geometry}, padding={padding}, unexplained={unexplained}"
    )


def _audit_one_aisle(
    aisle: AislePrimitive,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    direction: np.ndarray,
    perpendicular: np.ndarray,
    local_footprint,
    masks: NavigationOccupancySourceMasks,
    cfg: VehicleSafeLaneOccupancySourceConfig,
) -> VehicleSafeLaneOccupancySourceAisle:
    samples, _ = _resample_polyline(aisle, cfg.sample_spacing_m)
    required_width = float(vehicle.navigation_width_m + 2.0 * cfg.preview_footprint_padding_m)
    if float(aisle.geometric_width_m) + 1.0e-9 < required_width:
        return VehicleSafeLaneOccupancySourceAisle(
            aisle_id=aisle.aisle_id,
            classification="STRUCTURAL_WIDTH_BLOCKED",
            total_station_count=int(samples.shape[0]),
            free_candidate_station_count=0,
            blocked_candidate_station_count=int(samples.shape[0]),
            occupied_pose_cell_hits=0,
            raw_obstacle_direct_hits=0,
            geometry_direct_hits=0,
            padding_only_hits=0,
            unexplained_occupied_hits=0,
            slope_threshold_hits=0,
            step_threshold_hits=0,
            mean_selected_free_fraction=0.0,
            mean_selected_occupied_fraction=0.0,
            mean_selected_unknown_fraction=0.0,
            reason="structural width gate blocks vehicle-safe lane before occupancy-source attribution",
        )

    width_surplus = max(0.0, float(aisle.geometric_width_m) - required_width)
    allowed_shift = min(float(cfg.maximum_lateral_shift_m), 0.5 * width_surplus)
    offsets = _offset_candidates(allowed_shift, cfg.lateral_search_step_m)
    yaw = math.atan2(float(direction[1]), float(direction[0]))

    free_station = 0
    blocked_station = 0
    occupied_hits = 0
    raw_hits = 0
    geometry_hits = 0
    padding_hits = 0
    unexplained_hits = 0
    slope_hits = 0
    step_hits = 0
    free_fractions: list[float] = []
    occupied_fractions: list[float] = []
    unknown_fractions: list[float] = []

    for sample in samples:
        candidates: list[tuple[float, np.ndarray, float]] = []
        station_has_free = False
        for offset in offsets:
            x = float(sample[0] + float(offset) * perpendicular[0])
            y = float(sample[1] + float(offset) * perpendicular[1])
            mask, coverage = _pose_mask(x, y, yaw, navigation, local_footprint)
            candidates.append((float(offset), mask, coverage))
            if _fully_free(mask, coverage, navigation):
                station_has_free = True

        if station_has_free:
            free_station += 1
        else:
            blocked_station += 1

        offset, selected, coverage = min(
            candidates,
            key=lambda item: _candidate_key(item[1], item[2], navigation, item[0]),
        )
        del offset
        cells = navigation.occupancy[selected]
        if cells.size <= 0:
            continue
        free_fractions.append(float(np.count_nonzero(cells == FREE) / cells.size))
        occupied_fractions.append(float(np.count_nonzero(cells == OCCUPIED) / cells.size))
        unknown_fractions.append(float(np.count_nonzero(cells == UNKNOWN) / cells.size))

        occupied_mask = selected & (navigation.occupancy == OCCUPIED)
        count = int(np.count_nonzero(occupied_mask))
        occupied_hits += count
        raw_hits += int(np.count_nonzero(occupied_mask & masks.raw_obstacle_direct))
        geometry_hits += int(np.count_nonzero(occupied_mask & masks.geometry_direct))
        padding_hits += int(np.count_nonzero(occupied_mask & masks.padding_only))
        unexplained_hits += int(np.count_nonzero(occupied_mask & masks.unexplained_occupied))
        slope_hits += int(np.count_nonzero(occupied_mask & masks.slope_threshold_exceeded))
        step_hits += int(np.count_nonzero(occupied_mask & masks.step_threshold_exceeded))

    classification, reason = _classify_source(
        raw_hits,
        geometry_hits,
        padding_hits,
        unexplained_hits,
        occupied_hits,
    )
    return VehicleSafeLaneOccupancySourceAisle(
        aisle_id=aisle.aisle_id,
        classification=classification,
        total_station_count=int(samples.shape[0]),
        free_candidate_station_count=free_station,
        blocked_candidate_station_count=blocked_station,
        occupied_pose_cell_hits=occupied_hits,
        raw_obstacle_direct_hits=raw_hits,
        geometry_direct_hits=geometry_hits,
        padding_only_hits=padding_hits,
        unexplained_occupied_hits=unexplained_hits,
        slope_threshold_hits=slope_hits,
        step_threshold_hits=step_hits,
        mean_selected_free_fraction=float(np.mean(free_fractions)) if free_fractions else 0.0,
        mean_selected_occupied_fraction=float(np.mean(occupied_fractions)) if occupied_fractions else 0.0,
        mean_selected_unknown_fraction=float(np.mean(unknown_fractions)) if unknown_fractions else 0.0,
        reason=reason,
    )


def derive_vehicle_safe_lane_occupancy_source_audit(
    graph: AgriculturalAisleGraph,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    navigation_asset: str | Path,
    config: VehicleSafeLaneOccupancySourceConfig | None = None,
    *,
    source: Mapping[str, Any] | None = None,
) -> VehicleSafeLaneOccupancySourcePlan:
    cfg = config or VehicleSafeLaneOccupancySourceConfig()
    cfg.validate()
    if graph.frame_id != navigation.frame_id:
        raise ValueError("Aisle Graph and Navigation Grid frame_id must match")
    if not vehicle.planning_preview_ready:
        raise ValueError(f"vehicle profile {vehicle.profile_id} is not ready for planning preview")

    masks = load_navigation_occupancy_source_masks(navigation, navigation_asset)
    direction = _normalize(graph.row_direction_xy)
    perpendicular = np.array([-direction[1], direction[0]], dtype=np.float64)
    local_footprint = _preview_local_footprint(vehicle, cfg.preview_footprint_padding_m)
    aisles = tuple(
        _audit_one_aisle(
            aisle,
            navigation,
            vehicle,
            direction,
            perpendicular,
            local_footprint,
            masks,
            cfg,
        )
        for aisle in graph.aisles
    )

    actual_occupied = navigation.occupancy == OCCUPIED
    merged_source = dict(graph.source)
    merged_source.update(dict(navigation.source))
    merged_source.update(dict(source or {}))
    merged_source.update(
        {
            "audit_policy": "BEST_BOUNDED_LATERAL_POSE_OCCUPIED_SOURCE_ATTRIBUTION",
            "validation_scope": "DIAGNOSTIC_ONLY",
            "source_exactness": masks.source_exactness,
            "navigation_map_mutated": False,
            "aisle_graph_mutated": False,
        }
    )
    return VehicleSafeLaneOccupancySourcePlan(
        frame_id=graph.frame_id,
        platform_id=vehicle.profile_id,
        platform_profile_sha256=vehicle.profile_sha256,
        source_exactness=masks.source_exactness,
        aisles=aisles,
        global_actual_occupied_cells=int(np.count_nonzero(actual_occupied)),
        global_raw_obstacle_direct_cells=int(np.count_nonzero(masks.raw_obstacle_direct)),
        global_geometry_direct_cells=int(np.count_nonzero(masks.geometry_direct)),
        global_padding_only_cells=int(np.count_nonzero(masks.padding_only)),
        global_unexplained_occupied_cells=int(np.count_nonzero(masks.unexplained_occupied)),
        source=merged_source,
    )


def occupancy_source_plan_to_dict(plan: VehicleSafeLaneOccupancySourcePlan) -> dict[str, Any]:
    return {
        "schema": plan.schema,
        "status": plan.status,
        "frame_id": plan.frame_id,
        "platform_id": plan.platform_id,
        "platform_profile_sha256": plan.platform_profile_sha256,
        "source_exactness": plan.source_exactness,
        "source": dict(plan.source),
        "global": {
            "actual_occupied_cells": plan.global_actual_occupied_cells,
            "raw_obstacle_direct_cells": plan.global_raw_obstacle_direct_cells,
            "geometry_direct_cells": plan.global_geometry_direct_cells,
            "padding_only_cells": plan.global_padding_only_cells,
            "unexplained_occupied_cells": plan.global_unexplained_occupied_cells,
        },
        "aisles": [
            {
                "aisle_id": item.aisle_id,
                "classification": item.classification,
                "total_station_count": item.total_station_count,
                "free_candidate_station_count": item.free_candidate_station_count,
                "blocked_candidate_station_count": item.blocked_candidate_station_count,
                "occupied_pose_cell_hits": item.occupied_pose_cell_hits,
                "sources": {
                    "raw_obstacle_direct_hits": item.raw_obstacle_direct_hits,
                    "geometry_direct_hits": item.geometry_direct_hits,
                    "padding_only_hits": item.padding_only_hits,
                    "unexplained_occupied_hits": item.unexplained_occupied_hits,
                    "raw_obstacle_fraction": item.raw_obstacle_fraction,
                    "geometry_fraction": item.geometry_fraction,
                    "padding_fraction": item.padding_fraction,
                    "unexplained_fraction": item.unexplained_fraction,
                    "slope_threshold_hits": item.slope_threshold_hits,
                    "step_threshold_hits": item.step_threshold_hits,
                },
                "selected_pose_mean": {
                    "free_fraction": item.mean_selected_free_fraction,
                    "occupied_fraction": item.mean_selected_occupied_fraction,
                    "unknown_fraction": item.mean_selected_unknown_fraction,
                },
                "reason": item.reason,
            }
            for item in plan.aisles
        ],
    }


def write_vehicle_safe_lane_occupancy_source_audit(
    plan: VehicleSafeLaneOccupancySourcePlan,
    path: str | Path,
) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(occupancy_source_plan_to_dict(plan), sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return output
