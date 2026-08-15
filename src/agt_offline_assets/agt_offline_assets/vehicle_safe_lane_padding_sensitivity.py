"""Counterfactual padding sensitivity for vehicle-safe agricultural aisle lanes.

The frozen Navigation Map currently contains an obstacle-padding decision while
later route gates also evaluate the full canonical vehicle footprint.  This
module quantifies that interaction without overwriting the frozen map.

It reconstructs counterfactual trinary Navigation Grids from frozen derivation
sidecars and the already-audited direct occupied-source mask, then reruns the
vehicle-safe-lane evidence diagnostic for a small matrix of:

* map obstacle-padding radii expressed in integer grid cells
* preview-footprint padding values expressed in meters

This is DIAGNOSTIC_ONLY.  It does not change the production Navigation Map,
Aisle Graph, vehicle profile, route admission, or R6/R7 state.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Any, Iterable, Mapping

import numpy as np
import yaml

from .agricultural_aisle_graph import AgriculturalAisleGraph
from .navigation_grid import NavigationGridEvidence
from .navigation_map_derivation import FREE, OCCUPIED, UNKNOWN
from .vehicle_profile import CanonicalVehicleProfile
from .vehicle_safe_lane_diagnostics import (
    VehicleSafeLaneAisleDiagnostic,
    VehicleSafeLaneDiagnosticConfig,
    derive_vehicle_safe_lane_diagnostics,
)
from .vehicle_safe_lane_occupancy_sources import (
    load_navigation_occupancy_source_masks,
)


VEHICLE_SAFE_LANE_PADDING_SENSITIVITY_SCHEMA = (
    "agt_vehicle_safe_lane_padding_sensitivity/v1"
)


@dataclass(frozen=True)
class PaddingSensitivityCase:
    case_id: str
    map_padding_cells: int
    map_padding_axis_m: float
    map_padding_diagonal_m: float
    footprint_padding_m: float
    free_cell_count: int
    occupied_cell_count: int
    unknown_cell_count: int
    width_compatible_aisle_count: int
    zero_feasible_aisle_count: int
    mostly_configuration_space_free_count: int
    mean_any_lateral_free_fraction: float
    aisles: tuple[VehicleSafeLaneAisleDiagnostic, ...]


@dataclass(frozen=True)
class VehicleSafeLanePaddingSensitivityPlan:
    frame_id: str
    platform_id: str
    platform_profile_sha256: str
    source_exactness: str
    current_requested_obstacle_padding_m: float
    current_effective_padding_cells: int
    cases: tuple[PaddingSensitivityCase, ...]
    source: Mapping[str, Any] = field(default_factory=dict)
    schema: str = VEHICLE_SAFE_LANE_PADDING_SENSITIVITY_SCHEMA
    status: str = "DIAGNOSTIC_ONLY"


def _require_scipy():
    try:
        from scipy import ndimage
    except ImportError as exc:
        raise RuntimeError("padding sensitivity audit requires scipy") from exc
    return ndimage


def _resolve_navigation_dir(path: str | Path) -> Path:
    input_path = Path(path).expanduser().resolve()
    return input_path if input_path.is_dir() else input_path.parent


def _load_counterfactual_ground_evidence(
    navigation: NavigationGridEvidence,
    navigation_asset: str | Path,
) -> tuple[np.ndarray, np.ndarray, int, int]:
    asset_dir = _resolve_navigation_dir(navigation_asset)
    ground_height_path = asset_dir / "ground_height.npy"
    ground_support_path = asset_dir / "ground_support_count.npy"
    derivation_path = asset_dir / "derivation.yaml"
    missing = [
        path.name
        for path in (ground_height_path, ground_support_path, derivation_path)
        if not path.is_file()
    ]
    if missing:
        raise FileNotFoundError(
            "padding sensitivity audit requires frozen ground evidence sidecars: "
            + ", ".join(missing)
        )

    derivation = yaml.safe_load(derivation_path.read_text(encoding="utf-8")) or {}
    overrides = derivation.get("overrides") or []
    if overrides:
        raise ValueError(
            "padding sensitivity audit refuses counterfactual reconstruction when "
            "Navigation Map overrides are present"
        )
    config = derivation.get("config") or {}
    if not isinstance(config, Mapping):
        raise ValueError("derivation.yaml config must be a mapping")

    ground_height = np.load(ground_height_path)
    ground_support = np.load(ground_support_path)
    shape = navigation.occupancy.shape
    if ground_height.shape != shape or ground_support.shape != shape:
        raise ValueError("ground sidecar shape does not match Navigation Grid")

    minimum_support = int(config.get("minimum_ground_support_points", 2))
    current_padding_m = float(config.get("obstacle_padding_m", 0.0))
    current_padding_cells = int(
        math.ceil(current_padding_m / float(navigation.resolution_m))
    )
    return (
        np.isfinite(ground_height),
        np.asarray(ground_support),
        minimum_support,
        current_padding_cells,
    )


def _counterfactual_navigation(
    navigation: NavigationGridEvidence,
    direct_occupied: np.ndarray,
    ground_valid: np.ndarray,
    ground_support: np.ndarray,
    minimum_ground_support_points: int,
    map_padding_cells: int,
    *,
    source: Mapping[str, Any] | None = None,
) -> NavigationGridEvidence:
    """Build one immutable counterfactual grid from frozen evidence."""
    if map_padding_cells < 0:
        raise ValueError("map_padding_cells must be >= 0")
    shape = navigation.occupancy.shape
    for name, array in (
        ("direct_occupied", direct_occupied),
        ("ground_valid", ground_valid),
        ("ground_support", ground_support),
    ):
        if np.asarray(array).shape != shape:
            raise ValueError(f"{name} shape does not match Navigation Grid")

    direct = np.asarray(direct_occupied, dtype=bool)
    if map_padding_cells > 0:
        ndimage = _require_scipy()
        padded = ndimage.maximum_filter(
            direct.astype(np.uint8),
            size=2 * int(map_padding_cells) + 1,
            mode="constant",
        ).astype(bool)
    else:
        padded = direct.copy()

    occupancy = np.full(shape, UNKNOWN, dtype=np.uint8)
    free_evidence = (
        np.asarray(ground_valid, dtype=bool)
        & (np.asarray(ground_support) >= int(minimum_ground_support_points))
        & ~padded
    )
    occupancy[free_evidence] = FREE
    occupancy[padded] = OCCUPIED

    merged_source = dict(navigation.source)
    merged_source.update(dict(source or {}))
    merged_source.update(
        {
            "counterfactual_map_padding_cells": int(map_padding_cells),
            "counterfactual_map_padding_axis_m": float(
                map_padding_cells * navigation.resolution_m
            ),
            "counterfactual_map_padding_semantics": "SQUARE_MAXIMUM_FILTER_MATCHING_CURRENT_DERIVATION",
            "production_navigation_map_mutated": False,
        }
    )
    return NavigationGridEvidence(
        resolution_m=navigation.resolution_m,
        origin_x_m=navigation.origin_x_m,
        origin_y_m=navigation.origin_y_m,
        width=navigation.width,
        height=navigation.height,
        occupancy=occupancy,
        frame_id=navigation.frame_id,
        source=merged_source,
    )


def _case_id(map_padding_cells: int, footprint_padding_m: float) -> str:
    millimeters = int(round(float(footprint_padding_m) * 1000.0))
    return f"map_pad_{int(map_padding_cells)}cell__footprint_pad_{millimeters:03d}mm"


def derive_vehicle_safe_lane_padding_sensitivity(
    graph: AgriculturalAisleGraph,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    navigation_asset: str | Path,
    *,
    map_padding_cells: Iterable[int] = (0, 1, 2),
    footprint_padding_m: Iterable[float] = (0.0, 0.05),
    sample_spacing_m: float = 0.10,
    lateral_search_step_m: float = 0.05,
    maximum_lateral_shift_m: float = 0.50,
    source: Mapping[str, Any] | None = None,
) -> VehicleSafeLanePaddingSensitivityPlan:
    """Evaluate route-pose feasibility under frozen padding counterfactuals."""
    if graph.frame_id != navigation.frame_id:
        raise ValueError("Aisle Graph and Navigation Grid frame_id must match")
    if not vehicle.planning_preview_ready:
        raise ValueError(f"vehicle profile {vehicle.profile_id} is not preview-ready")

    padding_values = tuple(dict.fromkeys(int(v) for v in map_padding_cells))
    footprint_values = tuple(dict.fromkeys(float(v) for v in footprint_padding_m))
    if not padding_values or any(v < 0 for v in padding_values):
        raise ValueError("map_padding_cells must contain non-negative values")
    if not footprint_values or any((not math.isfinite(v) or v < 0.0) for v in footprint_values):
        raise ValueError("footprint_padding_m must contain finite non-negative values")

    masks = load_navigation_occupancy_source_masks(navigation, navigation_asset)
    direct_occupied = masks.raw_obstacle_direct | masks.geometry_direct
    (
        ground_valid,
        ground_support,
        minimum_support,
        current_padding_cells,
    ) = _load_counterfactual_ground_evidence(navigation, navigation_asset)
    requested_padding_m = float(masks.derivation_config.get("obstacle_padding_m", 0.0))

    cases: list[PaddingSensitivityCase] = []
    for map_cells in padding_values:
        counterfactual = _counterfactual_navigation(
            navigation,
            direct_occupied,
            ground_valid,
            ground_support,
            minimum_support,
            map_cells,
            source={
                "validation_scope": "COUNTERFACTUAL_DIAGNOSTIC_ONLY",
                "source_exactness": masks.source_exactness,
            },
        )
        counts = counterfactual.counts()
        for footprint_padding in footprint_values:
            diagnostic = derive_vehicle_safe_lane_diagnostics(
                graph,
                counterfactual,
                vehicle,
                VehicleSafeLaneDiagnosticConfig(
                    sample_spacing_m=float(sample_spacing_m),
                    lateral_search_step_m=float(lateral_search_step_m),
                    maximum_lateral_shift_m=float(maximum_lateral_shift_m),
                    preview_footprint_padding_m=float(footprint_padding),
                ),
                source={
                    "padding_sensitivity_case": _case_id(map_cells, footprint_padding),
                    "production_navigation_map_mutated": False,
                },
            )
            compatible = tuple(
                item
                for item in diagnostic.aisles
                if item.classification != "STRUCTURAL_WIDTH_BLOCKED"
            )
            fractions = [item.any_lateral_footprint_free_fraction for item in compatible]
            cases.append(
                PaddingSensitivityCase(
                    case_id=_case_id(map_cells, footprint_padding),
                    map_padding_cells=int(map_cells),
                    map_padding_axis_m=float(map_cells * navigation.resolution_m),
                    map_padding_diagonal_m=float(
                        math.sqrt(2.0) * map_cells * navigation.resolution_m
                    ),
                    footprint_padding_m=float(footprint_padding),
                    free_cell_count=int(counts["free"]),
                    occupied_cell_count=int(counts["occupied"]),
                    unknown_cell_count=int(counts["unknown"]),
                    width_compatible_aisle_count=len(compatible),
                    zero_feasible_aisle_count=sum(
                        item.any_lateral_footprint_free_pose_count == 0
                        for item in compatible
                    ),
                    mostly_configuration_space_free_count=sum(
                        item.any_lateral_footprint_free_fraction >= 0.80
                        for item in compatible
                    ),
                    mean_any_lateral_free_fraction=(
                        float(np.mean(fractions)) if fractions else 0.0
                    ),
                    aisles=diagnostic.aisles,
                )
            )

    merged_source = dict(graph.source)
    merged_source.update(dict(navigation.source))
    merged_source.update(dict(source or {}))
    merged_source.update(
        {
            "validation_scope": "COUNTERFACTUAL_DIAGNOSTIC_ONLY",
            "source_exactness": masks.source_exactness,
            "production_navigation_map_mutated": False,
            "structural_aisle_graph_mutated": False,
            "vehicle_profile_mutated": False,
            "padding_semantics": "INTEGER_CELL_SQUARE_DILATION_MATCHING_CURRENT_IMPLEMENTATION",
        }
    )
    return VehicleSafeLanePaddingSensitivityPlan(
        frame_id=graph.frame_id,
        platform_id=vehicle.profile_id,
        platform_profile_sha256=vehicle.profile_sha256,
        source_exactness=masks.source_exactness,
        current_requested_obstacle_padding_m=requested_padding_m,
        current_effective_padding_cells=current_padding_cells,
        cases=tuple(cases),
        source=merged_source,
    )


def vehicle_safe_lane_padding_sensitivity_to_dict(
    plan: VehicleSafeLanePaddingSensitivityPlan,
) -> dict[str, Any]:
    return {
        "schema": plan.schema,
        "status": plan.status,
        "frame_id": plan.frame_id,
        "platform_id": plan.platform_id,
        "platform_profile_sha256": plan.platform_profile_sha256,
        "source_exactness": plan.source_exactness,
        "current_requested_obstacle_padding_m": plan.current_requested_obstacle_padding_m,
        "current_effective_padding_cells": plan.current_effective_padding_cells,
        "source": dict(plan.source),
        "cases": [
            {
                "case_id": case.case_id,
                "map_padding_cells": case.map_padding_cells,
                "map_padding_axis_m": case.map_padding_axis_m,
                "map_padding_diagonal_m": case.map_padding_diagonal_m,
                "footprint_padding_m": case.footprint_padding_m,
                "grid_counts": {
                    "free": case.free_cell_count,
                    "occupied": case.occupied_cell_count,
                    "unknown": case.unknown_cell_count,
                },
                "summary": {
                    "width_compatible_aisles": case.width_compatible_aisle_count,
                    "zero_feasible_aisles": case.zero_feasible_aisle_count,
                    "mostly_configuration_space_free_aisles": case.mostly_configuration_space_free_count,
                    "mean_any_lateral_free_fraction": case.mean_any_lateral_free_fraction,
                },
                "aisles": [
                    {
                        "aisle_id": item.aisle_id,
                        "classification": item.classification,
                        "reference_free_fraction": item.reference_free_fraction,
                        "any_lateral_footprint_free_fraction": item.any_lateral_footprint_free_fraction,
                        "mean_best_candidate_free_fraction": item.mean_best_candidate_free_fraction,
                        "mean_best_candidate_occupied_fraction": item.mean_best_candidate_occupied_fraction,
                        "mean_best_candidate_unknown_fraction": item.mean_best_candidate_unknown_fraction,
                    }
                    for item in case.aisles
                ],
            }
            for case in plan.cases
        ],
    }


def write_vehicle_safe_lane_padding_sensitivity(
    plan: VehicleSafeLanePaddingSensitivityPlan,
    path: str | Path,
) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(
            vehicle_safe_lane_padding_sensitivity_to_dict(plan),
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )
    return output
