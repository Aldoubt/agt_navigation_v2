"""Counterfactual direct-source sensitivity for vehicle-safe agricultural lanes.

The padding sensitivity experiment showed that removing Navigation Map obstacle
padding improves route-pose feasibility but does not recover most early
Greenhouse aisles.  This module isolates the remaining direct occupied sources
without changing the frozen production map.

Four no-map-padding counterfactuals are evaluated against the same frozen ground
support evidence and canonical vehicle footprint:

* NO_DIRECT_OCCUPIED
* RAW_OBSTACLE_ONLY
* GEOMETRY_ONLY
* RAW_PLUS_GEOMETRY

The result answers whether raw obstacle projection, slope/step geometry evidence,
or their combination is the dominant remaining route bottleneck after obstacle
padding is removed.

This module is DIAGNOSTIC_ONLY.  It never mutates the production Navigation Map,
Aisle Graph, vehicle profile, R6 admission, or R7 state.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from .agricultural_aisle_graph import AgriculturalAisleGraph
from .navigation_grid import NavigationGridEvidence
from .vehicle_profile import CanonicalVehicleProfile
from .vehicle_safe_lane_diagnostics import (
    VehicleSafeLaneAisleDiagnostic,
    VehicleSafeLaneDiagnosticConfig,
    derive_vehicle_safe_lane_diagnostics,
)
from .vehicle_safe_lane_occupancy_sources import (
    load_navigation_occupancy_source_masks,
)
from .vehicle_safe_lane_padding_sensitivity import (
    _counterfactual_navigation,
    _load_counterfactual_ground_evidence,
)


VEHICLE_SAFE_LANE_DIRECT_SOURCE_SENSITIVITY_SCHEMA = (
    "agt_vehicle_safe_lane_direct_source_sensitivity/v1"
)


@dataclass(frozen=True)
class DirectSourceSensitivityConfig:
    sample_spacing_m: float = 0.10
    lateral_search_step_m: float = 0.05
    maximum_lateral_shift_m: float = 0.50
    preview_footprint_padding_m: float = 0.0

    def validate(self) -> None:
        for name in (
            "sample_spacing_m",
            "lateral_search_step_m",
            "maximum_lateral_shift_m",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")
        padding = float(self.preview_footprint_padding_m)
        if not math.isfinite(padding) or padding < 0.0:
            raise ValueError("preview_footprint_padding_m must be finite and >= 0")


@dataclass(frozen=True)
class DirectSourceSensitivityCase:
    case_id: str
    free_cell_count: int
    occupied_cell_count: int
    unknown_cell_count: int
    zero_feasible_aisle_count: int
    mostly_configuration_space_free_count: int
    mean_any_lateral_free_fraction: float
    aisles: tuple[VehicleSafeLaneAisleDiagnostic, ...]


@dataclass(frozen=True)
class VehicleSafeLaneDirectSourceSensitivityPlan:
    frame_id: str
    platform_id: str
    platform_profile_sha256: str
    source_exactness: str
    footprint_padding_m: float
    cases: tuple[DirectSourceSensitivityCase, ...]
    source: Mapping[str, Any] = field(default_factory=dict)
    schema: str = VEHICLE_SAFE_LANE_DIRECT_SOURCE_SENSITIVITY_SCHEMA
    status: str = "DIAGNOSTIC_ONLY"


def _validate_mask(name: str, array: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    mask = np.asarray(array, dtype=bool)
    if mask.shape != shape:
        raise ValueError(f"{name} shape {mask.shape} does not match Navigation Grid {shape}")
    return mask


def _case_masks(
    raw_obstacle_direct: np.ndarray,
    geometry_direct: np.ndarray,
) -> tuple[tuple[str, np.ndarray], ...]:
    shape = raw_obstacle_direct.shape
    none = np.zeros(shape, dtype=bool)
    return (
        ("NO_DIRECT_OCCUPIED", none),
        ("RAW_OBSTACLE_ONLY", raw_obstacle_direct.copy()),
        ("GEOMETRY_ONLY", geometry_direct.copy()),
        ("RAW_PLUS_GEOMETRY", raw_obstacle_direct | geometry_direct),
    )


def derive_vehicle_safe_lane_direct_source_sensitivity_from_masks(
    graph: AgriculturalAisleGraph,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    raw_obstacle_direct: np.ndarray,
    geometry_direct: np.ndarray,
    ground_valid: np.ndarray,
    ground_support: np.ndarray,
    *,
    minimum_ground_support_points: int,
    config: DirectSourceSensitivityConfig | None = None,
    source_exactness: str = "SYNTHETIC_OR_CALLER_SUPPLIED",
    source: Mapping[str, Any] | None = None,
) -> VehicleSafeLaneDirectSourceSensitivityPlan:
    """Evaluate direct occupied-source counterfactuals from explicit masks."""
    cfg = config or DirectSourceSensitivityConfig()
    cfg.validate()
    if graph.frame_id != navigation.frame_id:
        raise ValueError("Aisle Graph and Navigation Grid frame_id must match")
    if not vehicle.planning_preview_ready:
        raise ValueError(f"vehicle profile {vehicle.profile_id} is not preview-ready")
    if int(minimum_ground_support_points) < 1:
        raise ValueError("minimum_ground_support_points must be >= 1")

    shape = navigation.occupancy.shape
    raw = _validate_mask("raw_obstacle_direct", raw_obstacle_direct, shape)
    geometry = _validate_mask("geometry_direct", geometry_direct, shape)
    valid = _validate_mask("ground_valid", ground_valid, shape)
    support = np.asarray(ground_support)
    if support.shape != shape:
        raise ValueError(
            f"ground_support shape {support.shape} does not match Navigation Grid {shape}"
        )

    cases: list[DirectSourceSensitivityCase] = []
    for case_id, direct_mask in _case_masks(raw, geometry):
        counterfactual = _counterfactual_navigation(
            navigation,
            direct_mask,
            valid,
            support,
            int(minimum_ground_support_points),
            0,
            source={
                "direct_source_sensitivity_case": case_id,
                "counterfactual_map_padding_cells": 0,
                "production_navigation_map_mutated": False,
            },
        )
        diagnostic = derive_vehicle_safe_lane_diagnostics(
            graph,
            counterfactual,
            vehicle,
            VehicleSafeLaneDiagnosticConfig(
                sample_spacing_m=float(cfg.sample_spacing_m),
                lateral_search_step_m=float(cfg.lateral_search_step_m),
                maximum_lateral_shift_m=float(cfg.maximum_lateral_shift_m),
                preview_footprint_padding_m=float(cfg.preview_footprint_padding_m),
            ),
            source={
                "direct_source_sensitivity_case": case_id,
                "production_navigation_map_mutated": False,
            },
        )
        compatible = tuple(
            item
            for item in diagnostic.aisles
            if item.classification != "STRUCTURAL_WIDTH_BLOCKED"
        )
        fractions = [item.any_lateral_footprint_free_fraction for item in compatible]
        counts = counterfactual.counts()
        cases.append(
            DirectSourceSensitivityCase(
                case_id=case_id,
                free_cell_count=int(counts["free"]),
                occupied_cell_count=int(counts["occupied"]),
                unknown_cell_count=int(counts["unknown"]),
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
            "validation_scope": "COUNTERFACTUAL_DIRECT_SOURCE_DIAGNOSTIC_ONLY",
            "map_padding_cells": 0,
            "preview_footprint_padding_m": float(cfg.preview_footprint_padding_m),
            "production_navigation_map_mutated": False,
            "structural_aisle_graph_mutated": False,
            "vehicle_profile_mutated": False,
            "r6_r7_admission_mutated": False,
        }
    )
    return VehicleSafeLaneDirectSourceSensitivityPlan(
        frame_id=graph.frame_id,
        platform_id=vehicle.profile_id,
        platform_profile_sha256=vehicle.profile_sha256,
        source_exactness=str(source_exactness),
        footprint_padding_m=float(cfg.preview_footprint_padding_m),
        cases=tuple(cases),
        source=merged_source,
    )


def derive_vehicle_safe_lane_direct_source_sensitivity(
    graph: AgriculturalAisleGraph,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    navigation_asset: str | Path,
    config: DirectSourceSensitivityConfig | None = None,
    *,
    source: Mapping[str, Any] | None = None,
) -> VehicleSafeLaneDirectSourceSensitivityPlan:
    """Load frozen sidecars and evaluate no-padding direct-source cases."""
    masks = load_navigation_occupancy_source_masks(navigation, navigation_asset)
    (
        ground_valid,
        ground_support,
        minimum_support,
        _current_padding_cells,
    ) = _load_counterfactual_ground_evidence(navigation, navigation_asset)
    return derive_vehicle_safe_lane_direct_source_sensitivity_from_masks(
        graph,
        navigation,
        vehicle,
        masks.raw_obstacle_direct,
        masks.geometry_direct,
        ground_valid,
        ground_support,
        minimum_ground_support_points=minimum_support,
        config=config,
        source_exactness=masks.source_exactness,
        source=source,
    )


def vehicle_safe_lane_direct_source_sensitivity_to_dict(
    plan: VehicleSafeLaneDirectSourceSensitivityPlan,
) -> dict[str, Any]:
    return {
        "schema": plan.schema,
        "status": plan.status,
        "frame_id": plan.frame_id,
        "platform_id": plan.platform_id,
        "platform_profile_sha256": plan.platform_profile_sha256,
        "source_exactness": plan.source_exactness,
        "footprint_padding_m": plan.footprint_padding_m,
        "source": dict(plan.source),
        "cases": [
            {
                "case_id": case.case_id,
                "grid_counts": {
                    "free": case.free_cell_count,
                    "occupied": case.occupied_cell_count,
                    "unknown": case.unknown_cell_count,
                },
                "summary": {
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


def write_vehicle_safe_lane_direct_source_sensitivity(
    plan: VehicleSafeLaneDirectSourceSensitivityPlan,
    path: str | Path,
) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(
            vehicle_safe_lane_direct_source_sensitivity_to_dict(plan),
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )
    return output
