"""D3 vehicle-specific collision-envelope review over persistent vertical evidence.

This module is downstream of environment-map generation. It never mutates or
promotes the Formal/Accepted PGM. Instead it selects the ground-relative
vertical evidence that physically overlaps a vehicle collision envelope,
derives a review-only NavigationMapResult, and reuses D1.1 for aisle clearance.

The current D3 contract is intentionally aisle-aligned: lateral feasibility is
represented by vehicle half-width plus a safety margin. Yaw-dependent rectangle
sweeps and Ackermann turning envelopes are explicitly deferred.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from .height_layer_ablation import HeightLayerObstacleEvidence
from .navigation_map_derivation import FREE, OCCUPIED, UNKNOWN, NavigationMapResult
from .vehicle_feasibility import build_vehicle_feasible_aisle_audit


_LAYER_ORDER = ("LOW", "MID", "HIGH")


@dataclass(frozen=True)
class VehicleCollisionEnvelope:
    """Aisle-aligned static vehicle collision contract for review evidence."""

    half_width_m: float
    lateral_safety_margin_m: float = 0.0
    collision_z_min_m: float = 0.0
    collision_z_max_m: float = 0.60

    def __post_init__(self) -> None:
        values = (
            self.half_width_m,
            self.lateral_safety_margin_m,
            self.collision_z_min_m,
            self.collision_z_max_m,
        )
        if not all(np.isfinite(float(value)) for value in values):
            raise ValueError("vehicle collision envelope values must be finite")
        if float(self.half_width_m) <= 0.0:
            raise ValueError("vehicle half_width_m must be > 0")
        if float(self.lateral_safety_margin_m) < 0.0:
            raise ValueError("vehicle lateral_safety_margin_m must be >= 0")
        if float(self.collision_z_min_m) < 0.0:
            raise ValueError("vehicle collision_z_min_m must be >= 0")
        if float(self.collision_z_max_m) <= float(self.collision_z_min_m):
            raise ValueError("vehicle collision_z_max_m must be > collision_z_min_m")

    @property
    def effective_lateral_radius_m(self) -> float:
        return float(self.half_width_m) + float(self.lateral_safety_margin_m)

    def to_dict(self) -> dict[str, float]:
        return {
            "half_width_m": float(self.half_width_m),
            "lateral_safety_margin_m": float(self.lateral_safety_margin_m),
            "effective_lateral_radius_m": float(self.effective_lateral_radius_m),
            "collision_z_min_m": float(self.collision_z_min_m),
            "collision_z_max_m": float(self.collision_z_max_m),
        }


def _layer_intervals(evidence: HeightLayerObstacleEvidence) -> dict[str, tuple[float, float]]:
    evidence.validate()
    return {
        "LOW": (
            float(evidence.obstacle_min_height_m),
            float(evidence.low_max_height_m),
        ),
        "MID": (
            float(evidence.low_max_height_m),
            float(evidence.mid_max_height_m),
        ),
        "HIGH": (
            float(evidence.mid_max_height_m),
            float(evidence.obstacle_max_height_m),
        ),
    }


def select_overlapping_height_layers(
    evidence: HeightLayerObstacleEvidence,
    envelope: VehicleCollisionEnvelope,
) -> tuple[str, ...]:
    """Select layers with positive-height overlap with the vehicle envelope.

    Merely touching a layer boundary has zero physical thickness and therefore
    does not select the adjacent layer. For example an envelope ending at
    exactly 0.60 m does not include the HIGH layer that starts at 0.60 m.
    """

    selected: list[str] = []
    for key in _LAYER_ORDER:
        low, high = _layer_intervals(evidence)[key]
        overlap_low = max(low, float(envelope.collision_z_min_m))
        overlap_high = min(high, float(envelope.collision_z_max_m))
        if overlap_high > overlap_low + 1.0e-12:
            selected.append(key)
    return tuple(selected)


def _selected_count(
    evidence: HeightLayerObstacleEvidence,
    layers: tuple[str, ...],
) -> np.ndarray:
    evidence.validate()
    result = np.zeros_like(np.asarray(evidence.low_count, dtype=np.int32))
    arrays = {
        "LOW": np.asarray(evidence.low_count, dtype=np.int32),
        "MID": np.asarray(evidence.mid_count, dtype=np.int32),
        "HIGH": np.asarray(evidence.high_count, dtype=np.int32),
    }
    for layer in layers:
        if layer not in arrays:
            raise ValueError(f"unknown vertical evidence layer {layer!r}")
        result = result + arrays[layer]
    return result.astype(np.int32, copy=False)


def derive_vehicle_envelope_navigation(
    a3_navigation: NavigationMapResult,
    evidence: HeightLayerObstacleEvidence,
    envelope: VehicleCollisionEnvelope,
) -> NavigationMapResult:
    """Derive review-only occupancy using only vertically colliding evidence."""

    evidence.validate(expected_shape=tuple(np.asarray(a3_navigation.occupancy).shape))
    layers = select_overlapping_height_layers(evidence, envelope)
    selected_obstacle_count = _selected_count(evidence, layers)
    cfg = a3_navigation.config
    slope = np.asarray(a3_navigation.slope_deg, dtype=np.float64)
    step = np.asarray(a3_navigation.step_m, dtype=np.float64)

    direct_obstacle = selected_obstacle_count >= int(cfg.minimum_obstacle_points)
    geometry_bad = (
        np.asarray(a3_navigation.ground_valid, dtype=bool)
        & (
            (np.isfinite(slope) & (slope > float(cfg.maximum_slope_deg)))
            | (np.isfinite(step) & (step > float(cfg.maximum_step_m)))
        )
    )
    occupied = direct_obstacle | geometry_bad
    free = (
        np.asarray(a3_navigation.ground_valid, dtype=bool)
        & (
            np.asarray(a3_navigation.ground_support_count, dtype=np.int32)
            >= int(cfg.minimum_ground_support_points)
        )
        & ~occupied
    )
    occupancy = np.full(a3_navigation.occupancy.shape, UNKNOWN, dtype=np.uint8)
    occupancy[free] = FREE
    occupancy[occupied] = OCCUPIED
    return replace(
        a3_navigation,
        obstacle_count=selected_obstacle_count,
        occupancy=occupancy,
    )


def build_vehicle_collision_envelope_audit(
    a3_navigation: NavigationMapResult,
    evidence: HeightLayerObstacleEvidence,
    envelope: VehicleCollisionEnvelope,
    structure: Any,
    corridor: Any,
    accepted_occupancy: np.ndarray,
    *,
    terminal_inset_m: float = 0.50,
) -> dict[str, object]:
    """Build an additive D3 report for a caller-materialized vehicle map.

    ``accepted_occupancy`` must be the structure-aware accepted/review occupancy
    produced from ``derive_vehicle_envelope_navigation`` for the same envelope.
    Keeping materialization outside this module avoids coupling vehicle review to
    map-authority export code.
    """

    vehicle_navigation = derive_vehicle_envelope_navigation(a3_navigation, evidence, envelope)
    if np.asarray(accepted_occupancy).shape != np.asarray(vehicle_navigation.occupancy).shape:
        raise ValueError("vehicle envelope accepted occupancy/grid shape mismatch")
    reports = build_vehicle_feasible_aisle_audit(
        vehicle_navigation,
        structure,
        corridor,
        accepted_occupancy,
        clearance_radius_m=float(envelope.effective_lateral_radius_m),
        terminal_inset_m=float(terminal_inset_m),
    )
    selected_layers = select_overlapping_height_layers(evidence, envelope)
    return {
        "schema": "agt_vehicle_collision_envelope_audit/v1",
        "status": "EXPERIMENTAL_REVIEW_EVIDENCE",
        "authority": "DERIVED_VEHICLE_REVIEW_NOT_NAVIGATION_MAP_AUTHORITY",
        "envelope": envelope.to_dict(),
        "selected_vertical_layers": list(selected_layers),
        "connectivity_scope": "INTERIOR_TERMINAL_BANDS",
        "terminal_inset_m": float(terminal_inset_m),
        "interior_terminal_raster_connected_aisles": sum(
            bool(item.get("interior_terminal_raster_connectivity")) for item in reports
        ),
        "vehicle_feasible_connected_aisles": sum(
            bool(item.get("vehicle_feasible_connectivity")) for item in reports
        ),
        "aisles": reports,
    }
