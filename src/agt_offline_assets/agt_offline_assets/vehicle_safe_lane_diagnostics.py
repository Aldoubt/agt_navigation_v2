"""Evidence diagnostics for vehicle-safe agricultural aisle lanes.

This module explains *why* a structural aisle fails to become a vehicle-safe
lane.  It deliberately does not change the Navigation Grid, Aisle Graph, lane
acceptance thresholds, or R6/R7 admission.

For each structural longitudinal sample it distinguishes:

* the occupancy state of the reference point itself
* whether the zero-offset full preview footprint is FREE
* whether any bounded lateral candidate has a fully FREE preview footprint
* whether the best bounded candidate is blocked mainly by OCCUPIED, UNKNOWN, or
  out-of-grid evidence

The resulting asset is diagnostic evidence only.  It exists to decide whether
the next repair belongs to Navigation Map semantics, vehicle-width clearance,
lateral lane placement, or map coverage.
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
    GridPathEvidence,
    _cell_index,
    _evaluate_candidate,
    _preview_local_footprint,
)
from .navigation_grid import NavigationGridEvidence
from .navigation_map_derivation import FREE, OCCUPIED, UNKNOWN
from .vehicle_profile import CanonicalVehicleProfile
from .vehicle_safe_lane import _normalize, _offset_candidates, _resample_polyline


VEHICLE_SAFE_LANE_DIAGNOSTIC_SCHEMA = "agt_vehicle_safe_lane_diagnostic/v1"


@dataclass(frozen=True)
class VehicleSafeLaneDiagnosticConfig:
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
        value = float(self.preview_footprint_padding_m)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError("preview_footprint_padding_m must be finite and >= 0")


@dataclass(frozen=True)
class VehicleSafeLaneAisleDiagnostic:
    aisle_id: str
    classification: str
    structural_width_m: float
    required_preview_width_m: float
    allowed_lateral_shift_m: float
    total_sample_count: int
    reference_free_count: int
    reference_occupied_count: int
    reference_unknown_count: int
    reference_out_of_grid_count: int
    zero_offset_footprint_free_pose_count: int
    any_lateral_footprint_free_pose_count: int
    center_free_but_zero_footprint_blocked_count: int
    center_free_but_no_lateral_free_count: int
    best_candidate_occupied_pose_count: int
    best_candidate_unknown_pose_count: int
    best_candidate_out_of_grid_pose_count: int
    mean_best_candidate_free_fraction: float
    mean_best_candidate_occupied_fraction: float
    mean_best_candidate_unknown_fraction: float
    reason: str

    @property
    def reference_free_fraction(self) -> float:
        return 0.0 if self.total_sample_count <= 0 else self.reference_free_count / self.total_sample_count

    @property
    def zero_offset_footprint_free_fraction(self) -> float:
        return 0.0 if self.total_sample_count <= 0 else self.zero_offset_footprint_free_pose_count / self.total_sample_count

    @property
    def any_lateral_footprint_free_fraction(self) -> float:
        return 0.0 if self.total_sample_count <= 0 else self.any_lateral_footprint_free_pose_count / self.total_sample_count


@dataclass(frozen=True)
class VehicleSafeLaneDiagnosticPlan:
    frame_id: str
    platform_id: str
    platform_profile_sha256: str
    aisles: tuple[VehicleSafeLaneAisleDiagnostic, ...]
    source: Mapping[str, Any] = field(default_factory=dict)
    schema: str = VEHICLE_SAFE_LANE_DIAGNOSTIC_SCHEMA
    status: str = "DIAGNOSTIC_ONLY"


def _reference_state(
    navigation: NavigationGridEvidence,
    x: float,
    y: float,
) -> str:
    index = _cell_index(navigation, float(x), float(y))
    if index is None:
        return "OUT_OF_GRID"
    value = navigation.occupancy[index]
    if value == FREE:
        return "FREE"
    if value == OCCUPIED:
        return "OCCUPIED"
    if value == UNKNOWN:
        return "UNKNOWN"
    return "UNEXPECTED"


def _single_pose_evidence(
    x: float,
    y: float,
    z: float,
    yaw: float,
    navigation: NavigationGridEvidence,
    local_footprint,
) -> GridPathEvidence:
    sample = ForwardConnectorSample(x=float(x), y=float(y), z=float(z), yaw=float(yaw))
    _center, footprint = _evaluate_candidate((sample,), navigation, local_footprint)
    return footprint


def _fully_free(evidence: GridPathEvidence) -> bool:
    return (
        evidence.cell_count > 0
        and evidence.grid_coverage_fraction >= 1.0 - 1.0e-9
        and evidence.occupied_count == 0
        and evidence.unknown_count == 0
        and evidence.free_count == evidence.cell_count
    )


def _best_key(evidence: GridPathEvidence, offset: float) -> tuple[float, ...]:
    out_fraction = max(0.0, 1.0 - float(evidence.grid_coverage_fraction))
    return (
        float(evidence.occupied_fraction),
        float(evidence.unknown_fraction),
        out_fraction,
        abs(float(offset)),
    )


def _classification(
    *,
    width_blocked: bool,
    total: int,
    reference_free: int,
    reference_occupied: int,
    reference_unknown: int,
    reference_out: int,
    any_lateral_free: int,
    center_free_no_lateral: int,
    best_occ: int,
    best_unknown: int,
    best_out: int,
) -> tuple[str, str]:
    if width_blocked:
        return (
            "STRUCTURAL_WIDTH_BLOCKED",
            "structural aisle width is below the canonical preview vehicle width gate",
        )
    if total <= 0:
        return "NO_SAMPLES", "no structural aisle samples were available"

    reference_occ_fraction = reference_occupied / total
    reference_unknown_fraction = reference_unknown / total
    reference_out_fraction = reference_out / total
    lateral_free_fraction = any_lateral_free / total
    center_free_no_lateral_fraction = center_free_no_lateral / total

    if lateral_free_fraction >= 0.80:
        return (
            "MOSTLY_CONFIGURATION_SPACE_FREE",
            "most longitudinal stations have at least one fully FREE bounded lateral vehicle pose; remaining failure belongs to continuity/endpoints",
        )
    if reference_occ_fraction >= 0.50:
        return (
            "CENTER_REFERENCE_OCCUPIED_DOMINANT",
            "the structural centerline reference point itself is OCCUPIED at at least half of longitudinal samples",
        )
    if reference_unknown_fraction >= 0.50:
        return (
            "CENTER_REFERENCE_UNKNOWN_DOMINANT",
            "the structural centerline reference point itself is UNKNOWN at at least half of longitudinal samples",
        )
    if reference_out_fraction >= 0.20:
        return (
            "MAP_COVERAGE_LIMITED",
            "a material fraction of structural centerline samples is outside the frozen Navigation Grid",
        )
    if center_free_no_lateral_fraction >= 0.50:
        if best_occ > best_unknown and best_occ >= best_out:
            return (
                "FOOTPRINT_OCCUPIED_DOMINANT",
                "reference points are often FREE but the full vehicle footprint is predominantly blocked by OCCUPIED cells",
            )
        if best_unknown >= best_occ and best_unknown >= best_out:
            return (
                "FOOTPRINT_UNKNOWN_DOMINANT",
                "reference points are often FREE but the full vehicle footprint is predominantly blocked by UNKNOWN cells",
            )
        return (
            "FOOTPRINT_GRID_BOUNDARY_DOMINANT",
            "reference points are often FREE but the full vehicle footprint is predominantly limited by grid coverage",
        )
    if best_occ > best_unknown and best_occ >= best_out:
        return "MIXED_OCCUPIED_DOMINANT", "mixed evidence with OCCUPIED as the most common best-candidate blocker"
    if best_unknown >= best_occ and best_unknown >= best_out:
        return "MIXED_UNKNOWN_DOMINANT", "mixed evidence with UNKNOWN as the most common best-candidate blocker"
    return "MIXED_GRID_LIMITED", "mixed evidence with grid coverage as the most common best-candidate blocker"


def _diagnose_one(
    aisle: AislePrimitive,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    direction: np.ndarray,
    perpendicular: np.ndarray,
    local_footprint,
    cfg: VehicleSafeLaneDiagnosticConfig,
) -> VehicleSafeLaneAisleDiagnostic:
    samples, _distances = _resample_polyline(aisle, cfg.sample_spacing_m)
    total = int(samples.shape[0])
    required_width = float(vehicle.navigation_width_m + 2.0 * cfg.preview_footprint_padding_m)
    width_blocked = float(aisle.geometric_width_m) + 1.0e-9 < required_width
    width_surplus = max(0.0, float(aisle.geometric_width_m) - required_width)
    allowed_shift = min(float(cfg.maximum_lateral_shift_m), 0.5 * width_surplus)
    offsets = _offset_candidates(allowed_shift, cfg.lateral_search_step_m)
    yaw = math.atan2(float(direction[1]), float(direction[0]))

    reference = {"FREE": 0, "OCCUPIED": 0, "UNKNOWN": 0, "OUT_OF_GRID": 0}
    zero_free = 0
    lateral_free = 0
    center_free_zero_blocked = 0
    center_free_no_lateral = 0
    best_occ = 0
    best_unknown = 0
    best_out = 0
    best_free_fraction: list[float] = []
    best_occ_fraction: list[float] = []
    best_unknown_fraction: list[float] = []

    if width_blocked:
        classification, reason = _classification(
            width_blocked=True,
            total=total,
            reference_free=0,
            reference_occupied=0,
            reference_unknown=0,
            reference_out=0,
            any_lateral_free=0,
            center_free_no_lateral=0,
            best_occ=0,
            best_unknown=0,
            best_out=0,
        )
        return VehicleSafeLaneAisleDiagnostic(
            aisle_id=aisle.aisle_id,
            classification=classification,
            structural_width_m=float(aisle.geometric_width_m),
            required_preview_width_m=required_width,
            allowed_lateral_shift_m=0.0,
            total_sample_count=total,
            reference_free_count=0,
            reference_occupied_count=0,
            reference_unknown_count=0,
            reference_out_of_grid_count=0,
            zero_offset_footprint_free_pose_count=0,
            any_lateral_footprint_free_pose_count=0,
            center_free_but_zero_footprint_blocked_count=0,
            center_free_but_no_lateral_free_count=0,
            best_candidate_occupied_pose_count=0,
            best_candidate_unknown_pose_count=0,
            best_candidate_out_of_grid_pose_count=0,
            mean_best_candidate_free_fraction=0.0,
            mean_best_candidate_occupied_fraction=0.0,
            mean_best_candidate_unknown_fraction=0.0,
            reason=reason,
        )

    for sample in samples:
        state = _reference_state(navigation, float(sample[0]), float(sample[1]))
        if state in reference:
            reference[state] += 1
        else:
            reference["OUT_OF_GRID"] += 1

        candidate_evidence: list[tuple[float, GridPathEvidence]] = []
        zero_evidence: GridPathEvidence | None = None
        any_free = False
        for offset in offsets:
            x = float(sample[0] + float(offset) * perpendicular[0])
            y = float(sample[1] + float(offset) * perpendicular[1])
            evidence = _single_pose_evidence(
                x,
                y,
                float(sample[2]),
                yaw,
                navigation,
                local_footprint,
            )
            candidate_evidence.append((float(offset), evidence))
            if abs(float(offset)) <= 1.0e-12:
                zero_evidence = evidence
            if _fully_free(evidence):
                any_free = True

        if zero_evidence is not None and _fully_free(zero_evidence):
            zero_free += 1
        elif state == "FREE":
            center_free_zero_blocked += 1

        if any_free:
            lateral_free += 1
        elif state == "FREE":
            center_free_no_lateral += 1

        _best_offset, best = min(candidate_evidence, key=lambda item: _best_key(item[1], item[0]))
        best_free_fraction.append(float(best.free_fraction))
        best_occ_fraction.append(float(best.occupied_fraction))
        best_unknown_fraction.append(float(best.unknown_fraction))
        out_fraction = max(0.0, 1.0 - float(best.grid_coverage_fraction))
        dominant = max(
            (
                (float(best.occupied_fraction), "OCCUPIED"),
                (float(best.unknown_fraction), "UNKNOWN"),
                (out_fraction, "OUT_OF_GRID"),
            ),
            key=lambda item: item[0],
        )[1]
        if dominant == "OCCUPIED" and best.occupied_fraction > 0.0:
            best_occ += 1
        elif dominant == "UNKNOWN" and best.unknown_fraction > 0.0:
            best_unknown += 1
        elif dominant == "OUT_OF_GRID" and out_fraction > 0.0:
            best_out += 1

    classification, reason = _classification(
        width_blocked=False,
        total=total,
        reference_free=reference["FREE"],
        reference_occupied=reference["OCCUPIED"],
        reference_unknown=reference["UNKNOWN"],
        reference_out=reference["OUT_OF_GRID"],
        any_lateral_free=lateral_free,
        center_free_no_lateral=center_free_no_lateral,
        best_occ=best_occ,
        best_unknown=best_unknown,
        best_out=best_out,
    )

    return VehicleSafeLaneAisleDiagnostic(
        aisle_id=aisle.aisle_id,
        classification=classification,
        structural_width_m=float(aisle.geometric_width_m),
        required_preview_width_m=required_width,
        allowed_lateral_shift_m=allowed_shift,
        total_sample_count=total,
        reference_free_count=reference["FREE"],
        reference_occupied_count=reference["OCCUPIED"],
        reference_unknown_count=reference["UNKNOWN"],
        reference_out_of_grid_count=reference["OUT_OF_GRID"],
        zero_offset_footprint_free_pose_count=zero_free,
        any_lateral_footprint_free_pose_count=lateral_free,
        center_free_but_zero_footprint_blocked_count=center_free_zero_blocked,
        center_free_but_no_lateral_free_count=center_free_no_lateral,
        best_candidate_occupied_pose_count=best_occ,
        best_candidate_unknown_pose_count=best_unknown,
        best_candidate_out_of_grid_pose_count=best_out,
        mean_best_candidate_free_fraction=float(np.mean(best_free_fraction)) if best_free_fraction else 0.0,
        mean_best_candidate_occupied_fraction=float(np.mean(best_occ_fraction)) if best_occ_fraction else 0.0,
        mean_best_candidate_unknown_fraction=float(np.mean(best_unknown_fraction)) if best_unknown_fraction else 0.0,
        reason=reason,
    )


def derive_vehicle_safe_lane_diagnostics(
    graph: AgriculturalAisleGraph,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    config: VehicleSafeLaneDiagnosticConfig | None = None,
    *,
    source: Mapping[str, Any] | None = None,
) -> VehicleSafeLaneDiagnosticPlan:
    cfg = config or VehicleSafeLaneDiagnosticConfig()
    cfg.validate()
    if graph.frame_id != navigation.frame_id:
        raise ValueError("Aisle Graph and Navigation Grid frame_id must match")
    if not vehicle.planning_preview_ready:
        raise ValueError(f"vehicle profile {vehicle.profile_id} is not ready for planning preview")

    direction = _normalize(graph.row_direction_xy)
    perpendicular = np.array([-direction[1], direction[0]], dtype=np.float64)
    local_footprint = _preview_local_footprint(vehicle, cfg.preview_footprint_padding_m)
    aisles = tuple(
        _diagnose_one(
            aisle,
            navigation,
            vehicle,
            direction,
            perpendicular,
            local_footprint,
            cfg,
        )
        for aisle in graph.aisles
    )
    merged_source = dict(graph.source)
    merged_source.update(dict(navigation.source))
    merged_source.update(dict(source or {}))
    merged_source.update(
        {
            "diagnostic_scope": "EXPLAIN_VEHICLE_SAFE_LANE_FAILURE_ONLY",
            "validation_scope": "PREVIEW_ONLY_NOT_R8_VEHICLE_READY",
            "navigation_occupancy_mutated": False,
            "structural_aisle_graph_mutated": False,
        }
    )
    return VehicleSafeLaneDiagnosticPlan(
        frame_id=graph.frame_id,
        platform_id=vehicle.profile_id,
        platform_profile_sha256=vehicle.profile_sha256,
        aisles=aisles,
        source=merged_source,
    )


def vehicle_safe_lane_diagnostic_to_dict(plan: VehicleSafeLaneDiagnosticPlan) -> dict[str, Any]:
    return {
        "schema": plan.schema,
        "status": plan.status,
        "frame_id": plan.frame_id,
        "platform_id": plan.platform_id,
        "platform_profile_sha256": plan.platform_profile_sha256,
        "source": dict(plan.source),
        "aisle_count": len(plan.aisles),
        "aisles": [
            {
                "aisle_id": item.aisle_id,
                "classification": item.classification,
                "structural_width_m": item.structural_width_m,
                "required_preview_width_m": item.required_preview_width_m,
                "allowed_lateral_shift_m": item.allowed_lateral_shift_m,
                "total_sample_count": item.total_sample_count,
                "reference_point": {
                    "free_count": item.reference_free_count,
                    "occupied_count": item.reference_occupied_count,
                    "unknown_count": item.reference_unknown_count,
                    "out_of_grid_count": item.reference_out_of_grid_count,
                    "free_fraction": item.reference_free_fraction,
                },
                "pose_feasibility": {
                    "zero_offset_footprint_free_pose_count": item.zero_offset_footprint_free_pose_count,
                    "zero_offset_footprint_free_fraction": item.zero_offset_footprint_free_fraction,
                    "any_lateral_footprint_free_pose_count": item.any_lateral_footprint_free_pose_count,
                    "any_lateral_footprint_free_fraction": item.any_lateral_footprint_free_fraction,
                    "center_free_but_zero_footprint_blocked_count": item.center_free_but_zero_footprint_blocked_count,
                    "center_free_but_no_lateral_free_count": item.center_free_but_no_lateral_free_count,
                },
                "best_candidate_blockers": {
                    "occupied_pose_count": item.best_candidate_occupied_pose_count,
                    "unknown_pose_count": item.best_candidate_unknown_pose_count,
                    "out_of_grid_pose_count": item.best_candidate_out_of_grid_pose_count,
                    "mean_free_fraction": item.mean_best_candidate_free_fraction,
                    "mean_occupied_fraction": item.mean_best_candidate_occupied_fraction,
                    "mean_unknown_fraction": item.mean_best_candidate_unknown_fraction,
                },
                "reason": item.reason,
            }
            for item in plan.aisles
        ],
    }


def write_vehicle_safe_lane_diagnostics(
    plan: VehicleSafeLaneDiagnosticPlan,
    path: str | Path,
) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(vehicle_safe_lane_diagnostic_to_dict(plan), sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return output
