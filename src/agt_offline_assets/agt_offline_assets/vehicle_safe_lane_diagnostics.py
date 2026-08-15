"""Explain why structural agricultural aisles fail the vehicle-safe lane gate.

This is diagnostic-only evidence. It never mutates the Navigation Grid, Aisle
Graph, lane thresholds, or R6/R7 admission.
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
        for name in ("sample_spacing_m", "lateral_search_step_m", "maximum_lateral_shift_m"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")
        if self.preview_footprint_padding_m < 0.0:
            raise ValueError("preview_footprint_padding_m must be >= 0")


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


def _reference_state(navigation: NavigationGridEvidence, x: float, y: float) -> str:
    index = _cell_index(navigation, x, y)
    if index is None:
        return "OUT_OF_GRID"
    value = navigation.occupancy[index]
    if value == FREE:
        return "FREE"
    if value == OCCUPIED:
        return "OCCUPIED"
    if value == UNKNOWN:
        return "UNKNOWN"
    return "OUT_OF_GRID"


def _pose_evidence(
    x: float,
    y: float,
    z: float,
    yaw: float,
    navigation: NavigationGridEvidence,
    local_footprint,
) -> GridPathEvidence:
    sample = ForwardConnectorSample(x=x, y=y, z=z, yaw=yaw)
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
    """Prefer the least total non-FREE evidence, never OCCUPIED-only ranking."""
    out = max(0.0, 1.0 - float(evidence.grid_coverage_fraction))
    non_free = float(evidence.occupied_fraction + evidence.unknown_fraction + out)
    return (
        non_free,
        float(evidence.occupied_fraction),
        float(evidence.unknown_fraction),
        out,
        abs(float(offset)),
    )


def _classify(
    width_blocked: bool,
    total: int,
    ref_free: int,
    ref_occ: int,
    ref_unknown: int,
    ref_out: int,
    lateral_free: int,
    center_free_no_lateral: int,
    blocker_occ: int,
    blocker_unknown: int,
    blocker_out: int,
) -> tuple[str, str]:
    if width_blocked:
        return "STRUCTURAL_WIDTH_BLOCKED", "aisle structural width is below the canonical preview vehicle-width gate"
    if total <= 0:
        return "NO_SAMPLES", "no structural aisle samples were available"

    if lateral_free / total >= 0.80:
        return "MOSTLY_CONFIGURATION_SPACE_FREE", "most stations have a fully FREE bounded lateral vehicle pose"
    if ref_occ / total >= 0.50:
        return "CENTER_REFERENCE_OCCUPIED_DOMINANT", "the structural centerline reference point itself is OCCUPIED at at least half the stations"
    if ref_unknown / total >= 0.50:
        return "CENTER_REFERENCE_UNKNOWN_DOMINANT", "the structural centerline reference point itself is UNKNOWN at at least half the stations"
    if ref_out / total >= 0.20:
        return "MAP_COVERAGE_LIMITED", "a material fraction of structural centerline samples is outside the frozen grid"

    if ref_free / total >= 0.50 and center_free_no_lateral / total >= 0.30:
        if blocker_occ >= blocker_unknown and blocker_occ >= blocker_out:
            return "FOOTPRINT_OCCUPIED_DOMINANT", "reference points are often FREE but the full vehicle footprint is predominantly blocked by OCCUPIED cells"
        if blocker_unknown >= blocker_occ and blocker_unknown >= blocker_out:
            return "FOOTPRINT_UNKNOWN_DOMINANT", "reference points are often FREE but the full vehicle footprint is predominantly blocked by UNKNOWN cells"
        return "FOOTPRINT_GRID_BOUNDARY_DOMINANT", "reference points are often FREE but the footprint is predominantly limited by grid coverage"

    if blocker_occ >= blocker_unknown and blocker_occ >= blocker_out:
        return "MIXED_OCCUPIED_DOMINANT", "mixed route evidence with OCCUPIED as the most common best-candidate blocker"
    if blocker_unknown >= blocker_occ and blocker_unknown >= blocker_out:
        return "MIXED_UNKNOWN_DOMINANT", "mixed route evidence with UNKNOWN as the most common best-candidate blocker"
    return "MIXED_GRID_LIMITED", "mixed route evidence with grid coverage as the most common best-candidate blocker"


def _diagnose_one(
    aisle: AislePrimitive,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    direction: np.ndarray,
    perpendicular: np.ndarray,
    local_footprint,
    cfg: VehicleSafeLaneDiagnosticConfig,
) -> VehicleSafeLaneAisleDiagnostic:
    samples, _ = _resample_polyline(aisle, cfg.sample_spacing_m)
    total = int(samples.shape[0])
    required_width = float(vehicle.navigation_width_m + 2.0 * cfg.preview_footprint_padding_m)
    width_blocked = float(aisle.geometric_width_m) + 1.0e-9 < required_width
    surplus = max(0.0, float(aisle.geometric_width_m) - required_width)
    allowed_shift = 0.0 if width_blocked else min(float(cfg.maximum_lateral_shift_m), 0.5 * surplus)
    offsets = _offset_candidates(allowed_shift, cfg.lateral_search_step_m)
    yaw = math.atan2(float(direction[1]), float(direction[0]))

    ref = {"FREE": 0, "OCCUPIED": 0, "UNKNOWN": 0, "OUT_OF_GRID": 0}
    zero_free = 0
    lateral_free = 0
    center_free_no_lateral = 0
    blocker_occ = 0
    blocker_unknown = 0
    blocker_out = 0
    means_free: list[float] = []
    means_occ: list[float] = []
    means_unknown: list[float] = []

    if not width_blocked:
        for sample in samples:
            state = _reference_state(navigation, float(sample[0]), float(sample[1]))
            ref[state] += 1
            candidates: list[tuple[float, GridPathEvidence]] = []
            zero: GridPathEvidence | None = None
            any_free = False
            for offset in offsets:
                x = float(sample[0] + offset * perpendicular[0])
                y = float(sample[1] + offset * perpendicular[1])
                evidence = _pose_evidence(x, y, float(sample[2]), yaw, navigation, local_footprint)
                candidates.append((offset, evidence))
                if abs(offset) <= 1.0e-12:
                    zero = evidence
                any_free |= _fully_free(evidence)

            if zero is not None and _fully_free(zero):
                zero_free += 1
            if any_free:
                lateral_free += 1
            elif state == "FREE":
                center_free_no_lateral += 1

            _offset, best = min(candidates, key=lambda item: _best_key(item[1], item[0]))
            means_free.append(float(best.free_fraction))
            means_occ.append(float(best.occupied_fraction))
            means_unknown.append(float(best.unknown_fraction))
            out = max(0.0, 1.0 - float(best.grid_coverage_fraction))
            dominant_value, dominant_name = max(
                (
                    (float(best.occupied_fraction), "OCCUPIED"),
                    (float(best.unknown_fraction), "UNKNOWN"),
                    (out, "OUT_OF_GRID"),
                ),
                key=lambda item: item[0],
            )
            if dominant_value > 0.0:
                if dominant_name == "OCCUPIED":
                    blocker_occ += 1
                elif dominant_name == "UNKNOWN":
                    blocker_unknown += 1
                else:
                    blocker_out += 1

    classification, reason = _classify(
        width_blocked,
        total,
        ref["FREE"],
        ref["OCCUPIED"],
        ref["UNKNOWN"],
        ref["OUT_OF_GRID"],
        lateral_free,
        center_free_no_lateral,
        blocker_occ,
        blocker_unknown,
        blocker_out,
    )

    return VehicleSafeLaneAisleDiagnostic(
        aisle_id=aisle.aisle_id,
        classification=classification,
        structural_width_m=float(aisle.geometric_width_m),
        required_preview_width_m=required_width,
        allowed_lateral_shift_m=allowed_shift,
        total_sample_count=total,
        reference_free_count=ref["FREE"],
        reference_occupied_count=ref["OCCUPIED"],
        reference_unknown_count=ref["UNKNOWN"],
        reference_out_of_grid_count=ref["OUT_OF_GRID"],
        zero_offset_footprint_free_pose_count=zero_free,
        any_lateral_footprint_free_pose_count=lateral_free,
        center_free_but_no_lateral_free_count=center_free_no_lateral,
        best_candidate_occupied_pose_count=blocker_occ,
        best_candidate_unknown_pose_count=blocker_unknown,
        best_candidate_out_of_grid_pose_count=blocker_out,
        mean_best_candidate_free_fraction=float(np.mean(means_free)) if means_free else 0.0,
        mean_best_candidate_occupied_fraction=float(np.mean(means_occ)) if means_occ else 0.0,
        mean_best_candidate_unknown_fraction=float(np.mean(means_unknown)) if means_unknown else 0.0,
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
    footprint = _preview_local_footprint(vehicle, cfg.preview_footprint_padding_m)
    aisles = tuple(
        _diagnose_one(aisle, navigation, vehicle, direction, perpendicular, footprint, cfg)
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
                    "any_lateral_footprint_free_pose_count": item.any_lateral_footprint_free_pose_count,
                    "any_lateral_footprint_free_fraction": item.any_lateral_footprint_free_fraction,
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


def write_vehicle_safe_lane_diagnostics(plan: VehicleSafeLaneDiagnosticPlan, path: str | Path) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(vehicle_safe_lane_diagnostic_to_dict(plan), sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return output
