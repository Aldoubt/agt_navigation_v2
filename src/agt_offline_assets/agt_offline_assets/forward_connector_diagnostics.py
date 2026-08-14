"""Quantify why R5 forward Dubins candidates do not fit a Turn Zone.

This diagnostic is deliberately separate from ``agt_forward_connector_plan/v1``.
R5 keeps its frozen centerline/Turn-Zone acceptance semantics, while this module
answers the operator question: how much larger would the current row-frame Turn
Zone envelope need to be before at least one forward-only Dubins candidate fits?

The expansion values are measured in the Agricultural Aisle Graph row frame:
``u`` follows the canonical row direction and ``v`` is the lateral direction.
For the current AUTO_ENDPOINT_ENVELOPE rectangular Turn Zones this is an exact
envelope diagnostic. For future arbitrary/concave operator polygons the values
remain a bounding-envelope diagnostic and do not replace polygon containment.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Any, Iterable, Mapping

import numpy as np
import yaml

from .agricultural_coverage_ordering import ConnectorRequest
from .forward_connector import (
    ForwardConnectorConfig,
    _dubins_candidates,
    _inside_fraction,
    _sample_candidate,
    _zone_by_id,
)
from .turn_zones import TurnZone, TurnZoneSet
from .vehicle_profile import CanonicalVehicleProfile


FORWARD_CONNECTOR_ZONE_FIT_SCHEMA = "agt_forward_connector_zone_fit_diagnostic/v1"


@dataclass(frozen=True)
class ForwardConnectorZoneFitDiagnostic:
    connector_id: str
    from_aisle_id: str
    to_aisle_id: str
    turn_zone_id: str
    status: str
    candidate_count: int
    best_path_type: str | None
    best_candidate_length_m: float | None
    inside_turn_zone_fraction: float
    required_outward_extension_m: float
    required_inward_extension_m: float
    required_lateral_low_extension_m: float
    required_lateral_high_extension_m: float
    max_required_extension_m: float
    reason: str = ""


@dataclass(frozen=True)
class ForwardConnectorZoneFitReport:
    frame_id: str
    platform_id: str
    platform_profile_sha256: str
    diagnostics: tuple[ForwardConnectorZoneFitDiagnostic, ...]
    source: Mapping[str, Any] = field(default_factory=dict)
    schema: str = FORWARD_CONNECTOR_ZONE_FIT_SCHEMA
    status: str = "DRAFT"

    @property
    def fits_current_zone_count(self) -> int:
        return sum(item.status == "FITS_CURRENT_ZONE" for item in self.diagnostics)

    @property
    def expansion_required_count(self) -> int:
        return sum(item.status == "ZONE_EXPANSION_REQUIRED" for item in self.diagnostics)


def _normalize(direction_xy) -> np.ndarray:
    direction = np.asarray(direction_xy, dtype=np.float64).reshape(2)
    norm = float(np.linalg.norm(direction))
    if norm <= 1.0e-12:
        raise ValueError("row direction must be non-zero")
    return direction / norm


def _row_frame_bounds_xy(points_xy, direction: np.ndarray, perpendicular: np.ndarray):
    points = np.asarray(points_xy, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] == 0 or points.shape[1] != 2:
        raise ValueError("points_xy must be Nx2")
    uu = points @ direction
    vv = points @ perpendicular
    return float(np.min(uu)), float(np.max(uu)), float(np.min(vv)), float(np.max(vv))


def _candidate_expansion(
    samples,
    zone: TurnZone,
    direction: np.ndarray,
    perpendicular: np.ndarray,
) -> tuple[float, float, float, float, float]:
    zone_bounds = _row_frame_bounds_xy(zone.polygon_xy, direction, perpendicular)
    sample_xy = np.asarray([(sample.x, sample.y) for sample in samples], dtype=np.float64)
    sample_bounds = _row_frame_bounds_xy(sample_xy, direction, perpendicular)

    zu0, zu1, zv0, zv1 = zone_bounds
    su0, su1, sv0, sv1 = sample_bounds

    low_u = max(0.0, zu0 - su0)
    high_u = max(0.0, su1 - zu1)
    low_v = max(0.0, zv0 - sv0)
    high_v = max(0.0, sv1 - zv1)

    if zone.side == "LOW_U":
        outward = low_u
        inward = high_u
    elif zone.side == "HIGH_U":
        outward = high_u
        inward = low_u
    else:
        raise ValueError(f"unsupported Turn Zone side: {zone.side}")

    maximum = max(outward, inward, low_v, high_v)
    return float(outward), float(inward), float(low_v), float(high_v), float(maximum)


def diagnose_forward_connector_zone_fit(
    connector_requests: Iterable[ConnectorRequest],
    zones: TurnZoneSet,
    vehicle: CanonicalVehicleProfile,
    config: ForwardConnectorConfig | None = None,
    *,
    source: Mapping[str, Any] | None = None,
) -> ForwardConnectorZoneFitReport:
    """Find the forward Dubins candidate requiring the least zone expansion."""
    cfg = config or ForwardConnectorConfig()
    cfg.validate()
    requests = tuple(connector_requests)

    if vehicle.kinematics != "ackermann":
        raise ValueError("forward connector zone-fit diagnostic requires Ackermann kinematics")
    if not vehicle.planning_preview_ready:
        raise ValueError(f"vehicle profile {vehicle.profile_id} is not ready for planning preview")
    radius = float(vehicle.minimum_turning_radius_m)
    if not vehicle.minimum_turning_radius_verified or radius <= 0.0:
        raise ValueError("zone-fit diagnostic requires verified minimum turning radius")

    direction = _normalize(zones.row_direction_xy)
    perpendicular = np.array([-direction[1], direction[0]], dtype=np.float64)
    zone_map = _zone_by_id(zones)
    diagnostics: list[ForwardConnectorZoneFitDiagnostic] = []

    for request in requests:
        zone = zone_map.get(request.turn_zone_id)
        if zone is None:
            diagnostics.append(
                ForwardConnectorZoneFitDiagnostic(
                    connector_id=request.connector_id,
                    from_aisle_id=request.from_aisle_id,
                    to_aisle_id=request.to_aisle_id,
                    turn_zone_id=request.turn_zone_id,
                    status="TURN_ZONE_NOT_FOUND",
                    candidate_count=0,
                    best_path_type=None,
                    best_candidate_length_m=None,
                    inside_turn_zone_fraction=0.0,
                    required_outward_extension_m=0.0,
                    required_inward_extension_m=0.0,
                    required_lateral_low_extension_m=0.0,
                    required_lateral_high_extension_m=0.0,
                    max_required_extension_m=0.0,
                    reason="requested Turn Zone does not exist",
                )
            )
            continue
        if zone.side != request.side:
            diagnostics.append(
                ForwardConnectorZoneFitDiagnostic(
                    connector_id=request.connector_id,
                    from_aisle_id=request.from_aisle_id,
                    to_aisle_id=request.to_aisle_id,
                    turn_zone_id=request.turn_zone_id,
                    status="TURN_ZONE_SIDE_MISMATCH",
                    candidate_count=0,
                    best_path_type=None,
                    best_candidate_length_m=None,
                    inside_turn_zone_fraction=0.0,
                    required_outward_extension_m=0.0,
                    required_inward_extension_m=0.0,
                    required_lateral_low_extension_m=0.0,
                    required_lateral_high_extension_m=0.0,
                    max_required_extension_m=0.0,
                    reason=f"request side={request.side}, zone side={zone.side}",
                )
            )
            continue

        candidates = _dubins_candidates(request.start_pose, request.goal_pose, radius)
        if not candidates:
            diagnostics.append(
                ForwardConnectorZoneFitDiagnostic(
                    connector_id=request.connector_id,
                    from_aisle_id=request.from_aisle_id,
                    to_aisle_id=request.to_aisle_id,
                    turn_zone_id=request.turn_zone_id,
                    status="NO_FORWARD_DUBINS_CANDIDATE",
                    candidate_count=0,
                    best_path_type=None,
                    best_candidate_length_m=None,
                    inside_turn_zone_fraction=0.0,
                    required_outward_extension_m=0.0,
                    required_inward_extension_m=0.0,
                    required_lateral_low_extension_m=0.0,
                    required_lateral_high_extension_m=0.0,
                    max_required_extension_m=0.0,
                    reason="no analytic forward-only Dubins family is geometrically available",
                )
            )
            continue

        scored = []
        for path_type, normalized_lengths in candidates:
            samples, length_m = _sample_candidate(
                request.start_pose,
                request.goal_pose,
                path_type,
                normalized_lengths,
                radius,
                cfg.sample_step_m,
            )
            fraction = _inside_fraction(samples, zone, cfg.polygon_tolerance_m)
            outward, inward, low_v, high_v, maximum = _candidate_expansion(
                samples,
                zone,
                direction,
                perpendicular,
            )
            expansion_sum = outward + inward + low_v + high_v
            scored.append(
                (
                    maximum,
                    expansion_sum,
                    -fraction,
                    float(length_m),
                    path_type,
                    fraction,
                    outward,
                    inward,
                    low_v,
                    high_v,
                )
            )

        scored.sort(key=lambda item: item[:5])
        (
            maximum,
            _expansion_sum,
            _negative_fraction,
            length_m,
            path_type,
            fraction,
            outward,
            inward,
            low_v,
            high_v,
        ) = scored[0]

        fits_polygon = fraction >= 1.0 - 1.0e-12
        if fits_polygon:
            status = "FITS_CURRENT_ZONE"
            reason = "at least one forward Dubins candidate already fits the requested Turn Zone"
        elif maximum <= cfg.polygon_tolerance_m:
            status = "POLYGON_SHAPE_CONTAINMENT_GAP"
            reason = (
                "candidate fits the row-frame bounding envelope but leaves the polygon; "
                "do not infer that simple rectangular expansion is sufficient"
            )
        else:
            status = "ZONE_EXPANSION_REQUIRED"
            reason = "row-frame envelope must expand before this forward Dubins candidate can fit"

        diagnostics.append(
            ForwardConnectorZoneFitDiagnostic(
                connector_id=request.connector_id,
                from_aisle_id=request.from_aisle_id,
                to_aisle_id=request.to_aisle_id,
                turn_zone_id=request.turn_zone_id,
                status=status,
                candidate_count=len(candidates),
                best_path_type=path_type,
                best_candidate_length_m=float(length_m),
                inside_turn_zone_fraction=float(fraction),
                required_outward_extension_m=float(outward),
                required_inward_extension_m=float(inward),
                required_lateral_low_extension_m=float(low_v),
                required_lateral_high_extension_m=float(high_v),
                max_required_extension_m=float(maximum),
                reason=reason,
            )
        )

    merged_source = dict(zones.source)
    merged_source.update(dict(source or {}))
    merged_source.update(
        {
            "diagnostic_backend": "ANALYTIC_DUBINS_ROW_FRAME_ZONE_DEFICIT",
            "minimum_turning_radius_m": radius,
            "diagnostic_scope": "ROW_FRAME_ENVELOPE_ONLY_NOT_COLLISION_TRUTH",
        }
    )
    return ForwardConnectorZoneFitReport(
        frame_id=zones.frame_id,
        platform_id=vehicle.profile_id,
        platform_profile_sha256=vehicle.profile_sha256,
        diagnostics=tuple(diagnostics),
        source=merged_source,
    )


def forward_connector_zone_fit_to_dict(report: ForwardConnectorZoneFitReport) -> dict[str, Any]:
    return {
        "schema": report.schema,
        "status": report.status,
        "frame_id": report.frame_id,
        "platform_id": report.platform_id,
        "platform_profile_sha256": report.platform_profile_sha256,
        "source": dict(report.source),
        "diagnostic_count": len(report.diagnostics),
        "fits_current_zone_count": report.fits_current_zone_count,
        "expansion_required_count": report.expansion_required_count,
        "diagnostics": [
            {
                "connector_id": item.connector_id,
                "from_aisle_id": item.from_aisle_id,
                "to_aisle_id": item.to_aisle_id,
                "turn_zone_id": item.turn_zone_id,
                "status": item.status,
                "candidate_count": item.candidate_count,
                "best_path_type": item.best_path_type,
                "best_candidate_length_m": item.best_candidate_length_m,
                "inside_turn_zone_fraction": item.inside_turn_zone_fraction,
                "required_outward_extension_m": item.required_outward_extension_m,
                "required_inward_extension_m": item.required_inward_extension_m,
                "required_lateral_low_extension_m": item.required_lateral_low_extension_m,
                "required_lateral_high_extension_m": item.required_lateral_high_extension_m,
                "max_required_extension_m": item.max_required_extension_m,
                "reason": item.reason,
                "diagnostic_scope": "ROW_FRAME_ENVELOPE_ONLY_NOT_COLLISION_TRUTH",
            }
            for item in report.diagnostics
        ],
    }


def write_forward_connector_zone_fit_report(
    report: ForwardConnectorZoneFitReport,
    path: str | Path,
) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(
            forward_connector_zone_fit_to_dict(report),
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )
    return output
