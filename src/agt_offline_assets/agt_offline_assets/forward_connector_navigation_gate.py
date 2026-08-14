"""Connector-specific Navigation Grid gate for R5 forward Dubins candidates.

The shared Turn Zone is a search envelope spanning many aisle endpoints, so the
FREE/OCCUPIED ratio of an enlarged whole zone is not a valid proxy for whether a
particular Dubins connector is drivable.  This module evaluates each analytic
forward candidate directly against the frozen Navigation Grid.

For MK-mini preview the canonical navigation footprint is swept along each
sampled pose.  Because the exact on-vehicle ``base_footprint`` reference is not
yet verified, this remains PREVIEW evidence and cannot replace the formal R8
vehicle-READY swept-footprint gate.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping
import math

import numpy as np
import yaml

from .agricultural_coverage_ordering import ConnectorRequest
from .forward_connector import (
    ForwardConnectorConfig,
    ForwardConnectorSample,
    _dubins_candidates,
    _sample_candidate,
    _zone_by_id,
)
from .forward_connector_diagnostics import _candidate_expansion
from .navigation_grid import NavigationGridEvidence
from .navigation_map_derivation import FREE, OCCUPIED, UNKNOWN
from .turn_zones import TurnZoneSet, _points_inside_polygon
from .vehicle_profile import CanonicalVehicleProfile


FORWARD_CONNECTOR_NAVIGATION_GATE_SCHEMA = "agt_forward_connector_navigation_gate/v1"


@dataclass(frozen=True)
class ForwardConnectorNavigationGateConfig:
    sample_step_m: float = 0.05
    preview_footprint_padding_m: float = 0.05
    maximum_occupied_fraction: float = 0.0
    maximum_unknown_fraction: float = 0.0
    minimum_grid_coverage_fraction: float = 1.0

    def validate(self) -> None:
        if not math.isfinite(self.sample_step_m) or self.sample_step_m <= 0.0:
            raise ValueError("sample_step_m must be finite and > 0")
        if self.preview_footprint_padding_m < 0.0:
            raise ValueError("preview_footprint_padding_m must be >= 0")
        for name, value in (
            ("maximum_occupied_fraction", self.maximum_occupied_fraction),
            ("maximum_unknown_fraction", self.maximum_unknown_fraction),
            ("minimum_grid_coverage_fraction", self.minimum_grid_coverage_fraction),
        ):
            if not 0.0 <= float(value) <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]")


@dataclass(frozen=True)
class GridPathEvidence:
    cell_count: int
    free_count: int
    occupied_count: int
    unknown_count: int
    free_fraction: float
    occupied_fraction: float
    unknown_fraction: float
    grid_coverage_fraction: float


@dataclass(frozen=True)
class ForwardConnectorNavigationResult:
    connector_id: str
    from_aisle_id: str
    to_aisle_id: str
    turn_zone_id: str
    status: str
    path_type: str | None
    length_m: float | None
    minimum_turning_radius_m: float
    candidate_count: int
    samples: tuple[ForwardConnectorSample, ...]
    required_outward_extension_m: float
    required_inward_extension_m: float
    required_lateral_low_extension_m: float
    required_lateral_high_extension_m: float
    max_required_zone_extension_m: float
    centerline_evidence: GridPathEvidence
    footprint_evidence: GridPathEvidence
    reason: str = ""


@dataclass(frozen=True)
class ForwardConnectorNavigationPlan:
    frame_id: str
    platform_id: str
    platform_profile_sha256: str
    connectors: tuple[ForwardConnectorNavigationResult, ...]
    source: Mapping[str, Any] = field(default_factory=dict)
    schema: str = FORWARD_CONNECTOR_NAVIGATION_GATE_SCHEMA
    status: str = "DRAFT"

    @property
    def preview_free_count(self) -> int:
        return sum(item.status == "PREVIEW_FOOTPRINT_FREE" for item in self.connectors)

    @property
    def review_required_count(self) -> int:
        return len(self.connectors) - self.preview_free_count


def _empty_evidence() -> GridPathEvidence:
    return GridPathEvidence(
        cell_count=0,
        free_count=0,
        occupied_count=0,
        unknown_count=0,
        free_fraction=0.0,
        occupied_fraction=0.0,
        unknown_fraction=0.0,
        grid_coverage_fraction=0.0,
    )


def _evidence_from_mask(mask: np.ndarray, occupancy: np.ndarray, coverage: float) -> GridPathEvidence:
    count = int(np.count_nonzero(mask))
    if count <= 0:
        return GridPathEvidence(0, 0, 0, 0, 0.0, 0.0, 0.0, float(coverage))
    free = int(np.count_nonzero(mask & (occupancy == FREE)))
    occupied = int(np.count_nonzero(mask & (occupancy == OCCUPIED)))
    unknown = int(np.count_nonzero(mask & (occupancy == UNKNOWN)))
    return GridPathEvidence(
        cell_count=count,
        free_count=free,
        occupied_count=occupied,
        unknown_count=unknown,
        free_fraction=float(free / count),
        occupied_fraction=float(occupied / count),
        unknown_fraction=float(unknown / count),
        grid_coverage_fraction=float(coverage),
    )


def _preview_local_footprint(
    vehicle: CanonicalVehicleProfile,
    padding_m: float,
) -> tuple[tuple[float, float], ...]:
    """Return a conservative rectangle around canonical navigation footprint.

    MK-mini's canonical footprint is rectangular.  Using extents keeps the gate
    conservative for any future non-rectangular platform while avoiding a second
    vehicle geometry truth format.
    """
    points = np.asarray(vehicle.navigation_footprint_xy, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] < 3 or points.shape[1] != 2:
        raise ValueError("canonical navigation footprint is invalid")
    x0 = float(np.min(points[:, 0]) - padding_m)
    x1 = float(np.max(points[:, 0]) + padding_m)
    y0 = float(np.min(points[:, 1]) - padding_m)
    y1 = float(np.max(points[:, 1]) + padding_m)
    return ((x1, y1), (x1, y0), (x0, y0), (x0, y1))


def _transform_polygon(local_polygon, sample: ForwardConnectorSample):
    c = math.cos(float(sample.yaw))
    s = math.sin(float(sample.yaw))
    return tuple(
        (
            float(sample.x + c * lx - s * ly),
            float(sample.y + s * lx + c * ly),
        )
        for lx, ly in local_polygon
    )


def _cell_index(navigation: NavigationGridEvidence, x: float, y: float):
    col = int(math.floor((x - navigation.origin_x_m) / navigation.resolution_m))
    row = int(math.floor((y - navigation.origin_y_m) / navigation.resolution_m))
    if row < 0 or row >= navigation.height or col < 0 or col >= navigation.width:
        return None
    return row, col


def _evaluate_candidate(
    samples: tuple[ForwardConnectorSample, ...],
    navigation: NavigationGridEvidence,
    local_footprint,
) -> tuple[GridPathEvidence, GridPathEvidence]:
    occupancy = np.asarray(navigation.occupancy, dtype=np.uint8)
    center_mask = np.zeros(occupancy.shape, dtype=bool)
    swept_mask = np.zeros(occupancy.shape, dtype=bool)
    center_out = 0
    footprint_out = 0

    min_x, min_y, max_x, max_y = navigation.bounds_m()
    resolution = float(navigation.resolution_m)

    for sample in samples:
        index = _cell_index(navigation, float(sample.x), float(sample.y))
        if index is None:
            center_out += 1
        else:
            center_mask[index] = True

        polygon = _transform_polygon(local_footprint, sample)
        polygon_array = np.asarray(polygon, dtype=np.float64)
        px0, py0 = np.min(polygon_array, axis=0)
        px1, py1 = np.max(polygon_array, axis=0)
        if px0 < min_x or py0 < min_y or px1 > max_x or py1 > max_y:
            footprint_out += 1

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
            continue

        rows, cols = np.indices((row1 - row0 + 1, col1 - col0 + 1), dtype=np.float64)
        xx = navigation.origin_x_m + (cols + col0 + 0.5) * resolution
        yy = navigation.origin_y_m + (rows + row0 + 0.5) * resolution
        inside = _points_inside_polygon(xx, yy, polygon)
        swept_mask[row0 : row1 + 1, col0 : col1 + 1] |= inside

    sample_count = max(1, len(samples))
    center_coverage = 1.0 - center_out / sample_count
    footprint_coverage = 1.0 - footprint_out / sample_count
    return (
        _evidence_from_mask(center_mask, occupancy, center_coverage),
        _evidence_from_mask(swept_mask, occupancy, footprint_coverage),
    )


def _candidate_is_preview_free(
    evidence: GridPathEvidence,
    cfg: ForwardConnectorNavigationGateConfig,
) -> bool:
    return (
        evidence.cell_count > 0
        and evidence.grid_coverage_fraction >= cfg.minimum_grid_coverage_fraction
        and evidence.occupied_fraction <= cfg.maximum_occupied_fraction
        and evidence.unknown_fraction <= cfg.maximum_unknown_fraction
    )


def derive_forward_connector_navigation_gate(
    connector_requests: Iterable[ConnectorRequest],
    zones: TurnZoneSet,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    config: ForwardConnectorNavigationGateConfig | None = None,
    *,
    source: Mapping[str, Any] | None = None,
) -> ForwardConnectorNavigationPlan:
    """Evaluate forward Dubins candidates against frozen navigation evidence."""
    cfg = config or ForwardConnectorNavigationGateConfig()
    cfg.validate()
    requests = tuple(connector_requests)
    if vehicle.kinematics != "ackermann":
        raise ValueError("forward navigation gate currently requires Ackermann kinematics")
    if not vehicle.planning_preview_ready:
        raise ValueError(f"vehicle profile {vehicle.profile_id} is not ready for planning preview")
    radius = float(vehicle.minimum_turning_radius_m)
    if not vehicle.minimum_turning_radius_verified or radius <= 0.0:
        raise ValueError("forward navigation gate requires verified minimum turning radius")

    zone_map = _zone_by_id(zones)
    local_footprint = _preview_local_footprint(vehicle, cfg.preview_footprint_padding_m)
    forward_cfg = ForwardConnectorConfig(sample_step_m=cfg.sample_step_m)
    results: list[ForwardConnectorNavigationResult] = []

    for request in requests:
        zone = zone_map.get(request.turn_zone_id)
        if zone is None or zone.side != request.side:
            results.append(
                ForwardConnectorNavigationResult(
                    connector_id=request.connector_id,
                    from_aisle_id=request.from_aisle_id,
                    to_aisle_id=request.to_aisle_id,
                    turn_zone_id=request.turn_zone_id,
                    status="TURN_ZONE_METADATA_INVALID",
                    path_type=None,
                    length_m=None,
                    minimum_turning_radius_m=radius,
                    candidate_count=0,
                    samples=(),
                    required_outward_extension_m=0.0,
                    required_inward_extension_m=0.0,
                    required_lateral_low_extension_m=0.0,
                    required_lateral_high_extension_m=0.0,
                    max_required_zone_extension_m=0.0,
                    centerline_evidence=_empty_evidence(),
                    footprint_evidence=_empty_evidence(),
                    reason="requested Turn Zone is missing or side-mismatched",
                )
            )
            continue

        candidates = _dubins_candidates(request.start_pose, request.goal_pose, radius)
        scored = []
        for path_type, normalized_lengths in candidates:
            samples, length_m = _sample_candidate(
                request.start_pose,
                request.goal_pose,
                path_type,
                normalized_lengths,
                radius,
                forward_cfg.sample_step_m,
            )
            centerline, footprint = _evaluate_candidate(samples, navigation, local_footprint)
            outward, inward, low_v, high_v, maximum = _candidate_expansion(
                samples,
                zone,
                np.asarray(zones.row_direction_xy, dtype=np.float64)
                / np.linalg.norm(np.asarray(zones.row_direction_xy, dtype=np.float64)),
                np.array(
                    [
                        -zones.row_direction_xy[1],
                        zones.row_direction_xy[0],
                    ],
                    dtype=np.float64,
                )
                / np.linalg.norm(np.asarray(zones.row_direction_xy, dtype=np.float64)),
            )
            accepted = _candidate_is_preview_free(footprint, cfg)
            score = (
                0 if accepted else 1,
                1.0 - footprint.grid_coverage_fraction,
                footprint.occupied_fraction,
                footprint.unknown_fraction,
                maximum,
                float(length_m),
                path_type,
            )
            scored.append(
                (
                    score,
                    path_type,
                    float(length_m),
                    samples,
                    centerline,
                    footprint,
                    float(outward),
                    float(inward),
                    float(low_v),
                    float(high_v),
                    float(maximum),
                    accepted,
                )
            )

        if not scored:
            results.append(
                ForwardConnectorNavigationResult(
                    connector_id=request.connector_id,
                    from_aisle_id=request.from_aisle_id,
                    to_aisle_id=request.to_aisle_id,
                    turn_zone_id=request.turn_zone_id,
                    status="NO_FORWARD_DUBINS_CANDIDATE",
                    path_type=None,
                    length_m=None,
                    minimum_turning_radius_m=radius,
                    candidate_count=0,
                    samples=(),
                    required_outward_extension_m=0.0,
                    required_inward_extension_m=0.0,
                    required_lateral_low_extension_m=0.0,
                    required_lateral_high_extension_m=0.0,
                    max_required_zone_extension_m=0.0,
                    centerline_evidence=_empty_evidence(),
                    footprint_evidence=_empty_evidence(),
                    reason="no analytic forward-only Dubins candidate exists",
                )
            )
            continue

        scored.sort(key=lambda item: item[0])
        (
            _score,
            path_type,
            length_m,
            samples,
            centerline,
            footprint,
            outward,
            inward,
            low_v,
            high_v,
            maximum,
            accepted,
        ) = scored[0]
        status = "PREVIEW_FOOTPRINT_FREE" if accepted else "NO_FORWARD_PREVIEW_FREE_CANDIDATE"
        if accepted:
            reason = (
                "selected forward Dubins candidate is free in the frozen Navigation Grid "
                "under the preview navigation-footprint assumption"
            )
        else:
            reason = (
                "all forward Dubins candidates have occupied/unknown/out-of-grid preview footprint evidence; "
                "do not infer that enlarging the shared Turn Zone makes them drivable"
            )
        results.append(
            ForwardConnectorNavigationResult(
                connector_id=request.connector_id,
                from_aisle_id=request.from_aisle_id,
                to_aisle_id=request.to_aisle_id,
                turn_zone_id=request.turn_zone_id,
                status=status,
                path_type=path_type,
                length_m=length_m,
                minimum_turning_radius_m=radius,
                candidate_count=len(candidates),
                samples=samples,
                required_outward_extension_m=outward,
                required_inward_extension_m=inward,
                required_lateral_low_extension_m=low_v,
                required_lateral_high_extension_m=high_v,
                max_required_zone_extension_m=maximum,
                centerline_evidence=centerline,
                footprint_evidence=footprint,
                reason=reason,
            )
        )

    merged_source = dict(zones.source)
    merged_source.update(dict(navigation.source))
    merged_source.update(dict(source or {}))
    merged_source.update(
        {
            "backend": "ANALYTIC_DUBINS_WITH_NAVIGATION_GRID_PREVIEW_GATE",
            "preview_footprint_padding_m": cfg.preview_footprint_padding_m,
            "footprint_semantics": "CANONICAL_NAVIGATION_FOOTPRINT_BOUNDING_RECTANGLE_PREVIEW",
            "validation_scope": "PREVIEW_ONLY_NOT_R8_VEHICLE_READY",
        }
    )
    return ForwardConnectorNavigationPlan(
        frame_id=zones.frame_id,
        platform_id=vehicle.profile_id,
        platform_profile_sha256=vehicle.profile_sha256,
        connectors=tuple(results),
        source=merged_source,
    )


def _evidence_to_dict(item: GridPathEvidence) -> dict[str, Any]:
    return {
        "cell_count": item.cell_count,
        "free_count": item.free_count,
        "occupied_count": item.occupied_count,
        "unknown_count": item.unknown_count,
        "free_fraction": item.free_fraction,
        "occupied_fraction": item.occupied_fraction,
        "unknown_fraction": item.unknown_fraction,
        "grid_coverage_fraction": item.grid_coverage_fraction,
    }


def forward_connector_navigation_gate_to_dict(
    plan: ForwardConnectorNavigationPlan,
) -> dict[str, Any]:
    return {
        "schema": plan.schema,
        "status": plan.status,
        "frame_id": plan.frame_id,
        "platform_id": plan.platform_id,
        "platform_profile_sha256": plan.platform_profile_sha256,
        "source": dict(plan.source),
        "connector_count": len(plan.connectors),
        "preview_free_count": plan.preview_free_count,
        "review_required_count": plan.review_required_count,
        "connectors": [
            {
                "connector_id": item.connector_id,
                "from_aisle_id": item.from_aisle_id,
                "to_aisle_id": item.to_aisle_id,
                "turn_zone_id": item.turn_zone_id,
                "status": item.status,
                "path_type": item.path_type,
                "length_m": item.length_m,
                "minimum_turning_radius_m": item.minimum_turning_radius_m,
                "candidate_count": item.candidate_count,
                "required_zone_extension": {
                    "outward_m": item.required_outward_extension_m,
                    "inward_m": item.required_inward_extension_m,
                    "lateral_low_m": item.required_lateral_low_extension_m,
                    "lateral_high_m": item.required_lateral_high_extension_m,
                    "max_m": item.max_required_zone_extension_m,
                },
                "centerline_evidence": _evidence_to_dict(item.centerline_evidence),
                "preview_footprint_evidence": _evidence_to_dict(item.footprint_evidence),
                "samples": [
                    {
                        "x": sample.x,
                        "y": sample.y,
                        "z": sample.z,
                        "yaw": sample.yaw,
                        "motion_direction": sample.motion_direction,
                    }
                    for sample in item.samples
                ],
                "reason": item.reason,
                "validation_scope": "PREVIEW_ONLY_NOT_R8_VEHICLE_READY",
            }
            for item in plan.connectors
        ],
    }


def write_forward_connector_navigation_gate(
    plan: ForwardConnectorNavigationPlan,
    path: str | Path,
) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(
            forward_connector_navigation_gate_to_dict(plan),
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )
    return output
