"""Audit every forward Dubins candidate after the R5.5 Navigation Grid gate.

R5.5 correctly answers whether *any* forward candidate is preview-footprint free,
but a single representative rejected candidate can be misleading when all
candidates fail: minimizing OCCUPIED fraction can prefer a large loop through
UNKNOWN over the locally relevant headland candidate.

This module keeps all six Dubins families visible, sorts them by Turn-Zone
extension before path length, and classifies the locally relevant candidate set
without promoting any preview result to R8 vehicle-READY truth.
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
from .forward_connector_navigation_gate import (
    ForwardConnectorNavigationGateConfig,
    GridPathEvidence,
    _candidate_inside_site_boundary,
    _candidate_is_preview_free,
    _evaluate_candidate,
    _preview_local_footprint,
)
from .navigation_grid import NavigationGridEvidence
from .site_boundary import SiteBoundary
from .turn_zones import TurnZoneSet
from .vehicle_profile import CanonicalVehicleProfile


FORWARD_CONNECTOR_CANDIDATE_AUDIT_SCHEMA = "agt_forward_connector_candidate_audit/v1"


@dataclass(frozen=True)
class ForwardConnectorCandidateAuditConfig:
    sample_step_m: float = 0.05
    preview_footprint_padding_m: float = 0.05
    local_zone_extension_slack_m: float = 0.25
    known_map_max_unknown_fraction: float = 0.02
    known_map_min_grid_coverage_fraction: float = 1.0

    def validate(self) -> None:
        if not math.isfinite(self.sample_step_m) or self.sample_step_m <= 0.0:
            raise ValueError("sample_step_m must be finite and > 0")
        if self.preview_footprint_padding_m < 0.0:
            raise ValueError("preview_footprint_padding_m must be >= 0")
        if self.local_zone_extension_slack_m < 0.0:
            raise ValueError("local_zone_extension_slack_m must be >= 0")
        if not 0.0 <= self.known_map_max_unknown_fraction <= 1.0:
            raise ValueError("known_map_max_unknown_fraction must be in [0, 1]")
        if not 0.0 <= self.known_map_min_grid_coverage_fraction <= 1.0:
            raise ValueError("known_map_min_grid_coverage_fraction must be in [0, 1]")


@dataclass(frozen=True)
class ForwardCandidateAuditItem:
    path_type: str
    length_m: float
    required_outward_extension_m: float
    required_inward_extension_m: float
    required_lateral_low_extension_m: float
    required_lateral_high_extension_m: float
    max_required_zone_extension_m: float
    centerline_evidence: GridPathEvidence
    footprint_evidence: GridPathEvidence
    preview_footprint_free: bool
    local_headland_candidate: bool
    site_boundary_free: bool = True


@dataclass(frozen=True)
class ForwardConnectorCandidateAuditResult:
    connector_id: str
    from_aisle_id: str
    to_aisle_id: str
    turn_zone_id: str
    status: str
    minimum_zone_extension_m: float
    local_zone_extension_limit_m: float
    candidate_count: int
    local_candidate_count: int
    local_known_occupied_count: int
    local_map_insufficient_count: int
    candidates: tuple[ForwardCandidateAuditItem, ...]
    reason: str
    start_endpoint_site_boundary_free: bool = True
    goal_endpoint_site_boundary_free: bool = True


@dataclass(frozen=True)
class ForwardConnectorCandidateAuditPlan:
    frame_id: str
    platform_id: str
    platform_profile_sha256: str
    connectors: tuple[ForwardConnectorCandidateAuditResult, ...]
    source: Mapping[str, Any] = field(default_factory=dict)
    schema: str = FORWARD_CONNECTOR_CANDIDATE_AUDIT_SCHEMA
    status: str = "DRAFT"

    @property
    def forward_preview_free_count(self) -> int:
        return sum(item.status == "FORWARD_PREVIEW_FREE" for item in self.connectors)

    @property
    def local_occupancy_blocked_count(self) -> int:
        return sum(item.status == "LOCAL_FORWARD_OCCUPANCY_BLOCKED" for item in self.connectors)

    @property
    def local_map_insufficient_count(self) -> int:
        return sum(item.status == "LOCAL_FORWARD_MAP_EVIDENCE_INSUFFICIENT" for item in self.connectors)

    @property
    def local_mixed_evidence_count(self) -> int:
        return sum(item.status == "LOCAL_FORWARD_MIXED_EVIDENCE" for item in self.connectors)


def _known_map(evidence: GridPathEvidence, cfg: ForwardConnectorCandidateAuditConfig) -> bool:
    return (
        evidence.grid_coverage_fraction >= cfg.known_map_min_grid_coverage_fraction
        and evidence.unknown_fraction <= cfg.known_map_max_unknown_fraction
    )


def _endpoint_inside_site_boundary(pose, local_footprint, site_boundary) -> bool:
    if site_boundary is None:
        return True
    sample = ForwardConnectorSample(
        x=float(pose[0]),
        y=float(pose[1]),
        z=float(pose[2]),
        yaw=float(pose[3]),
    )
    return _candidate_inside_site_boundary(
        (sample,),
        local_footprint,
        site_boundary,
    )


def derive_forward_connector_candidate_audit(
    connector_requests: Iterable[ConnectorRequest],
    zones: TurnZoneSet,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    config: ForwardConnectorCandidateAuditConfig | None = None,
    *,
    site_boundary: SiteBoundary | None = None,
    source: Mapping[str, Any] | None = None,
) -> ForwardConnectorCandidateAuditPlan:
    """Expose all forward candidates and classify the locally relevant set."""
    cfg = config or ForwardConnectorCandidateAuditConfig()
    cfg.validate()
    requests = tuple(connector_requests)
    if vehicle.kinematics != "ackermann":
        raise ValueError("forward candidate audit currently requires Ackermann kinematics")
    if not vehicle.planning_preview_ready:
        raise ValueError(f"vehicle profile {vehicle.profile_id} is not ready for planning preview")
    if site_boundary is not None:
        site_boundary.validate(expected_frame_id=zones.frame_id)
    radius = float(vehicle.minimum_turning_radius_m)
    if not vehicle.minimum_turning_radius_verified or radius <= 0.0:
        raise ValueError("forward candidate audit requires verified minimum turning radius")

    zone_map = _zone_by_id(zones)
    local_footprint = _preview_local_footprint(vehicle, cfg.preview_footprint_padding_m)
    forward_cfg = ForwardConnectorConfig(sample_step_m=cfg.sample_step_m)
    gate_cfg = ForwardConnectorNavigationGateConfig(
        sample_step_m=cfg.sample_step_m,
        preview_footprint_padding_m=cfg.preview_footprint_padding_m,
        maximum_occupied_fraction=0.0,
        maximum_unknown_fraction=0.0,
        minimum_grid_coverage_fraction=1.0,
    )
    direction = np.asarray(zones.row_direction_xy, dtype=np.float64)
    direction /= np.linalg.norm(direction)
    perpendicular = np.array([-direction[1], direction[0]], dtype=np.float64)

    results: list[ForwardConnectorCandidateAuditResult] = []
    for request in requests:
        zone = zone_map.get(request.turn_zone_id)
        if zone is None or zone.side != request.side:
            results.append(
                ForwardConnectorCandidateAuditResult(
                    connector_id=request.connector_id,
                    from_aisle_id=request.from_aisle_id,
                    to_aisle_id=request.to_aisle_id,
                    turn_zone_id=request.turn_zone_id,
                    status="TURN_ZONE_METADATA_INVALID",
                    minimum_zone_extension_m=0.0,
                    local_zone_extension_limit_m=0.0,
                    candidate_count=0,
                    local_candidate_count=0,
                    local_known_occupied_count=0,
                    local_map_insufficient_count=0,
                    candidates=(),
                    reason="requested Turn Zone is missing or side-mismatched",
                )
            )
            continue

        start_boundary_free = _endpoint_inside_site_boundary(
            request.start_pose,
            local_footprint,
            site_boundary,
        )
        goal_boundary_free = _endpoint_inside_site_boundary(
            request.goal_pose,
            local_footprint,
            site_boundary,
        )
        if site_boundary is not None and not (
            start_boundary_free and goal_boundary_free
        ):
            results.append(
                ForwardConnectorCandidateAuditResult(
                    connector_id=request.connector_id,
                    from_aisle_id=request.from_aisle_id,
                    to_aisle_id=request.to_aisle_id,
                    turn_zone_id=request.turn_zone_id,
                    status="CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT",
                    minimum_zone_extension_m=0.0,
                    local_zone_extension_limit_m=0.0,
                    candidate_count=0,
                    local_candidate_count=0,
                    local_known_occupied_count=0,
                    local_map_insufficient_count=0,
                    candidates=(),
                    reason=(
                        "connector start or goal footprint touches or crosses "
                        "the hard Site Boundary"
                    ),
                    start_endpoint_site_boundary_free=start_boundary_free,
                    goal_endpoint_site_boundary_free=goal_boundary_free,
                )
            )
            continue

        raw_items = []
        for path_type, normalized_lengths in _dubins_candidates(
            request.start_pose, request.goal_pose, radius
        ):
            samples, length_m = _sample_candidate(
                request.start_pose,
                request.goal_pose,
                path_type,
                normalized_lengths,
                radius,
                forward_cfg.sample_step_m,
            )
            centerline, footprint = _evaluate_candidate(
                samples,
                navigation,
                local_footprint,
            )
            boundary_free = _candidate_inside_site_boundary(
                samples,
                local_footprint,
                site_boundary,
            )
            outward, inward, low_v, high_v, maximum = _candidate_expansion(
                samples, zone, direction, perpendicular
            )
            raw_items.append(
                (
                    float(maximum),
                    float(length_m),
                    path_type,
                    float(outward),
                    float(inward),
                    float(low_v),
                    float(high_v),
                    centerline,
                    footprint,
                    _candidate_is_preview_free(footprint, gate_cfg) and boundary_free,
                    boundary_free,
                )
            )

        raw_items.sort(key=lambda item: (item[0], item[1], item[2]))
        if not raw_items:
            results.append(
                ForwardConnectorCandidateAuditResult(
                    connector_id=request.connector_id,
                    from_aisle_id=request.from_aisle_id,
                    to_aisle_id=request.to_aisle_id,
                    turn_zone_id=request.turn_zone_id,
                    status="NO_FORWARD_DUBINS_CANDIDATE",
                    minimum_zone_extension_m=0.0,
                    local_zone_extension_limit_m=0.0,
                    candidate_count=0,
                    local_candidate_count=0,
                    local_known_occupied_count=0,
                    local_map_insufficient_count=0,
                    candidates=(),
                    reason="no analytic forward-only Dubins candidate exists",
                    start_endpoint_site_boundary_free=start_boundary_free,
                    goal_endpoint_site_boundary_free=goal_boundary_free,
                )
            )
            continue

        minimum_extension = raw_items[0][0]
        local_limit = minimum_extension + cfg.local_zone_extension_slack_m
        items: list[ForwardCandidateAuditItem] = []
        for (
            maximum,
            length_m,
            path_type,
            outward,
            inward,
            low_v,
            high_v,
            centerline,
            footprint,
            preview_free,
            boundary_free,
        ) in raw_items:
            items.append(
                ForwardCandidateAuditItem(
                    path_type=path_type,
                    length_m=length_m,
                    required_outward_extension_m=outward,
                    required_inward_extension_m=inward,
                    required_lateral_low_extension_m=low_v,
                    required_lateral_high_extension_m=high_v,
                    max_required_zone_extension_m=maximum,
                    centerline_evidence=centerline,
                    footprint_evidence=footprint,
                    preview_footprint_free=preview_free,
                    local_headland_candidate=maximum <= local_limit + 1.0e-12,
                    site_boundary_free=boundary_free,
                )
            )

        local = [item for item in items if item.local_headland_candidate]
        local_safe = [item for item in local if item.site_boundary_free]
        local_known = [
            item for item in local_safe if _known_map(item.footprint_evidence, cfg)
        ]
        local_known_occupied = [
            item
            for item in local_known
            if item.footprint_evidence.occupied_fraction > 0.0
        ]
        local_map_insufficient = [
            item for item in local_safe if not _known_map(item.footprint_evidence, cfg)
        ]

        if any(item.preview_footprint_free for item in items):
            status = "FORWARD_PREVIEW_FREE"
            reason = "at least one forward Dubins candidate is preview-footprint free"
        elif site_boundary is not None and local and not local_safe:
            status = "LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT"
            reason = (
                "all locally relevant forward candidates touch or cross the hard "
                "Site Boundary while connector endpoints remain legal"
            )
        elif local_safe and len(local_known_occupied) == len(local_safe):
            status = "LOCAL_FORWARD_OCCUPANCY_BLOCKED"
            reason = (
                "all boundary-safe locally relevant forward candidates are "
                "sufficiently mapped and intersect OCCUPIED cells"
            )
        elif local_known_occupied and local_map_insufficient:
            status = "LOCAL_FORWARD_MIXED_EVIDENCE"
            reason = (
                "boundary-safe locally relevant candidates include known OCCUPIED "
                "conflicts and insufficient-map alternatives"
            )
        elif local_map_insufficient:
            status = "LOCAL_FORWARD_MAP_EVIDENCE_INSUFFICIENT"
            reason = (
                "local forward candidates require UNKNOWN or incompletely covered "
                "map evidence"
            )
        else:
            status = "LOCAL_FORWARD_POLICY_REVIEW"
            reason = (
                "local forward candidates are rejected but do not fit a stronger "
                "diagnostic class"
            )

        results.append(
            ForwardConnectorCandidateAuditResult(
                connector_id=request.connector_id,
                from_aisle_id=request.from_aisle_id,
                to_aisle_id=request.to_aisle_id,
                turn_zone_id=request.turn_zone_id,
                status=status,
                minimum_zone_extension_m=minimum_extension,
                local_zone_extension_limit_m=local_limit,
                candidate_count=len(items),
                local_candidate_count=len(local),
                local_known_occupied_count=len(local_known_occupied),
                local_map_insufficient_count=len(local_map_insufficient),
                candidates=tuple(items),
                reason=reason,
                start_endpoint_site_boundary_free=start_boundary_free,
                goal_endpoint_site_boundary_free=goal_boundary_free,
            )
        )

    merged_source = dict(zones.source)
    merged_source.update(dict(navigation.source))
    merged_source.update(dict(source or {}))
    merged_source.update(
        {
            "audit_scope": "ALL_DUBINS_CANDIDATES_LOCAL_HEADLAND_CLASSIFICATION",
            "validation_scope": "PREVIEW_ONLY_NOT_R8_VEHICLE_READY",
            "minimum_turning_radius_m": radius,
            "local_zone_extension_slack_m": cfg.local_zone_extension_slack_m,
            "site_boundary_enforced": site_boundary is not None,
        }
    )
    return ForwardConnectorCandidateAuditPlan(
        frame_id=zones.frame_id,
        platform_id=vehicle.profile_id,
        platform_profile_sha256=vehicle.profile_sha256,
        connectors=tuple(results),
        source=merged_source,
    )


def _evidence_dict(evidence: GridPathEvidence) -> dict[str, Any]:
    return {
        "cell_count": evidence.cell_count,
        "free_count": evidence.free_count,
        "occupied_count": evidence.occupied_count,
        "unknown_count": evidence.unknown_count,
        "free_fraction": evidence.free_fraction,
        "occupied_fraction": evidence.occupied_fraction,
        "unknown_fraction": evidence.unknown_fraction,
        "grid_coverage_fraction": evidence.grid_coverage_fraction,
    }


def forward_connector_candidate_audit_to_dict(
    plan: ForwardConnectorCandidateAuditPlan,
) -> dict[str, Any]:
    return {
        "schema": plan.schema,
        "status": plan.status,
        "frame_id": plan.frame_id,
        "platform_id": plan.platform_id,
        "platform_profile_sha256": plan.platform_profile_sha256,
        "source": dict(plan.source),
        "connector_count": len(plan.connectors),
        "summary": {
            "forward_preview_free": plan.forward_preview_free_count,
            "local_occupancy_blocked": plan.local_occupancy_blocked_count,
            "local_map_insufficient": plan.local_map_insufficient_count,
            "local_mixed_evidence": plan.local_mixed_evidence_count,
        },
        "connectors": [
            {
                "connector_id": result.connector_id,
                "from_aisle_id": result.from_aisle_id,
                "to_aisle_id": result.to_aisle_id,
                "turn_zone_id": result.turn_zone_id,
                "status": result.status,
                "reason": result.reason,
                "minimum_zone_extension_m": result.minimum_zone_extension_m,
                "local_zone_extension_limit_m": result.local_zone_extension_limit_m,
                "candidate_count": result.candidate_count,
                "local_candidate_count": result.local_candidate_count,
                "local_known_occupied_count": result.local_known_occupied_count,
                "local_map_insufficient_count": result.local_map_insufficient_count,
                "start_endpoint_site_boundary_free": (
                    result.start_endpoint_site_boundary_free
                ),
                "goal_endpoint_site_boundary_free": (
                    result.goal_endpoint_site_boundary_free
                ),
                "validation_scope": "PREVIEW_ONLY_NOT_R8_VEHICLE_READY",
                "candidates": [
                    {
                        "path_type": item.path_type,
                        "length_m": item.length_m,
                        "local_headland_candidate": item.local_headland_candidate,
                        "preview_footprint_free": item.preview_footprint_free,
                        "site_boundary_free": item.site_boundary_free,
                        "required_zone_extension": {
                            "outward_m": item.required_outward_extension_m,
                            "inward_m": item.required_inward_extension_m,
                            "lateral_low_m": item.required_lateral_low_extension_m,
                            "lateral_high_m": item.required_lateral_high_extension_m,
                            "max_m": item.max_required_zone_extension_m,
                        },
                        "centerline_evidence": _evidence_dict(item.centerline_evidence),
                        "preview_footprint_evidence": _evidence_dict(
                            item.footprint_evidence
                        ),
                    }
                    for item in result.candidates
                ],
            }
            for result in plan.connectors
        ],
    }


def write_forward_connector_candidate_audit(
    plan: ForwardConnectorCandidateAuditPlan,
    path: str | Path,
) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(
            forward_connector_candidate_audit_to_dict(plan),
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )
    return output