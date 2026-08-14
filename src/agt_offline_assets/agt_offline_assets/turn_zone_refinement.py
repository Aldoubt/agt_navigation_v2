"""Propose evidence-gated Turn Zone expansion after R5 zone-fit diagnostics.

This module does not mutate the frozen R2 Turn Zone asset.  It converts the
per-connector row-frame deficits into one proposal per shared LOW_U/HIGH_U zone
and, when a frozen Navigation Grid is supplied, measures FREE/OCCUPIED/UNKNOWN
evidence specifically in the newly added area.

A proposal remains DRAFT evidence.  It is not permission to drive and cannot
promote a Route Asset to READY.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping
import math

import numpy as np
import yaml

from .forward_connector_diagnostics import ForwardConnectorZoneFitReport
from .navigation_grid import NavigationGridEvidence
from .navigation_map_derivation import FREE, OCCUPIED, UNKNOWN
from .turn_zones import TurnZone, TurnZoneSet


TURN_ZONE_REFINEMENT_SCHEMA = "agt_turn_zone_refinement_proposal/v1"


@dataclass(frozen=True)
class TurnZoneRefinementConfig:
    expansion_margin_m: float = 0.10
    minimum_added_free_fraction: float = 0.90
    maximum_added_occupied_fraction: float = 0.02
    maximum_added_unknown_fraction: float = 0.10
    minimum_grid_coverage_fraction: float = 0.85

    def validate(self) -> None:
        if self.expansion_margin_m < 0.0:
            raise ValueError("expansion_margin_m must be >= 0")
        for name, value in (
            ("minimum_added_free_fraction", self.minimum_added_free_fraction),
            ("maximum_added_occupied_fraction", self.maximum_added_occupied_fraction),
            ("maximum_added_unknown_fraction", self.maximum_added_unknown_fraction),
            ("minimum_grid_coverage_fraction", self.minimum_grid_coverage_fraction),
        ):
            if not 0.0 <= float(value) <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]")


@dataclass(frozen=True)
class TurnZoneExpansionEvidence:
    grid_available: bool
    added_cell_count: int
    added_free_count: int
    added_occupied_count: int
    added_unknown_count: int
    added_free_fraction: float
    added_occupied_fraction: float
    added_unknown_fraction: float
    estimated_grid_coverage_fraction: float


@dataclass(frozen=True)
class TurnZoneRefinementProposal:
    zone_id: str
    side: str
    status: str
    current_polygon_xy: tuple[tuple[float, float], ...]
    proposed_polygon_xy: tuple[tuple[float, float], ...]
    outward_extension_delta_m: float
    inward_extension_delta_m: float
    lateral_low_extension_delta_m: float
    lateral_high_extension_delta_m: float
    driving_outward_connector_id: str | None
    driving_inward_connector_id: str | None
    driving_lateral_low_connector_id: str | None
    driving_lateral_high_connector_id: str | None
    evidence: TurnZoneExpansionEvidence
    reason: str


@dataclass(frozen=True)
class TurnZoneRefinementPlan:
    frame_id: str
    row_direction_xy: tuple[float, float]
    proposals: tuple[TurnZoneRefinementProposal, ...]
    source: Mapping[str, Any] = field(default_factory=dict)
    schema: str = TURN_ZONE_REFINEMENT_SCHEMA
    status: str = "DRAFT"

    @property
    def evidence_supported_count(self) -> int:
        return sum(item.status == "EVIDENCE_SUPPORTS_EXPANSION" for item in self.proposals)

    @property
    def review_required_count(self) -> int:
        return len(self.proposals) - self.evidence_supported_count


def _normalize(direction_xy) -> np.ndarray:
    direction = np.asarray(direction_xy, dtype=np.float64).reshape(2)
    norm = float(np.linalg.norm(direction))
    if norm <= 1.0e-12:
        raise ValueError("row direction must be non-zero")
    return direction / norm


def _row_bounds(points_xy, direction: np.ndarray, perpendicular: np.ndarray):
    points = np.asarray(points_xy, dtype=np.float64)
    uu = points @ direction
    vv = points @ perpendicular
    return float(np.min(uu)), float(np.max(uu)), float(np.min(vv)), float(np.max(vv))


def _to_world_polygon(direction, perpendicular, u0, u1, v0, v1):
    return tuple(
        (
            float(u * direction[0] + v * perpendicular[0]),
            float(u * direction[1] + v * perpendicular[1]),
        )
        for u, v in ((u0, v0), (u1, v0), (u1, v1), (u0, v1))
    )


def _polygon_area(polygon_xy) -> float:
    points = np.asarray(polygon_xy, dtype=np.float64)
    x = points[:, 0]
    y = points[:, 1]
    return float(0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def _inside_polygon(xx: np.ndarray, yy: np.ndarray, polygon_xy) -> np.ndarray:
    polygon = np.asarray(polygon_xy, dtype=np.float64)
    x = xx.ravel()
    y = yy.ravel()
    inside = np.zeros(x.shape, dtype=bool)
    j = polygon.shape[0] - 1
    for i in range(polygon.shape[0]):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        crossing = ((yi > y) != (yj > y)) & (
            x < (xj - xi) * (y - yi) / (yj - yi + 1.0e-15) + xi
        )
        inside ^= crossing
        j = i
    return inside.reshape(xx.shape)


def _grid_evidence(
    navigation: NavigationGridEvidence | None,
    current_polygon,
    proposed_polygon,
) -> TurnZoneExpansionEvidence:
    if navigation is None:
        nan = float("nan")
        return TurnZoneExpansionEvidence(
            grid_available=False,
            added_cell_count=0,
            added_free_count=0,
            added_occupied_count=0,
            added_unknown_count=0,
            added_free_fraction=nan,
            added_occupied_fraction=nan,
            added_unknown_fraction=nan,
            estimated_grid_coverage_fraction=nan,
        )

    rows, cols = np.indices((navigation.height, navigation.width), dtype=np.float64)
    xx = navigation.origin_x_m + (cols + 0.5) * navigation.resolution_m
    yy = navigation.origin_y_m + (rows + 0.5) * navigation.resolution_m
    current = _inside_polygon(xx, yy, current_polygon)
    proposed = _inside_polygon(xx, yy, proposed_polygon)
    added = proposed & ~current
    count = int(np.count_nonzero(added))
    occupancy = np.asarray(navigation.occupancy, dtype=np.uint8)
    free_count = int(np.count_nonzero(added & (occupancy == FREE)))
    occupied_count = int(np.count_nonzero(added & (occupancy == OCCUPIED)))
    unknown_count = int(np.count_nonzero(added & (occupancy == UNKNOWN)))

    if count > 0:
        free_fraction = free_count / count
        occupied_fraction = occupied_count / count
        unknown_fraction = unknown_count / count
    else:
        free_fraction = occupied_fraction = unknown_fraction = 0.0

    added_area = max(0.0, _polygon_area(proposed_polygon) - _polygon_area(current_polygon))
    represented_area = count * navigation.resolution_m * navigation.resolution_m
    coverage = 1.0 if added_area <= 1.0e-12 else min(1.0, represented_area / added_area)
    return TurnZoneExpansionEvidence(
        grid_available=True,
        added_cell_count=count,
        added_free_count=free_count,
        added_occupied_count=occupied_count,
        added_unknown_count=unknown_count,
        added_free_fraction=float(free_fraction),
        added_occupied_fraction=float(occupied_fraction),
        added_unknown_fraction=float(unknown_fraction),
        estimated_grid_coverage_fraction=float(coverage),
    )


def _max_driver(items, attribute: str):
    if not items:
        return 0.0, None
    selected = max(items, key=lambda item: (float(getattr(item, attribute)), item.connector_id))
    return float(getattr(selected, attribute)), selected.connector_id


def derive_turn_zone_refinement_proposal(
    zones: TurnZoneSet,
    zone_fit: ForwardConnectorZoneFitReport,
    navigation: NavigationGridEvidence | None = None,
    config: TurnZoneRefinementConfig | None = None,
    *,
    source: Mapping[str, Any] | None = None,
) -> TurnZoneRefinementPlan:
    """Aggregate connector deficits into reviewable LOW_U/HIGH_U zone proposals."""
    cfg = config or TurnZoneRefinementConfig()
    cfg.validate()
    if zones.frame_id != zone_fit.frame_id:
        raise ValueError("Turn Zone and zone-fit frame_id mismatch")

    direction = _normalize(zones.row_direction_xy)
    perpendicular = np.array([-direction[1], direction[0]], dtype=np.float64)
    proposals: list[TurnZoneRefinementProposal] = []

    for zone in zones.zones:
        diagnostics = [
            item
            for item in zone_fit.diagnostics
            if item.turn_zone_id == zone.zone_id
            and item.status in {"ZONE_EXPANSION_REQUIRED", "FITS_CURRENT_ZONE"}
        ]
        outward, outward_driver = _max_driver(diagnostics, "required_outward_extension_m")
        inward, inward_driver = _max_driver(diagnostics, "required_inward_extension_m")
        low_v, low_v_driver = _max_driver(diagnostics, "required_lateral_low_extension_m")
        high_v, high_v_driver = _max_driver(diagnostics, "required_lateral_high_extension_m")

        # Add one explicit discretization / operator margin only to dimensions that
        # actually need expansion.  Zero-deficit dimensions remain unchanged.
        margin = float(cfg.expansion_margin_m)
        outward_delta = outward + margin if outward > 0.0 else 0.0
        inward_delta = inward + margin if inward > 0.0 else 0.0
        low_v_delta = low_v + margin if low_v > 0.0 else 0.0
        high_v_delta = high_v + margin if high_v > 0.0 else 0.0

        u0, u1, v0, v1 = _row_bounds(zone.polygon_xy, direction, perpendicular)
        if zone.side == "LOW_U":
            proposed_u0 = u0 - outward_delta
            proposed_u1 = u1 + inward_delta
        elif zone.side == "HIGH_U":
            proposed_u0 = u0 - inward_delta
            proposed_u1 = u1 + outward_delta
        else:
            raise ValueError(f"unsupported Turn Zone side: {zone.side}")
        proposed_v0 = v0 - low_v_delta
        proposed_v1 = v1 + high_v_delta
        proposed_polygon = _to_world_polygon(
            direction,
            perpendicular,
            proposed_u0,
            proposed_u1,
            proposed_v0,
            proposed_v1,
        )
        evidence = _grid_evidence(navigation, zone.polygon_xy, proposed_polygon)

        any_expansion = max(outward_delta, inward_delta, low_v_delta, high_v_delta) > 0.0
        if not any_expansion:
            status = "NO_EXPANSION_REQUIRED"
            reason = "all relevant forward candidates already fit the current Turn Zone envelope"
        elif not evidence.grid_available:
            status = "NAVIGATION_EVIDENCE_REQUIRED"
            reason = "geometry proposes expansion, but frozen Navigation Grid evidence was not supplied"
        elif evidence.added_cell_count == 0 or evidence.estimated_grid_coverage_fraction < cfg.minimum_grid_coverage_fraction:
            status = "REVIEW_REQUIRED_OUT_OF_GRID"
            reason = "proposed expansion is insufficiently covered by the frozen Navigation Grid"
        elif (
            evidence.added_free_fraction >= cfg.minimum_added_free_fraction
            and evidence.added_occupied_fraction <= cfg.maximum_added_occupied_fraction
            and evidence.added_unknown_fraction <= cfg.maximum_added_unknown_fraction
        ):
            status = "EVIDENCE_SUPPORTS_EXPANSION"
            reason = "newly added Turn Zone area is sufficiently supported by FREE Navigation Grid evidence"
        else:
            status = "REVIEW_REQUIRED_NAVIGATION_CONFLICT"
            reason = "newly added Turn Zone area contains too much OCCUPIED or UNKNOWN evidence"

        proposals.append(
            TurnZoneRefinementProposal(
                zone_id=zone.zone_id,
                side=zone.side,
                status=status,
                current_polygon_xy=zone.polygon_xy,
                proposed_polygon_xy=proposed_polygon,
                outward_extension_delta_m=float(outward_delta),
                inward_extension_delta_m=float(inward_delta),
                lateral_low_extension_delta_m=float(low_v_delta),
                lateral_high_extension_delta_m=float(high_v_delta),
                driving_outward_connector_id=outward_driver,
                driving_inward_connector_id=inward_driver,
                driving_lateral_low_connector_id=low_v_driver,
                driving_lateral_high_connector_id=high_v_driver,
                evidence=evidence,
                reason=reason,
            )
        )

    merged_source = dict(zones.source)
    merged_source.update(dict(zone_fit.source))
    if navigation is not None:
        merged_source.update(dict(navigation.source))
    merged_source.update(dict(source or {}))
    merged_source.update(
        {
            "proposal_scope": "GEOMETRY_PLUS_FROZEN_NAVIGATION_GRID_EVIDENCE_NOT_DRIVE_PERMISSION",
            "expansion_margin_m": float(cfg.expansion_margin_m),
        }
    )
    return TurnZoneRefinementPlan(
        frame_id=zones.frame_id,
        row_direction_xy=zones.row_direction_xy,
        proposals=tuple(proposals),
        source=merged_source,
    )


def turn_zone_refinement_to_dict(plan: TurnZoneRefinementPlan) -> dict[str, Any]:
    def finite(value: float):
        return None if not math.isfinite(float(value)) else float(value)

    return {
        "schema": plan.schema,
        "status": plan.status,
        "frame_id": plan.frame_id,
        "row_direction_xy": list(plan.row_direction_xy),
        "source": dict(plan.source),
        "proposal_count": len(plan.proposals),
        "evidence_supported_count": plan.evidence_supported_count,
        "review_required_count": plan.review_required_count,
        "proposals": [
            {
                "zone_id": item.zone_id,
                "side": item.side,
                "status": item.status,
                "current_polygon_xy": [list(point) for point in item.current_polygon_xy],
                "proposed_polygon_xy": [list(point) for point in item.proposed_polygon_xy],
                "expansion_delta_m": {
                    "outward": item.outward_extension_delta_m,
                    "inward": item.inward_extension_delta_m,
                    "lateral_low": item.lateral_low_extension_delta_m,
                    "lateral_high": item.lateral_high_extension_delta_m,
                },
                "driving_connectors": {
                    "outward": item.driving_outward_connector_id,
                    "inward": item.driving_inward_connector_id,
                    "lateral_low": item.driving_lateral_low_connector_id,
                    "lateral_high": item.driving_lateral_high_connector_id,
                },
                "navigation_evidence": {
                    "grid_available": item.evidence.grid_available,
                    "added_cell_count": item.evidence.added_cell_count,
                    "free_count": item.evidence.added_free_count,
                    "occupied_count": item.evidence.added_occupied_count,
                    "unknown_count": item.evidence.added_unknown_count,
                    "free_fraction": finite(item.evidence.added_free_fraction),
                    "occupied_fraction": finite(item.evidence.added_occupied_fraction),
                    "unknown_fraction": finite(item.evidence.added_unknown_fraction),
                    "estimated_grid_coverage_fraction": finite(
                        item.evidence.estimated_grid_coverage_fraction
                    ),
                },
                "reason": item.reason,
                "semantics": "PROPOSAL_NOT_DRIVE_PERMISSION",
            }
            for item in plan.proposals
        ],
    }


def write_turn_zone_refinement_proposal(
    plan: TurnZoneRefinementPlan,
    path: str | Path,
) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(turn_zone_refinement_to_dict(plan), sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return output
