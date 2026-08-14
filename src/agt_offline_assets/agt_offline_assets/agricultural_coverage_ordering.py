"""Deterministic coverage ordering over an Agricultural Aisle Graph.

R4 chooses which aisles are compatible with the canonical vehicle profile and
assigns a boustrophedon traversal order.  It deliberately does not synthesize a
Dubins/Reeds-Shepp connector; instead it emits explicit connector requests for
R5/R6/R7.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

import numpy as np

from .agricultural_aisle_graph import AislePrimitive, AgriculturalAisleGraph
from .vehicle_profile import CanonicalVehicleProfile


COVERAGE_ORDER_SCHEMA = "agt_agricultural_coverage_order/v1"


@dataclass(frozen=True)
class CoverageOrderingConfig:
    minimum_side_clearance_m: float = 0.05
    include_boundary_aisles: bool = True
    start_side: str = "LOW_U"

    def validate(self) -> None:
        if self.minimum_side_clearance_m < 0.0:
            raise ValueError("minimum_side_clearance_m must be >= 0")
        if self.start_side not in {"LOW_U", "HIGH_U"}:
            raise ValueError("start_side must be LOW_U or HIGH_U")


@dataclass(frozen=True)
class AisleRejection:
    aisle_id: str
    reason: str
    geometric_width_m: float
    required_width_m: float


@dataclass(frozen=True)
class AisleTraversal:
    sequence: int
    aisle_id: str
    aisle_kind: str
    direction: str
    entry_side: str
    exit_side: str
    entry_pose: tuple[float, float, float, float]
    exit_pose: tuple[float, float, float, float]
    centerline_xyz: tuple[tuple[float, float, float], ...]
    geometric_width_m: float
    required_width_m: float
    width_surplus_m: float


@dataclass(frozen=True)
class ConnectorRequest:
    connector_id: str
    from_aisle_id: str
    to_aisle_id: str
    turn_zone_id: str
    side: str
    start_pose: tuple[float, float, float, float]
    goal_pose: tuple[float, float, float, float]


@dataclass(frozen=True)
class AgriculturalCoverageOrder:
    frame_id: str
    platform_id: str
    platform_profile_sha256: str
    traversals: tuple[AisleTraversal, ...]
    connector_requests: tuple[ConnectorRequest, ...]
    rejected_aisles: tuple[AisleRejection, ...]
    source: Mapping[str, Any] = field(default_factory=dict)
    schema: str = COVERAGE_ORDER_SCHEMA
    status: str = "DRAFT"


def _normalize(direction_xy) -> np.ndarray:
    direction = np.asarray(direction_xy, dtype=np.float64).reshape(2)
    norm = float(np.linalg.norm(direction))
    if norm <= 1.0e-12:
        raise ValueError("row direction must be non-zero")
    return direction / norm


def _lateral_coordinate(aisle: AislePrimitive, perpendicular: np.ndarray) -> float:
    points = np.asarray(aisle.centerline_xyz, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] < 2:
        return float("inf")
    midpoint = np.mean(points[:, :2], axis=0)
    return float(midpoint @ perpendicular)


def _reverse_pose(pose: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, yaw = pose
    reverse_yaw = float(np.arctan2(np.sin(yaw + np.pi), np.cos(yaw + np.pi)))
    return (float(x), float(y), float(z), reverse_yaw)


def derive_agricultural_coverage_order(
    graph: AgriculturalAisleGraph,
    vehicle: CanonicalVehicleProfile,
    config: CoverageOrderingConfig | None = None,
    *,
    source: Mapping[str, Any] | None = None,
) -> AgriculturalCoverageOrder:
    """Filter aisles by vehicle width and assign deterministic snake traversal."""
    cfg = config or CoverageOrderingConfig()
    cfg.validate()
    if not vehicle.planning_preview_ready:
        raise ValueError(
            f"vehicle profile {vehicle.profile_id} is not ready for planning preview: "
            f"{vehicle.blocked_reason or 'missing kinematic truth'}"
        )

    direction = _normalize(graph.row_direction_xy)
    perpendicular = np.array([-direction[1], direction[0]], dtype=np.float64)
    required_vehicle_width = float(vehicle.navigation_width_m + 2.0 * cfg.minimum_side_clearance_m)

    eligible: list[AislePrimitive] = []
    rejected: list[AisleRejection] = []
    for aisle in graph.aisles:
        required_width = max(required_vehicle_width, float(aisle.minimum_required_width_m))
        if aisle.kind == "boundary" and not cfg.include_boundary_aisles:
            rejected.append(
                AisleRejection(
                    aisle_id=aisle.aisle_id,
                    reason="BOUNDARY_AISLE_DISABLED_BY_POLICY",
                    geometric_width_m=float(aisle.geometric_width_m),
                    required_width_m=required_width,
                )
            )
            continue
        if float(aisle.geometric_width_m) + 1.0e-9 < required_width:
            rejected.append(
                AisleRejection(
                    aisle_id=aisle.aisle_id,
                    reason="VEHICLE_WIDTH_INFEASIBLE",
                    geometric_width_m=float(aisle.geometric_width_m),
                    required_width_m=required_width,
                )
            )
            continue
        eligible.append(aisle)

    eligible.sort(key=lambda aisle: (_lateral_coordinate(aisle, perpendicular), aisle.aisle_id))

    traversals: list[AisleTraversal] = []
    for index, aisle in enumerate(eligible):
        forward = (index % 2 == 0) == (cfg.start_side == "LOW_U")
        required_width = max(required_vehicle_width, float(aisle.minimum_required_width_m))
        if forward:
            entry_pose = tuple(float(v) for v in aisle.start_pose)
            exit_pose = tuple(float(v) for v in aisle.end_pose)
            centerline = aisle.centerline_xyz
            direction_name = "FORWARD"
            entry_side = "LOW_U"
            exit_side = "HIGH_U"
        else:
            entry_pose = _reverse_pose(aisle.end_pose)
            exit_pose = _reverse_pose(aisle.start_pose)
            centerline = tuple(reversed(aisle.centerline_xyz))
            direction_name = "REVERSE_ORIENTATION"
            entry_side = "HIGH_U"
            exit_side = "LOW_U"
        traversals.append(
            AisleTraversal(
                sequence=index + 1,
                aisle_id=aisle.aisle_id,
                aisle_kind=aisle.kind,
                direction=direction_name,
                entry_side=entry_side,
                exit_side=exit_side,
                entry_pose=entry_pose,
                exit_pose=exit_pose,
                centerline_xyz=centerline,
                geometric_width_m=float(aisle.geometric_width_m),
                required_width_m=required_width,
                width_surplus_m=float(aisle.geometric_width_m) - required_width,
            )
        )

    connector_requests: list[ConnectorRequest] = []
    for index, (current, nxt) in enumerate(zip(traversals, traversals[1:]), start=1):
        if current.exit_side != nxt.entry_side:
            raise RuntimeError("boustrophedon ordering produced mismatched connector sides")
        side = current.exit_side
        connector_requests.append(
            ConnectorRequest(
                connector_id=f"connector_{index:03d}",
                from_aisle_id=current.aisle_id,
                to_aisle_id=nxt.aisle_id,
                turn_zone_id="turn_low_u" if side == "LOW_U" else "turn_high_u",
                side=side,
                start_pose=current.exit_pose,
                goal_pose=nxt.entry_pose,
            )
        )

    merged_source = dict(graph.source)
    merged_source.update(dict(source or {}))
    merged_source.update(
        {
            "ordering_policy": "DETERMINISTIC_BOUSTROPHEDON",
            "minimum_side_clearance_m": float(cfg.minimum_side_clearance_m),
            "vehicle_navigation_width_m": float(vehicle.navigation_width_m),
        }
    )
    return AgriculturalCoverageOrder(
        frame_id=graph.frame_id,
        platform_id=vehicle.profile_id,
        platform_profile_sha256=vehicle.profile_sha256,
        traversals=tuple(traversals),
        connector_requests=tuple(connector_requests),
        rejected_aisles=tuple(rejected),
        source=merged_source,
    )


def coverage_order_to_dict(order: AgriculturalCoverageOrder) -> dict[str, Any]:
    def pose_dict(pose):
        return {"x": pose[0], "y": pose[1], "z": pose[2], "yaw": pose[3]}

    return {
        "schema": order.schema,
        "status": order.status,
        "frame_id": order.frame_id,
        "platform_id": order.platform_id,
        "platform_profile_sha256": order.platform_profile_sha256,
        "source": dict(order.source),
        "traversal_count": len(order.traversals),
        "connector_request_count": len(order.connector_requests),
        "rejected_aisle_count": len(order.rejected_aisles),
        "traversals": [
            {
                "sequence": traversal.sequence,
                "aisle_id": traversal.aisle_id,
                "aisle_kind": traversal.aisle_kind,
                "direction": traversal.direction,
                "entry_side": traversal.entry_side,
                "exit_side": traversal.exit_side,
                "entry_pose": pose_dict(traversal.entry_pose),
                "exit_pose": pose_dict(traversal.exit_pose),
                "geometric_width_m": traversal.geometric_width_m,
                "required_width_m": traversal.required_width_m,
                "width_surplus_m": traversal.width_surplus_m,
            }
            for traversal in order.traversals
        ],
        "connector_requests": [
            {
                "connector_id": connector.connector_id,
                "from_aisle_id": connector.from_aisle_id,
                "to_aisle_id": connector.to_aisle_id,
                "turn_zone_id": connector.turn_zone_id,
                "side": connector.side,
                "start_pose": pose_dict(connector.start_pose),
                "goal_pose": pose_dict(connector.goal_pose),
            }
            for connector in order.connector_requests
        ],
        "rejected_aisles": [
            {
                "aisle_id": rejection.aisle_id,
                "reason": rejection.reason,
                "geometric_width_m": rejection.geometric_width_m,
                "required_width_m": rejection.required_width_m,
            }
            for rejection in order.rejected_aisles
        ],
    }
