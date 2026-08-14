"""Vehicle-safe connector anchors for V25-12E agricultural route production.

A raw Agricultural Aisle Graph endpoint is structural centerline evidence, not a
promise that the canonical vehicle footprint is still fully FREE when its
reference point is placed exactly at that endpoint.  R6B exposed this distinction
on the real greenhouse map: most admitted connectors were rejected before search
because the raw endpoint preview footprint overlapped a row end.

This module derives explicit connector anchors by walking *inward along the
existing aisle centerline* until the canonical preview footprint is stably FREE.
It never mutates the Aisle Graph or Navigation Map.  Later Route assembly must
trim the aisle traversal to the selected anchor rather than driving through the
unsafe raw tail.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Any, Iterable, Mapping

import numpy as np
import yaml

from .agricultural_aisle_graph import AislePrimitive, AgriculturalAisleGraph
from .agricultural_coverage_ordering import ConnectorRequest
from .contracts import AssetContractError
from .forward_connector_navigation_gate import _preview_local_footprint
from .navigation_grid import NavigationGridEvidence
from .reverse_fallback_admission import ReverseFallbackAdmissionPlan
from .reverse_primitive_connector import _preview_pose_free
from .vehicle_profile import CanonicalVehicleProfile


CONNECTOR_ANCHOR_SCHEMA = "agt_connector_anchor_plan/v1"


@dataclass(frozen=True)
class ConnectorAnchorConfig:
    search_step_m: float = 0.05
    maximum_retreat_m: float = 1.50
    stable_free_span_m: float = 0.30
    endpoint_match_tolerance_m: float = 0.35
    preview_footprint_padding_m: float = 0.05

    def validate(self) -> None:
        for name in (
            "search_step_m",
            "maximum_retreat_m",
            "endpoint_match_tolerance_m",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")
        if not math.isfinite(self.stable_free_span_m) or self.stable_free_span_m < 0.0:
            raise ValueError("stable_free_span_m must be finite and >= 0")
        if not math.isfinite(self.preview_footprint_padding_m) or self.preview_footprint_padding_m < 0.0:
            raise ValueError("preview_footprint_padding_m must be finite and >= 0")


@dataclass(frozen=True)
class ConnectorAnchorEndpoint:
    raw_pose: tuple[float, float, float, float]
    anchor_pose: tuple[float, float, float, float] | None
    status: str
    retreat_m: float | None
    raw_pose_free: bool
    endpoint_kind: str


@dataclass(frozen=True)
class ConnectorAnchorItem:
    connector_id: str
    from_aisle_id: str
    to_aisle_id: str
    turn_zone_id: str
    side: str
    status: str
    start: ConnectorAnchorEndpoint
    goal: ConnectorAnchorEndpoint
    reason: str

    @property
    def adjusted_request(self) -> ConnectorRequest | None:
        if self.status != "READY_FOR_LOCAL_CONNECTOR":
            return None
        if self.start.anchor_pose is None or self.goal.anchor_pose is None:
            return None
        return ConnectorRequest(
            connector_id=self.connector_id,
            from_aisle_id=self.from_aisle_id,
            to_aisle_id=self.to_aisle_id,
            turn_zone_id=self.turn_zone_id,
            side=self.side,
            start_pose=self.start.anchor_pose,
            goal_pose=self.goal.anchor_pose,
        )


@dataclass(frozen=True)
class ConnectorAnchorPlan:
    frame_id: str
    platform_id: str
    platform_profile_sha256: str
    items: tuple[ConnectorAnchorItem, ...]
    source: Mapping[str, Any] = field(default_factory=dict)
    schema: str = CONNECTOR_ANCHOR_SCHEMA
    status: str = "DRAFT"

    @property
    def ready_count(self) -> int:
        return sum(item.status == "READY_FOR_LOCAL_CONNECTOR" for item in self.items)

    @property
    def failed_count(self) -> int:
        return len(self.items) - self.ready_count

    @property
    def adjusted_requests(self) -> tuple[ConnectorRequest, ...]:
        requests: list[ConnectorRequest] = []
        for item in self.items:
            adjusted = item.adjusted_request
            if adjusted is not None:
                requests.append(adjusted)
        return tuple(requests)


def _polyline_from_endpoint(
    aisle: AislePrimitive,
    raw_pose: tuple[float, float, float, float],
    tolerance_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    points = np.asarray(aisle.centerline_xyz, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] < 2 or points.shape[1] < 3:
        raise ValueError(f"aisle {aisle.aisle_id} centerline is invalid")
    raw_xy = np.asarray(raw_pose[:2], dtype=np.float64)
    d_start = float(np.linalg.norm(points[0, :2] - raw_xy))
    d_end = float(np.linalg.norm(points[-1, :2] - raw_xy))
    if min(d_start, d_end) > tolerance_m:
        raise ValueError(
            f"connector endpoint is not close to an endpoint of {aisle.aisle_id}: "
            f"nearest={min(d_start, d_end):.3f} m"
        )
    ordered = points if d_start <= d_end else points[::-1].copy()
    segment_lengths = np.linalg.norm(np.diff(ordered[:, :2], axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    return ordered, cumulative


def _point_at_distance(points: np.ndarray, cumulative: np.ndarray, distance_m: float) -> np.ndarray:
    distance = float(np.clip(distance_m, 0.0, cumulative[-1]))
    index = int(np.searchsorted(cumulative, distance, side="right") - 1)
    index = min(max(index, 0), len(cumulative) - 2)
    span = float(cumulative[index + 1] - cumulative[index])
    if span <= 1.0e-12:
        return points[index].copy()
    ratio = (distance - float(cumulative[index])) / span
    return points[index] + ratio * (points[index + 1] - points[index])


def _stable_pose_free(
    points: np.ndarray,
    cumulative: np.ndarray,
    retreat_m: float,
    raw_yaw: float,
    navigation: NavigationGridEvidence,
    local_footprint,
    cfg: ConnectorAnchorConfig,
) -> bool:
    remaining = float(cumulative[-1] - retreat_m)
    required_span = min(float(cfg.stable_free_span_m), remaining)
    sample_count = max(1, int(math.ceil(required_span / cfg.search_step_m)))
    offsets = np.linspace(0.0, required_span, sample_count + 1)
    for offset in offsets:
        point = _point_at_distance(points, cumulative, retreat_m + float(offset))
        if not _preview_pose_free(
            float(point[0]),
            float(point[1]),
            float(raw_yaw),
            navigation,
            local_footprint,
        ):
            return False
    return True


def _derive_endpoint(
    aisle: AislePrimitive,
    raw_pose: tuple[float, float, float, float],
    endpoint_kind: str,
    navigation: NavigationGridEvidence,
    local_footprint,
    cfg: ConnectorAnchorConfig,
) -> ConnectorAnchorEndpoint:
    points, cumulative = _polyline_from_endpoint(
        aisle,
        raw_pose,
        cfg.endpoint_match_tolerance_m,
    )
    raw_free = _preview_pose_free(
        float(raw_pose[0]),
        float(raw_pose[1]),
        float(raw_pose[3]),
        navigation,
        local_footprint,
    )
    maximum = min(float(cfg.maximum_retreat_m), float(cumulative[-1]))
    steps = max(1, int(math.ceil(maximum / cfg.search_step_m)))
    distances = [min(maximum, index * cfg.search_step_m) for index in range(steps + 1)]
    if distances[-1] < maximum - 1.0e-12:
        distances.append(maximum)

    for retreat in distances:
        if not _stable_pose_free(
            points,
            cumulative,
            float(retreat),
            float(raw_pose[3]),
            navigation,
            local_footprint,
            cfg,
        ):
            continue
        point = _point_at_distance(points, cumulative, float(retreat))
        anchor = (
            float(point[0]),
            float(point[1]),
            float(point[2]),
            float(raw_pose[3]),
        )
        return ConnectorAnchorEndpoint(
            raw_pose=tuple(float(v) for v in raw_pose),
            anchor_pose=anchor,
            status="RAW_ENDPOINT_STABLY_FREE" if retreat <= 1.0e-12 else "RETREATED_TO_STABLE_FREE_ANCHOR",
            retreat_m=float(retreat),
            raw_pose_free=bool(raw_free),
            endpoint_kind=endpoint_kind,
        )

    return ConnectorAnchorEndpoint(
        raw_pose=tuple(float(v) for v in raw_pose),
        anchor_pose=None,
        status="NO_STABLE_FREE_ANCHOR_WITHIN_LIMIT",
        retreat_m=None,
        raw_pose_free=bool(raw_free),
        endpoint_kind=endpoint_kind,
    )


def derive_connector_anchor_plan(
    graph: AgriculturalAisleGraph,
    connector_requests: Iterable[ConnectorRequest],
    admission: ReverseFallbackAdmissionPlan,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    config: ConnectorAnchorConfig | None = None,
    *,
    source: Mapping[str, Any] | None = None,
) -> ConnectorAnchorPlan:
    """Derive nearest stable FREE connector anchors for R6A-admitted connectors."""
    cfg = config or ConnectorAnchorConfig()
    cfg.validate()
    if not vehicle.planning_preview_ready:
        raise ValueError(f"vehicle profile {vehicle.profile_id} is not ready for planning preview")
    if admission.platform_id and admission.platform_id != vehicle.profile_id:
        raise ValueError("R6A admission platform does not match canonical vehicle profile")
    if graph.frame_id != navigation.frame_id:
        raise ValueError("Aisle Graph and Navigation Grid frame_id must match")

    request_map = {request.connector_id: request for request in connector_requests}
    aisle_map = {aisle.aisle_id: aisle for aisle in graph.aisles}
    local_footprint = _preview_local_footprint(vehicle, cfg.preview_footprint_padding_m)
    items: list[ConnectorAnchorItem] = []

    for connector_id in admission.eligible_connector_ids:
        request = request_map.get(connector_id)
        if request is None:
            raise ValueError(f"R6A admitted connector missing from coverage requests: {connector_id}")
        from_aisle = aisle_map.get(request.from_aisle_id)
        to_aisle = aisle_map.get(request.to_aisle_id)
        if from_aisle is None or to_aisle is None:
            raise ValueError(f"connector {connector_id} references missing aisle geometry")

        start = _derive_endpoint(
            from_aisle,
            request.start_pose,
            "START_EXIT_ANCHOR",
            navigation,
            local_footprint,
            cfg,
        )
        goal = _derive_endpoint(
            to_aisle,
            request.goal_pose,
            "GOAL_ENTRY_ANCHOR",
            navigation,
            local_footprint,
            cfg,
        )
        if start.anchor_pose is not None and goal.anchor_pose is not None:
            status = "READY_FOR_LOCAL_CONNECTOR"
            reason = (
                "both connector endpoints have stable FREE preview-footprint anchors on their existing aisle centerlines"
            )
        else:
            status = "HOLD_ANCHOR_REVIEW"
            reason = (
                "at least one endpoint has no stable FREE anchor within the bounded inward-retreat limit; "
                "do not widen occupancy or hand this endpoint to R7 unchanged"
            )
        items.append(
            ConnectorAnchorItem(
                connector_id=request.connector_id,
                from_aisle_id=request.from_aisle_id,
                to_aisle_id=request.to_aisle_id,
                turn_zone_id=request.turn_zone_id,
                side=request.side,
                status=status,
                start=start,
                goal=goal,
                reason=reason,
            )
        )

    merged_source = dict(graph.source)
    merged_source.update(dict(admission.source))
    merged_source.update(dict(navigation.source))
    merged_source.update(dict(source or {}))
    merged_source.update(
        {
            "anchor_policy": "NEAREST_STABLE_FREE_POSE_INWARD_ON_EXISTING_AISLE_CENTERLINE",
            "validation_scope": "PREVIEW_ONLY_NOT_R8_VEHICLE_READY",
            "maximum_retreat_m": float(cfg.maximum_retreat_m),
            "stable_free_span_m": float(cfg.stable_free_span_m),
            "preview_footprint_padding_m": float(cfg.preview_footprint_padding_m),
            "route_assembly_requirement": "TRIM_AISLE_TRAVERSAL_TO_SELECTED_CONNECTOR_ANCHOR",
        }
    )
    return ConnectorAnchorPlan(
        frame_id=graph.frame_id,
        platform_id=vehicle.profile_id,
        platform_profile_sha256=vehicle.profile_sha256,
        items=tuple(items),
        source=merged_source,
    )


def apply_connector_anchor_plan(
    connector_requests: Iterable[ConnectorRequest],
    plan: ConnectorAnchorPlan,
) -> tuple[ConnectorRequest, ...]:
    """Return all requests with READY anchor-adjusted requests replacing raw endpoints."""
    adjusted = {
        request.connector_id: request
        for request in plan.adjusted_requests
    }
    return tuple(adjusted.get(request.connector_id, request) for request in connector_requests)


def _pose_dict(pose: tuple[float, float, float, float] | None) -> dict[str, float] | None:
    if pose is None:
        return None
    return {"x": pose[0], "y": pose[1], "z": pose[2], "yaw": pose[3]}


def connector_anchor_plan_to_dict(plan: ConnectorAnchorPlan) -> dict[str, Any]:
    def endpoint_dict(endpoint: ConnectorAnchorEndpoint) -> dict[str, Any]:
        return {
            "endpoint_kind": endpoint.endpoint_kind,
            "status": endpoint.status,
            "raw_pose_free": endpoint.raw_pose_free,
            "retreat_m": endpoint.retreat_m,
            "raw_pose": _pose_dict(endpoint.raw_pose),
            "anchor_pose": _pose_dict(endpoint.anchor_pose),
        }

    return {
        "schema": plan.schema,
        "status": plan.status,
        "frame_id": plan.frame_id,
        "platform_id": plan.platform_id,
        "platform_profile_sha256": plan.platform_profile_sha256,
        "source": dict(plan.source),
        "connector_count": len(plan.items),
        "summary": {
            "ready_for_local_connector": plan.ready_count,
            "hold_anchor_review": plan.failed_count,
        },
        "connectors": [
            {
                "connector_id": item.connector_id,
                "from_aisle_id": item.from_aisle_id,
                "to_aisle_id": item.to_aisle_id,
                "turn_zone_id": item.turn_zone_id,
                "side": item.side,
                "status": item.status,
                "reason": item.reason,
                "start": endpoint_dict(item.start),
                "goal": endpoint_dict(item.goal),
                "adjusted_request": (
                    None
                    if item.adjusted_request is None
                    else {
                        "start_pose": _pose_dict(item.adjusted_request.start_pose),
                        "goal_pose": _pose_dict(item.adjusted_request.goal_pose),
                    }
                ),
            }
            for item in plan.items
        ],
    }


def write_connector_anchor_plan(plan: ConnectorAnchorPlan, path: str | Path) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(connector_anchor_plan_to_dict(plan), sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return output


def load_connector_anchor_plan(path: str | Path) -> ConnectorAnchorPlan:
    source_path = Path(path).expanduser().resolve()
    payload = yaml.safe_load(source_path.read_text(encoding="utf-8")) or {}
    if not isinstance(payload, Mapping):
        raise AssetContractError("connector_anchor_invalid", "connector anchor asset must be a mapping")
    schema = str(payload.get("schema", ""))
    if schema != CONNECTOR_ANCHOR_SCHEMA:
        raise AssetContractError(
            "connector_anchor_schema_mismatch",
            f"expected {CONNECTOR_ANCHOR_SCHEMA}, got {schema or '<empty>'}",
        )

    def pose(value: Any) -> tuple[float, float, float, float] | None:
        if value is None:
            return None
        if not isinstance(value, Mapping):
            raise AssetContractError("connector_anchor_pose_invalid", "anchor pose must be a mapping")
        return (float(value["x"]), float(value["y"]), float(value["z"]), float(value["yaw"]))

    items: list[ConnectorAnchorItem] = []
    for raw_item in payload.get("connectors") or []:
        if not isinstance(raw_item, Mapping):
            raise AssetContractError("connector_anchor_item_invalid", "connector item must be a mapping")

        def endpoint(name: str) -> ConnectorAnchorEndpoint:
            value = raw_item.get(name) or {}
            if not isinstance(value, Mapping):
                raise AssetContractError("connector_anchor_endpoint_invalid", f"{name} must be a mapping")
            raw_pose = pose(value.get("raw_pose"))
            if raw_pose is None:
                raise AssetContractError("connector_anchor_endpoint_invalid", f"{name}.raw_pose is required")
            return ConnectorAnchorEndpoint(
                raw_pose=raw_pose,
                anchor_pose=pose(value.get("anchor_pose")),
                status=str(value.get("status", "")),
                retreat_m=None if value.get("retreat_m") is None else float(value["retreat_m"]),
                raw_pose_free=bool(value.get("raw_pose_free", False)),
                endpoint_kind=str(value.get("endpoint_kind", "")),
            )

        items.append(
            ConnectorAnchorItem(
                connector_id=str(raw_item["connector_id"]),
                from_aisle_id=str(raw_item["from_aisle_id"]),
                to_aisle_id=str(raw_item["to_aisle_id"]),
                turn_zone_id=str(raw_item["turn_zone_id"]),
                side=str(raw_item["side"]),
                status=str(raw_item["status"]),
                start=endpoint("start"),
                goal=endpoint("goal"),
                reason=str(raw_item.get("reason", "")),
            )
        )

    declared = payload.get("connector_count")
    if declared is not None and int(declared) != len(items):
        raise AssetContractError(
            "connector_anchor_count_mismatch",
            f"declared connector_count={declared}, parsed={len(items)}",
        )
    return ConnectorAnchorPlan(
        frame_id=str(payload.get("frame_id", "map")),
        platform_id=str(payload.get("platform_id", "")),
        platform_profile_sha256=str(payload.get("platform_profile_sha256", "")),
        items=tuple(items),
        source=dict(payload.get("source") or {}),
        schema=schema,
        status=str(payload.get("status", "DRAFT")),
    )
