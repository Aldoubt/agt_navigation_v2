"""R6B bounded reverse-aware local connector preview for agricultural aisles.

This backend is deliberately not labelled analytic Reeds-Shepp. It performs a
bounded deterministic search over Ackermann motion primitives inside one known
aisle-pair headland neighborhood and may use FORWARD/REVERSE with explicit cusp
actions.

V25-12F adds an optional hard Site Boundary invariant. Every preview footprint,
including the start pose, primitive samples, and forward goal-shot samples, must
remain strictly inside the vehicle-permitted inner perimeter. This is enforced
in addition to the existing FREE-only Navigation Grid gate; no R6B search
parameter is relaxed.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace as dataclass_replace
import heapq
import math
from pathlib import Path
from typing import Any, Iterable, Mapping

import numpy as np
import yaml

from .agricultural_coverage_ordering import ConnectorRequest
from .forward_connector import (
    ForwardConnectorSample,
    _dubins_candidates,
    _sample_candidate,
    _wrap_pi,
)
from .forward_connector_navigation_gate import (
    GridPathEvidence,
    _evaluate_candidate,
    _preview_local_footprint,
    _transform_polygon,
)
from .navigation_grid import NavigationGridEvidence
from .navigation_map_derivation import FREE
from .reverse_fallback_admission import ReverseFallbackAdmissionPlan
from .site_boundary import SiteBoundary, polygon_strictly_inside_site_boundary
from .turn_zones import TurnZone, TurnZoneSet, _points_inside_polygon
from .vehicle_profile import CanonicalVehicleProfile
from .reverse_primitive_diagnostics import (
    R6B_DIAGNOSTIC_SCHEMA,
    SEARCH_ENVELOPE_LIMITED,
    SITE_BOUNDARY_LIMITED,
    FOOTPRINT_GRID_LIMITED,
    STATE_DISCRETIZATION_OR_DOMINANCE_LIMITED,
    GOAL_CONNECTION_LIMITED,
    MOTION_PRIMITIVE_LIMITED,
    PATH_OR_CUSP_ENVELOPE_LIMITED,
    MIXED_LIMITATION,
    INCONCLUSIVE,
    ReversePrimitiveSearchDiagnostics,
    classify_reverse_primitive_failure,
    reverse_primitive_search_diagnostics_to_dict,
    reverse_primitive_search_diagnostics_from_dict,
)

EDGE_FREE = "FREE"
EDGE_SEARCH_ENVELOPE = "SEARCH_ENVELOPE"
EDGE_SITE_BOUNDARY = "SITE_BOUNDARY"
EDGE_NAVIGATION_GRID = "NAVIGATION_GRID"

_PRIMITIVE_CURVATURE_FRACTIONS = (-1.0, -0.5, 0.0, 0.5, 1.0)


def _primitive_curvature_values(radius: float) -> tuple[float, ...]:
    radius = float(radius)
    if not math.isfinite(radius) or radius <= 0.0:
        raise ValueError("primitive curvature radius must be finite and > 0")
    return tuple(fraction / radius for fraction in _PRIMITIVE_CURVATURE_FRACTIONS)


REVERSE_PRIMITIVE_CONNECTOR_SCHEMA = "agt_reverse_primitive_connector_plan/v1"
REVERSE_PRIMITIVE_BACKEND = (
    "BOUNDED_REVERSE_PRIMITIVE_SEARCH_NOT_ANALYTIC_REEDS_SHEPP"
)


@dataclass(frozen=True)
class ReversePrimitiveConnectorConfig:
    primitive_length_m: float = 0.30
    collision_sample_step_m: float = 0.10
    state_xy_resolution_m: float = 0.15
    state_yaw_resolution_deg: float = 15.0
    goal_position_tolerance_m: float = 0.18
    goal_yaw_tolerance_deg: float = 12.0
    goal_shot_distance_m: float = 3.0
    max_cusps: int = 2
    max_expansions: int = 30000
    max_path_length_m: float = 18.0
    reverse_cost_multiplier: float = 1.15
    cusp_penalty_m: float = 0.75
    steering_change_penalty_m: float = 0.05
    longitudinal_zone_padding_m: float = 0.80
    lateral_pair_padding_m: float = 1.25
    preview_footprint_padding_m: float = 0.05

    def validate(self) -> None:
        positive = (
            "primitive_length_m",
            "collision_sample_step_m",
            "state_xy_resolution_m",
            "state_yaw_resolution_deg",
            "goal_position_tolerance_m",
            "goal_yaw_tolerance_deg",
            "goal_shot_distance_m",
            "max_path_length_m",
        )
        for name in positive:
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")
        for name in (
            "reverse_cost_multiplier",
            "cusp_penalty_m",
            "steering_change_penalty_m",
            "longitudinal_zone_padding_m",
            "lateral_pair_padding_m",
            "preview_footprint_padding_m",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and >= 0")
        if self.max_cusps < 1:
            raise ValueError("max_cusps must be >= 1")
        if self.max_expansions <= 0:
            raise ValueError("max_expansions must be > 0")


@dataclass(frozen=True)
class ReversePrimitiveSample:
    x: float
    y: float
    z: float
    yaw: float
    motion_direction: str
    curvature_per_m: float
    segment_index: int
    is_cusp: bool = False


@dataclass(frozen=True)
class ReversePrimitiveConnectorResult:
    connector_id: str
    from_aisle_id: str
    to_aisle_id: str
    turn_zone_id: str
    status: str
    backend: str
    minimum_turning_radius_m: float
    samples: tuple[ReversePrimitiveSample, ...]
    path_length_m: float | None
    forward_distance_m: float
    reverse_distance_m: float
    cusp_count: int
    search_expansions: int
    goal_position_error_m: float | None
    goal_yaw_error_rad: float | None
    centerline_evidence: GridPathEvidence
    footprint_evidence: GridPathEvidence
    reason: str = ""
    diagnostics: ReversePrimitiveSearchDiagnostics = field(
        default_factory=ReversePrimitiveSearchDiagnostics
    )


@dataclass
class _MutableReversePrimitiveSearchDiagnostics:
    failure_class: str | None = None
    search_started: bool = False
    search_expansions: int = 0
    queue_exhausted: bool = False
    expansion_budget_reached: bool = False
    start_goal_position_error_m: float | None = None
    best_goal_position_error_m: float | None = None
    best_goal_yaw_error_rad: float | None = None
    best_goal_distance_state_direction: str | None = None
    best_goal_distance_cusp_count: int | None = None
    nodes_popped: int = 0
    stale_nodes_skipped: int = 0
    cusp_switches_considered: int = 0
    cusp_switches_rejected_state_dominance: int = 0
    cusp_switches_enqueued: int = 0
    nodes_at_max_cusps: int = 0
    primitive_edges_considered: int = 0
    primitive_edges_rejected_search_envelope: int = 0
    primitive_edges_rejected_site_boundary: int = 0
    primitive_edges_rejected_navigation_grid: int = 0
    primitive_edges_rejected_path_length: int = 0
    primitive_edges_rejected_state_dominance: int = 0
    primitive_edges_enqueued: int = 0
    goal_tolerance_checks: int = 0
    goal_tolerance_successes: int = 0
    goal_shot_attempts: int = 0
    goal_shot_reverse_direction_blocked: int = 0
    goal_shot_candidates_considered: int = 0
    goal_shot_rejected_path_length: int = 0
    goal_shot_rejected_search_envelope: int = 0
    goal_shot_rejected_site_boundary: int = 0
    goal_shot_rejected_navigation_grid: int = 0
    goal_shot_successes: int = 0

    def freeze(self) -> ReversePrimitiveSearchDiagnostics:
        values = {name: getattr(self, name) for name in ReversePrimitiveSearchDiagnostics.__dataclass_fields__ if name != "schema"}
        return ReversePrimitiveSearchDiagnostics(**values)


@dataclass(frozen=True)
class ReversePrimitiveConnectorPlan:
    frame_id: str
    platform_id: str
    platform_profile_sha256: str
    connectors: tuple[ReversePrimitiveConnectorResult, ...]
    source: Mapping[str, Any] = field(default_factory=dict)
    schema: str = REVERSE_PRIMITIVE_CONNECTOR_SCHEMA
    status: str = "DRAFT"

    @property
    def solved_count(self) -> int:
        return sum(
            item.status == "REVERSE_PRIMITIVE_PREVIEW_FREE"
            for item in self.connectors
        )

    @property
    def unsolved_count(self) -> int:
        return len(self.connectors) - self.solved_count


@dataclass
class _SearchNode:
    x: float
    y: float
    yaw: float
    direction: int
    cusp_count: int
    last_curvature_index: int
    cost: float
    travel_m: float
    parent_index: int | None
    edge_samples: tuple[tuple[float, float, float, int, float, bool], ...]


def _empty_evidence() -> GridPathEvidence:
    return GridPathEvidence(0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0)


def _direction_name(direction: int) -> str:
    return "FORWARD" if direction >= 0 else "REVERSE"


def _row_frame_bounds(
    request: ConnectorRequest,
    zone: TurnZone,
    row_direction_xy: tuple[float, float],
    cfg: ReversePrimitiveConnectorConfig,
) -> tuple[np.ndarray, np.ndarray, float, float, float, float]:
    u = np.asarray(row_direction_xy, dtype=np.float64)
    norm = float(np.linalg.norm(u))
    if norm <= 1.0e-12:
        raise ValueError("Turn Zone row direction is degenerate")
    u /= norm
    v = np.array([-u[1], u[0]], dtype=np.float64)
    polygon = np.asarray(zone.polygon_xy, dtype=np.float64)
    zone_u = polygon @ u
    start_xy = np.asarray(request.start_pose[:2], dtype=np.float64)
    goal_xy = np.asarray(request.goal_pose[:2], dtype=np.float64)
    start_v = float(start_xy @ v)
    goal_v = float(goal_xy @ v)
    return (
        u,
        v,
        float(np.min(zone_u) - cfg.longitudinal_zone_padding_m),
        float(np.max(zone_u) + cfg.longitudinal_zone_padding_m),
        float(min(start_v, goal_v) - cfg.lateral_pair_padding_m),
        float(max(start_v, goal_v) + cfg.lateral_pair_padding_m),
    )


def _inside_search_envelope(
    x: float,
    y: float,
    bounds: tuple[np.ndarray, np.ndarray, float, float, float, float],
) -> bool:
    u, v, u0, u1, v0, v1 = bounds
    point = np.array([x, y], dtype=np.float64)
    pu = float(point @ u)
    pv = float(point @ v)
    return u0 <= pu <= u1 and v0 <= pv <= v1


def _preview_pose_status(
    x: float,
    y: float,
    yaw: float,
    navigation: NavigationGridEvidence,
    local_footprint: tuple[tuple[float, float], ...],
    site_boundary: SiteBoundary | None = None,
) -> str:
    probe = ForwardConnectorSample(
        x=float(x),
        y=float(y),
        z=0.0,
        yaw=float(yaw),
    )
    polygon = _transform_polygon(local_footprint, probe)
    if site_boundary is not None and not polygon_strictly_inside_site_boundary(
        site_boundary,
        polygon,
    ):
        return "SITE_BOUNDARY_CONFLICT"

    poly = np.asarray(polygon, dtype=np.float64)
    px0, py0 = np.min(poly, axis=0)
    px1, py1 = np.max(poly, axis=0)
    min_x, min_y, max_x, max_y = navigation.bounds_m()
    if px0 < min_x or py0 < min_y or px1 > max_x or py1 > max_y:
        return "GRID_NOT_FREE"

    resolution = float(navigation.resolution_m)
    col0 = max(
        0,
        int(math.floor((float(px0) - navigation.origin_x_m) / resolution)),
    )
    col1 = min(
        navigation.width - 1,
        int(math.floor((float(px1) - navigation.origin_x_m) / resolution)),
    )
    row0 = max(
        0,
        int(math.floor((float(py0) - navigation.origin_y_m) / resolution)),
    )
    row1 = min(
        navigation.height - 1,
        int(math.floor((float(py1) - navigation.origin_y_m) / resolution)),
    )
    if row0 > row1 or col0 > col1:
        return "GRID_NOT_FREE"

    rows, cols = np.indices(
        (row1 - row0 + 1, col1 - col0 + 1),
        dtype=np.float64,
    )
    xx = navigation.origin_x_m + (cols + col0 + 0.5) * resolution
    yy = navigation.origin_y_m + (rows + row0 + 0.5) * resolution
    inside = _points_inside_polygon(xx, yy, polygon)
    if not bool(np.any(inside)):
        return "GRID_NOT_FREE"
    cells = navigation.occupancy[row0 : row1 + 1, col0 : col1 + 1][inside]
    if cells.size <= 0 or not np.all(cells == FREE):
        return "GRID_NOT_FREE"
    return "FREE"


def _preview_pose_free(
    x: float,
    y: float,
    yaw: float,
    navigation: NavigationGridEvidence,
    local_footprint: tuple[tuple[float, float], ...],
    site_boundary: SiteBoundary | None = None,
) -> bool:
    return (
        _preview_pose_status(
            x,
            y,
            yaw,
            navigation,
            local_footprint,
            site_boundary,
        )
        == "FREE"
    )


def _integrate_primitive(
    node: _SearchNode,
    curvature: float,
    length_m: float,
    sample_step_m: float,
) -> tuple[tuple[float, float, float, int, float, bool], ...]:
    x, y, yaw = float(node.x), float(node.y), float(node.yaw)
    remaining = float(length_m)
    output: list[tuple[float, float, float, int, float, bool]] = []
    while remaining > 1.0e-12:
        ds_abs = min(sample_step_m, remaining)
        signed_ds = float(node.direction) * ds_abs
        if abs(curvature) <= 1.0e-12:
            x += signed_ds * math.cos(yaw)
            y += signed_ds * math.sin(yaw)
        else:
            yaw_next = yaw + curvature * signed_ds
            x += (math.sin(yaw_next) - math.sin(yaw)) / curvature
            y += (-math.cos(yaw_next) + math.cos(yaw)) / curvature
            yaw = yaw_next
        yaw = _wrap_pi(yaw)
        output.append((x, y, yaw, node.direction, curvature, False))
        remaining -= ds_abs
    return tuple(output)


def _state_key(
    node: _SearchNode,
    cfg: ReversePrimitiveConnectorConfig,
) -> tuple[int, ...]:
    yaw_resolution = math.radians(cfg.state_yaw_resolution_deg)
    yaw_index = int(round(_wrap_pi(node.yaw) / yaw_resolution))
    return (
        int(round(node.x / cfg.state_xy_resolution_m)),
        int(round(node.y / cfg.state_xy_resolution_m)),
        yaw_index,
        int(node.direction),
        int(node.cusp_count),
        int(node.last_curvature_index),
    )


def _heuristic(
    node: _SearchNode,
    goal_pose: tuple[float, float, float, float],
    radius: float,
) -> float:
    dx = float(goal_pose[0] - node.x)
    dy = float(goal_pose[1] - node.y)
    yaw_error = abs(_wrap_pi(float(goal_pose[3] - node.yaw)))
    return math.hypot(dx, dy) + 0.20 * radius * yaw_error


def _goal_error(
    node: _SearchNode,
    goal_pose: tuple[float, float, float, float],
) -> tuple[float, float]:
    position = math.hypot(
        float(goal_pose[0] - node.x),
        float(goal_pose[1] - node.y),
    )
    yaw = abs(_wrap_pi(float(goal_pose[3] - node.yaw)))
    return position, yaw


def _edge_is_free(
    samples: tuple[tuple[float, float, float, int, float, bool], ...],
    bounds,
    navigation: NavigationGridEvidence,
    local_footprint,
    site_boundary: SiteBoundary | None = None,
) -> bool:
    return _edge_rejection_reason(
        samples, bounds, navigation, local_footprint, site_boundary
    ) == EDGE_FREE


def _edge_rejection_reason(
    samples,
    bounds,
    navigation: NavigationGridEvidence,
    local_footprint,
    site_boundary: SiteBoundary | None = None,
) -> str:
    for x, y, yaw, _direction, _curvature, _is_cusp in samples:
        if not _inside_search_envelope(x, y, bounds):
            return EDGE_SEARCH_ENVELOPE
        status = _preview_pose_status(
            x,
            y,
            yaw,
            navigation,
            local_footprint,
            site_boundary,
        )
        if status == "SITE_BOUNDARY_CONFLICT":
            return EDGE_SITE_BOUNDARY
        if status != "FREE":
            return EDGE_NAVIGATION_GRID
    return EDGE_FREE


def _try_forward_goal_shot(
    node: _SearchNode,
    request: ConnectorRequest,
    radius: float,
    cfg: ReversePrimitiveConnectorConfig,
    bounds,
    navigation: NavigationGridEvidence,
    local_footprint,
    site_boundary: SiteBoundary | None = None,
    diagnostics: _MutableReversePrimitiveSearchDiagnostics | None = None,
) -> tuple[tuple[tuple[float, float, float, int, float, bool], ...], float] | None:
    distance = math.hypot(
        request.goal_pose[0] - node.x,
        request.goal_pose[1] - node.y,
    )
    if distance > cfg.goal_shot_distance_m:
        return None
    if diagnostics is not None:
        diagnostics.goal_shot_attempts += 1
    if node.direction != 1:
        if diagnostics is not None:
            diagnostics.goal_shot_reverse_direction_blocked += 1
        return None
    start = (node.x, node.y, request.start_pose[2], node.yaw)
    candidates = _dubins_candidates(start, request.goal_pose, radius)
    for path_type, normalized_lengths in candidates:
        sampled, length_m = _sample_candidate(
            start,
            request.goal_pose,
            path_type,
            normalized_lengths,
            radius,
            cfg.collision_sample_step_m,
        )
        if node.travel_m + float(length_m) > cfg.max_path_length_m:
            if diagnostics is not None:
                diagnostics.goal_shot_candidates_considered += 1
                diagnostics.goal_shot_rejected_path_length += 1
            continue
        converted = tuple(
            (float(s.x), float(s.y), float(s.yaw), 1, 0.0, False)
            for s in sampled[1:]
        )
        reason = _edge_rejection_reason(
            converted,
            bounds,
            navigation,
            local_footprint,
            site_boundary,
        )
        if diagnostics is not None:
            diagnostics.goal_shot_candidates_considered += 1
            if reason == EDGE_SEARCH_ENVELOPE:
                diagnostics.goal_shot_rejected_search_envelope += 1
            elif reason == EDGE_SITE_BOUNDARY:
                diagnostics.goal_shot_rejected_site_boundary += 1
            elif reason == EDGE_NAVIGATION_GRID:
                diagnostics.goal_shot_rejected_navigation_grid += 1
        if reason == EDGE_FREE:
            if diagnostics is not None:
                diagnostics.goal_shot_successes += 1
            return converted, float(length_m)
    return None


def _reconstruct_raw(nodes: list[_SearchNode], node_index: int):
    edges = []
    current = node_index
    while nodes[current].parent_index is not None:
        edges.append(nodes[current].edge_samples)
        current = int(nodes[current].parent_index)
    edges.reverse()
    output: list[tuple[float, float, float, int, float, bool]] = []
    for edge in edges:
        output.extend(edge)
    return output


def _finalize_samples(
    request: ConnectorRequest,
    raw_samples: Iterable[tuple[float, float, float, int, float, bool]],
) -> tuple[ReversePrimitiveSample, ...]:
    raw = list(raw_samples)
    start = (
        float(request.start_pose[0]),
        float(request.start_pose[1]),
        float(request.start_pose[3]),
        1,
        0.0,
        False,
    )
    combined = [start] + raw

    distances = [0.0]
    for previous, current in zip(combined[:-1], combined[1:]):
        distances.append(
            distances[-1]
            + math.hypot(
                float(current[0] - previous[0]),
                float(current[1] - previous[1]),
            )
        )
    total = distances[-1]
    z0 = float(request.start_pose[2])
    z1 = float(request.goal_pose[2])
    segment = 0
    output: list[ReversePrimitiveSample] = []
    for index, item in enumerate(combined):
        x, y, yaw, direction, curvature, is_cusp = item
        if index > 0 and is_cusp:
            segment += 1
        ratio = 0.0 if total <= 1.0e-12 else distances[index] / total
        output.append(
            ReversePrimitiveSample(
                x=float(x),
                y=float(y),
                z=float(z0 + ratio * (z1 - z0)),
                yaw=float(_wrap_pi(yaw)),
                motion_direction=_direction_name(direction),
                curvature_per_m=float(curvature),
                segment_index=segment,
                is_cusp=bool(is_cusp),
            )
        )
    return tuple(output)


def _path_metrics(
    samples: tuple[ReversePrimitiveSample, ...],
) -> tuple[float, float, float, int]:
    forward = 0.0
    reverse = 0.0
    for previous, current in zip(samples[:-1], samples[1:]):
        distance = math.hypot(
            current.x - previous.x,
            current.y - previous.y,
        )
        if current.motion_direction == "REVERSE":
            reverse += distance
        else:
            forward += distance
    return (
        forward + reverse,
        forward,
        reverse,
        sum(sample.is_cusp for sample in samples),
    )


def _path_evidence(
    samples: tuple[ReversePrimitiveSample, ...],
    navigation: NavigationGridEvidence,
    local_footprint,
) -> tuple[GridPathEvidence, GridPathEvidence]:
    forward_samples = tuple(
        ForwardConnectorSample(
            x=sample.x,
            y=sample.y,
            z=sample.z,
            yaw=sample.yaw,
            motion_direction=sample.motion_direction,
        )
        for sample in samples
    )
    return _evaluate_candidate(forward_samples, navigation, local_footprint)


def _boundary_conflict_result(
    request: ConnectorRequest,
    radius: float,
) -> ReversePrimitiveConnectorResult:
    return ReversePrimitiveConnectorResult(
        connector_id=request.connector_id,
        from_aisle_id=request.from_aisle_id,
        to_aisle_id=request.to_aisle_id,
        turn_zone_id=request.turn_zone_id,
        status="SITE_BOUNDARY_CONFLICT",
        backend=REVERSE_PRIMITIVE_BACKEND,
        minimum_turning_radius_m=radius,
        samples=(),
        path_length_m=None,
        forward_distance_m=0.0,
        reverse_distance_m=0.0,
        cusp_count=0,
        search_expansions=0,
        goal_position_error_m=None,
        goal_yaw_error_rad=None,
        centerline_evidence=_empty_evidence(),
        footprint_evidence=_empty_evidence(),
        reason=(
            "start preview footprint touches or crosses the Site Boundary; "
            "the vehicle-permitted inner perimeter is a hard constraint"
        ),
    )


def _search_one(
    request: ConnectorRequest,
    zone: TurnZone,
    zones: TurnZoneSet,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    cfg: ReversePrimitiveConnectorConfig,
    site_boundary: SiteBoundary | None = None,
) -> ReversePrimitiveConnectorResult:
    radius = float(vehicle.minimum_turning_radius_m)
    local_footprint = _preview_local_footprint(
        vehicle,
        cfg.preview_footprint_padding_m,
    )
    bounds = _row_frame_bounds(request, zone, zones.row_direction_xy, cfg)
    diagnostics = _MutableReversePrimitiveSearchDiagnostics()
    diagnostics.start_goal_position_error_m = math.hypot(
        float(request.goal_pose[0] - request.start_pose[0]),
        float(request.goal_pose[1] - request.start_pose[1]),
    )
    start = _SearchNode(
        x=float(request.start_pose[0]),
        y=float(request.start_pose[1]),
        yaw=float(request.start_pose[3]),
        direction=1,
        cusp_count=0,
        last_curvature_index=0,
        cost=0.0,
        travel_m=0.0,
        parent_index=None,
        edge_samples=(),
    )
    start_status = _preview_pose_status(
        start.x,
        start.y,
        start.yaw,
        navigation,
        local_footprint,
        site_boundary,
    )
    if start_status == "SITE_BOUNDARY_CONFLICT":
        result = _boundary_conflict_result(request, radius)
        return dataclass_replace(result, diagnostics=diagnostics.freeze())
    if start_status != "FREE":
        return ReversePrimitiveConnectorResult(
            connector_id=request.connector_id,
            from_aisle_id=request.from_aisle_id,
            to_aisle_id=request.to_aisle_id,
            turn_zone_id=request.turn_zone_id,
            status="R6B_START_FOOTPRINT_NOT_FREE",
            backend=REVERSE_PRIMITIVE_BACKEND,
            minimum_turning_radius_m=radius,
            samples=(),
            path_length_m=None,
            forward_distance_m=0.0,
            reverse_distance_m=0.0,
            cusp_count=0,
            search_expansions=0,
            goal_position_error_m=None,
            goal_yaw_error_rad=None,
            centerline_evidence=_empty_evidence(),
            footprint_evidence=_empty_evidence(),
            reason="start preview footprint is not FREE in the frozen Navigation Grid",
            diagnostics=diagnostics.freeze(),
        )

    nodes = [start]
    queue: list[tuple[float, int, int]] = []
    counter = 0
    heapq.heappush(
        queue,
        (_heuristic(start, request.goal_pose, radius), counter, 0),
    )
    best_cost = {_state_key(start, cfg): 0.0}
    expansions = 0
    diagnostics.search_started = True
    goal_yaw_tol = math.radians(cfg.goal_yaw_tolerance_deg)
    curvature_values = _primitive_curvature_values(radius)

    solved_index: int | None = None
    solved_shot: tuple[tuple[float, float, float, int, float, bool], ...] = ()

    while queue and expansions < cfg.max_expansions:
        _priority, _tie, node_index = heapq.heappop(queue)
        diagnostics.nodes_popped += 1
        node = nodes[node_index]
        key = _state_key(node, cfg)
        if node.cost > best_cost.get(key, float("inf")) + 1.0e-9:
            diagnostics.stale_nodes_skipped += 1
            continue
        expansions += 1
        diagnostics.search_expansions = expansions

        position_error, yaw_error = _goal_error(node, request.goal_pose)
        if (
            diagnostics.best_goal_position_error_m is None
            or position_error < diagnostics.best_goal_position_error_m - 1.0e-12
            or (
                abs(position_error - diagnostics.best_goal_position_error_m) <= 1.0e-12
                and yaw_error < (diagnostics.best_goal_yaw_error_rad or float("inf")) - 1.0e-12
            )
        ):
            diagnostics.best_goal_position_error_m = position_error
            diagnostics.best_goal_yaw_error_rad = yaw_error
            diagnostics.best_goal_distance_state_direction = _direction_name(node.direction)
            diagnostics.best_goal_distance_cusp_count = node.cusp_count
        diagnostics.goal_tolerance_checks += 1
        if (
            node.direction == 1
            and position_error <= cfg.goal_position_tolerance_m
            and yaw_error <= goal_yaw_tol
        ):
            diagnostics.goal_tolerance_successes += 1
            solved_index = node_index
            break

        shot = _try_forward_goal_shot(
            node,
            request,
            radius,
            cfg,
            bounds,
            navigation,
            local_footprint,
            site_boundary,
            diagnostics,
        )
        if shot is not None:
            solved_index = node_index
            solved_shot = shot[0]
            break

        if node.cusp_count < cfg.max_cusps:
            diagnostics.cusp_switches_considered += 1
            switched = _SearchNode(
                x=node.x,
                y=node.y,
                yaw=node.yaw,
                direction=-node.direction,
                cusp_count=node.cusp_count + 1,
                last_curvature_index=0,
                cost=node.cost + cfg.cusp_penalty_m,
                travel_m=node.travel_m,
                parent_index=node_index,
                edge_samples=(
                    (
                        node.x,
                        node.y,
                        node.yaw,
                        -node.direction,
                        0.0,
                        True,
                    ),
                ),
            )
            switched_key = _state_key(switched, cfg)
            if switched.cost + 1.0e-9 < best_cost.get(
                switched_key,
                float("inf"),
            ):
                best_cost[switched_key] = switched.cost
                nodes.append(switched)
                counter += 1
                heapq.heappush(
                    queue,
                    (
                        switched.cost
                        + _heuristic(switched, request.goal_pose, radius),
                        counter,
                        len(nodes) - 1,
                    ),
                )
                diagnostics.cusp_switches_enqueued += 1
            else:
                diagnostics.cusp_switches_rejected_state_dominance += 1
        else:
            diagnostics.nodes_at_max_cusps += 1

        if node.travel_m + cfg.primitive_length_m > cfg.max_path_length_m:
            primitive_count = len(curvature_values)
            diagnostics.primitive_edges_considered += primitive_count
            diagnostics.primitive_edges_rejected_path_length += primitive_count
            continue
        for curvature_index, curvature in enumerate(curvature_values, start=-2):
            diagnostics.primitive_edges_considered += 1
            edge = _integrate_primitive(
                node,
                curvature,
                cfg.primitive_length_m,
                cfg.collision_sample_step_m,
            )
            rejection = _edge_rejection_reason(
                edge,
                bounds,
                navigation,
                local_footprint,
                site_boundary,
            )
            if rejection == EDGE_SEARCH_ENVELOPE:
                diagnostics.primitive_edges_rejected_search_envelope += 1
                continue
            if rejection == EDGE_SITE_BOUNDARY:
                diagnostics.primitive_edges_rejected_site_boundary += 1
                continue
            if rejection == EDGE_NAVIGATION_GRID:
                diagnostics.primitive_edges_rejected_navigation_grid += 1
                continue
            x, y, yaw, _direction, _curvature, _cusp = edge[-1]
            motion_cost = cfg.primitive_length_m * (
                cfg.reverse_cost_multiplier if node.direction < 0 else 1.0
            )
            steering_change = (
                cfg.steering_change_penalty_m
                if curvature_index != node.last_curvature_index
                else 0.0
            )
            child = _SearchNode(
                x=x,
                y=y,
                yaw=yaw,
                direction=node.direction,
                cusp_count=node.cusp_count,
                last_curvature_index=curvature_index,
                cost=node.cost + motion_cost + steering_change,
                travel_m=node.travel_m + cfg.primitive_length_m,
                parent_index=node_index,
                edge_samples=edge,
            )
            child_key = _state_key(child, cfg)
            if child.cost + 1.0e-9 >= best_cost.get(
                child_key,
                float("inf"),
            ):
                diagnostics.primitive_edges_rejected_state_dominance += 1
                continue
            best_cost[child_key] = child.cost
            nodes.append(child)
            counter += 1
            heapq.heappush(
                queue,
                (
                    child.cost + _heuristic(child, request.goal_pose, radius),
                    counter,
                    len(nodes) - 1,
                ),
            )
            diagnostics.primitive_edges_enqueued += 1

    if solved_index is None:
        if not queue:
            diagnostics.queue_exhausted = True
        elif expansions >= cfg.max_expansions:
            diagnostics.expansion_budget_reached = True
        diagnostics.failure_class = classify_reverse_primitive_failure(
            diagnostics.freeze(), max_cusps=cfg.max_cusps
        )
        return ReversePrimitiveConnectorResult(
            connector_id=request.connector_id,
            from_aisle_id=request.from_aisle_id,
            to_aisle_id=request.to_aisle_id,
            turn_zone_id=request.turn_zone_id,
            status="NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION",
            backend=REVERSE_PRIMITIVE_BACKEND,
            minimum_turning_radius_m=radius,
            samples=(),
            path_length_m=None,
            forward_distance_m=0.0,
            reverse_distance_m=0.0,
            cusp_count=0,
            search_expansions=expansions,
            goal_position_error_m=None,
            goal_yaw_error_rad=None,
            centerline_evidence=_empty_evidence(),
            footprint_evidence=_empty_evidence(),
            reason=(
                "bounded F/R/F primitive search found no preview-footprint-free solution; "
                "Navigation Grid and Site Boundary gates remain fail-closed; hand this "
                "connector to R7 rather than expanding R6B into a global planner"
            ),
            diagnostics=diagnostics.freeze(),
        )

    raw = _reconstruct_raw(nodes, solved_index)
    raw.extend(solved_shot)
    samples = _finalize_samples(request, raw)
    path_length, forward_distance, reverse_distance, cusp_count = _path_metrics(
        samples
    )
    final = samples[-1]
    goal_position_error = math.hypot(
        request.goal_pose[0] - final.x,
        request.goal_pose[1] - final.y,
    )
    goal_yaw_error = abs(_wrap_pi(request.goal_pose[3] - final.yaw))
    centerline_evidence, footprint_evidence = _path_evidence(
        samples,
        navigation,
        local_footprint,
    )
    has_reverse = reverse_distance > 1.0e-6
    status = (
        "REVERSE_PRIMITIVE_PREVIEW_FREE"
        if has_reverse
        else "FORWARD_PRIMITIVE_PREVIEW_FREE"
    )
    reason = (
        "bounded local primitive path is FREE in frozen Navigation Grid"
        + (
            " and strictly inside the Site Boundary"
            if site_boundary is not None
            else ""
        )
        + " under the preview footprint assumption; this is not analytic Reeds-Shepp "
        "and is not R8 vehicle-READY evidence"
    )
    return ReversePrimitiveConnectorResult(
        connector_id=request.connector_id,
        from_aisle_id=request.from_aisle_id,
        to_aisle_id=request.to_aisle_id,
        turn_zone_id=request.turn_zone_id,
        status=status,
        backend=REVERSE_PRIMITIVE_BACKEND,
        minimum_turning_radius_m=radius,
        samples=samples,
        path_length_m=path_length,
        forward_distance_m=forward_distance,
        reverse_distance_m=reverse_distance,
        cusp_count=cusp_count,
        search_expansions=expansions,
        goal_position_error_m=goal_position_error,
        goal_yaw_error_rad=goal_yaw_error,
        centerline_evidence=centerline_evidence,
        footprint_evidence=footprint_evidence,
        reason=reason,
        diagnostics=diagnostics.freeze(),
    )


def derive_reverse_primitive_connector_plan(
    connector_requests: Iterable[ConnectorRequest],
    admission: ReverseFallbackAdmissionPlan,
    zones: TurnZoneSet,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    config: ReversePrimitiveConnectorConfig | None = None,
    *,
    site_boundary: SiteBoundary | None = None,
    source: Mapping[str, Any] | None = None,
) -> ReversePrimitiveConnectorPlan:
    """Plan bounded reverse-aware connectors for R6A-admitted IDs only."""
    cfg = config or ReversePrimitiveConnectorConfig()
    cfg.validate()
    if vehicle.kinematics != "ackermann":
        raise ValueError(
            "R6B reverse primitive backend currently requires Ackermann kinematics"
        )
    if not vehicle.planning_preview_ready:
        raise ValueError(
            f"vehicle profile {vehicle.profile_id} is not ready for planning preview"
        )
    if (
        not vehicle.minimum_turning_radius_verified
        or vehicle.minimum_turning_radius_m <= 0.0
    ):
        raise ValueError("R6B requires verified Ackermann minimum turning radius")
    if admission.platform_id and admission.platform_id != vehicle.profile_id:
        raise ValueError(
            "R6A admission platform does not match canonical vehicle profile"
        )
    if site_boundary is not None:
        site_boundary.validate(expected_frame_id=zones.frame_id)

    request_map = {
        request.connector_id: request for request in connector_requests
    }
    zone_map = {zone.zone_id: zone for zone in zones.zones}
    results: list[ReversePrimitiveConnectorResult] = []
    for connector_id in admission.eligible_connector_ids:
        request = request_map.get(connector_id)
        if request is None:
            raise ValueError(
                "R6A admitted connector is missing from coverage requests: "
                f"{connector_id}"
            )
        zone = zone_map.get(request.turn_zone_id)
        if zone is None or zone.side != request.side:
            raise ValueError(
                "R6A admitted connector has invalid Turn Zone metadata: "
                f"{connector_id}"
            )
        results.append(
            _search_one(
                request,
                zone,
                zones,
                navigation,
                vehicle,
                cfg,
                site_boundary,
            )
        )

    merged_source = dict(admission.source)
    merged_source.update(dict(navigation.source))
    merged_source.update(dict(source or {}))
    merged_source.update(
        {
            "backend": REVERSE_PRIMITIVE_BACKEND,
            "admission_scope": "R6A_ELIGIBLE_CONNECTOR_IDS_ONLY",
            "validation_scope": "PREVIEW_ONLY_NOT_R8_VEHICLE_READY",
            "z_semantics": "LINEAR_ENDPOINT_INTERPOLATION_PREVIEW_ONLY",
            "max_cusps": cfg.max_cusps,
            "minimum_turning_radius_m": float(vehicle.minimum_turning_radius_m),
            "site_boundary_enforced": site_boundary is not None,
        }
    )
    return ReversePrimitiveConnectorPlan(
        frame_id=zones.frame_id,
        platform_id=vehicle.profile_id,
        platform_profile_sha256=vehicle.profile_sha256,
        connectors=tuple(results),
        source=merged_source,
    )


def _evidence_to_dict(evidence: GridPathEvidence) -> dict[str, Any]:
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


def reverse_primitive_connector_plan_to_dict(
    plan: ReversePrimitiveConnectorPlan,
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
            "solved": plan.solved_count,
            "unsolved": plan.unsolved_count,
        },
        "connectors": [
            {
                "connector_id": item.connector_id,
                "from_aisle_id": item.from_aisle_id,
                "to_aisle_id": item.to_aisle_id,
                "turn_zone_id": item.turn_zone_id,
                "status": item.status,
                "backend": item.backend,
                "validation_scope": "PREVIEW_ONLY_NOT_R8_VEHICLE_READY",
                "minimum_turning_radius_m": item.minimum_turning_radius_m,
                "path_length_m": item.path_length_m,
                "forward_distance_m": item.forward_distance_m,
                "reverse_distance_m": item.reverse_distance_m,
                "cusp_count": item.cusp_count,
                "search_expansions": item.search_expansions,
                "goal_position_error_m": item.goal_position_error_m,
                "goal_yaw_error_rad": item.goal_yaw_error_rad,
                "centerline_evidence": _evidence_to_dict(
                    item.centerline_evidence
                ),
                "preview_footprint_evidence": _evidence_to_dict(
                    item.footprint_evidence
                ),
                "reason": item.reason,
                "diagnostics": reverse_primitive_search_diagnostics_to_dict(
                    item.diagnostics
                ),
                "samples": [
                    {
                        "x": sample.x,
                        "y": sample.y,
                        "z": sample.z,
                        "yaw": sample.yaw,
                        "motion_direction": sample.motion_direction,
                        "curvature_per_m": sample.curvature_per_m,
                        "segment_index": sample.segment_index,
                        "is_cusp": sample.is_cusp,
                    }
                    for sample in item.samples
                ],
            }
            for item in plan.connectors
        ],
    }


def write_reverse_primitive_connector_plan(
    plan: ReversePrimitiveConnectorPlan,
    path: str | Path,
) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(
            reverse_primitive_connector_plan_to_dict(plan),
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )
    return output
