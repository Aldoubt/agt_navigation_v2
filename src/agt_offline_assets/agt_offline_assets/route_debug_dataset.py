"""Read-only aggregation of frozen route-production evidence for 2D Route Debug.

Route Debug is an explainability layer, not a planner.  This module only reads
already-frozen route assets, validates their contracts, joins cause-chain
records, and exposes immutable data for the renderer.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

from .agricultural_aisle_graph import AislePrimitive, AgriculturalAisleGraph
from .agricultural_coverage_ordering import COVERAGE_ORDER_SCHEMA, ConnectorRequest
from .agricultural_route_io import load_agricultural_aisle_graph, load_turn_zones
from .forward_connector import FORWARD_CONNECTOR_SCHEMA
from .forward_connector_candidate_audit import FORWARD_CONNECTOR_CANDIDATE_AUDIT_SCHEMA
from .forward_connector_diagnostics import FORWARD_CONNECTOR_ZONE_FIT_SCHEMA
from .navigation_grid import NavigationGridEvidence, load_navigation_grid
from .navigation_map_derivation import NAVIGATION_DERIVATION_SCHEMA
from .reverse_fallback_admission import REVERSE_FALLBACK_ADMISSION_SCHEMA
from .reverse_primitive_connector import REVERSE_PRIMITIVE_CONNECTOR_SCHEMA
from .route_diagnostic_io import (
    load_forward_connector_candidate_audit,
    load_forward_connector_zone_fit_report,
)
from .turn_zones import TurnZoneSet
from .vehicle_profile import CanonicalVehicleProfile, load_canonical_vehicle_profile
from .vehicle_safe_lane import VEHICLE_SAFE_LANE_SCHEMA
from .vehicle_safe_lane_diagnostics import VEHICLE_SAFE_LANE_DIAGNOSTIC_SCHEMA
from .vehicle_safe_lane_occupancy_sources import (
    VEHICLE_SAFE_LANE_OCCUPANCY_SOURCE_SCHEMA,
    NavigationOccupancySourceMasks,
    load_navigation_occupancy_source_masks,
)

ASSET_LOADED = "LOADED"
ASSET_MISSING = "MISSING"
ASSET_INVALID = "INVALID"


@dataclass(frozen=True)
class RouteDebugAssetState:
    key: str
    availability: str
    path: str | None
    schema: str | None
    frame_id: str | None
    status: str | None
    error: str = ""


@dataclass(frozen=True)
class RouteDebugNoGoRegion:
    region_id: str
    polygon_xy: tuple[tuple[float, float], ...]
    source_asset: str


@dataclass(frozen=True)
class RouteDebugCoverageTraversal:
    sequence: int
    aisle_id: str
    motion_direction: str
    graph_orientation: str
    entry_side: str
    exit_side: str
    geometric_width_m: float
    required_width_m: float


@dataclass(frozen=True)
class RouteDebugCoverageRejection:
    aisle_id: str
    reason: str
    geometric_width_m: float
    required_width_m: float


@dataclass(frozen=True)
class RouteDebugMotionSample:
    x: float
    y: float
    z: float
    yaw: float
    motion_direction: str
    segment_index: int | None = None
    is_cusp: bool = False


@dataclass(frozen=True)
class RouteDebugForwardCandidateRecord:
    path_type: str
    length_m: float
    local_headland_candidate: bool
    preview_footprint_free: bool
    max_required_zone_extension_m: float


@dataclass(frozen=True)
class RouteDebugVehicleLaneRecord:
    aisle_id: str
    status: str
    coverage_fraction: float
    centerline_xyz: tuple[tuple[float, float, float], ...]
    reason: str


@dataclass(frozen=True)
class RouteDebugLaneDiagnosticRecord:
    aisle_id: str
    classification: str
    reference_free_fraction: float
    any_lateral_footprint_free_fraction: float
    reason: str = ""


@dataclass(frozen=True)
class RouteDebugOccupancySourceRecord:
    aisle_id: str
    classification: str
    raw_obstacle_fraction: float
    geometry_fraction: float
    padding_fraction: float
    unexplained_fraction: float
    reason: str = ""


@dataclass(frozen=True)
class RouteDebugAisleRecord:
    aisle_id: str
    aisle: AislePrimitive
    traversal: RouteDebugCoverageTraversal | None
    rejection: RouteDebugCoverageRejection | None
    vehicle_lane: RouteDebugVehicleLaneRecord | None = None
    diagnostic: RouteDebugLaneDiagnosticRecord | None = None
    occupancy_source: RouteDebugOccupancySourceRecord | None = None


@dataclass(frozen=True)
class RouteDebugConnectorRecord:
    connector_id: str
    from_aisle_id: str
    to_aisle_id: str
    turn_zone_id: str
    request: ConnectorRequest | None
    forward_status: str | None = None
    forward_backend: str | None = None
    forward_path_type: str | None = None
    forward_samples: tuple[RouteDebugMotionSample, ...] = ()
    zone_fit_status: str | None = None
    forward_audit_status: str | None = None
    forward_candidates: tuple[RouteDebugForwardCandidateRecord, ...] = ()
    r6a_decision: str | None = None
    r6b_status: str | None = None
    r6b_backend: str | None = None
    r6b_samples: tuple[RouteDebugMotionSample, ...] = ()
    r6b_path_length_m: float | None = None
    r6b_forward_distance_m: float | None = None
    r6b_reverse_distance_m: float | None = None
    r6b_cusp_count: int | None = None
    r6b_search_expansions: int | None = None
    r6b_goal_position_error_m: float | None = None
    r6b_goal_yaw_error_rad: float | None = None


@dataclass(frozen=True)
class RouteDebugDataset:
    run_dir: Path
    frame_id: str
    navigation: NavigationGridEvidence | None
    aisle_graph: AgriculturalAisleGraph | None
    turn_zones: TurnZoneSet | None
    vehicle_profile: CanonicalVehicleProfile | None
    no_go_regions: tuple[RouteDebugNoGoRegion, ...]
    aisles: tuple[RouteDebugAisleRecord, ...]
    connector_requests: tuple[ConnectorRequest, ...]
    connectors: tuple[RouteDebugConnectorRecord, ...]
    occupancy_source_masks: NavigationOccupancySourceMasks | None
    asset_states: tuple[RouteDebugAssetState, ...]

    def asset_state(self, key: str) -> RouteDebugAssetState:
        for state in self.asset_states:
            if state.key == key:
                return state
        return RouteDebugAssetState(key, ASSET_MISSING, None, None, None, None)

    def aisle_by_id(self, aisle_id: str) -> RouteDebugAisleRecord | None:
        return next((record for record in self.aisles if record.aisle_id == aisle_id), None)

    def connector_by_id(self, connector_id: str) -> RouteDebugConnectorRecord | None:
        return next((record for record in self.connectors if record.connector_id == connector_id), None)


@dataclass(frozen=True)
class _CoverageDebug:
    frame_id: str
    platform_id: str
    status: str
    traversals: tuple[RouteDebugCoverageTraversal, ...]
    requests: tuple[ConnectorRequest, ...]
    rejections: tuple[RouteDebugCoverageRejection, ...]


def _load_yaml(path: Path) -> Mapping[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(payload, Mapping):
        raise ValueError(f"{path.name} must contain a YAML mapping")
    return payload


def _state(
    key: str,
    availability: str,
    path: Path | None,
    *,
    payload: Mapping[str, Any] | None = None,
    error: str = "",
) -> RouteDebugAssetState:
    return RouteDebugAssetState(
        key=key,
        availability=availability,
        path=None if path is None else str(path),
        schema=None if payload is None else str(payload.get("schema") or "") or None,
        frame_id=None if payload is None else str(payload.get("frame_id") or "") or None,
        status=None if payload is None else str(payload.get("status") or "") or None,
        error=error,
    )


def _frame_accepts(authoritative: str | None, candidate: str | None) -> bool:
    return not authoritative or not candidate or str(authoritative) == str(candidate)


def _validate_schema_frame(
    path: Path,
    expected_schema: str,
    frame_id: str,
) -> Mapping[str, Any]:
    payload = _load_yaml(path)
    schema = str(payload.get("schema") or "")
    if schema != expected_schema:
        raise ValueError(f"expected {expected_schema}, got {schema or '<empty>'}")
    candidate_frame = str(payload.get("frame_id") or frame_id)
    if not _frame_accepts(frame_id, candidate_frame):
        raise ValueError(f"frame_id mismatch: expected {frame_id}, got {candidate_frame}")
    return payload


def _pose(value: Any, field: str) -> tuple[float, float, float, float]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field} must be a mapping")
    return (
        float(value["x"]),
        float(value["y"]),
        float(value.get("z", 0.0)),
        float(value["yaw"]),
    )


def _load_debug_coverage(path: Path) -> _CoverageDebug:
    payload = _load_yaml(path)
    schema = str(payload.get("schema") or "")
    if schema != COVERAGE_ORDER_SCHEMA:
        raise ValueError(f"expected {COVERAGE_ORDER_SCHEMA}, got {schema or '<empty>'}")

    traversals: list[RouteDebugCoverageTraversal] = []
    raw_traversals = payload.get("traversals") or []
    if not isinstance(raw_traversals, list):
        raise ValueError("traversals must be a list")
    for index, raw in enumerate(raw_traversals):
        if not isinstance(raw, Mapping):
            raise ValueError(f"traversals[{index}] must be a mapping")
        traversals.append(
            RouteDebugCoverageTraversal(
                sequence=int(raw["sequence"]),
                aisle_id=str(raw["aisle_id"]),
                motion_direction=str(raw.get("motion_direction", "FORWARD")),
                graph_orientation=str(raw.get("graph_orientation", "")),
                entry_side=str(raw.get("entry_side", "")),
                exit_side=str(raw.get("exit_side", "")),
                geometric_width_m=float(raw.get("geometric_width_m", 0.0)),
                required_width_m=float(raw.get("required_width_m", 0.0)),
            )
        )

    requests: list[ConnectorRequest] = []
    raw_requests = payload.get("connector_requests") or []
    if not isinstance(raw_requests, list):
        raise ValueError("connector_requests must be a list")
    for index, raw in enumerate(raw_requests):
        if not isinstance(raw, Mapping):
            raise ValueError(f"connector_requests[{index}] must be a mapping")
        requests.append(
            ConnectorRequest(
                connector_id=str(raw["connector_id"]),
                from_aisle_id=str(raw["from_aisle_id"]),
                to_aisle_id=str(raw["to_aisle_id"]),
                turn_zone_id=str(raw["turn_zone_id"]),
                side=str(raw.get("side", "")),
                start_pose=_pose(raw["start_pose"], f"connector_requests[{index}].start_pose"),
                goal_pose=_pose(raw["goal_pose"], f"connector_requests[{index}].goal_pose"),
            )
        )

    rejections: list[RouteDebugCoverageRejection] = []
    raw_rejections = payload.get("rejected_aisles") or []
    if not isinstance(raw_rejections, list):
        raise ValueError("rejected_aisles must be a list")
    for index, raw in enumerate(raw_rejections):
        if not isinstance(raw, Mapping):
            raise ValueError(f"rejected_aisles[{index}] must be a mapping")
        rejections.append(
            RouteDebugCoverageRejection(
                aisle_id=str(raw["aisle_id"]),
                reason=str(raw.get("reason", "")),
                geometric_width_m=float(raw.get("geometric_width_m", 0.0)),
                required_width_m=float(raw.get("required_width_m", 0.0)),
            )
        )

    return _CoverageDebug(
        frame_id=str(payload.get("frame_id", "map")),
        platform_id=str(payload.get("platform_id", "")),
        status=str(payload.get("status", "DRAFT")),
        traversals=tuple(traversals),
        requests=tuple(requests),
        rejections=tuple(rejections),
    )


def _load_no_go_regions(path: Path) -> tuple[RouteDebugNoGoRegion, ...]:
    payload = _load_yaml(path)
    if str(payload.get("schema") or "") != NAVIGATION_DERIVATION_SCHEMA:
        raise ValueError(f"expected {NAVIGATION_DERIVATION_SCHEMA}")
    raw_overrides = payload.get("overrides") or []
    if not isinstance(raw_overrides, list):
        raise ValueError("derivation overrides must be a list")
    output: list[RouteDebugNoGoRegion] = []
    for index, raw in enumerate(raw_overrides):
        if not isinstance(raw, Mapping):
            raise ValueError(f"overrides[{index}] must be a mapping")
        if str(raw.get("mode", "")).lower() != "no_go":
            continue
        polygon = raw.get("polygon_xy") or []
        if not isinstance(polygon, Sequence) or isinstance(polygon, (str, bytes)) or len(polygon) < 3:
            raise ValueError(f"no_go override {index} has invalid polygon_xy")
        points: list[tuple[float, float]] = []
        for vertex in polygon:
            if not isinstance(vertex, Sequence) or isinstance(vertex, (str, bytes)) or len(vertex) < 2:
                raise ValueError(f"no_go override {index} contains an invalid vertex")
            points.append((float(vertex[0]), float(vertex[1])))
        output.append(
            RouteDebugNoGoRegion(
                region_id=str(
                    raw.get("region_id")
                    or raw.get("override_id")
                    or raw.get("id")
                    or f"no_go_{index + 1:03d}"
                ),
                polygon_xy=tuple(points),
                source_asset=path.name,
            )
        )
    return tuple(output)


def _discover_yaml_by_schema(run_dir: Path) -> dict[str, tuple[Path, ...]]:
    found: dict[str, list[Path]] = {}
    for path in sorted(run_dir.glob("*.y*ml")):
        try:
            schema = str(_load_yaml(path).get("schema") or "")
        except Exception:
            continue
        if schema:
            found.setdefault(schema, []).append(path)
    return {schema: tuple(paths) for schema, paths in found.items()}


def _choose_reverse_primitive_asset(paths: Sequence[Path]) -> Path | None:
    if not paths:
        return None
    by_name = {path.name: path for path in paths}
    for preferred in (
        "reverse_primitive_connectors_anchored.yaml",
        "reverse_primitive_connectors.yaml",
    ):
        if preferred in by_name:
            return by_name[preferred]
    return sorted(paths, key=lambda item: item.name)[0]


def _discover_vehicle_profile(run_dir: Path, platform_id: str) -> Path | None:
    if not platform_id:
        return None
    for root in (run_dir, *run_dir.parents):
        path = root / "profiles" / "platforms" / f"{platform_id}.yaml"
        if path.is_file():
            return path.resolve()
    return None


def _motion_samples(raw_samples: Any, *, field: str = "samples") -> tuple[RouteDebugMotionSample, ...]:
    if raw_samples is None:
        return ()
    if not isinstance(raw_samples, list):
        raise ValueError(f"{field} must be a list")
    output: list[RouteDebugMotionSample] = []
    for index, raw in enumerate(raw_samples):
        if not isinstance(raw, Mapping):
            raise ValueError(f"{field}[{index}] must be a mapping")
        output.append(
            RouteDebugMotionSample(
                x=float(raw["x"]),
                y=float(raw["y"]),
                z=float(raw.get("z", 0.0)),
                yaw=float(raw["yaw"]),
                motion_direction=str(raw.get("motion_direction", "FORWARD")),
                segment_index=(
                    None if raw.get("segment_index") is None else int(raw["segment_index"])
                ),
                is_cusp=bool(raw.get("is_cusp", False)),
            )
        )
    return tuple(output)


def _connector_mapping(payload: Mapping[str, Any], *, validate_samples: bool = False) -> dict[str, Mapping[str, Any]]:
    raw_items = payload.get("connectors") or []
    if not isinstance(raw_items, list):
        raise ValueError("connectors must be a list")
    output: dict[str, Mapping[str, Any]] = {}
    for index, raw in enumerate(raw_items):
        if not isinstance(raw, Mapping):
            raise ValueError(f"connectors[{index}] must be a mapping")
        connector_id = str(raw.get("connector_id") or "")
        if not connector_id:
            raise ValueError(f"connectors[{index}].connector_id is required")
        if validate_samples:
            _motion_samples(raw.get("samples"), field=f"connectors[{index}].samples")
        output[connector_id] = raw
    return output


def _parse_lane_records(payload: Mapping[str, Any]) -> dict[str, RouteDebugVehicleLaneRecord]:
    raw_items = payload.get("aisles") or []
    if not isinstance(raw_items, list):
        raise ValueError("vehicle-safe-lane aisles must be a list")
    output: dict[str, RouteDebugVehicleLaneRecord] = {}
    for index, raw in enumerate(raw_items):
        if not isinstance(raw, Mapping):
            raise ValueError(f"aisles[{index}] must be a mapping")
        aisle_id = str(raw.get("aisle_id") or "")
        if not aisle_id:
            raise ValueError(f"aisles[{index}].aisle_id is required")
        raw_points = raw.get("centerline_xyz") or []
        if not isinstance(raw_points, list):
            raise ValueError(f"aisles[{index}].centerline_xyz must be a list")
        points: list[tuple[float, float, float]] = []
        for point_index, point in enumerate(raw_points):
            if not isinstance(point, Sequence) or isinstance(point, (str, bytes)) or len(point) < 3:
                raise ValueError(f"aisles[{index}].centerline_xyz[{point_index}] is invalid")
            points.append((float(point[0]), float(point[1]), float(point[2])))
        output[aisle_id] = RouteDebugVehicleLaneRecord(
            aisle_id=aisle_id,
            status=str(raw.get("status", "")),
            coverage_fraction=float(raw.get("coverage_fraction", 0.0)),
            centerline_xyz=tuple(points),
            reason=str(raw.get("reason", "")),
        )
    return output


def _parse_lane_diagnostics(payload: Mapping[str, Any]) -> dict[str, RouteDebugLaneDiagnosticRecord]:
    raw_items = payload.get("aisles") or []
    if not isinstance(raw_items, list):
        raise ValueError("vehicle-safe-lane diagnostic aisles must be a list")
    output: dict[str, RouteDebugLaneDiagnosticRecord] = {}
    for index, raw in enumerate(raw_items):
        if not isinstance(raw, Mapping):
            raise ValueError(f"aisles[{index}] must be a mapping")
        aisle_id = str(raw.get("aisle_id") or "")
        if not aisle_id:
            raise ValueError(f"aisles[{index}].aisle_id is required")
        reference = raw.get("reference_point") or {}
        feasibility = raw.get("pose_feasibility") or {}
        if not isinstance(reference, Mapping) or not isinstance(feasibility, Mapping):
            raise ValueError(f"aisles[{index}] diagnostic subrecords must be mappings")
        output[aisle_id] = RouteDebugLaneDiagnosticRecord(
            aisle_id=aisle_id,
            classification=str(raw.get("classification", "")),
            reference_free_fraction=float(reference.get("free_fraction", 0.0)),
            any_lateral_footprint_free_fraction=float(
                feasibility.get("any_lateral_footprint_free_fraction", 0.0)
            ),
            reason=str(raw.get("reason", "")),
        )
    return output


def _parse_source_records(payload: Mapping[str, Any]) -> dict[str, RouteDebugOccupancySourceRecord]:
    raw_items = payload.get("aisles") or []
    if not isinstance(raw_items, list):
        raise ValueError("occupancy-source aisles must be a list")
    output: dict[str, RouteDebugOccupancySourceRecord] = {}
    for index, raw in enumerate(raw_items):
        if not isinstance(raw, Mapping):
            raise ValueError(f"aisles[{index}] must be a mapping")
        aisle_id = str(raw.get("aisle_id") or "")
        if not aisle_id:
            raise ValueError(f"aisles[{index}].aisle_id is required")
        sources = raw.get("sources") or {}
        if not isinstance(sources, Mapping):
            raise ValueError(f"aisles[{index}].sources must be a mapping")
        output[aisle_id] = RouteDebugOccupancySourceRecord(
            aisle_id=aisle_id,
            classification=str(raw.get("classification", "")),
            raw_obstacle_fraction=float(sources.get("raw_obstacle_fraction", 0.0)),
            geometry_fraction=float(sources.get("geometry_fraction", 0.0)),
            padding_fraction=float(sources.get("padding_fraction", 0.0)),
            unexplained_fraction=float(sources.get("unexplained_fraction", 0.0)),
            reason=str(raw.get("reason", "")),
        )
    return output


def _load_optional_raw(
    key: str,
    path: Path | None,
    schema: str,
    frame_id: str,
    states: list[RouteDebugAssetState],
    parser,
):
    if path is None:
        states.append(_state(key, ASSET_MISSING, None))
        return None, None
    try:
        payload = _validate_schema_frame(path, schema, frame_id)
        parsed = parser(payload)
    except Exception as exc:
        try:
            payload = _load_yaml(path)
        except Exception:
            payload = None
        states.append(_state(key, ASSET_INVALID, path, payload=payload, error=str(exc)))
        return None, None
    states.append(_state(key, ASSET_LOADED, path, payload=payload))
    return payload, parsed


def _load_optional_diagnostic(
    key: str,
    path: Path | None,
    schema: str,
    frame_id: str,
    states: list[RouteDebugAssetState],
    loader,
):
    if path is None:
        states.append(_state(key, ASSET_MISSING, None))
        return None, None
    try:
        payload = _validate_schema_frame(path, schema, frame_id)
        parsed = loader(path)
    except Exception as exc:
        try:
            payload = _load_yaml(path)
        except Exception:
            payload = None
        states.append(_state(key, ASSET_INVALID, path, payload=payload, error=str(exc)))
        return None, None
    states.append(_state(key, ASSET_LOADED, path, payload=payload))
    return payload, parsed


def _record_value(record: Any, field: str) -> Any:
    if record is None:
        return None
    if isinstance(record, Mapping):
        return record.get(field)
    return getattr(record, field, None)


def _join_connector_records(
    requests: tuple[ConnectorRequest, ...],
    forward: Mapping[str, Mapping[str, Any]],
    zone_fit: Mapping[str, Any],
    audit: Mapping[str, Any],
    r6a: Mapping[str, Mapping[str, Any]],
    r6b: Mapping[str, Mapping[str, Any]],
) -> tuple[RouteDebugConnectorRecord, ...]:
    request_map = {request.connector_id: request for request in requests}
    ids = set(request_map) | set(forward) | set(zone_fit) | set(audit) | set(r6a) | set(r6b)
    output: list[RouteDebugConnectorRecord] = []

    for connector_id in sorted(ids):
        request = request_map.get(connector_id)
        forward_item = forward.get(connector_id)
        zone_item = zone_fit.get(connector_id)
        audit_item = audit.get(connector_id)
        r6a_item = r6a.get(connector_id)
        r6b_item = r6b.get(connector_id)

        def choose(field: str) -> str:
            if request is not None and hasattr(request, field):
                value = getattr(request, field)
                if value:
                    return str(value)
            for record in (forward_item, zone_item, audit_item, r6a_item, r6b_item):
                value = _record_value(record, field)
                if value:
                    return str(value)
            return ""

        candidates: tuple[RouteDebugForwardCandidateRecord, ...] = ()
        if audit_item is not None:
            candidates = tuple(
                RouteDebugForwardCandidateRecord(
                    path_type=str(item.path_type),
                    length_m=float(item.length_m),
                    local_headland_candidate=bool(item.local_headland_candidate),
                    preview_footprint_free=bool(item.preview_footprint_free),
                    max_required_zone_extension_m=float(item.max_required_zone_extension_m),
                )
                for item in audit_item.candidates
            )

        output.append(
            RouteDebugConnectorRecord(
                connector_id=connector_id,
                from_aisle_id=choose("from_aisle_id"),
                to_aisle_id=choose("to_aisle_id"),
                turn_zone_id=choose("turn_zone_id"),
                request=request,
                forward_status=(
                    None if forward_item is None else str(forward_item.get("status", "")) or None
                ),
                forward_backend=(
                    None if forward_item is None else str(forward_item.get("backend", "")) or None
                ),
                forward_path_type=(
                    None
                    if forward_item is None or forward_item.get("path_type") is None
                    else str(forward_item["path_type"])
                ),
                forward_samples=(
                    ()
                    if forward_item is None
                    else _motion_samples(forward_item.get("samples"), field=f"{connector_id}.forward.samples")
                ),
                zone_fit_status=(None if zone_item is None else str(zone_item.status)),
                forward_audit_status=(None if audit_item is None else str(audit_item.status)),
                forward_candidates=candidates,
                r6a_decision=(
                    None if r6a_item is None else str(r6a_item.get("decision", "")) or None
                ),
                r6b_status=(
                    None if r6b_item is None else str(r6b_item.get("status", "")) or None
                ),
                r6b_backend=(
                    None if r6b_item is None else str(r6b_item.get("backend", "")) or None
                ),
                r6b_samples=(
                    ()
                    if r6b_item is None
                    else _motion_samples(r6b_item.get("samples"), field=f"{connector_id}.r6b.samples")
                ),
                r6b_path_length_m=(
                    None if r6b_item is None or r6b_item.get("path_length_m") is None
                    else float(r6b_item["path_length_m"])
                ),
                r6b_forward_distance_m=(
                    None if r6b_item is None or r6b_item.get("forward_distance_m") is None
                    else float(r6b_item["forward_distance_m"])
                ),
                r6b_reverse_distance_m=(
                    None if r6b_item is None or r6b_item.get("reverse_distance_m") is None
                    else float(r6b_item["reverse_distance_m"])
                ),
                r6b_cusp_count=(
                    None if r6b_item is None or r6b_item.get("cusp_count") is None
                    else int(r6b_item["cusp_count"])
                ),
                r6b_search_expansions=(
                    None if r6b_item is None or r6b_item.get("search_expansions") is None
                    else int(r6b_item["search_expansions"])
                ),
                r6b_goal_position_error_m=(
                    None if r6b_item is None or r6b_item.get("goal_position_error_m") is None
                    else float(r6b_item["goal_position_error_m"])
                ),
                r6b_goal_yaw_error_rad=(
                    None if r6b_item is None or r6b_item.get("goal_yaw_error_rad") is None
                    else float(r6b_item["goal_yaw_error_rad"])
                ),
            )
        )
    return tuple(output)


def _platform_id_from_payloads(
    coverage: _CoverageDebug | None,
    payloads: Sequence[Mapping[str, Any] | None],
) -> str:
    if coverage is not None and coverage.platform_id:
        return coverage.platform_id
    for payload in payloads:
        if payload is None:
            continue
        platform_id = str(payload.get("platform_id") or "")
        if platform_id:
            return platform_id
    return ""


def load_route_debug_dataset(
    run_dir: str | Path,
    *,
    vehicle_profile_path: str | Path | None = None,
) -> RouteDebugDataset:
    """Load one frozen run directory for read-only Route Debug rendering."""
    root = Path(run_dir).expanduser().resolve()
    if not root.is_dir():
        raise NotADirectoryError(f"route debug run directory not found: {root}")

    states: list[RouteDebugAssetState] = []
    authoritative_frame: str | None = None
    navigation: NavigationGridEvidence | None = None
    aisle_graph: AgriculturalAisleGraph | None = None
    turn_zones: TurnZoneSet | None = None
    coverage: _CoverageDebug | None = None

    nav_path = root / "navigation_map.yaml"
    if nav_path.is_file():
        try:
            navigation = load_navigation_grid(nav_path)
            authoritative_frame = navigation.frame_id
            states.append(_state("navigation", ASSET_LOADED, nav_path, payload={"frame_id": navigation.frame_id}))
        except Exception as exc:
            states.append(_state("navigation", ASSET_INVALID, nav_path, error=str(exc)))
    else:
        states.append(_state("navigation", ASSET_MISSING, None))

    aisle_path = root / "aisle_graph.yaml"
    if aisle_path.is_file():
        try:
            candidate = load_agricultural_aisle_graph(aisle_path)
            if not _frame_accepts(authoritative_frame, candidate.frame_id):
                raise ValueError(
                    f"frame_id mismatch: expected {authoritative_frame}, got {candidate.frame_id}"
                )
            aisle_graph = candidate
            authoritative_frame = authoritative_frame or candidate.frame_id
            states.append(_state("aisle_graph", ASSET_LOADED, aisle_path, payload=_load_yaml(aisle_path)))
        except Exception as exc:
            states.append(_state("aisle_graph", ASSET_INVALID, aisle_path, error=str(exc)))
    else:
        states.append(_state("aisle_graph", ASSET_MISSING, None))

    coverage_path = root / "coverage_order.yaml"
    if coverage_path.is_file():
        try:
            candidate = _load_debug_coverage(coverage_path)
            if not _frame_accepts(authoritative_frame, candidate.frame_id):
                raise ValueError(
                    f"frame_id mismatch: expected {authoritative_frame}, got {candidate.frame_id}"
                )
            coverage = candidate
            authoritative_frame = authoritative_frame or candidate.frame_id
            states.append(_state("coverage_order", ASSET_LOADED, coverage_path, payload=_load_yaml(coverage_path)))
        except Exception as exc:
            states.append(_state("coverage_order", ASSET_INVALID, coverage_path, error=str(exc)))
    else:
        states.append(_state("coverage_order", ASSET_MISSING, None))

    frame_id = authoritative_frame or "map"

    turn_path = root / "turn_zones.yaml"
    if turn_path.is_file():
        try:
            candidate = load_turn_zones(turn_path)
            if not _frame_accepts(frame_id, candidate.frame_id):
                raise ValueError(f"frame_id mismatch: expected {frame_id}, got {candidate.frame_id}")
            turn_zones = candidate
            states.append(_state("turn_zones", ASSET_LOADED, turn_path, payload=_load_yaml(turn_path)))
        except Exception as exc:
            states.append(_state("turn_zones", ASSET_INVALID, turn_path, error=str(exc)))
    else:
        states.append(_state("turn_zones", ASSET_MISSING, None))

    derivation_path = root / "derivation.yaml"
    no_go_regions: tuple[RouteDebugNoGoRegion, ...] = ()
    if derivation_path.is_file():
        try:
            payload = _load_yaml(derivation_path)
            if str(payload.get("schema") or "") != NAVIGATION_DERIVATION_SCHEMA:
                raise ValueError(f"expected {NAVIGATION_DERIVATION_SCHEMA}")
            derivation_frame = str(payload.get("frame_id") or frame_id)
            if not _frame_accepts(frame_id, derivation_frame):
                raise ValueError(f"frame_id mismatch: expected {frame_id}, got {derivation_frame}")
            no_go_regions = _load_no_go_regions(derivation_path)
            states.append(_state("derivation", ASSET_LOADED, derivation_path, payload=payload))
        except Exception as exc:
            try:
                payload = _load_yaml(derivation_path)
            except Exception:
                payload = None
            states.append(_state("derivation", ASSET_INVALID, derivation_path, payload=payload, error=str(exc)))
    else:
        states.append(_state("derivation", ASSET_MISSING, None))

    discovered = _discover_yaml_by_schema(root)

    def first(schema: str) -> Path | None:
        paths = discovered.get(schema, ())
        return paths[0] if paths else None

    forward_path = first(FORWARD_CONNECTOR_SCHEMA)
    zone_fit_path = first(FORWARD_CONNECTOR_ZONE_FIT_SCHEMA)
    audit_path = first(FORWARD_CONNECTOR_CANDIDATE_AUDIT_SCHEMA)
    r6a_path = first(REVERSE_FALLBACK_ADMISSION_SCHEMA)
    r6b_path = _choose_reverse_primitive_asset(discovered.get(REVERSE_PRIMITIVE_CONNECTOR_SCHEMA, ()))
    lane_path = first(VEHICLE_SAFE_LANE_SCHEMA)
    diagnostic_path = first(VEHICLE_SAFE_LANE_DIAGNOSTIC_SCHEMA)
    source_path = first(VEHICLE_SAFE_LANE_OCCUPANCY_SOURCE_SCHEMA)

    forward_payload, forward_map = _load_optional_raw(
        "forward_connector",
        forward_path,
        FORWARD_CONNECTOR_SCHEMA,
        frame_id,
        states,
        lambda payload: _connector_mapping(payload, validate_samples=True),
    )
    zone_payload, zone_report = _load_optional_diagnostic(
        "forward_zone_fit",
        zone_fit_path,
        FORWARD_CONNECTOR_ZONE_FIT_SCHEMA,
        frame_id,
        states,
        load_forward_connector_zone_fit_report,
    )
    audit_payload, audit_plan = _load_optional_diagnostic(
        "forward_candidate_audit",
        audit_path,
        FORWARD_CONNECTOR_CANDIDATE_AUDIT_SCHEMA,
        frame_id,
        states,
        load_forward_connector_candidate_audit,
    )
    r6a_payload, r6a_map = _load_optional_raw(
        "reverse_fallback_admission",
        r6a_path,
        REVERSE_FALLBACK_ADMISSION_SCHEMA,
        frame_id,
        states,
        lambda payload: _connector_mapping(payload),
    )
    r6b_payload, r6b_map = _load_optional_raw(
        "reverse_primitive_connectors",
        r6b_path,
        REVERSE_PRIMITIVE_CONNECTOR_SCHEMA,
        frame_id,
        states,
        lambda payload: _connector_mapping(payload, validate_samples=True),
    )
    lane_payload, lane_map = _load_optional_raw(
        "vehicle_safe_lane",
        lane_path,
        VEHICLE_SAFE_LANE_SCHEMA,
        frame_id,
        states,
        _parse_lane_records,
    )
    diagnostic_payload, diagnostic_map = _load_optional_raw(
        "vehicle_safe_lane_diagnostic",
        diagnostic_path,
        VEHICLE_SAFE_LANE_DIAGNOSTIC_SCHEMA,
        frame_id,
        states,
        _parse_lane_diagnostics,
    )
    source_payload, source_map = _load_optional_raw(
        "vehicle_safe_lane_occupancy_source",
        source_path,
        VEHICLE_SAFE_LANE_OCCUPANCY_SOURCE_SCHEMA,
        frame_id,
        states,
        _parse_source_records,
    )

    forward_map = forward_map or {}
    r6a_map = r6a_map or {}
    r6b_map = r6b_map or {}
    lane_map = lane_map or {}
    diagnostic_map = diagnostic_map or {}
    source_map = source_map or {}
    zone_map = (
        {} if zone_report is None else {item.connector_id: item for item in zone_report.diagnostics}
    )
    audit_map = (
        {} if audit_plan is None else {item.connector_id: item for item in audit_plan.connectors}
    )

    platform_id = _platform_id_from_payloads(
        coverage,
        (
            forward_payload,
            zone_payload,
            audit_payload,
            r6a_payload,
            r6b_payload,
            lane_payload,
            diagnostic_payload,
            source_payload,
        ),
    )
    profile_path = (
        Path(vehicle_profile_path).expanduser().resolve()
        if vehicle_profile_path is not None
        else _discover_vehicle_profile(root, platform_id)
    )
    vehicle_profile: CanonicalVehicleProfile | None = None
    if profile_path is None:
        states.append(_state("vehicle_profile", ASSET_MISSING, None))
    else:
        try:
            vehicle_profile = load_canonical_vehicle_profile(profile_path)
            if platform_id and vehicle_profile.profile_id != platform_id:
                raise ValueError(
                    f"platform_id mismatch: route assets={platform_id}, profile={vehicle_profile.profile_id}"
                )
            states.append(_state("vehicle_profile", ASSET_LOADED, profile_path))
        except Exception as exc:
            states.append(_state("vehicle_profile", ASSET_INVALID, profile_path, error=str(exc)))

    occupancy_source_masks: NavigationOccupancySourceMasks | None = None
    if navigation is None:
        states.append(_state("occupancy_source_masks", ASSET_MISSING, None))
    else:
        try:
            occupancy_source_masks = load_navigation_occupancy_source_masks(navigation, root)
            states.append(_state("occupancy_source_masks", ASSET_LOADED, root))
        except FileNotFoundError as exc:
            states.append(_state("occupancy_source_masks", ASSET_MISSING, None, error=str(exc)))
        except Exception as exc:
            states.append(_state("occupancy_source_masks", ASSET_INVALID, root, error=str(exc)))

    traversal_map = (
        {} if coverage is None else {record.aisle_id: record for record in coverage.traversals}
    )
    rejection_map = (
        {} if coverage is None else {record.aisle_id: record for record in coverage.rejections}
    )
    aisles = (
        ()
        if aisle_graph is None
        else tuple(
            RouteDebugAisleRecord(
                aisle_id=aisle.aisle_id,
                aisle=aisle,
                traversal=traversal_map.get(aisle.aisle_id),
                rejection=rejection_map.get(aisle.aisle_id),
                vehicle_lane=lane_map.get(aisle.aisle_id),
                diagnostic=diagnostic_map.get(aisle.aisle_id),
                occupancy_source=source_map.get(aisle.aisle_id),
            )
            for aisle in aisle_graph.aisles
        )
    )
    requests = () if coverage is None else coverage.requests
    connectors = _join_connector_records(
        requests,
        forward_map,
        zone_map,
        audit_map,
        r6a_map,
        r6b_map,
    )

    return RouteDebugDataset(
        run_dir=root,
        frame_id=frame_id,
        navigation=navigation,
        aisle_graph=aisle_graph,
        turn_zones=turn_zones,
        vehicle_profile=vehicle_profile,
        no_go_regions=no_go_regions,
        aisles=aisles,
        connector_requests=requests,
        connectors=connectors,
        occupancy_source_masks=occupancy_source_masks,
        asset_states=tuple(states),
    )
