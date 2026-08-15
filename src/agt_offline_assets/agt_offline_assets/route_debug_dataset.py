"""Read-only aggregation of frozen route-production evidence for 2D Route Debug.

The loader intentionally does not run any planner or mutate route truth. It joins
already-frozen assets into a small immutable model that the Workbench can render
and inspect.
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
        return next((item for item in self.aisles if item.aisle_id == aisle_id), None)

    def connector_by_id(self, connector_id: str) -> RouteDebugConnectorRecord | None:
        return next((item for item in self.connectors if item.connector_id == connector_id), None)


@dataclass
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


def _asset_state(key, availability, path, *, payload=None, error="") -> RouteDebugAssetState:
    return RouteDebugAssetState(
        key=key,
        availability=availability,
        path=None if path is None else str(path),
        schema=None if payload is None else str(payload.get("schema") or "") or None,
        frame_id=None if payload is None else str(payload.get("frame_id") or "") or None,
        status=None if payload is None else str(payload.get("status") or "") or None,
        error=error,
    )


def _pose(value: Any, field: str) -> tuple[float, float, float, float]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field} must be a mapping")
    return (float(value["x"]), float(value["y"]), float(value.get("z", 0.0)), float(value["yaw"]))


def _frame_accepts(authoritative: str | None, candidate: str | None) -> bool:
    return not authoritative or not candidate or str(authoritative) == str(candidate)


def _load_debug_coverage(path: Path) -> _CoverageDebug:
    payload = _load_yaml(path)
    schema = str(payload.get("schema") or "")
    if schema != COVERAGE_ORDER_SCHEMA:
        raise ValueError(f"expected {COVERAGE_ORDER_SCHEMA}, got {schema or '<empty>'}")
    traversals = []
    for index, raw in enumerate(payload.get("traversals") or []):
        if not isinstance(raw, Mapping):
            raise ValueError(f"traversals[{index}] must be a mapping")
        traversals.append(RouteDebugCoverageTraversal(
            sequence=int(raw["sequence"]), aisle_id=str(raw["aisle_id"]),
            motion_direction=str(raw.get("motion_direction", "FORWARD")),
            graph_orientation=str(raw.get("graph_orientation", "")),
            entry_side=str(raw.get("entry_side", "")), exit_side=str(raw.get("exit_side", "")),
            geometric_width_m=float(raw.get("geometric_width_m", 0.0)),
            required_width_m=float(raw.get("required_width_m", 0.0)),
        ))
    requests = []
    for index, raw in enumerate(payload.get("connector_requests") or []):
        if not isinstance(raw, Mapping):
            raise ValueError(f"connector_requests[{index}] must be a mapping")
        requests.append(ConnectorRequest(
            connector_id=str(raw["connector_id"]), from_aisle_id=str(raw["from_aisle_id"]),
            to_aisle_id=str(raw["to_aisle_id"]), turn_zone_id=str(raw["turn_zone_id"]),
            side=str(raw.get("side", "")),
            start_pose=_pose(raw["start_pose"], f"connector_requests[{index}].start_pose"),
            goal_pose=_pose(raw["goal_pose"], f"connector_requests[{index}].goal_pose"),
        ))
    rejections = []
    for index, raw in enumerate(payload.get("rejected_aisles") or []):
        if not isinstance(raw, Mapping):
            raise ValueError(f"rejected_aisles[{index}] must be a mapping")
        rejections.append(RouteDebugCoverageRejection(
            aisle_id=str(raw["aisle_id"]), reason=str(raw.get("reason", "")),
            geometric_width_m=float(raw.get("geometric_width_m", 0.0)),
            required_width_m=float(raw.get("required_width_m", 0.0)),
        ))
    return _CoverageDebug(
        frame_id=str(payload.get("frame_id", "map")), platform_id=str(payload.get("platform_id", "")),
        status=str(payload.get("status", "DRAFT")), traversals=tuple(traversals),
        requests=tuple(requests), rejections=tuple(rejections),
    )


def _load_no_go_regions(path: Path) -> tuple[RouteDebugNoGoRegion, ...]:
    if not path.is_file():
        return ()
    payload = _load_yaml(path)
    if str(payload.get("schema") or "") != NAVIGATION_DERIVATION_SCHEMA:
        raise ValueError("derivation.yaml schema mismatch")
    output = []
    for index, raw in enumerate(payload.get("overrides") or []):
        if not isinstance(raw, Mapping) or str(raw.get("mode", "")).lower() != "no_go":
            continue
        polygon = raw.get("polygon_xy") or []
        if not isinstance(polygon, Sequence) or len(polygon) < 3:
            raise ValueError(f"no_go override {index} has invalid polygon_xy")
        output.append(RouteDebugNoGoRegion(
            region_id=str(raw.get("region_id") or raw.get("id") or f"no_go_{index + 1:03d}"),
            polygon_xy=tuple((float(p[0]), float(p[1])) for p in polygon), source_asset=path.name,
        ))
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
    for preferred in ("reverse_primitive_connectors_anchored.yaml", "reverse_primitive_connectors.yaml"):
        if preferred in by_name:
            return by_name[preferred]
    return sorted(paths, key=lambda path: path.name)[0]


def _discover_vehicle_profile(run_dir: Path, platform_id: str) -> Path | None:
    if not platform_id:
        return None
    for root in [run_dir, *run_dir.parents]:
        path = root / "profiles" / "platforms" / f"{platform_id}.yaml"
        if path.is_file():
            return path.resolve()
    return None


def _motion_samples(raw_samples: Any) -> tuple[RouteDebugMotionSample, ...]:
    if not isinstance(raw_samples, list):
        return ()
    return tuple(RouteDebugMotionSample(
        x=float(raw.get("x", 0.0)), y=float(raw.get("y", 0.0)), z=float(raw.get("z", 0.0)),
        yaw=float(raw.get("yaw", 0.0)), motion_direction=str(raw.get("motion_direction", "FORWARD")),
        segment_index=None if raw.get("segment_index") is None else int(raw["segment_index"]),
        is_cusp=bool(raw.get("is_cusp", False)),
    ) for raw in raw_samples if isinstance(raw, Mapping))


def _items_by_id(payload: Mapping[str, Any], key: str = "connectors") -> dict[str, Mapping[str, Any]]:
    raw_items = payload.get(key) or []
    if not isinstance(raw_items, list):
        return {}
    return {str(raw["connector_id"]): raw for raw in raw_items
            if isinstance(raw, Mapping) and raw.get("connector_id") is not None}


def _load_optional_payload(key, path, expected_schema, frame_id, states):
    if path is None:
        states.append(_asset_state(key, ASSET_MISSING, None))
        return None
    try:
        payload = _load_yaml(path)
        schema = str(payload.get("schema") or "")
        if schema != expected_schema:
            raise ValueError(f"expected {expected_schema}, got {schema or '<empty>'}")
        candidate_frame = str(payload.get("frame_id") or frame_id)
        if not _frame_accepts(frame_id, candidate_frame):
            raise ValueError(f"frame_id mismatch: expected {frame_id}, got {candidate_frame}")
    except Exception as exc:
        try:
            payload = _load_yaml(path)
        except Exception:
            payload = None
        states.append(_asset_state(key, ASSET_INVALID, path, payload=payload, error=str(exc)))
        return None
    states.append(_asset_state(key, ASSET_LOADED, path, payload=payload))
    return payload


def _parse_lane_records(payload):
    output = {}
    if payload is None:
        return output
    for raw in payload.get("aisles") or []:
        if not isinstance(raw, Mapping) or not raw.get("aisle_id"):
            continue
        aisle_id = str(raw["aisle_id"])
        points = raw.get("centerline_xyz") or []
        output[aisle_id] = RouteDebugVehicleLaneRecord(
            aisle_id=aisle_id, status=str(raw.get("status", "")),
            coverage_fraction=float(raw.get("coverage_fraction", 0.0)),
            centerline_xyz=tuple((float(p[0]), float(p[1]), float(p[2])) for p in points),
            reason=str(raw.get("reason", "")),
        )
    return output


def _parse_lane_diagnostics(payload):
    output = {}
    if payload is None:
        return output
    for raw in payload.get("aisles") or []:
        if not isinstance(raw, Mapping) or not raw.get("aisle_id"):
            continue
        aisle_id = str(raw["aisle_id"]); reference = raw.get("reference_point") or {}; feasibility = raw.get("pose_feasibility") or {}
        output[aisle_id] = RouteDebugLaneDiagnosticRecord(
            aisle_id=aisle_id, classification=str(raw.get("classification", "")),
            reference_free_fraction=float(reference.get("free_fraction", 0.0)),
            any_lateral_footprint_free_fraction=float(feasibility.get("any_lateral_footprint_free_fraction", 0.0)),
            reason=str(raw.get("reason", "")),
        )
    return output


def _parse_source_records(payload):
    output = {}
    if payload is None:
        return output
    for raw in payload.get("aisles") or []:
        if not isinstance(raw, Mapping) or not raw.get("aisle_id"):
            continue
        aisle_id = str(raw["aisle_id"]); sources = raw.get("sources") or {}
        output[aisle_id] = RouteDebugOccupancySourceRecord(
            aisle_id=aisle_id, classification=str(raw.get("classification", "")),
            raw_obstacle_fraction=float(sources.get("raw_obstacle_fraction", 0.0)),
            geometry_fraction=float(sources.get("geometry_fraction", 0.0)),
            padding_fraction=float(sources.get("padding_fraction", 0.0)),
            unexplained_fraction=float(sources.get("unexplained_fraction", 0.0)), reason=str(raw.get("reason", "")),
        )
    return output


def _parse_connector_records(requests, forward, zone_fit_path, audit_path, r6a, r6b):
    request_map = {item.connector_id: item for item in requests}
    forward_map = {} if forward is None else _items_by_id(forward)
    r6a_map = {} if r6a is None else _items_by_id(r6a)
    r6b_map = {} if r6b is None else _items_by_id(r6b)
    zone_fit_map = {}
    if zone_fit_path is not None:
        try:
            report = load_forward_connector_zone_fit_report(zone_fit_path)
            zone_fit_map = {item.connector_id: item for item in report.diagnostics}
        except Exception:
            pass
    audit_map = {}
    if audit_path is not None:
        try:
            plan = load_forward_connector_candidate_audit(audit_path)
            audit_map = {item.connector_id: item for item in plan.connectors}
        except Exception:
            pass
    ids = set(request_map) | set(forward_map) | set(r6a_map) | set(r6b_map) | set(zone_fit_map) | set(audit_map)
    output = []
    for connector_id in sorted(ids):
        request = request_map.get(connector_id); fwd = forward_map.get(connector_id) or {}; audit = audit_map.get(connector_id)
        r6a_item = r6a_map.get(connector_id) or {}; r6b_item = r6b_map.get(connector_id) or {}; zfit = zone_fit_map.get(connector_id)
        candidates = () if audit is None else tuple(RouteDebugForwardCandidateRecord(
            path_type=str(item.path_type), length_m=float(item.length_m),
            local_headland_candidate=bool(item.local_headland_candidate), preview_footprint_free=bool(item.preview_footprint_free),
            max_required_zone_extension_m=float(item.max_required_zone_extension_m),
        ) for item in audit.candidates)
        def choose(attr, *records):
            if request is not None and hasattr(request, attr):
                return getattr(request, attr)
            for record in records:
                value = record.get(attr)
                if value:
                    return value
            return ""
        output.append(RouteDebugConnectorRecord(
            connector_id=connector_id,
            from_aisle_id=str(choose("from_aisle_id", fwd, r6b_item, r6a_item)),
            to_aisle_id=str(choose("to_aisle_id", fwd, r6b_item, r6a_item)),
            turn_zone_id=str(choose("turn_zone_id", fwd, r6b_item, r6a_item)), request=request,
            forward_status=None if not fwd else str(fwd.get("status", "")) or None,
            forward_backend=None if not fwd else str(fwd.get("backend", "")) or None,
            forward_path_type=None if fwd.get("path_type") is None else str(fwd["path_type"]),
            forward_samples=_motion_samples(fwd.get("samples")),
            zone_fit_status=None if zfit is None else str(zfit.status),
            forward_audit_status=None if audit is None else str(audit.status), forward_candidates=candidates,
            r6a_decision=None if not r6a_item else str(r6a_item.get("decision", "")) or None,
            r6b_status=None if not r6b_item else str(r6b_item.get("status", "")) or None,
            r6b_backend=None if not r6b_item else str(r6b_item.get("backend", "")) or None,
            r6b_samples=_motion_samples(r6b_item.get("samples")),
            r6b_path_length_m=None if r6b_item.get("path_length_m") is None else float(r6b_item["path_length_m"]),
            r6b_forward_distance_m=None if r6b_item.get("forward_distance_m") is None else float(r6b_item["forward_distance_m"]),
            r6b_reverse_distance_m=None if r6b_item.get("reverse_distance_m") is None else float(r6b_item["reverse_distance_m"]),
            r6b_cusp_count=None if r6b_item.get("cusp_count") is None else int(r6b_item["cusp_count"]),
            r6b_search_expansions=None if r6b_item.get("search_expansions") is None else int(r6b_item["search_expansions"]),
            r6b_goal_position_error_m=None if r6b_item.get("goal_position_error_m") is None else float(r6b_item["goal_position_error_m"]),
            r6b_goal_yaw_error_rad=None if r6b_item.get("goal_yaw_error_rad") is None else float(r6b_item["goal_yaw_error_rad"]),
        ))
    return tuple(output)


def load_route_debug_dataset(run_dir: str | Path, *, vehicle_profile_path: str | Path | None = None) -> RouteDebugDataset:
    """Load one frozen run directory for read-only Route Debug rendering."""
    root = Path(run_dir).expanduser().resolve()
    if not root.is_dir():
        raise NotADirectoryError(f"route debug run directory not found: {root}")
    states = []; authoritative = None; navigation = None; aisle_graph = None; turn_zones = None; coverage = None

    nav_yaml = root / "navigation_map.yaml"
    if nav_yaml.is_file():
        try:
            navigation = load_navigation_grid(nav_yaml); frame = "map"; derivation = root / "derivation.yaml"
            if derivation.is_file():
                frame = str(_load_yaml(derivation).get("frame_id") or "map")
            navigation = NavigationGridEvidence(
                resolution_m=navigation.resolution_m, origin_x_m=navigation.origin_x_m, origin_y_m=navigation.origin_y_m,
                width=navigation.width, height=navigation.height, occupancy=navigation.occupancy, frame_id=frame, source=navigation.source,
            )
            authoritative = frame; states.append(_asset_state("navigation", ASSET_LOADED, nav_yaml, payload={"frame_id": frame}))
        except Exception as exc:
            states.append(_asset_state("navigation", ASSET_INVALID, nav_yaml, error=str(exc)))
    else:
        states.append(_asset_state("navigation", ASSET_MISSING, None))

    aisle_path = root / "aisle_graph.yaml"
    if aisle_path.is_file():
        try:
            candidate = load_agricultural_aisle_graph(aisle_path)
            if not _frame_accepts(authoritative, candidate.frame_id): raise ValueError(f"frame_id mismatch: expected {authoritative}, got {candidate.frame_id}")
            aisle_graph = candidate; authoritative = authoritative or candidate.frame_id
            states.append(_asset_state("aisle_graph", ASSET_LOADED, aisle_path, payload=_load_yaml(aisle_path)))
        except Exception as exc:
            states.append(_asset_state("aisle_graph", ASSET_INVALID, aisle_path, error=str(exc)))
    else: states.append(_asset_state("aisle_graph", ASSET_MISSING, None))

    coverage_path = root / "coverage_order.yaml"
    if coverage_path.is_file():
        try:
            candidate = _load_debug_coverage(coverage_path)
            if not _frame_accepts(authoritative, candidate.frame_id): raise ValueError(f"frame_id mismatch: expected {authoritative}, got {candidate.frame_id}")
            coverage = candidate; authoritative = authoritative or candidate.frame_id
            states.append(_asset_state("coverage_order", ASSET_LOADED, coverage_path, payload=_load_yaml(coverage_path)))
        except Exception as exc:
            states.append(_asset_state("coverage_order", ASSET_INVALID, coverage_path, error=str(exc)))
    else: states.append(_asset_state("coverage_order", ASSET_MISSING, None))
    frame_id = authoritative or "map"

    turn_path = root / "turn_zones.yaml"
    if turn_path.is_file():
        try:
            candidate = load_turn_zones(turn_path)
            if not _frame_accepts(frame_id, candidate.frame_id): raise ValueError(f"frame_id mismatch: expected {frame_id}, got {candidate.frame_id}")
            turn_zones = candidate; states.append(_asset_state("turn_zones", ASSET_LOADED, turn_path, payload=_load_yaml(turn_path)))
        except Exception as exc: states.append(_asset_state("turn_zones", ASSET_INVALID, turn_path, error=str(exc)))
    else: states.append(_asset_state("turn_zones", ASSET_MISSING, None))

    derivation_path = root / "derivation.yaml"; no_go = ()
    if derivation_path.is_file():
        try:
            no_go = _load_no_go_regions(derivation_path); payload = _load_yaml(derivation_path)
            if not _frame_accepts(frame_id, str(payload.get("frame_id") or frame_id)): raise ValueError("derivation frame_id mismatch")
            states.append(_asset_state("derivation", ASSET_LOADED, derivation_path, payload=payload))
        except Exception as exc:
            states.append(_asset_state("derivation", ASSET_INVALID, derivation_path, error=str(exc))); no_go = ()
    else: states.append(_asset_state("derivation", ASSET_MISSING, None))

    platform_id = "" if coverage is None else coverage.platform_id
    profile_path = Path(vehicle_profile_path).expanduser().resolve() if vehicle_profile_path is not None else _discover_vehicle_profile(root, platform_id)
    vehicle_profile = None
    if profile_path is not None:
        try:
            vehicle_profile = load_canonical_vehicle_profile(profile_path); states.append(_asset_state("vehicle_profile", ASSET_LOADED, profile_path))
        except Exception as exc: states.append(_asset_state("vehicle_profile", ASSET_INVALID, profile_path, error=str(exc)))
    else: states.append(_asset_state("vehicle_profile", ASSET_MISSING, None))

    discovered = _discover_yaml_by_schema(root)
    first = lambda schema: next(iter(discovered.get(schema, ())), None)
    forward_path = first(FORWARD_CONNECTOR_SCHEMA); zone_fit_path = first(FORWARD_CONNECTOR_ZONE_FIT_SCHEMA)
    audit_path = first(FORWARD_CONNECTOR_CANDIDATE_AUDIT_SCHEMA); r6a_path = first(REVERSE_FALLBACK_ADMISSION_SCHEMA)
    r6b_path = _choose_reverse_primitive_asset(discovered.get(REVERSE_PRIMITIVE_CONNECTOR_SCHEMA, ()))
    lane_path = first(VEHICLE_SAFE_LANE_SCHEMA); diag_path = first(VEHICLE_SAFE_LANE_DIAGNOSTIC_SCHEMA); source_path = first(VEHICLE_SAFE_LANE_OCCUPANCY_SOURCE_SCHEMA)
    forward_payload = _load_optional_payload("forward_connector", forward_path, FORWARD_CONNECTOR_SCHEMA, frame_id, states)
    zone_fit_payload = _load_optional_payload("forward_zone_fit", zone_fit_path, FORWARD_CONNECTOR_ZONE_FIT_SCHEMA, frame_id, states)
    audit_payload = _load_optional_payload("forward_candidate_audit", audit_path, FORWARD_CONNECTOR_CANDIDATE_AUDIT_SCHEMA, frame_id, states)
    r6a_payload = _load_optional_payload("reverse_fallback_admission", r6a_path, REVERSE_FALLBACK_ADMISSION_SCHEMA, frame_id, states)
    r6b_payload = _load_optional_payload("reverse_primitive_connectors", r6b_path, REVERSE_PRIMITIVE_CONNECTOR_SCHEMA, frame_id, states)
    lane_payload = _load_optional_payload("vehicle_safe_lane", lane_path, VEHICLE_SAFE_LANE_SCHEMA, frame_id, states)
    diag_payload = _load_optional_payload("vehicle_safe_lane_diagnostic", diag_path, VEHICLE_SAFE_LANE_DIAGNOSTIC_SCHEMA, frame_id, states)
    source_payload = _load_optional_payload("vehicle_safe_lane_occupancy_source", source_path, VEHICLE_SAFE_LANE_OCCUPANCY_SOURCE_SCHEMA, frame_id, states)

    masks = None
    if navigation is not None:
        try:
            masks = load_navigation_occupancy_source_masks(navigation, root)
            states.append(_asset_state("occupancy_source_masks", ASSET_LOADED, derivation_path if derivation_path.is_file() else root))
        except FileNotFoundError as exc: states.append(_asset_state("occupancy_source_masks", ASSET_MISSING, None, error=str(exc)))
        except Exception as exc: states.append(_asset_state("occupancy_source_masks", ASSET_INVALID, root, error=str(exc)))
    else: states.append(_asset_state("occupancy_source_masks", ASSET_MISSING, None))

    traversal_map = {} if coverage is None else {item.aisle_id: item for item in coverage.traversals}
    rejection_map = {} if coverage is None else {item.aisle_id: item for item in coverage.rejections}
    lane_map = _parse_lane_records(lane_payload); diag_map = _parse_lane_diagnostics(diag_payload); source_map = _parse_source_records(source_payload)
    aisles = () if aisle_graph is None else tuple(RouteDebugAisleRecord(
        aisle_id=aisle.aisle_id, aisle=aisle, traversal=traversal_map.get(aisle.aisle_id), rejection=rejection_map.get(aisle.aisle_id),
        vehicle_lane=lane_map.get(aisle.aisle_id), diagnostic=diag_map.get(aisle.aisle_id), occupancy_source=source_map.get(aisle.aisle_id),
    ) for aisle in aisle_graph.aisles)
    requests = () if coverage is None else coverage.requests
    connectors = _parse_connector_records(
        requests, forward_payload, zone_fit_path if zone_fit_payload is not None else None,
        audit_path if audit_payload is not None else None, r6a_payload, r6b_payload,
    )
    return RouteDebugDataset(
        run_dir=root, frame_id=frame_id, navigation=navigation, aisle_graph=aisle_graph, turn_zones=turn_zones,
        vehicle_profile=vehicle_profile, no_go_regions=no_go, aisles=aisles, connector_requests=requests,
        connectors=connectors, occupancy_source_masks=masks, asset_states=tuple(states),
    )
