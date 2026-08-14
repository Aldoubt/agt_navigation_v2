"""YAML IO for V25-12E agricultural route-production intermediate assets."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import yaml

from .agricultural_aisle_graph import (
    AISLE_GRAPH_SCHEMA,
    AislePrimitive,
    AgriculturalAisleGraph,
)
from .agricultural_coverage_ordering import (
    COVERAGE_ORDER_SCHEMA,
    AgriculturalCoverageOrder,
    ConnectorRequest,
    coverage_order_to_dict,
)
from .contracts import AssetContractError
from .forward_connector import ForwardConnectorPlan, forward_connector_plan_to_dict
from .turn_zones import TURN_ZONE_SCHEMA, TurnZone, TurnZoneSet


def _mapping(value: Any, field: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise AssetContractError("agricultural_asset_invalid", f"{field} must be a mapping")
    return value


def _pose(value: Any, field: str) -> tuple[float, float, float, float]:
    payload = _mapping(value, field)
    return (
        float(payload["x"]),
        float(payload["y"]),
        float(payload["z"]),
        float(payload["yaw"]),
    )


def load_agricultural_aisle_graph(path: str | Path) -> AgriculturalAisleGraph:
    """Load one previously exported DRAFT aisle graph without rerunning PCD analysis."""
    source_path = Path(path).expanduser().resolve()
    raw = yaml.safe_load(source_path.read_text(encoding="utf-8")) or {}
    payload = _mapping(raw, "aisle_graph")
    schema = str(payload.get("schema", ""))
    if schema != AISLE_GRAPH_SCHEMA:
        raise AssetContractError(
            "aisle_graph_schema_mismatch",
            f"expected {AISLE_GRAPH_SCHEMA}, got {schema or '<empty>'}",
        )
    status = str(payload.get("status", "DRAFT"))
    frame_id = str(payload.get("frame_id", "map"))
    direction = payload.get("row_direction_xy")
    if not isinstance(direction, (list, tuple)) or len(direction) != 2:
        raise AssetContractError("aisle_graph_direction_invalid", "row_direction_xy must be [x, y]")

    aisles: list[AislePrimitive] = []
    raw_aisles = payload.get("aisles") or []
    if not isinstance(raw_aisles, list):
        raise AssetContractError("aisle_graph_aisles_invalid", "aisles must be a list")
    for index, raw_aisle in enumerate(raw_aisles):
        aisle = _mapping(raw_aisle, f"aisles[{index}]")
        adjacent = _mapping(aisle.get("adjacent_structure"), f"aisles[{index}].adjacent_structure")
        evidence = _mapping(aisle.get("evidence"), f"aisles[{index}].evidence")
        points = aisle.get("centerline_xyz") or []
        if not isinstance(points, list) or len(points) < 2:
            raise AssetContractError(
                "aisle_graph_centerline_invalid",
                f"aisles[{index}].centerline_xyz must contain at least two points",
            )
        centerline = tuple((float(p[0]), float(p[1]), float(p[2])) for p in points)
        aisles.append(
            AislePrimitive(
                aisle_id=str(aisle["aisle_id"]),
                kind=str(aisle["kind"]),
                pair_kind=str(aisle["pair_kind"]),
                left_structure_ref=str(adjacent["left"]),
                right_structure_ref=str(adjacent["right"]),
                centerline_xyz=centerline,
                start_pose=_pose(aisle["start_pose"], f"aisles[{index}].start_pose"),
                end_pose=_pose(aisle["end_pose"], f"aisles[{index}].end_pose"),
                length_m=float(aisle["length_m"]),
                geometric_width_m=float(aisle["geometric_width_m"]),
                minimum_required_width_m=float(aisle["minimum_required_width_m"]),
                center_distance_m=float(aisle["center_distance_m"]),
                longitudinal_overlap_m=(
                    None
                    if aisle.get("longitudinal_overlap_m") is None
                    else float(aisle["longitudinal_overlap_m"])
                ),
                safe_cell_count=int(evidence["safe_cell_count"]),
                centerline_cell_count=int(evidence["centerline_cell_count"]),
                diagnostic_status=str(evidence["diagnostic_status"]),
            )
        )

    declared_count = payload.get("aisle_count")
    if declared_count is not None and int(declared_count) != len(aisles):
        raise AssetContractError(
            "aisle_graph_count_mismatch",
            f"declared aisle_count={declared_count}, parsed={len(aisles)}",
        )

    spacing = payload.get("nominal_row_spacing_m")
    return AgriculturalAisleGraph(
        frame_id=frame_id,
        row_direction_xy=(float(direction[0]), float(direction[1])),
        nominal_row_spacing_m=None if spacing is None else float(spacing),
        aisles=tuple(aisles),
        source=dict(payload.get("source") or {}),
        schema=schema,
        status=status,
    )


def load_turn_zones(path: str | Path) -> TurnZoneSet:
    """Load frozen R2 turn-zone search envelopes."""
    source_path = Path(path).expanduser().resolve()
    raw = yaml.safe_load(source_path.read_text(encoding="utf-8")) or {}
    payload = _mapping(raw, "turn_zones")
    schema = str(payload.get("schema", ""))
    if schema != TURN_ZONE_SCHEMA:
        raise AssetContractError(
            "turn_zone_schema_mismatch",
            f"expected {TURN_ZONE_SCHEMA}, got {schema or '<empty>'}",
        )
    direction = payload.get("row_direction_xy")
    if not isinstance(direction, (list, tuple)) or len(direction) != 2:
        raise AssetContractError("turn_zone_direction_invalid", "row_direction_xy must be [x, y]")
    zones: list[TurnZone] = []
    for index, raw_zone in enumerate(payload.get("zones") or []):
        zone = _mapping(raw_zone, f"zones[{index}]")
        polygon = zone.get("polygon_xy") or []
        if not isinstance(polygon, list) or len(polygon) < 3:
            raise AssetContractError("turn_zone_polygon_invalid", f"zones[{index}].polygon_xy is invalid")
        permissions = _mapping(zone.get("permissions") or {}, f"zones[{index}].permissions")
        free_fraction = zone.get("free_fraction")
        zones.append(
            TurnZone(
                zone_id=str(zone["zone_id"]),
                side=str(zone["side"]),
                polygon_xy=tuple((float(p[0]), float(p[1])) for p in polygon),
                supported_aisle_ids=tuple(str(v) for v in zone.get("supported_aisle_ids") or []),
                endpoint_count=int(zone.get("endpoint_count", 0)),
                free_fraction=float("nan") if free_fraction is None else float(free_fraction),
                allow_turn=bool(permissions.get("turn", True)),
                allow_reverse=bool(permissions.get("reverse", True)),
                allow_direction_change=bool(permissions.get("direction_change", True)),
                source_kind=str(zone.get("source_kind", "AUTO_ENDPOINT_ENVELOPE")),
            )
        )
    declared = payload.get("zone_count")
    if declared is not None and int(declared) != len(zones):
        raise AssetContractError("turn_zone_count_mismatch", f"declared zone_count={declared}, parsed={len(zones)}")
    return TurnZoneSet(
        frame_id=str(payload.get("frame_id", "map")),
        row_direction_xy=(float(direction[0]), float(direction[1])),
        zones=tuple(zones),
        source=dict(payload.get("source") or {}),
        schema=schema,
        status=str(payload.get("status", "DRAFT")),
    )


def load_coverage_connector_requests(path: str | Path) -> tuple[ConnectorRequest, ...]:
    """Load only the frozen R4 connector requests needed by R5 without duplicating aisle geometry."""
    source_path = Path(path).expanduser().resolve()
    raw = yaml.safe_load(source_path.read_text(encoding="utf-8")) or {}
    payload = _mapping(raw, "coverage_order")
    schema = str(payload.get("schema", ""))
    if schema != COVERAGE_ORDER_SCHEMA:
        raise AssetContractError(
            "coverage_order_schema_mismatch",
            f"expected {COVERAGE_ORDER_SCHEMA}, got {schema or '<empty>'}",
        )
    requests: list[ConnectorRequest] = []
    for index, raw_request in enumerate(payload.get("connector_requests") or []):
        request = _mapping(raw_request, f"connector_requests[{index}]")
        requests.append(
            ConnectorRequest(
                connector_id=str(request["connector_id"]),
                from_aisle_id=str(request["from_aisle_id"]),
                to_aisle_id=str(request["to_aisle_id"]),
                turn_zone_id=str(request["turn_zone_id"]),
                side=str(request["side"]),
                start_pose=_pose(request["start_pose"], f"connector_requests[{index}].start_pose"),
                goal_pose=_pose(request["goal_pose"], f"connector_requests[{index}].goal_pose"),
            )
        )
    declared = payload.get("connector_request_count")
    if declared is not None and int(declared) != len(requests):
        raise AssetContractError(
            "coverage_connector_count_mismatch",
            f"declared connector_request_count={declared}, parsed={len(requests)}",
        )
    return tuple(requests)


def write_agricultural_coverage_order(
    order: AgriculturalCoverageOrder,
    path: str | Path,
) -> Path:
    """Write deterministic R4 order / connector-request evidence."""
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(coverage_order_to_dict(order), sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return output


def write_forward_connector_plan(plan: ForwardConnectorPlan, path: str | Path) -> Path:
    """Write DRAFT R5 centerline kinematic evidence; this is not a READY route."""
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(forward_connector_plan_to_dict(plan), sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return output
