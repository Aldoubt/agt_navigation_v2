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
    AgriculturalCoverageOrder,
    coverage_order_to_dict,
)
from .contracts import AssetContractError


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
