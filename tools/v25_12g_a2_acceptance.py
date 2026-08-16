#!/usr/bin/env python3
"""Diagnostic-only V25-12G-A2 static service-topology acceptance harness."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from agt_offline_assets.turn_zones import load_turn_zones
from agt_offline_assets.vehicle_feasible_segment import (
    INTERIOR_BLOCKED_END,
    load_vehicle_feasible_segment_plan,
)
from agt_offline_assets.vehicle_feasible_service_graph import (
    derive_vehicle_feasible_service_graph,
    write_vehicle_feasible_service_graph,
)


REPORT_SCHEMA = "agt_v25_12g_a2_acceptance_report/v1"
VALIDATION_SCOPE = "A2_STATIC_SERVICE_TOPOLOGY_DIAGNOSTIC_NOT_ROUTE_READY"
GRAPH_ASSET = "vehicle_feasible_service_graph.yaml"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Inspect V25-12G-A2 static service topology from frozen A1 assets"
        )
    )
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--segments", default="vehicle_feasible_segments.yaml")
    parser.add_argument("--turn-zones", default="turn_zones.yaml")
    parser.add_argument("--write-graph", action="store_true")
    parser.add_argument("--overwrite-graph", action="store_true")
    parser.add_argument("--pretty", action="store_true")
    return parser


def _require_file(path: Path) -> Path:
    if not path.is_file():
        raise FileNotFoundError(str(path))
    return path


def _resource_records(graph) -> list[dict[str, Any]]:
    state_ids_by_segment: dict[str, list[str]] = {}
    dead_end_count_by_segment: dict[str, int] = {}
    for state in graph.service_states:
        state_ids_by_segment.setdefault(state.segment_id, []).append(
            state.service_state_id
        )
        if state.service_type == "DEAD_END_FORWARD_IN_REVERSE_OUT":
            dead_end_count_by_segment[state.segment_id] = (
                dead_end_count_by_segment.get(state.segment_id, 0) + 1
            )

    component_id_by_segment: dict[str, str] = {}
    for component in graph.candidate_components:
        for segment_id in component.segment_ids:
            component_id_by_segment[segment_id] = component.component_id

    records: list[dict[str, Any]] = []
    for resource in graph.service_resources:
        records.append(
            {
                "segment_id": resource.segment_id,
                "aisle_id": resource.aisle_id,
                "coverage_length_m": resource.coverage_length_m,
                "low_endpoint_type": resource.low_endpoint_type,
                "high_endpoint_type": resource.high_endpoint_type,
                "service_state_ids": sorted(
                    state_ids_by_segment.get(resource.segment_id, [])
                ),
                "dead_end_candidate_count": dead_end_count_by_segment.get(
                    resource.segment_id, 0
                ),
                "candidate_component_id": component_id_by_segment.get(
                    resource.segment_id
                ),
            }
        )
    return records


def _component_records(graph) -> list[dict[str, Any]]:
    return [
        {
            "component_id": component.component_id,
            "classification": component.classification,
            "service_state_count": len(component.service_state_ids),
            "unique_segment_count": len(component.segment_ids),
            "connector_candidate_count": len(component.connector_candidate_ids),
            "unique_coverage_length_m": component.total_unique_coverage_length_m,
            "distinct_aisle_count": component.distinct_aisle_count,
        }
        for component in graph.candidate_components
    ]


def _illegal_connector_counts(graph) -> tuple[int, int]:
    by_state_id = {
        state.service_state_id: state for state in graph.service_states
    }
    interior_cross_aisle = 0
    cross_side = 0

    for candidate in graph.connector_candidates:
        source = by_state_id[candidate.from_service_state_id]
        target = by_state_id[candidate.to_service_state_id]

        if (
            source.aisle_id != target.aisle_id
            and (
                source.exit_endpoint_type == INTERIOR_BLOCKED_END
                or target.entry_endpoint_type == INTERIOR_BLOCKED_END
            )
        ):
            interior_cross_aisle += 1

        source_side = (
            "LOW_U"
            if source.exit_endpoint_type == "LOW_U_HEADLAND"
            else "HIGH_U"
            if source.exit_endpoint_type == "HIGH_U_HEADLAND"
            else None
        )
        target_side = (
            "LOW_U"
            if target.entry_endpoint_type == "LOW_U_HEADLAND"
            else "HIGH_U"
            if target.entry_endpoint_type == "HIGH_U_HEADLAND"
            else None
        )
        if source_side != target_side:
            cross_side += 1

    return interior_cross_aisle, cross_side


def _summary(segment_plan, graph) -> dict[str, Any]:
    a1_segments = [
        segment
        for aisle in segment_plan.aisles
        for segment in aisle.active_segments
    ]
    a1_ids = {str(segment.segment_id) for segment in a1_segments}
    graph_ids = {resource.segment_id for resource in graph.service_resources}
    a1_length = sum(float(segment.length_m) for segment in a1_segments)
    graph_length = sum(
        float(resource.coverage_length_m) for resource in graph.service_resources
    )

    interior_cross_aisle, cross_side = _illegal_connector_counts(graph)
    if interior_cross_aisle:
        raise ValueError(
            "A2 graph contains interior cross-aisle connector candidates"
        )
    if cross_side:
        raise ValueError("A2 graph contains cross-side connector candidates")

    diagnostics = graph.diagnostics
    return {
        "resource_count": diagnostics.resource_count,
        "ordinary_service_state_count": diagnostics.ordinary_service_state_count,
        "dead_end_candidate_count": diagnostics.dead_end_candidate_count,
        "total_service_state_count": diagnostics.total_service_state_count,
        "connector_candidate_count": diagnostics.connector_candidate_count,
        "low_u_connector_candidate_count": diagnostics.low_u_connector_candidate_count,
        "high_u_connector_candidate_count": diagnostics.high_u_connector_candidate_count,
        "candidate_component_count": diagnostics.candidate_component_count,
        "isolated_component_count": diagnostics.isolated_component_count,
        "externally_unproven_state_count": diagnostics.externally_unproven_state_count,
        "unique_coverage_length_m": diagnostics.unique_coverage_length_m,
        "distinct_aisle_count": diagnostics.distinct_aisle_count,
        "a1_active_segment_count": len(a1_segments),
        "a1_active_segment_length_m": a1_length,
        "coverage_preserved": abs(graph_length - a1_length) <= 1.0e-9,
        "all_a1_segment_ids_preserved": a1_ids == graph_ids,
        "interior_cross_aisle_connector_count": interior_cross_aisle,
        "cross_side_connector_count": cross_side,
    }


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    run_dir = Path(args.run_dir).expanduser().resolve()
    segment_path = run_dir / args.segments
    turn_zone_path = run_dir / args.turn_zones
    output_path = run_dir / GRAPH_ASSET

    _require_file(segment_path)
    _require_file(turn_zone_path)

    if args.write_graph and output_path.exists() and not args.overwrite_graph:
        raise FileExistsError(str(output_path))

    segment_plan = load_vehicle_feasible_segment_plan(segment_path)
    turn_zones = load_turn_zones(turn_zone_path)
    graph = derive_vehicle_feasible_service_graph(
        segment_plan,
        turn_zones,
        source={
            "acceptance_stage": "v25_12g_a2",
            "vehicle_feasible_segments_asset": args.segments,
            "turn_zones_asset": args.turn_zones,
        },
    )

    report = {
        "schema": REPORT_SCHEMA,
        "validation_scope": VALIDATION_SCOPE,
        "run_dir": str(run_dir),
        "frame_id": graph.frame_id,
        "platform_id": graph.platform_id,
        "segments_asset": args.segments,
        "turn_zones_asset": args.turn_zones,
        "resources": _resource_records(graph),
        "components": _component_records(graph),
        "summary": _summary(segment_plan, graph),
    }

    if args.write_graph:
        write_vehicle_feasible_service_graph(
            graph,
            output_path,
            overwrite=args.overwrite_graph,
        )

    if args.pretty:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(json.dumps(report, separators=(",", ":"), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
