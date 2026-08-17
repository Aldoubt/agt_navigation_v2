#!/usr/bin/env python3
"""Diagnostic-only V25-12G-A3 local vehicle-motion acceptance harness."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from agt_offline_assets.navigation_grid import load_navigation_grid
from agt_offline_assets.site_boundary import load_site_boundary
from agt_offline_assets.turn_zones import load_turn_zones
from agt_offline_assets.vehicle_profile import load_canonical_vehicle_profile
from agt_offline_assets.vehicle_feasible_motion_graph import (
    A1_CENTERLINE_EXACT_REVERSE_RETRACE,
    BOUNDED_REVERSE_PRIMITIVE_SEARCH,
    BOUNDED_SEARCH_NO_SOLUTION,
    EXECUTABLE,
    derive_vehicle_feasible_motion_graph,
    write_vehicle_feasible_motion_graph,
)
from agt_offline_assets.vehicle_feasible_service_graph_io import (
    load_vehicle_feasible_service_graph,
)


REPORT_SCHEMA = "agt_v25_12g_a3_acceptance_report/v1"
VALIDATION_SCOPE = "A3_LOCAL_MOTION_EVIDENCE_DIAGNOSTIC_NOT_ROUTE_READY"
MOTION_GRAPH_ASSET = "vehicle_feasible_motion_graph.yaml"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate V25-12G-A3 local vehicle motion evidence"
    )
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--vehicle-profile", required=True)
    parser.add_argument("--service-graph", default="vehicle_feasible_service_graph.yaml")
    parser.add_argument("--turn-zones", default="turn_zones.yaml")
    parser.add_argument("--navigation-map", default="navigation_map.yaml")
    parser.add_argument("--site-boundary", default="site_boundary.yaml")
    parser.add_argument("--write-motion-graph", action="store_true")
    parser.add_argument("--overwrite-motion-graph", action="store_true")
    parser.add_argument("--pretty", action="store_true")
    return parser


def _require_file(path: Path) -> Path:
    if not path.is_file():
        raise FileNotFoundError(str(path))
    return path


def _preflight(args: argparse.Namespace):
    run_dir = Path(args.run_dir).expanduser().resolve()
    service_path = _require_file(run_dir / str(args.service_graph))
    zones_path = _require_file(run_dir / str(args.turn_zones))
    navigation_path = _require_file(run_dir / str(args.navigation_map))
    boundary_path = _require_file(run_dir / str(args.site_boundary))
    profile_path = _require_file(Path(args.vehicle_profile).expanduser().resolve())
    output_path = run_dir / MOTION_GRAPH_ASSET
    if (
        args.write_motion_graph
        and output_path.exists()
        and not args.overwrite_motion_graph
    ):
        raise FileExistsError(str(output_path))
    return (
        run_dir,
        service_path,
        zones_path,
        navigation_path,
        boundary_path,
        profile_path,
        output_path,
    )


def _sample_record(sample) -> dict[str, Any]:
    return {
        "x": float(sample.x),
        "y": float(sample.y),
        "z": float(sample.z),
        "yaw": float(sample.yaw),
        "motion_direction": str(sample.motion_direction),
        "segment_index": int(sample.segment_index),
        "is_cusp": bool(sample.is_cusp),
    }


def _service_record(item) -> dict[str, Any]:
    return {
        "service_state_id": item.service_state_id,
        "segment_id": item.segment_id,
        "aisle_id": item.aisle_id,
        "service_type": item.service_type,
        "status": item.status,
        "proof_scope": item.proof_scope,
        "backend": item.backend,
        "backend_status": item.backend_status,
        "coverage_segment_id": item.coverage_segment_id,
        "coverage_reward_length_m": float(item.coverage_reward_length_m),
        "path_length_m": float(item.path_length_m),
        "forward_distance_m": float(item.forward_distance_m),
        "reverse_distance_m": float(item.reverse_distance_m),
        "cusp_count": int(item.cusp_count),
        "samples": [_sample_record(sample) for sample in item.samples],
        "reason": item.reason,
    }


def _transition_record(item) -> dict[str, Any]:
    return {
        "connector_candidate_id": item.connector_candidate_id,
        "from_service_state_id": item.from_service_state_id,
        "to_service_state_id": item.to_service_state_id,
        "from_segment_id": item.from_segment_id,
        "to_segment_id": item.to_segment_id,
        "side": item.side,
        "turn_zone_id": item.turn_zone_id,
        "status": item.status,
        "proof_scope": item.proof_scope,
        "backend": item.backend,
        "backend_status": item.backend_status,
        "path_length_m": float(item.path_length_m),
        "forward_distance_m": float(item.forward_distance_m),
        "reverse_distance_m": float(item.reverse_distance_m),
        "cusp_count": int(item.cusp_count),
        "search_expansions": int(item.search_expansions),
        "forward_evidence": dict(item.forward_evidence),
        "reverse_admission_evidence": dict(item.reverse_admission_evidence),
        "reason": item.reason,
    }


def _diagnostics_dict(diagnostics) -> dict[str, Any]:
    fields = (
        "a2_service_state_count",
        "service_validation_count",
        "executable_service_action_count",
        "rejected_service_action_count",
        "unresolved_service_action_count",
        "ordinary_service_action_count",
        "dead_end_service_action_count",
        "executable_dead_end_service_action_count",
        "a2_connector_candidate_count",
        "transition_validation_count",
        "executable_transition_count",
        "forward_executable_transition_count",
        "reverse_executable_transition_count",
        "rejected_transition_count",
        "unresolved_transition_count",
        "locally_validated_segment_count",
        "locally_validated_unique_coverage_length_m",
        "distinct_locally_validated_aisle_count",
    )
    return {field: getattr(diagnostics, field) for field in fields}


def _dead_end_is_exact_retrace(item) -> bool:
    if item.service_type != "DEAD_END_FORWARD_IN_REVERSE_OUT":
        return True
    if item.backend != A1_CENTERLINE_EXACT_REVERSE_RETRACE or item.cusp_count != 1:
        return False
    cusp_indices = [
        index for index, sample in enumerate(item.samples) if sample.is_cusp
    ]
    if len(cusp_indices) != 1:
        return False
    cusp = cusp_indices[0]
    if cusp <= 0:
        return False
    terminal = item.samples[cusp - 1]
    marker = item.samples[cusp]
    if (
        abs(terminal.x - marker.x) > 1.0e-9
        or abs(terminal.y - marker.y) > 1.0e-9
        or abs(terminal.z - marker.z) > 1.0e-9
        or abs(terminal.yaw - marker.yaw) > 1.0e-9
        or marker.motion_direction != "REVERSE"
    ):
        return False
    forward = [
        (sample.x, sample.y, sample.z, sample.yaw)
        for sample in item.samples[:cusp]
    ]
    reverse = [
        (sample.x, sample.y, sample.z, sample.yaw)
        for sample in item.samples[cusp + 1 :]
    ]
    return reverse == list(reversed(forward[:-1]))


def _forbidden_key_count(value: Any) -> int:
    forbidden = {"route_ready", "reachable_from_start", "optimal"}
    if isinstance(value, dict):
        return sum(key in forbidden for key in value) + sum(
            _forbidden_key_count(child) for child in value.values()
        )
    if isinstance(value, (list, tuple)):
        return sum(_forbidden_key_count(child) for child in value)
    return 0


def _summary(service_graph, motion_graph, service_records, transition_records):
    summary = _diagnostics_dict(motion_graph.diagnostics)
    service_ids = {str(item.service_state_id) for item in service_graph.service_states}
    motion_service_ids = {
        str(item.service_state_id) for item in motion_graph.service_actions
    }
    connector_ids = {
        str(item.connector_candidate_id) for item in service_graph.connector_candidates
    }
    motion_connector_ids = {
        str(item.connector_candidate_id)
        for item in motion_graph.transition_validations
    }
    dead_end_non_retrace = sum(
        not _dead_end_is_exact_retrace(item)
        for item in motion_graph.service_actions
        if item.service_type == "DEAD_END_FORWARD_IN_REVERSE_OUT"
    )

    endpoint_conflicts = 0
    path_conflicts = 0
    path_reverse_admitted = 0
    path_reverse_executable = 0
    path_reverse_bounded_no_solution = 0
    endpoint_reverse_admission = 0
    for item in motion_graph.transition_validations:
        forward_status = str(item.forward_evidence.get("audit_status", ""))
        admission_decision = str(
            item.reverse_admission_evidence.get("decision", "")
        )
        if forward_status == "CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT":
            endpoint_conflicts += 1
            if (
                admission_decision == "ELIGIBLE_REVERSE_FALLBACK"
                or item.backend == BOUNDED_REVERSE_PRIMITIVE_SEARCH
            ):
                endpoint_reverse_admission += 1
        elif forward_status == "LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT":
            path_conflicts += 1
            if admission_decision == "ELIGIBLE_REVERSE_FALLBACK":
                path_reverse_admitted += 1
                if (
                    item.backend == BOUNDED_REVERSE_PRIMITIVE_SEARCH
                    and item.status == EXECUTABLE
                ):
                    path_reverse_executable += 1
                if (
                    item.backend == BOUNDED_REVERSE_PRIMITIVE_SEARCH
                    and item.proof_scope == BOUNDED_SEARCH_NO_SOLUTION
                ):
                    path_reverse_bounded_no_solution += 1

    forbidden = _forbidden_key_count(
        {"service_actions": service_records, "transitions": transition_records}
    )
    summary.update(
        {
            "all_a2_service_states_preserved": service_ids == motion_service_ids,
            "all_a2_connector_candidates_preserved": connector_ids == motion_connector_ids,
            "dead_end_non_retrace_count": int(dead_end_non_retrace),
            "endpoint_boundary_conflict_count": int(endpoint_conflicts),
            "forward_path_boundary_conflict_count": int(path_conflicts),
            "forward_path_boundary_reverse_admitted_count": int(
                path_reverse_admitted
            ),
            "forward_path_boundary_reverse_executable_count": int(
                path_reverse_executable
            ),
            "forward_path_boundary_reverse_bounded_no_solution_count": int(
                path_reverse_bounded_no_solution
            ),
            "endpoint_boundary_conflict_reverse_admission_count": int(
                endpoint_reverse_admission
            ),
            # Backward-compatible key with corrected semantics: only a true
            # endpoint-hard-conflict reverse admission counts as a bypass.
            "site_boundary_reverse_bypass_count": int(endpoint_reverse_admission),
            "forbidden_semantic_key_count": int(forbidden),
        }
    )
    if dead_end_non_retrace:
        raise ValueError("A3 dead-end validation contains non-retrace semantics")
    if endpoint_reverse_admission:
        raise ValueError(
            "A3 endpoint Site Boundary conflict was admitted to reverse fallback"
        )
    if forbidden:
        raise ValueError("A3 report contains forbidden route-readiness semantics")
    return summary


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    (
        run_dir,
        service_path,
        zones_path,
        navigation_path,
        boundary_path,
        profile_path,
        output_path,
    ) = _preflight(args)

    service_graph = load_vehicle_feasible_service_graph(service_path)
    turn_zones = load_turn_zones(zones_path)
    navigation = load_navigation_grid(navigation_path)
    boundary = load_site_boundary(boundary_path)
    vehicle = load_canonical_vehicle_profile(profile_path)
    motion_graph = derive_vehicle_feasible_motion_graph(
        service_graph,
        turn_zones,
        navigation,
        vehicle,
        site_boundary=boundary,
        source={
            "acceptance_stage": "v25_12g_a3",
            "service_graph_asset": str(args.service_graph),
            "turn_zones_asset": str(args.turn_zones),
            "navigation_map_asset": str(args.navigation_map),
            "site_boundary_asset": str(args.site_boundary),
        },
    )

    service_records = [
        _service_record(item) for item in motion_graph.service_actions
    ]
    transition_records = [
        _transition_record(item) for item in motion_graph.transition_validations
    ]
    report = {
        "schema": REPORT_SCHEMA,
        "validation_scope": VALIDATION_SCOPE,
        "run_dir": str(run_dir),
        "frame_id": motion_graph.frame_id,
        "platform_id": motion_graph.platform_id,
        "service_graph_asset": str(args.service_graph),
        "turn_zones_asset": str(args.turn_zones),
        "navigation_map_asset": str(args.navigation_map),
        "site_boundary_asset": str(args.site_boundary),
        "service_actions": service_records,
        "transitions": transition_records,
        "summary": _summary(
            service_graph,
            motion_graph,
            service_records,
            transition_records,
        ),
    }

    if args.write_motion_graph:
        write_vehicle_feasible_motion_graph(
            motion_graph,
            output_path,
            overwrite=args.overwrite_motion_graph,
        )

    if args.pretty:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(json.dumps(report, separators=(",", ":"), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())