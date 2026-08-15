#!/usr/bin/env python3
"""Repeatable V25-12F current-vs-candidate greenhouse acceptance harness.

This tool does not regenerate the candidate map.  It consumes already frozen
12F artifacts, evaluates the current and candidate Navigation Grids with the
same canonical MK-mini lane configuration, and replays connector_015/017 with
one unchanged default ReversePrimitiveConnectorConfig plus the same Site
Boundary.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

from agt_offline_assets.agricultural_route_io import (
    load_agricultural_aisle_graph,
    load_coverage_connector_requests,
    load_turn_zones,
)
from agt_offline_assets.navigation_grid import load_navigation_grid
from agt_offline_assets.reverse_fallback_admission import (
    ReverseFallbackAdmissionItem,
    ReverseFallbackAdmissionPlan,
)
from agt_offline_assets.reverse_primitive_connector import (
    ReversePrimitiveConnectorConfig,
    derive_reverse_primitive_connector_plan,
)
from agt_offline_assets.site_boundary import load_site_boundary
from agt_offline_assets.vehicle_profile import load_canonical_vehicle_profile
from agt_offline_assets.vehicle_safe_lane import (
    VehicleSafeLaneConfig,
    derive_vehicle_safe_lane_plan,
)

REGRESSION_CONNECTOR_IDS = ("connector_015", "connector_017")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compare frozen current and V25-12F candidate Navigation Maps"
    )
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--vehicle-profile", required=True)
    parser.add_argument(
        "--pretty",
        action="store_true",
        help="pretty-print JSON instead of compact deterministic JSON",
    )
    return parser


def _require_file(run_dir: Path, name: str) -> Path:
    path = run_dir / name
    if not path.is_file():
        raise FileNotFoundError(f"required V25-12F acceptance asset not found: {path}")
    return path


def _lane_summary(plan) -> dict[str, Any]:
    lanes = []
    coverages = []
    boundary_conflicts = 0
    for lane in plan.lanes:
        coverage = float(lane.coverage_fraction)
        coverages.append(coverage)
        if "SITE_BOUNDARY_CONFLICT" in str(lane.reason):
            boundary_conflicts += 1
        lanes.append(
            {
                "aisle_id": lane.aisle_id,
                "status": lane.status,
                "coverage_fraction": coverage,
                "selected_span_m": float(lane.selected_span_m),
                "structural_length_m": float(lane.structural_length_m),
                "low_u_retreat_m": lane.low_u_retreat_m,
                "high_u_retreat_m": lane.high_u_retreat_m,
                "maximum_used_lateral_shift_m": float(
                    lane.maximum_used_lateral_shift_m
                ),
                "reason": lane.reason,
            }
        )
    mean_coverage = (
        0.0 if not coverages else float(sum(coverages) / len(coverages))
    )
    return {
        "ready": int(plan.ready_count),
        "partial": int(plan.partial_count),
        "unavailable": int(plan.unavailable_count),
        "mean_coverage_fraction": mean_coverage,
        "site_boundary_conflict_aisles": int(boundary_conflicts),
        "aisles": lanes,
    }


def _r6b_summary(plan) -> dict[str, Any]:
    output: dict[str, Any] = {}
    for item in plan.connectors:
        output[item.connector_id] = {
            "status": item.status,
            "backend": item.backend,
            "path_length_m": item.path_length_m,
            "forward_distance_m": float(item.forward_distance_m),
            "reverse_distance_m": float(item.reverse_distance_m),
            "cusp_count": int(item.cusp_count),
            "search_expansions": int(item.search_expansions),
            "goal_position_error_m": item.goal_position_error_m,
            "goal_yaw_error_rad": item.goal_yaw_error_rad,
            "goal_yaw_error_deg": (
                None
                if item.goal_yaw_error_rad is None
                else math.degrees(float(item.goal_yaw_error_rad))
            ),
            "reason": item.reason,
        }
    return output


def _regression_admission(selected, zones, vehicle) -> ReverseFallbackAdmissionPlan:
    items = tuple(
        ReverseFallbackAdmissionItem(
            connector_id=request.connector_id,
            from_aisle_id=request.from_aisle_id,
            to_aisle_id=request.to_aisle_id,
            turn_zone_id=request.turn_zone_id,
            forward_audit_status="FROZEN_REGRESSION_REPLAY",
            decision="ELIGIBLE_REVERSE_FALLBACK",
            reason=(
                "V25-12F candidate-map regression replay of a previously selected "
                "connector; this does not change R6A production admission"
            ),
        )
        for request in selected
    )
    return ReverseFallbackAdmissionPlan(
        frame_id=zones.frame_id,
        platform_id=vehicle.profile_id,
        platform_profile_sha256=vehicle.profile_sha256,
        items=items,
        source={"validation_scope": "V25_12F_REGRESSION_REPLAY_ONLY"},
    )


def run_acceptance(run_dir: Path, vehicle_profile_path: Path) -> dict[str, Any]:
    run_dir = run_dir.expanduser().resolve()
    vehicle_profile_path = vehicle_profile_path.expanduser().resolve()
    if not run_dir.is_dir():
        raise NotADirectoryError(f"run directory not found: {run_dir}")
    if not vehicle_profile_path.is_file():
        raise FileNotFoundError(
            f"canonical vehicle profile not found: {vehicle_profile_path}"
        )

    graph_path = _require_file(run_dir, "aisle_graph.yaml")
    current_nav_path = _require_file(run_dir, "navigation_map.yaml")
    candidate_nav_path = _require_file(run_dir, "navigation_map_12f.yaml")
    boundary_path = _require_file(run_dir, "site_boundary.yaml")
    coverage_path = _require_file(run_dir, "coverage_order.yaml")
    zones_path = _require_file(run_dir, "turn_zones.yaml")
    _require_file(run_dir, "traversability_evidence.yaml")
    _require_file(run_dir, "traversability_evidence.npz")
    _require_file(run_dir, "navigation_map_12f_derivation.yaml")

    graph = load_agricultural_aisle_graph(graph_path)
    current_nav = load_navigation_grid(current_nav_path)
    candidate_nav = load_navigation_grid(candidate_nav_path)
    boundary = load_site_boundary(boundary_path, expected_frame_id=graph.frame_id)
    vehicle = load_canonical_vehicle_profile(vehicle_profile_path)

    if current_nav.frame_id != graph.frame_id:
        raise ValueError("current Navigation Grid frame does not match Aisle Graph")
    if candidate_nav.frame_id != graph.frame_id:
        raise ValueError("candidate Navigation Grid frame does not match Aisle Graph")
    if current_nav.occupancy.shape != candidate_nav.occupancy.shape:
        raise ValueError("current and candidate Navigation Grids have different shapes")
    if not math.isclose(
        current_nav.resolution_m,
        candidate_nav.resolution_m,
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        raise ValueError("current and candidate Navigation Grids have different resolution")
    if not (
        math.isclose(
            current_nav.origin_x_m,
            candidate_nav.origin_x_m,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        )
        and math.isclose(
            current_nav.origin_y_m,
            candidate_nav.origin_y_m,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        )
    ):
        raise ValueError("current and candidate Navigation Grids have different origins")

    lane_cfg = VehicleSafeLaneConfig(
        sample_spacing_m=0.10,
        lateral_search_step_m=0.05,
        maximum_lateral_shift_m=0.50,
        maximum_lateral_step_m=0.15,
        preview_footprint_padding_m=0.05,
        minimum_lane_coverage_fraction=0.70,
        maximum_endpoint_retreat_m=2.00,
        minimum_contiguous_span_m=1.00,
    )
    current_lane = derive_vehicle_safe_lane_plan(
        graph,
        current_nav,
        vehicle,
        lane_cfg,
        site_boundary=boundary,
        source={"acceptance_map": "current"},
    )
    candidate_lane = derive_vehicle_safe_lane_plan(
        graph,
        candidate_nav,
        vehicle,
        lane_cfg,
        site_boundary=boundary,
        source={"acceptance_map": "candidate_12f"},
    )

    requests = load_coverage_connector_requests(coverage_path)
    zones = load_turn_zones(zones_path)
    request_by_id = {request.connector_id: request for request in requests}
    missing_connectors = [
        connector_id
        for connector_id in REGRESSION_CONNECTOR_IDS
        if connector_id not in request_by_id
    ]
    if missing_connectors:
        raise ValueError(
            "required regression connector requests are missing: "
            + ", ".join(missing_connectors)
        )
    selected = tuple(
        request_by_id[connector_id] for connector_id in REGRESSION_CONNECTOR_IDS
    )
    admission = _regression_admission(selected, zones, vehicle)

    # Intentionally use one unchanged default configuration for both maps.
    r6b_cfg = ReversePrimitiveConnectorConfig()
    current_r6b = derive_reverse_primitive_connector_plan(
        selected,
        admission,
        zones,
        current_nav,
        vehicle,
        r6b_cfg,
        site_boundary=boundary,
        source={"acceptance_map": "current"},
    )
    candidate_r6b = derive_reverse_primitive_connector_plan(
        selected,
        admission,
        zones,
        candidate_nav,
        vehicle,
        r6b_cfg,
        site_boundary=boundary,
        source={"acceptance_map": "candidate_12f"},
    )

    current_lane_summary = _lane_summary(current_lane)
    candidate_lane_summary = _lane_summary(candidate_lane)
    result = {
        "schema": "agt_v25_12f_acceptance_report/v1",
        "validation_scope": "OFFLINE_A_B_REVIEW_NOT_ROUTE_READY",
        "run_dir": str(run_dir),
        "vehicle_profile": str(vehicle_profile_path),
        "platform_id": vehicle.profile_id,
        "site_boundary": {
            "path": str(boundary_path),
            "boundary_semantics": boundary.boundary_semantics,
            "vertex_count": len(boundary.outer_boundary_xy),
            "edge_policy": "TOUCH_OR_CROSS_IS_SITE_BOUNDARY_CONFLICT",
        },
        "lane_config": {
            "sample_spacing_m": lane_cfg.sample_spacing_m,
            "lateral_search_step_m": lane_cfg.lateral_search_step_m,
            "maximum_lateral_shift_m": lane_cfg.maximum_lateral_shift_m,
            "maximum_lateral_step_m": lane_cfg.maximum_lateral_step_m,
            "preview_footprint_padding_m": lane_cfg.preview_footprint_padding_m,
            "minimum_lane_coverage_fraction": lane_cfg.minimum_lane_coverage_fraction,
            "maximum_endpoint_retreat_m": lane_cfg.maximum_endpoint_retreat_m,
            "minimum_contiguous_span_m": lane_cfg.minimum_contiguous_span_m,
        },
        "vehicle_safe_lane": {
            "current": current_lane_summary,
            "candidate_12f": candidate_lane_summary,
            "delta": {
                "ready": (
                    candidate_lane_summary["ready"] - current_lane_summary["ready"]
                ),
                "partial": (
                    candidate_lane_summary["partial"] - current_lane_summary["partial"]
                ),
                "unavailable": (
                    candidate_lane_summary["unavailable"]
                    - current_lane_summary["unavailable"]
                ),
                "mean_coverage_fraction": (
                    candidate_lane_summary["mean_coverage_fraction"]
                    - current_lane_summary["mean_coverage_fraction"]
                ),
            },
        },
        "r6b_config": {
            "primitive_length_m": r6b_cfg.primitive_length_m,
            "collision_sample_step_m": r6b_cfg.collision_sample_step_m,
            "state_xy_resolution_m": r6b_cfg.state_xy_resolution_m,
            "state_yaw_resolution_deg": r6b_cfg.state_yaw_resolution_deg,
            "goal_position_tolerance_m": r6b_cfg.goal_position_tolerance_m,
            "goal_yaw_tolerance_deg": r6b_cfg.goal_yaw_tolerance_deg,
            "goal_shot_distance_m": r6b_cfg.goal_shot_distance_m,
            "max_cusps": r6b_cfg.max_cusps,
            "max_expansions": r6b_cfg.max_expansions,
            "max_path_length_m": r6b_cfg.max_path_length_m,
            "reverse_cost_multiplier": r6b_cfg.reverse_cost_multiplier,
            "cusp_penalty_m": r6b_cfg.cusp_penalty_m,
            "steering_change_penalty_m": r6b_cfg.steering_change_penalty_m,
            "longitudinal_zone_padding_m": r6b_cfg.longitudinal_zone_padding_m,
            "lateral_pair_padding_m": r6b_cfg.lateral_pair_padding_m,
            "preview_footprint_padding_m": r6b_cfg.preview_footprint_padding_m,
        },
        "connector_regression": {
            "current": _r6b_summary(current_r6b),
            "candidate_12f": _r6b_summary(candidate_r6b),
        },
        "safety_summary": {
            "current_lane_site_boundary_conflicts": current_lane_summary[
                "site_boundary_conflict_aisles"
            ],
            "candidate_lane_site_boundary_conflicts": candidate_lane_summary[
                "site_boundary_conflict_aisles"
            ],
            "candidate_route_ready_claimed": False,
        },
    }
    return result


def main(argv=None) -> int:
    args = _parser().parse_args(argv)
    try:
        report = run_acceptance(
            Path(args.run_dir),
            Path(args.vehicle_profile),
        )
    except (FileNotFoundError, NotADirectoryError, KeyError, OSError, TypeError, ValueError) as exc:
        print(
            json.dumps(
                {
                    "status": "ERROR",
                    "message": str(exc),
                },
                ensure_ascii=False,
            )
        )
        return 2

    print(
        json.dumps(
            report,
            ensure_ascii=False,
            sort_keys=True,
            indent=2 if args.pretty else None,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
