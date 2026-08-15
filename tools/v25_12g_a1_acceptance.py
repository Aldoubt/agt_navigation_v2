"""V25-12G-A1 real-data acceptance harness.

The harness is intentionally diagnostic-only. It compares the frozen legacy
longest-only vehicle-safe lane result with the A1 all-feasible-segment result
without promoting either artifact to a route-ready contract.
"""

from __future__ import annotations

import argparse
import json
from collections.abc import Sequence
from pathlib import Path

from agt_offline_assets.agricultural_route_io import load_agricultural_aisle_graph
from agt_offline_assets.navigation_grid import load_navigation_grid
from agt_offline_assets.site_boundary import load_site_boundary
from agt_offline_assets.vehicle_feasible_segment import (
    derive_vehicle_feasible_segment_plan,
    write_vehicle_feasible_segment_plan,
)
from agt_offline_assets.vehicle_profile import load_canonical_vehicle_profile
from agt_offline_assets.vehicle_safe_lane import (
    VehicleSafeLaneConfig,
    derive_vehicle_safe_lane_plan,
)


REPORT_SCHEMA = "agt_v25_12g_a1_acceptance_report/v1"
VALIDATION_SCOPE = "A1_SEGMENT_EXTRACTION_DIAGNOSTIC_NOT_ROUTE_READY"
DIAGNOSTIC_AISLE_IDS = (
    "aisle_016",
    "aisle_017",
    "aisle_018",
    "aisle_019",
    "aisle_020",
)


def build_parser() -> argparse.ArgumentParser:
    """Build the frozen CLI surface for the A1 acceptance harness."""
    parser = argparse.ArgumentParser(
        description="Compare V25-12G-A1 all-segment coverage with legacy longest-only lanes",
    )
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--vehicle-profile", required=True)
    parser.add_argument("--navigation-map", default="navigation_map.yaml")
    parser.add_argument("--write-segments", action="store_true")
    parser.add_argument("--overwrite-segments", action="store_true")
    parser.add_argument("--pretty", action="store_true")
    return parser


def _a1_config() -> VehicleSafeLaneConfig:
    """Return the frozen configuration used by the V25-12G-A1 experiment."""
    return VehicleSafeLaneConfig(
        sample_spacing_m=0.10,
        lateral_search_step_m=0.05,
        maximum_lateral_shift_m=0.50,
        maximum_lateral_step_m=0.15,
        preview_footprint_padding_m=0.05,
        minimum_lane_coverage_fraction=0.70,
        maximum_endpoint_retreat_m=2.00,
        minimum_contiguous_span_m=1.00,
    )


def _require_file(path: Path) -> Path:
    if not path.is_file():
        raise FileNotFoundError(f"required A1 acceptance input not found: {path}")
    return path


def _preflight(args: argparse.Namespace) -> tuple[Path, Path, Path, Path, Path]:
    """Resolve frozen input/output paths without parsing heavy asset content."""
    run_dir = Path(args.run_dir).expanduser().resolve()
    aisle_graph = _require_file(run_dir / "aisle_graph.yaml")
    site_boundary = _require_file(run_dir / "site_boundary.yaml")
    navigation_map = _require_file(run_dir / str(args.navigation_map))
    vehicle_profile = _require_file(Path(args.vehicle_profile).expanduser().resolve())

    segment_output = run_dir / "vehicle_feasible_segments.yaml"
    if args.write_segments and segment_output.exists() and not args.overwrite_segments:
        raise FileExistsError(
            f"vehicle-feasible segment asset already exists: {segment_output}"
        )

    return aisle_graph, site_boundary, navigation_map, vehicle_profile, segment_output


def _load_frozen_inputs(args: argparse.Namespace):
    """Load the four frozen A1 inputs and reject mixed map frames."""
    aisle_graph_path, boundary_path, navigation_path, vehicle_path, segment_output = (
        _preflight(args)
    )
    graph = load_agricultural_aisle_graph(aisle_graph_path)
    boundary = load_site_boundary(boundary_path)
    navigation = load_navigation_grid(navigation_path)
    vehicle = load_canonical_vehicle_profile(vehicle_path)

    if not graph.frame_id == navigation.frame_id == boundary.frame_id:
        raise ValueError(
            "A1 acceptance frame mismatch: "
            f"aisle_graph={graph.frame_id}, "
            f"navigation={navigation.frame_id}, "
            f"site_boundary={boundary.frame_id}"
        )

    return graph, boundary, navigation, vehicle, _a1_config(), segment_output


def _index_unique(items, *, label: str) -> dict[str, object]:
    """Index aisle-like records and fail closed on duplicate IDs."""
    result: dict[str, object] = {}
    for item in items:
        aisle_id = str(item.aisle_id)
        if aisle_id in result:
            raise ValueError(f"duplicate aisle_id in {label}: {aisle_id}")
        result[aisle_id] = item
    return result


def _derive_comparable_plans(graph, boundary, navigation, vehicle, cfg):
    """Derive legacy and A1 plans from identical frozen inputs."""
    source = {"acceptance_stage": "v25_12g_a1"}
    lane_plan = derive_vehicle_safe_lane_plan(
        graph,
        navigation,
        vehicle,
        cfg,
        site_boundary=boundary,
        source=source,
    )
    segment_plan = derive_vehicle_feasible_segment_plan(
        graph,
        navigation,
        vehicle,
        cfg,
        site_boundary=boundary,
        source=source,
    )

    graph_ids = [str(aisle.aisle_id) for aisle in graph.aisles]
    if len(graph_ids) != len(set(graph_ids)):
        raise ValueError("duplicate aisle_id in Agricultural Aisle Graph")

    lanes = _index_unique(lane_plan.lanes, label="VehicleSafeLanePlan")
    segments = _index_unique(segment_plan.aisles, label="VehicleFeasibleSegmentPlan")
    expected = set(graph_ids)
    if set(lanes) != expected:
        missing = sorted(expected - set(lanes))
        extra = sorted(set(lanes) - expected)
        raise ValueError(
            f"VehicleSafeLanePlan aisle IDs do not match graph: missing={missing} extra={extra}"
        )
    if set(segments) != expected:
        missing = sorted(expected - set(segments))
        extra = sorted(set(segments) - expected)
        raise ValueError(
            "VehicleFeasibleSegmentPlan aisle IDs do not match graph: "
            f"missing={missing} extra={extra}"
        )
    return lane_plan, segment_plan, lanes, segments


def _aisle_metrics(aisle_id: str, lane, segment_result) -> dict[str, object]:
    """Build the exact frozen A1 per-aisle comparison metrics."""
    active_segment_length_m = float(
        sum(float(segment.length_m) for segment in segment_result.active_segments)
    )
    longest_only_span_m = float(lane.selected_span_m)
    recoverable_additional_length_m = max(
        0.0,
        active_segment_length_m - longest_only_span_m,
    )
    endpoint_classifications = [
        {
            "segment_id": segment.segment_id,
            "low_endpoint_type": segment.low_endpoint_type,
            "high_endpoint_type": segment.high_endpoint_type,
        }
        for segment in segment_result.active_segments
    ]
    return {
        "aisle_id": aisle_id,
        "structural_length_m": float(segment_result.structural_length_m),
        "raw_feasible_fragment_count": int(segment_result.raw_feasible_fragment_count),
        "active_segment_count": len(segment_result.active_segments),
        "rejected_fragment_count": len(segment_result.rejected_fragments),
        "active_segment_length_m": active_segment_length_m,
        "current_longest_only_selected_span_m": longest_only_span_m,
        "recoverable_additional_length_m": recoverable_additional_length_m,
        "active_segment_ids": [
            segment.segment_id for segment in segment_result.active_segments
        ],
        "endpoint_classifications": endpoint_classifications,
    }


def _build_report(args, graph, vehicle, cfg, lanes, segments) -> dict[str, object]:
    """Build the deterministic diagnostic report for the frozen aisle subset."""
    metrics = [
        _aisle_metrics(aisle_id, lanes[aisle_id], segments[aisle_id])
        for aisle_id in DIAGNOSTIC_AISLE_IDS
        if aisle_id in lanes and aisle_id in segments
    ]

    structural_length_total = float(
        sum(float(item["structural_length_m"]) for item in metrics)
    )
    active_segment_length_total = float(
        sum(float(item["active_segment_length_m"]) for item in metrics)
    )
    longest_only_total = float(
        sum(float(item["current_longest_only_selected_span_m"]) for item in metrics)
    )
    recoverable_total = float(
        sum(float(item["recoverable_additional_length_m"]) for item in metrics)
    )
    rejected_fragment_count = int(
        sum(int(item["rejected_fragment_count"]) for item in metrics)
    )
    active_segment_count = int(
        sum(int(item["active_segment_count"]) for item in metrics)
    )
    segment_recovery_fraction = (
        0.0
        if structural_length_total <= 1.0e-12
        else float(active_segment_length_total / structural_length_total)
    )

    return {
        "schema": REPORT_SCHEMA,
        "validation_scope": VALIDATION_SCOPE,
        "diagnostic_aisle_ids": list(DIAGNOSTIC_AISLE_IDS),
        "frame_id": graph.frame_id,
        "platform_id": vehicle.profile_id,
        "run_dir": str(Path(args.run_dir).expanduser().resolve()),
        "navigation_map": str(args.navigation_map),
        "configuration": {
            "sample_spacing_m": float(cfg.sample_spacing_m),
            "lateral_search_step_m": float(cfg.lateral_search_step_m),
            "maximum_lateral_shift_m": float(cfg.maximum_lateral_shift_m),
            "maximum_lateral_step_m": float(cfg.maximum_lateral_step_m),
            "preview_footprint_padding_m": float(cfg.preview_footprint_padding_m),
            "minimum_lane_coverage_fraction": float(
                cfg.minimum_lane_coverage_fraction
            ),
            "maximum_endpoint_retreat_m": float(cfg.maximum_endpoint_retreat_m),
            "minimum_contiguous_span_m": float(cfg.minimum_contiguous_span_m),
        },
        "aisles": metrics,
        "summary": {
            "diagnostic_aisle_count": len(metrics),
            "active_segment_count": active_segment_count,
            "rejected_fragment_count": rejected_fragment_count,
            "structural_length_m": structural_length_total,
            "active_segment_length_m": active_segment_length_total,
            "current_longest_only_selected_span_m": longest_only_total,
            "recoverable_additional_length_m": recoverable_total,
            "segment_recovery_fraction": segment_recovery_fraction,
        },
    }


def _emit_report(report: dict[str, object], *, pretty: bool) -> None:
    if pretty:
        text = json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False)
    else:
        text = json.dumps(
            report,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
        )
    print(text)


def main(argv: Sequence[str] | None = None) -> int:
    """Run the frozen A1 longest-only versus all-segment diagnostic comparison."""
    args = build_parser().parse_args(argv)
    graph, boundary, navigation, vehicle, cfg, segment_output = _load_frozen_inputs(args)
    _lane_plan, segment_plan, lanes, segments = _derive_comparable_plans(
        graph,
        boundary,
        navigation,
        vehicle,
        cfg,
    )
    if args.write_segments:
        write_vehicle_feasible_segment_plan(
            segment_plan,
            segment_output,
            overwrite=bool(args.overwrite_segments),
        )
    report = _build_report(args, graph, vehicle, cfg, lanes, segments)
    _emit_report(report, pretty=bool(args.pretty))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
