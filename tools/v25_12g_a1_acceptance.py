"""V25-12G-A1 real-data acceptance harness.

The harness is intentionally diagnostic-only. It compares the frozen legacy
longest-only vehicle-safe lane result with the A1 all-feasible-segment result
without promoting either artifact to a route-ready contract.
"""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from pathlib import Path

from agt_offline_assets.agricultural_route_io import load_agricultural_aisle_graph
from agt_offline_assets.navigation_grid import load_navigation_grid
from agt_offline_assets.site_boundary import load_site_boundary
from agt_offline_assets.vehicle_profile import load_canonical_vehicle_profile
from agt_offline_assets.vehicle_safe_lane import VehicleSafeLaneConfig


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

    if not (
        graph.frame_id == navigation.frame_id == boundary.frame_id
    ):
        raise ValueError(
            "A1 acceptance frame mismatch: "
            f"aisle_graph={graph.frame_id}, "
            f"navigation={navigation.frame_id}, "
            f"site_boundary={boundary.frame_id}"
        )

    return graph, boundary, navigation, vehicle, _a1_config(), segment_output


def main(argv: Sequence[str] | None = None) -> int:
    """Load and validate frozen A1 inputs; derivation follows in later cycles."""
    args = build_parser().parse_args(argv)
    _load_frozen_inputs(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
