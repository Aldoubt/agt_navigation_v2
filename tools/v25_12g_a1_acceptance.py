"""V25-12G-A1 real-data acceptance harness.

The harness is intentionally diagnostic-only. It compares the frozen legacy
longest-only vehicle-safe lane result with the A1 all-feasible-segment result
without promoting either artifact to a route-ready contract.
"""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from pathlib import Path


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


def main(argv: Sequence[str] | None = None) -> int:
    """Validate frozen A1 input/output paths; derivation follows in later cycles."""
    args = build_parser().parse_args(argv)
    _preflight(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
