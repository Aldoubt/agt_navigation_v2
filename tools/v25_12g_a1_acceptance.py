"""V25-12G-A1 real-data acceptance harness.

The harness is intentionally diagnostic-only. It compares the frozen legacy
longest-only vehicle-safe lane result with the A1 all-feasible-segment result
without promoting either artifact to a route-ready contract.
"""

from __future__ import annotations

import argparse
from collections.abc import Sequence


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


def main(argv: Sequence[str] | None = None) -> int:
    """Parse the frozen CLI; real-data execution is added in later TDD cycles."""
    build_parser().parse_args(argv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
