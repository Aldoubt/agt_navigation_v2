#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from agt_route_benchmark.map_quality import write_map_quality_evidence


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Generate planner-independent map-only QA evidence from one "
            "generated/accepted navigation-map revision"
        )
    )
    parser.add_argument("--generated-map-yaml", required=True, type=Path)
    parser.add_argument("--accepted-map-yaml", required=True, type=Path)
    parser.add_argument("--derivation-yaml", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    report = write_map_quality_evidence(
        args.generated_map_yaml,
        args.accepted_map_yaml,
        args.derivation_yaml,
        output_dir=args.output_dir,
    )
    print(args.output_dir)
    print(
        "accepted_matches_replay="
        f"{str(bool(report['accepted_matches_replay'])).lower()}"
    )
    print(
        "unexplained_changed_cell_count="
        f"{int(report['unexplained_changed_cell_count'])}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
