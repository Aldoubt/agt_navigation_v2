#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

from agt_route_benchmark.paper1_baseline_snapshot import (
    generate_p1_baseline_snapshot,
    verify_p1_baseline_snapshot,
    write_p1_acceptance,
)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Generate or verify a Paper I P1 baseline snapshot")
    sub = parser.add_subparsers(dest="command", required=True)

    acceptance = sub.add_parser("acceptance")
    acceptance.add_argument("--output", required=True, type=Path)
    acceptance.add_argument("--accepted-by", default="Xuan Yang")
    acceptance.add_argument("--accepted-at")

    generate = sub.add_parser("generate")
    for name in (
        "pcd",
        "map-yaml",
        "semantic-map",
        "coverage-yaml",
        "platform-profile",
        "acceptance",
        "curation-manifest",
        "map-project",
        "output",
    ):
        generate.add_argument(f"--{name}", required=True, type=Path)

    verify = sub.add_parser("verify")
    verify.add_argument("--snapshot", required=True, type=Path)

    args = parser.parse_args(argv)
    if args.command == "acceptance":
        print(write_p1_acceptance(args.output, accepted_by=args.accepted_by, accepted_at=args.accepted_at))
        return 0
    if args.command == "generate":
        snapshot = generate_p1_baseline_snapshot(
            pcd=args.pcd,
            map_yaml=args.map_yaml,
            semantic_map=args.semantic_map,
            coverage_yaml=args.coverage_yaml,
            platform_profile=args.platform_profile,
            acceptance=args.acceptance,
            curation_manifest=args.curation_manifest,
            map_project=args.map_project,
            output=args.output,
        )
        print(json.dumps({"status": "PASS", "snapshot_sha256": snapshot["snapshot_sha256"]}, sort_keys=True))
        return 0
    print(json.dumps(verify_p1_baseline_snapshot(args.snapshot), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
