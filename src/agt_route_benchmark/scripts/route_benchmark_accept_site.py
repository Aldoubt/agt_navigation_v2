#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from agt_route_benchmark.site_snapshot import create_site_snapshot


def main() -> int:
    parser = argparse.ArgumentParser(description="Freeze an accepted greenhouse site snapshot for Paper I formal experiments")
    parser.add_argument("--site", default="greenhouse_01")
    parser.add_argument("--pcd", required=True, type=Path)
    parser.add_argument("--map-yaml", required=True, type=Path)
    parser.add_argument("--semantic-map", required=True, type=Path)
    parser.add_argument("--coverage-yaml", required=True, type=Path)
    parser.add_argument("--platform-profile", type=Path, default=Path("profiles/platforms/mk_mini.yaml"))
    parser.add_argument("--acceptance", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    snapshot = create_site_snapshot(
        args.site,
        args.pcd,
        args.map_yaml,
        args.semantic_map,
        args.coverage_yaml,
        args.platform_profile,
        args.acceptance,
        output_path=args.output,
    )
    print(args.output)
    print(snapshot["snapshot_sha256"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
