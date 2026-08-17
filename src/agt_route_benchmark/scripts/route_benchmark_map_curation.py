#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

from agt_route_benchmark.map_curation import build_map_curation_manifest


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build a planner-independent real-map curation evidence manifest"
    )
    parser.add_argument("--site", required=True)
    parser.add_argument("--source-pcd", required=True, type=Path)
    parser.add_argument("--generated-map-yaml", required=True, type=Path)
    parser.add_argument("--override-geojson", required=True, type=Path)
    parser.add_argument("--accepted-map-yaml", required=True, type=Path)
    parser.add_argument("--semantic-map", required=True, type=Path)
    parser.add_argument("--platform-profile", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    manifest = build_map_curation_manifest(
        site_id=args.site,
        source_pcd=args.source_pcd,
        generated_map_yaml=args.generated_map_yaml,
        override_geojson=args.override_geojson,
        accepted_map_yaml=args.accepted_map_yaml,
        semantic_map=args.semantic_map,
        platform_profile=args.platform_profile,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(manifest, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
