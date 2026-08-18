#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

from agt_route_benchmark.map_curation import build_formal_map_curation_manifest
from agt_route_benchmark.map_quality import write_map_quality_evidence


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Audit and freeze one planner-independent Paper I real-map curation revision"
    )
    parser.add_argument("--site", required=True)
    parser.add_argument("--source-pcd", required=True, type=Path)
    parser.add_argument("--generated-map-yaml", required=True, type=Path)
    parser.add_argument("--accepted-map-yaml", required=True, type=Path)
    parser.add_argument("--derivation-yaml", required=True, type=Path)
    parser.add_argument("--semantic-map", required=True, type=Path)
    parser.add_argument("--platform-profile", required=True, type=Path)
    parser.add_argument("--qa-output-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    qa = write_map_quality_evidence(
        args.generated_map_yaml,
        args.accepted_map_yaml,
        args.derivation_yaml,
        output_dir=args.qa_output_dir,
    )
    manifest = build_formal_map_curation_manifest(
        site_id=args.site,
        source_pcd=args.source_pcd,
        generated_map_yaml=args.generated_map_yaml,
        accepted_map_yaml=args.accepted_map_yaml,
        derivation_yaml=args.derivation_yaml,
        override_geojson=args.qa_output_dir / "overrides.geojson",
        qa_report=args.qa_output_dir / "map_qa_report.json",
        qa_figures=[
            args.qa_output_dir / "map_curation_qa.svg",
            args.qa_output_dir / "map_curation_qa.pdf",
            args.qa_output_dir / "map_curation_qa.png",
        ],
        semantic_map=args.semantic_map,
        platform_profile=args.platform_profile,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(manifest, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(args.qa_output_dir / "map_qa_report.json")
    print(args.output)
    print(
        "accepted_matches_replay=",
        qa["accepted_matches_replay"],
        "unexplained_changed_cell_count=",
        qa["unexplained_changed_cell_count"],
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
