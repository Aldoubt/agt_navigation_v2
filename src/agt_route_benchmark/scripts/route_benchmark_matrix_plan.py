#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from agt_route_benchmark.matrix_plan import build_execution_plan


def main() -> int:
    package_share = Path(get_package_share_directory("agt_route_benchmark"))
    parser = argparse.ArgumentParser(description="Generate the auditable 23-cell Paper I execution plan")
    parser.add_argument("--site", default="greenhouse_01")
    parser.add_argument("--scenario-dir", type=Path, default=package_share / "scenarios")
    parser.add_argument("--result-root", type=Path, required=True)
    parser.add_argument("--map", dest="map_yaml", type=Path)
    parser.add_argument("--platform-profile", type=Path)
    parser.add_argument("--semantic-map", type=Path)
    parser.add_argument("--manual-waypoints", type=Path)
    parser.add_argument("--ours-route-csv", type=Path)
    parser.add_argument("--lattice-filepath", type=Path)
    parser.add_argument("--site-snapshot", type=Path)
    parser.add_argument("--formal", action="store_true")
    parser.add_argument("--run-id", default="run_001")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--show-commands", action="store_true")
    parser.add_argument("--strict", action="store_true", help="Return non-zero while any canonical cell is blocked")
    args = parser.parse_args()

    plan = build_execution_plan(
        result_root=args.result_root,
        site_id=args.site,
        scenario_dir=args.scenario_dir,
        map_yaml=args.map_yaml,
        platform_profile=args.platform_profile,
        semantic_map=args.semantic_map,
        manual_waypoints=args.manual_waypoints,
        ours_route_csv=args.ours_route_csv,
        lattice_filepath=args.lattice_filepath,
        site_snapshot=args.site_snapshot,
        formal=args.formal,
        run_id=args.run_id,
    )
    payload = {
        "schema_version": "1.0",
        "site_id": args.site,
        "formal": args.formal,
        "expected_cell_count": len(plan),
        "complete_cell_count": sum(cell.status == "COMPLETE" for cell in plan),
        "ready_cell_count": sum(cell.status == "READY" for cell in plan),
        "blocked_cell_count": sum(cell.status == "BLOCKED_INPUT" for cell in plan),
        "cells": [asdict(cell) for cell in plan],
    }
    if args.output is not None:
        output = args.output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(output)
    print(
        f"matrix cells={payload['expected_cell_count']} complete={payload['complete_cell_count']} "
        f"ready={payload['ready_cell_count']} blocked={payload['blocked_cell_count']}"
    )
    for cell in plan:
        missing = ",".join(cell.missing_inputs) if cell.missing_inputs else "-"
        print(f"{cell.scenario_id:24s} {cell.planner_id:28s} {cell.status:13s} missing={missing}")
        if args.show_commands and cell.status == "READY":
            print(f"  {cell.command}")
    if args.strict and any(cell.status == "BLOCKED_INPUT" for cell in plan):
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
