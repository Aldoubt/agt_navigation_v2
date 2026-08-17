#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

from agt_route_benchmark.batch import (
    MatrixCell,
    canonical_expected_cells,
    validate_batch_completeness,
    write_comparison_summary,
)
from agt_route_benchmark.result_selection import load_site_snapshot_identity, select_result_run


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize Paper I benchmark result cells")
    parser.add_argument("--result-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--site", default="greenhouse_01")
    parser.add_argument("--formal", action="store_true")
    parser.add_argument("--site-snapshot", type=Path)
    parser.add_argument("--scenario", action="append", dest="scenarios")
    parser.add_argument("--planner", action="append", dest="planners")
    args = parser.parse_args()

    if bool(args.scenarios) != bool(args.planners):
        parser.error("--scenario and --planner must either both be omitted or both be provided")
    if args.formal and args.site_snapshot is None:
        parser.error("--formal requires --site-snapshot so result revisions cannot be mixed")

    if args.scenarios:
        expected = tuple((scenario, planner) for scenario in args.scenarios for planner in args.planners)
    else:
        expected = canonical_expected_cells()

    snapshot_sha256 = (
        load_site_snapshot_identity(args.site_snapshot)
        if args.formal and args.site_snapshot is not None
        else None
    )
    cells = []
    for scenario, planner in expected:
        run = select_result_run(
            args.result_root,
            args.site,
            scenario,
            planner,
            formal=args.formal,
            site_snapshot_sha256=snapshot_sha256,
        )
        if run is None:
            continue
        report = json.loads((run / "planner_report.json").read_text(encoding="utf-8"))
        metrics = json.loads((run / "metrics.json").read_text(encoding="utf-8"))
        cells.append(MatrixCell(scenario, planner, str(report["error_code"]), metrics))

    validate_batch_completeness(cells, expected, formal=args.formal)
    csv_path, json_path = write_comparison_summary(cells, args.output)
    print(csv_path)
    print(json_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
