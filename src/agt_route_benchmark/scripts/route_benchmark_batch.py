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


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize Paper I benchmark result cells")
    parser.add_argument("--result-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--site", default="greenhouse_01")
    parser.add_argument("--formal", action="store_true")
    parser.add_argument("--scenario", action="append", dest="scenarios")
    parser.add_argument("--planner", action="append", dest="planners")
    args = parser.parse_args()

    if bool(args.scenarios) != bool(args.planners):
        parser.error("--scenario and --planner must either both be omitted or both be provided")

    if args.scenarios:
        expected = tuple((scenario, planner) for scenario in args.scenarios for planner in args.planners)
    else:
        expected = canonical_expected_cells()

    cells = []
    for scenario, planner in expected:
        base = args.result_root / args.site / scenario
        candidates = sorted(base.glob(f"{planner}-*")) if base.exists() else []
        if not candidates:
            continue
        run = candidates[-1]
        report_path = run / "planner_report.json"
        metrics_path = run / "metrics.json"
        if not report_path.is_file() or not metrics_path.is_file():
            continue
        report = json.loads(report_path.read_text(encoding="utf-8"))
        metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
        cells.append(MatrixCell(scenario, planner, str(report["error_code"]), metrics))

    validate_batch_completeness(cells, expected, formal=args.formal)
    csv_path, json_path = write_comparison_summary(cells, args.output)
    print(csv_path)
    print(json_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
