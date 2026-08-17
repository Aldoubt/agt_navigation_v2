#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

from agt_route_benchmark.batch import MatrixCell, validate_batch_completeness, write_comparison_summary


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize Paper I benchmark result cells")
    parser.add_argument("--result-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--formal", action="store_true")
    parser.add_argument("--scenario", action="append", dest="scenarios", required=True)
    parser.add_argument("--planner", action="append", dest="planners", required=True)
    args = parser.parse_args()

    cells = []
    for scenario in args.scenarios:
        for planner in args.planners:
            base = args.result_root / "greenhouse_01" / scenario
            candidates = sorted(base.glob(f"{planner}-*")) if base.exists() else []
            if not candidates:
                continue
            run = candidates[-1]
            report = json.loads((run / "planner_report.json").read_text(encoding="utf-8"))
            metrics = json.loads((run / "metrics.json").read_text(encoding="utf-8"))
            cells.append(MatrixCell(scenario, planner, str(report["error_code"]), metrics))

    validate_batch_completeness(cells, args.scenarios, args.planners, formal=args.formal)
    csv_path, json_path = write_comparison_summary(cells, args.output)
    print(csv_path)
    print(json_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
