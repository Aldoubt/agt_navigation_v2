#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

from agt_route_benchmark.path_io import read_path_csv
from agt_route_benchmark.tracking_io import read_executed_trajectory_csv
from agt_route_benchmark.tracking_metrics import compute_tracking_metrics


def main() -> int:
    parser = argparse.ArgumentParser(description="Evaluate simulation/real trajectory against immutable Paper I path.csv")
    parser.add_argument("--reference", required=True, type=Path, help="Paper I benchmark path.csv")
    parser.add_argument("--executed", required=True, type=Path, help="stamp_s,x_m,y_m,yaw_rad trajectory CSV in the same frame")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    reference = read_path_csv(args.reference)
    executed = read_executed_trajectory_csv(args.executed)
    metrics = compute_tracking_metrics(reference, executed)
    metrics.update(
        {
            "reference_path_csv": str(args.reference.expanduser().resolve()),
            "executed_trajectory_csv": str(args.executed.expanduser().resolve()),
        }
    )
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(metrics, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
