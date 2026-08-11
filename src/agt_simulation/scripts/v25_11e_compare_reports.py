#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


FAULT_CASES = (
    "localization_lost",
    "lidar_dropout",
    "imu_dropout",
)


def _read_json(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    return data if isinstance(data, dict) else {}


def _atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Aggregate V25-11E active fault comparison reports"
    )
    parser.add_argument("--report-dir", default="/tmp")
    parser.add_argument(
        "--output", default="/tmp/agt_v25_11e_comparison_matrix.json"
    )
    args = parser.parse_args()

    report_dir = Path(args.report_dir)
    output = Path(args.output)
    rows: list[dict[str, Any]] = []
    missing: list[str] = []

    for fault_case in FAULT_CASES:
        path = report_dir / f"agt_v25_11e_compare_{fault_case}.json"
        if not path.is_file():
            missing.append(str(path))
            continue
        report = _read_json(path)
        metrics = report.get("metrics", {}) if isinstance(report, dict) else {}
        rows.append(
            {
                "fault_case": fault_case,
                "status": report.get("status"),
                "fault_to_safety_zero_ms": metrics.get("fault_to_safety_zero_ms"),
                "fault_to_route_terminal_ms": metrics.get("fault_to_route_terminal_ms"),
                "safety_zero_to_route_terminal_ms": metrics.get(
                    "safety_zero_to_route_terminal_ms"
                ),
                "fault_ground_truth_speed_mps": metrics.get(
                    "fault_ground_truth_speed_mps"
                ),
                "max_post_fault_cmd_norm": metrics.get("max_post_fault_cmd_norm"),
                "max_post_fault_ground_truth_speed_mps": metrics.get(
                    "max_post_fault_ground_truth_speed_mps"
                ),
                "max_post_fault_distance_m": metrics.get(
                    "max_post_fault_distance_m"
                ),
                "route_completion_ratio": metrics.get("route_completion_ratio"),
                "report": str(path),
            }
        )

    all_present = not missing and len(rows) == len(FAULT_CASES)
    all_passed = all_present and all(row.get("status") == "PASS" for row in rows)
    payload = {
        "schema": "agt_v25_11e_comparison_matrix/v1",
        "gate": "V25-11E",
        "stage": "active_fault_comparison_matrix",
        "status": "PASS" if all_passed else "FAIL",
        "checks": {
            "all_fault_reports_present": all_present,
            "all_fault_reports_passed": all_passed,
        },
        "missing_reports": missing,
        "rows": rows,
    }
    _atomic_write_json(output, payload)
    print(json.dumps(payload, indent=2))
    raise SystemExit(0 if all_passed else 1)


if __name__ == "__main__":
    main()
