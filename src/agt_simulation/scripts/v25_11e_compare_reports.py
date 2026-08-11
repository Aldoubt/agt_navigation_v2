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

MAX_FAULT_TO_SAFETY_ZERO_MS = 750.0
MAX_FAULT_TO_ROUTE_TERMINAL_MS = 1000.0
MAX_POST_FAULT_DISTANCE_M = 0.75


def _read_json(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    return data if isinstance(data, dict) else {}


def _atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def _bounded(value: Any, upper: float) -> bool:
    return isinstance(value, (int, float)) and 0.0 <= float(value) <= upper


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Aggregate and gate V25-11E active fault comparison reports"
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
        zero_ms = metrics.get("fault_to_safety_zero_ms")
        terminal_ms = metrics.get("fault_to_route_terminal_ms")
        distance_m = metrics.get("max_post_fault_distance_m")
        rows.append(
            {
                "fault_case": fault_case,
                "status": report.get("status"),
                "fault_to_safety_zero_ms": zero_ms,
                "fault_to_route_terminal_ms": terminal_ms,
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
                "max_post_fault_distance_m": distance_m,
                "route_completion_ratio": metrics.get("route_completion_ratio"),
                "response_checks": {
                    "safety_zero_bounded": _bounded(
                        zero_ms, MAX_FAULT_TO_SAFETY_ZERO_MS
                    ),
                    "route_terminal_bounded": _bounded(
                        terminal_ms, MAX_FAULT_TO_ROUTE_TERMINAL_MS
                    ),
                    "post_fault_distance_bounded": _bounded(
                        distance_m, MAX_POST_FAULT_DISTANCE_M
                    ),
                },
                "report": str(path),
            }
        )

    all_present = not missing and len(rows) == len(FAULT_CASES)
    all_passed = all_present and all(row.get("status") == "PASS" for row in rows)
    safety_zero_bounded = all_present and all(
        row["response_checks"]["safety_zero_bounded"] for row in rows
    )
    route_terminal_bounded = all_present and all(
        row["response_checks"]["route_terminal_bounded"] for row in rows
    )
    post_fault_distance_bounded = all_present and all(
        row["response_checks"]["post_fault_distance_bounded"] for row in rows
    )

    checks = {
        "all_fault_reports_present": all_present,
        "all_fault_reports_passed": all_passed,
        "all_safety_zero_bounded": safety_zero_bounded,
        "all_route_terminal_bounded": route_terminal_bounded,
        "all_post_fault_distance_bounded": post_fault_distance_bounded,
    }
    passed = all(bool(value) for value in checks.values())
    payload = {
        "schema": "agt_v25_11e_comparison_matrix/v2",
        "gate": "V25-11E",
        "stage": "safety_response_latency_gate",
        "status": "PASS" if passed else "FAIL",
        "thresholds": {
            "max_fault_to_safety_zero_ms": MAX_FAULT_TO_SAFETY_ZERO_MS,
            "max_fault_to_route_terminal_ms": MAX_FAULT_TO_ROUTE_TERMINAL_MS,
            "max_post_fault_distance_m": MAX_POST_FAULT_DISTANCE_M,
        },
        "checks": checks,
        "missing_reports": missing,
        "rows": rows,
    }
    _atomic_write_json(output, payload)
    print(json.dumps(payload, indent=2))
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
