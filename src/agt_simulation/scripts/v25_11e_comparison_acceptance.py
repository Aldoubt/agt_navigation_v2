#!/usr/bin/env python3

from __future__ import annotations

import json
from pathlib import Path
import time
from typing import Any

import rclpy
from rclpy.node import Node


ACTIVE_FAULT_CASES = {
    "localization_lost",
    "lidar_dropout",
    "imu_dropout",
}


def _read_json(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    return data if isinstance(data, dict) else {}


def _read_live_timeline_events(path: Path) -> tuple[set[str], int]:
    """Read an append-only JSONL timeline without racing its writer.

    The observability node remains alive while comparison acceptance reads the
    timeline. A snapshot can therefore end with one partially written JSON line.
    Ignore only that final malformed line; malformed complete records in the
    middle of the file remain a hard error because they indicate artifact
    corruption rather than a normal concurrent-write boundary.
    """
    events: set[str] = set()
    skipped_tail_records = 0
    if not path.is_file():
        return events, skipped_tail_records

    lines = path.read_text(encoding="utf-8").splitlines()
    last_index = len(lines) - 1
    for index, line in enumerate(lines):
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            if index == last_index:
                skipped_tail_records += 1
                continue
            raise
        if isinstance(record, dict):
            events.add(str(record.get("event")))
    return events, skipped_tail_records


def _atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


class ComparisonAcceptance(Node):
    def __init__(self) -> None:
        super().__init__("agt_v25_11e_comparison_acceptance")
        self.fault_case = str(
            self.declare_parameter("fault_case", "localization_lost").value
        )
        if self.fault_case not in ACTIVE_FAULT_CASES:
            raise RuntimeError(
                f"unsupported V25-11E fault_case={self.fault_case!r}; "
                f"expected one of {sorted(ACTIVE_FAULT_CASES)}"
            )
        self.timeout_s = float(self.declare_parameter("timeout_s", 50.0).value)
        self.settle_s = float(self.declare_parameter("settle_s", 1.0).value)
        if self.timeout_s <= 0.0 or self.settle_s < 0.0:
            raise RuntimeError("comparison acceptance timing parameters are invalid")

        self.metrics_path = Path(
            str(
                self.declare_parameter(
                    "metrics_path",
                    f"/tmp/agt_v25_11e_{self.fault_case}_fault_metrics.json",
                ).value
            )
        )
        self.summary_path = Path(
            str(
                self.declare_parameter(
                    "summary_path",
                    f"/tmp/agt_v25_11e_{self.fault_case}_summary.json",
                ).value
            )
        )
        self.timeline_path = Path(
            str(
                self.declare_parameter(
                    "timeline_path",
                    f"/tmp/agt_v25_11e_{self.fault_case}_timeline.jsonl",
                ).value
            )
        )
        self.v25_11d_result_path = Path(
            str(
                self.declare_parameter(
                    "v25_11d_result_path",
                    f"/tmp/agt_v25_11d_{self.fault_case}_result.json",
                ).value
            )
        )
        self.report_path = Path(
            str(
                self.declare_parameter(
                    "report_path",
                    f"/tmp/agt_v25_11e_compare_{self.fault_case}.json",
                ).value
            )
        )


def _metrics_ready(node: ComparisonAcceptance) -> bool:
    if not node.metrics_path.is_file():
        return False
    try:
        metrics = _read_json(node.metrics_path)
    except (OSError, json.JSONDecodeError):
        return False
    checks = metrics.get("checks", {})
    return bool(
        isinstance(checks, dict)
        and checks.get("fault_injection_seen")
        and checks.get("route_terminal_after_fault_seen")
    )


def _wait_until(node: Node, predicate, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return bool(predicate())


def _spin_for(node: Node, duration_s: float) -> None:
    deadline = time.monotonic() + duration_s
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ComparisonAcceptance()
    result: dict[str, Any] = {
        "gate": "V25-11E",
        "stage": "active_fault_comparison",
        "fault_case": node.fault_case,
        "status": "FAIL",
        "checks": {},
        "metrics": {},
        "artifacts": {
            "fault_metrics": str(node.metrics_path),
            "observability_summary": str(node.summary_path),
            "observability_timeline": str(node.timeline_path),
            "v25_11d_result": str(node.v25_11d_result_path),
        },
    }
    exit_code = 1

    try:
        metrics_ready = _wait_until(node, lambda: _metrics_ready(node), node.timeout_s)
        result["checks"]["fault_metrics_terminal_available"] = metrics_ready
        result["checks"]["v25_11d_result_available"] = _wait_until(
            node, node.v25_11d_result_path.is_file, 5.0
        )
        _spin_for(node, node.settle_s)

        metrics_doc = _read_json(node.metrics_path) if node.metrics_path.is_file() else {}
        summary = _read_json(node.summary_path) if node.summary_path.is_file() else {}
        v25_11d = (
            _read_json(node.v25_11d_result_path)
            if node.v25_11d_result_path.is_file()
            else {}
        )

        checks = metrics_doc.get("checks", {}) if isinstance(metrics_doc, dict) else {}
        metrics = metrics_doc.get("metrics", {}) if isinstance(metrics_doc, dict) else {}
        route = metrics_doc.get("route_state", {}) if isinstance(metrics_doc, dict) else {}

        fault_ns = metrics.get("fault_fired_ros_ns")
        first_motion_ns = metrics.get("first_nonzero_cmd_ros_ns")
        zero_ns = metrics.get("first_zero_cmd_after_fault_ros_ns")
        terminal_ns = metrics.get("route_terminal_ros_ns")
        fault_to_zero_ms = metrics.get("fault_to_safety_zero_ms")
        fault_to_terminal_ms = metrics.get("fault_to_route_terminal_ms")
        post_fault_distance_m = metrics.get("max_post_fault_distance_m")

        # Populate the report metrics before reading the live timeline. Even if
        # auxiliary observability evidence is malformed, the primary fault timing
        # measurements remain visible instead of collapsing to an all-null row.
        result["metrics"] = {
            "fault_to_safety_zero_ms": fault_to_zero_ms,
            "fault_to_route_terminal_ms": fault_to_terminal_ms,
            "safety_zero_to_route_terminal_ms": metrics.get(
                "safety_zero_to_route_terminal_ms"
            ),
            "fault_cmd_norm": metrics.get("fault_cmd_norm"),
            "fault_ground_truth_speed_mps": metrics.get(
                "fault_ground_truth_speed_mps"
            ),
            "max_post_fault_cmd_norm": metrics.get("max_post_fault_cmd_norm"),
            "max_post_fault_ground_truth_speed_mps": metrics.get(
                "max_post_fault_ground_truth_speed_mps"
            ),
            "max_post_fault_distance_m": post_fault_distance_m,
            "route_completion_ratio": metrics.get("route_completion_ratio"),
        }
        result["sources"] = {
            "fault_metrics": metrics_doc,
            "v25_11d_result": v25_11d,
        }

        timeline_events, skipped_timeline_tail_records = _read_live_timeline_events(
            node.timeline_path
        )
        result["metrics"]["skipped_timeline_tail_records"] = (
            skipped_timeline_tail_records
        )

        result["checks"]["v25_11d_gate_passed"] = bool(
            v25_11d.get("gate") == "V25-11D"
            and v25_11d.get("fault_case") == node.fault_case
            and v25_11d.get("status") == "PASS"
        )
        result["checks"]["fault_metrics_schema_valid"] = bool(
            metrics_doc.get("schema") == "agt_v25_11e_fault_metrics/v1"
            and metrics_doc.get("fault_case") == node.fault_case
        )
        result["checks"]["pre_fault_motion_observed"] = bool(
            checks.get("pre_fault_motion_seen")
        )
        result["checks"]["fault_injection_observed"] = bool(
            checks.get("fault_injection_seen")
        )
        result["checks"]["safety_zero_observed"] = bool(
            checks.get("safety_zero_after_fault_seen")
        )
        result["checks"]["route_terminal_after_fault"] = bool(
            checks.get("route_terminal_after_fault_seen")
            and isinstance(route, dict)
            and route.get("state") == "FAILED"
        )
        # Safety zeroing and route cancellation are independent consumers of the
        # same fault. Require both to happen after the injected fault, but do not
        # impose an artificial ordering between them.
        result["checks"]["timing_order_valid"] = bool(
            isinstance(first_motion_ns, int)
            and isinstance(fault_ns, int)
            and isinstance(zero_ns, int)
            and isinstance(terminal_ns, int)
            and first_motion_ns < fault_ns <= zero_ns
            and fault_ns <= terminal_ns
        )
        result["checks"]["response_metrics_available"] = all(
            isinstance(value, (int, float))
            for value in (
                fault_to_zero_ms,
                fault_to_terminal_ms,
                post_fault_distance_m,
            )
        )
        result["checks"]["fault_to_zero_bounded"] = bool(
            isinstance(fault_to_zero_ms, (int, float))
            and 0.0 <= float(fault_to_zero_ms) <= 2000.0
        )
        result["checks"]["post_fault_motion_bounded"] = bool(
            isinstance(post_fault_distance_m, (int, float))
            and 0.0 <= float(post_fault_distance_m) <= 2.50
        )
        result["checks"]["observability_runtime_healthy"] = bool(
            summary.get("schema") == "agt_v25_11e_summary/v1"
            and not summary.get("fatal_error")
            and "observer_fatal" not in timeline_events
        )
        result["checks"]["observability_core_events_present"] = {
            "observer_started",
            "localization",
            "route_state",
            "sensor_health",
            "safety",
            "safety_cmd_motion",
        }.issubset(timeline_events)

        passed = all(bool(value) for value in result["checks"].values())
        result["status"] = "PASS" if passed else "FAIL"
        exit_code = 0 if passed else 1
    except Exception as error:  # noqa: BLE001
        result["error"] = f"{type(error).__name__}: {error}"
    finally:
        _atomic_write_json(node.report_path, result)
        print(json.dumps(result, indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
