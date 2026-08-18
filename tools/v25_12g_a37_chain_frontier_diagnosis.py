#!/usr/bin/env python3
"""Read-only V25-12G A3.7 A4 chain-frontier diagnosis.

This tool does not run any planner. It overlays the saved A3.6 connector
outcomes onto the frozen A3.5/A3 motion topology and identifies rejected
connectors that would complete the frozen two-transition A4 minimum chain if
exactly one such connector were hypothetically recovered.

A3.6 outcome/termination evidence and A3.5 historical detailed diagnostics are
kept in separate namespaces so their provenance cannot be confused.
"""

from __future__ import annotations

import argparse
import copy
import json
from collections import Counter
from pathlib import Path
from typing import Any, Mapping

from agt_offline_assets.vehicle_feasible_motion_graph_io import (
    load_vehicle_feasible_motion_graph,
    vehicle_feasible_motion_graph_to_dict,
)


REPORT_SCHEMA = "agt_v25_12g_a37_a4_chain_frontier_diagnosis/v1"
VALIDATION_SCOPE = (
    "A37_A4_CHAIN_FRONTIER_DIAGNOSTIC_ONLY_NOT_PLANNER_AMENDMENT_NOT_ROUTE_READY"
)
A36_REPORT_SCHEMA = "agt_v25_12g_a36_intermediate_curvature_acceptance/v1"
EXPECTED_CONNECTOR_COUNT = 40
BASELINE_MOTION_GRAPH_ASSET = "vehicle_feasible_motion_graph.yaml"

FORBIDDEN_SEMANTIC_KEYS = frozenset(
    {"route_ready", "reachable_from_start", "optimal"}
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Diagnose the A3.7 one-edge A4 chain frontier from frozen evidence"
    )
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--a36-report", required=True)
    parser.add_argument(
        "--baseline-motion-graph",
        default=BASELINE_MOTION_GRAPH_ASSET,
    )
    parser.add_argument("--pretty", action="store_true")
    return parser


def assert_no_forbidden_semantic_keys(value: Any, path: str = "root") -> None:
    if isinstance(value, Mapping):
        for key, child in value.items():
            name = str(key)
            if name in FORBIDDEN_SEMANTIC_KEYS:
                raise ValueError(f"forbidden A3.7 semantic key {path}.{name}")
            assert_no_forbidden_semantic_keys(child, f"{path}.{name}")
    elif isinstance(value, (list, tuple)):
        for index, child in enumerate(value):
            assert_no_forbidden_semantic_keys(child, f"{path}[{index}]")


def _transition_by_id(payload: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    output: dict[str, dict[str, Any]] = {}
    for raw in payload.get("transition_validations", []):
        item = dict(raw)
        connector_id = str(item.get("connector_candidate_id", ""))
        if not connector_id or connector_id in output:
            raise ValueError(f"duplicate or empty connector id: {connector_id}")
        output[connector_id] = item
    return output


def _a36_by_id(report: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    output: dict[str, dict[str, Any]] = {}
    for raw in report.get("r6b_connectors", []):
        item = dict(raw)
        connector_id = str(item.get("connector_candidate_id", ""))
        if not connector_id or connector_id in output:
            raise ValueError(f"duplicate or empty A3.6 connector id: {connector_id}")
        output[connector_id] = item
    return output


def _validate_a36_integrity(report: Mapping[str, Any]) -> None:
    if str(report.get("schema", "")) != A36_REPORT_SCHEMA:
        raise ValueError("A3.7 requires the frozen A3.6 report schema")
    summary = report.get("summary", {})
    if not isinstance(summary, Mapping):
        raise ValueError("A3.6 summary must be a mapping")
    if not bool(summary.get("service_behavior_equal", False)):
        raise ValueError("A3.6 service behavior integrity failed")
    if not bool(summary.get("connector_universe_preserved", False)):
        raise ValueError("A3.6 connector universe integrity failed")
    if not bool(summary.get("protected_input_hashes_unchanged", False)):
        raise ValueError("A3.6 protected input hash integrity failed")
    if int(summary.get("forbidden_semantic_key_count", -1)) != 0:
        raise ValueError("A3.6 forbidden semantic integrity failed")
    if list(summary.get("unexpectedly_regressed_connector_ids", [])):
        raise ValueError("A3.6 contains unexpected connector regressions")


def _a36_evidence(item: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "status": str(item.get("status", "")),
        "backend_status": str(item.get("backend_status", "")),
        "search_expansions": int(item.get("search_expansions", 0)),
        "queue_exhausted": bool(item.get("queue_exhausted", False)),
        "expansion_budget_reached": bool(
            item.get("expansion_budget_reached", False)
        ),
        "failure_class": item.get("failure_class"),
    }


def build_transition_view(
    baseline_payload: Mapping[str, Any],
    a36_report: Mapping[str, Any],
) -> dict[str, dict[str, Any]]:
    """Overlay saved A3.6 outcomes on immutable A3.5 topology/evidence."""
    assert_no_forbidden_semantic_keys(baseline_payload)
    assert_no_forbidden_semantic_keys(a36_report)
    _validate_a36_integrity(a36_report)

    baseline = _transition_by_id(baseline_payload)
    amended = _a36_by_id(a36_report)
    if set(baseline) != set(amended):
        raise ValueError("A3.7 connector universe mismatch between A3.5 and A3.6")

    view: dict[str, dict[str, Any]] = {}
    for connector_id in sorted(baseline):
        before = baseline[connector_id]
        view[connector_id] = {
            "connector_candidate_id": connector_id,
            "from_service_state_id": str(before.get("from_service_state_id", "")),
            "to_service_state_id": str(before.get("to_service_state_id", "")),
            "from_segment_id": str(before.get("from_segment_id", "")),
            "to_segment_id": str(before.get("to_segment_id", "")),
            "side": str(before.get("side", "")),
            "turn_zone_id": str(before.get("turn_zone_id", "")),
            "a36_evidence": _a36_evidence(amended[connector_id]),
            "a35_reference": copy.deepcopy(before),
        }
    return view


def _service_by_id(payload: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    output: dict[str, dict[str, Any]] = {}
    for raw in payload.get("service_actions", []):
        item = dict(raw)
        state_id = str(item.get("service_state_id", ""))
        if not state_id or state_id in output:
            raise ValueError(f"duplicate or empty service state id: {state_id}")
        output[state_id] = item
    return output


def _witness(
    first: Mapping[str, Any],
    second: Mapping[str, Any],
    services: Mapping[str, Mapping[str, Any]],
    *,
    position: str,
) -> dict[str, Any] | None:
    if first.get("to_service_state_id") != second.get("from_service_state_id"):
        return None

    state_ids = [
        str(first.get("from_service_state_id", "")),
        str(first.get("to_service_state_id", "")),
        str(second.get("to_service_state_id", "")),
    ]
    if any(state_id not in services for state_id in state_ids):
        return None

    segment_ids = [
        str(first.get("from_segment_id", "")),
        str(first.get("to_segment_id", "")),
        str(second.get("to_segment_id", "")),
    ]
    if len(set(segment_ids)) != 3:
        return None

    aisle_ids = [str(services[state_id].get("aisle_id", "")) for state_id in state_ids]
    if len(set(aisle_ids)) < 2:
        return None

    return {
        "position": position,
        "connector_candidate_ids": [
            str(first.get("connector_candidate_id", "")),
            str(second.get("connector_candidate_id", "")),
        ],
        "service_state_ids": state_ids,
        "segment_ids": segment_ids,
        "aisle_ids": aisle_ids,
    }


def find_a4_frontier_candidates(
    baseline_payload: Mapping[str, Any],
    a36_report: Mapping[str, Any],
) -> list[dict[str, Any]]:
    """Return rejected connectors that are one hypothetical edge from A4 entry."""
    view = build_transition_view(baseline_payload, a36_report)
    services = _service_by_id(baseline_payload)
    executable = [
        item for item in view.values() if item["a36_evidence"]["status"] == "EXECUTABLE"
    ]
    rejected = [
        item for item in view.values() if item["a36_evidence"]["status"] == "REJECTED"
    ]

    candidates: dict[str, dict[str, Any]] = {}
    for candidate in sorted(rejected, key=lambda item: item["connector_candidate_id"]):
        witnesses: list[dict[str, Any]] = []
        for current in sorted(
            executable, key=lambda item: item["connector_candidate_id"]
        ):
            predecessor = _witness(
                candidate,
                current,
                services,
                position="PREDECESSOR_TO_EXECUTABLE",
            )
            if predecessor is not None:
                witnesses.append(predecessor)

            successor = _witness(
                current,
                candidate,
                services,
                position="SUCCESSOR_FROM_EXECUTABLE",
            )
            if successor is not None:
                witnesses.append(successor)

        if not witnesses:
            continue
        witnesses.sort(
            key=lambda item: (
                item["position"],
                tuple(item["connector_candidate_ids"]),
                tuple(item["service_state_ids"]),
            )
        )
        connector_id = str(candidate["connector_candidate_id"])
        candidates[connector_id] = {
            "candidate_connector_id": connector_id,
            "from_service_state_id": candidate["from_service_state_id"],
            "to_service_state_id": candidate["to_service_state_id"],
            "from_segment_id": candidate["from_segment_id"],
            "to_segment_id": candidate["to_segment_id"],
            "side": candidate["side"],
            "turn_zone_id": candidate["turn_zone_id"],
            "a36_evidence": copy.deepcopy(candidate["a36_evidence"]),
            "a35_reference": copy.deepcopy(candidate["a35_reference"]),
            "hypothetical_witnesses": witnesses,
        }

    return [candidates[key] for key in sorted(candidates)]


def build_frontier_report(
    baseline_payload: Mapping[str, Any],
    a36_report: Mapping[str, Any],
) -> dict[str, Any]:
    view = build_transition_view(baseline_payload, a36_report)
    candidates = find_a4_frontier_candidates(baseline_payload, a36_report)
    executable_ids = sorted(
        connector_id
        for connector_id, item in view.items()
        if item["a36_evidence"]["status"] == "EXECUTABLE"
    )
    rejected_ids = sorted(
        connector_id
        for connector_id, item in view.items()
        if item["a36_evidence"]["status"] == "REJECTED"
    )
    candidate_ids = [item["candidate_connector_id"] for item in candidates]
    failure_counts = Counter(
        str(item["a36_evidence"].get("failure_class")) for item in candidates
    )
    a4 = a36_report.get("summary", {}).get("a4_minimum_chain", {})

    report = {
        "schema": REPORT_SCHEMA,
        "validation_scope": VALIDATION_SCOPE,
        "evidence_provenance": {
            "a36_evidence": (
                "saved A3.6 current outcome/termination fields only; no planner rerun"
            ),
            "a35_reference": (
                "frozen pre-amendment A3.5/A3 topology and detailed historical diagnostics"
            ),
        },
        "summary": {
            "a36_executable_connector_ids": executable_ids,
            "a36_rejected_connector_count": len(rejected_ids),
            "frontier_candidate_ids": candidate_ids,
            "frontier_candidate_count": len(candidate_ids),
            "frontier_queue_exhausted_count": sum(
                bool(item["a36_evidence"]["queue_exhausted"])
                for item in candidates
            ),
            "frontier_expansion_budget_reached_count": sum(
                bool(item["a36_evidence"]["expansion_budget_reached"])
                for item in candidates
            ),
            "frontier_failure_class_counts": dict(sorted(failure_counts.items())),
            "current_a4_entry_gate_passed": bool(
                isinstance(a4, Mapping) and a4.get("a4_entry_gate_passed", False)
            ),
        },
        "frontier_candidates": candidates,
    }
    assert_no_forbidden_semantic_keys(report)
    return report


def _require_file(path: Path) -> Path:
    if not path.is_file():
        raise FileNotFoundError(str(path))
    return path


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    run_dir = Path(args.run_dir).expanduser().resolve()
    if not run_dir.is_dir():
        raise FileNotFoundError(str(run_dir))

    baseline_path = Path(args.baseline_motion_graph).expanduser()
    if not baseline_path.is_absolute():
        baseline_path = run_dir / baseline_path
    baseline_path = _require_file(baseline_path.resolve())
    a36_path = _require_file(Path(args.a36_report).expanduser().resolve())

    baseline_graph = load_vehicle_feasible_motion_graph(baseline_path)
    baseline_payload = vehicle_feasible_motion_graph_to_dict(baseline_graph)
    a36_report = json.loads(a36_path.read_text(encoding="utf-8"))

    if len(baseline_payload.get("transition_validations", [])) != EXPECTED_CONNECTOR_COUNT:
        raise ValueError(
            f"A3.7 real baseline requires {EXPECTED_CONNECTOR_COUNT} connectors"
        )
    if len(a36_report.get("r6b_connectors", [])) != EXPECTED_CONNECTOR_COUNT:
        raise ValueError(
            f"A3.7 real A3.6 report requires {EXPECTED_CONNECTOR_COUNT} connectors"
        )

    report = build_frontier_report(baseline_payload, a36_report)
    report["run_dir"] = str(run_dir)
    report["baseline_motion_graph"] = str(baseline_path)
    report["a36_report"] = str(a36_path)
    assert_no_forbidden_semantic_keys(report)

    if args.pretty:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(json.dumps(report, separators=(",", ":"), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
