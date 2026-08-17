#!/usr/bin/env python3
"""Read-only V25-12G-A3.6 intermediate-curvature acceptance harness.

The harness re-derives A3 motion evidence in memory with the approved five-level
R6B curvature family, compares it with the frozen A3.5 baseline, and reports
only experiment evidence.  It never writes runtime/map assets and it does not
make route-ready, optimal, field-ready, global-connectivity, or global-
infeasibility claims.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import resource
import time
from collections import Counter
from pathlib import Path
from typing import Any, Iterable, Mapping

from agt_offline_assets.navigation_grid import load_navigation_grid
from agt_offline_assets.site_boundary import load_site_boundary
from agt_offline_assets.turn_zones import load_turn_zones
from agt_offline_assets.vehicle_profile import load_canonical_vehicle_profile
from agt_offline_assets.vehicle_feasible_motion_graph import (
    BOUNDED_REVERSE_PRIMITIVE_SEARCH,
    EXECUTABLE,
    REJECTED,
    UNRESOLVED,
    derive_vehicle_feasible_motion_graph,
)
from agt_offline_assets.vehicle_feasible_motion_graph_io import (
    load_vehicle_feasible_motion_graph,
    vehicle_feasible_motion_graph_to_dict,
)
from agt_offline_assets.vehicle_feasible_service_graph_io import (
    load_vehicle_feasible_service_graph,
)


REPORT_SCHEMA = "agt_v25_12g_a36_intermediate_curvature_acceptance/v1"
VALIDATION_SCOPE = (
    "A36_INTERMEDIATE_CURVATURE_EXPERIMENT_NOT_ROUTE_READY_NOT_GLOBAL_INFEASIBILITY_PROOF"
)
BASELINE_MOTION_GRAPH_ASSET = "vehicle_feasible_motion_graph.yaml"
EXPECTED_CONNECTOR_COUNT = 40
EXPECTED_SERVICE_ACTION_COUNT = 32

FORBIDDEN_SEMANTIC_KEYS = frozenset(
    {"route_ready", "reachable_from_start", "optimal"}
)
EXPANSION_HISTOGRAM_BINS = (
    ("0-9", 0, 9),
    ("10-99", 10, 99),
    ("100-499", 100, 499),
    ("500-999", 500, 999),
    ("1000-4999", 1000, 4999),
    ("5000-9999", 5000, 9999),
    ("10000-19999", 10000, 19999),
    ("20000-30000", 20000, 30000),
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the read-only V25-12G-A3.6 intermediate-curvature experiment"
    )
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--vehicle-profile", required=True)
    parser.add_argument(
        "--baseline-motion-graph",
        default=BASELINE_MOTION_GRAPH_ASSET,
    )
    parser.add_argument("--service-graph", default="vehicle_feasible_service_graph.yaml")
    parser.add_argument("--segments", default="vehicle_feasible_segments.yaml")
    parser.add_argument("--turn-zones", default="turn_zones.yaml")
    parser.add_argument("--navigation-map", default="navigation_map.yaml")
    parser.add_argument("--navigation-pgm", default="navigation_map.pgm")
    parser.add_argument("--site-boundary", default="site_boundary.yaml")
    parser.add_argument("--derivation", default="derivation.yaml")
    parser.add_argument("--aisle-graph", default="aisle_graph.yaml")
    parser.add_argument("--pretty", action="store_true")
    return parser


def _require_file(path: Path) -> Path:
    if not path.is_file():
        raise FileNotFoundError(str(path))
    return path


def _resolve_run_asset(run_dir: Path, value: str) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (run_dir / path).resolve()


def _preflight(args: argparse.Namespace) -> dict[str, Path]:
    run_dir = Path(args.run_dir).expanduser().resolve()
    if not run_dir.is_dir():
        raise FileNotFoundError(str(run_dir))
    paths = {
        "baseline_motion_graph": _resolve_run_asset(
            run_dir, str(args.baseline_motion_graph)
        ),
        "service_graph": _resolve_run_asset(run_dir, str(args.service_graph)),
        "segments": _resolve_run_asset(run_dir, str(args.segments)),
        "turn_zones": _resolve_run_asset(run_dir, str(args.turn_zones)),
        "navigation_map": _resolve_run_asset(run_dir, str(args.navigation_map)),
        "navigation_pgm": _resolve_run_asset(run_dir, str(args.navigation_pgm)),
        "site_boundary": _resolve_run_asset(run_dir, str(args.site_boundary)),
        "derivation": _resolve_run_asset(run_dir, str(args.derivation)),
        "aisle_graph": _resolve_run_asset(run_dir, str(args.aisle_graph)),
        "vehicle_profile": Path(args.vehicle_profile).expanduser().resolve(),
    }
    for path in paths.values():
        _require_file(path)
    paths["run_dir"] = run_dir
    return paths


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _input_hashes(paths: Mapping[str, Path]) -> dict[str, str]:
    return {
        name: _sha256(path)
        for name, path in sorted(paths.items())
        if name != "run_dir"
    }


def assert_no_forbidden_semantic_keys(value: Any, path: str = "root") -> None:
    if isinstance(value, Mapping):
        for key, child in value.items():
            name = str(key)
            if name in FORBIDDEN_SEMANTIC_KEYS:
                raise ValueError(f"forbidden A3.6 semantic key {path}.{name}")
            assert_no_forbidden_semantic_keys(child, f"{path}.{name}")
    elif isinstance(value, (list, tuple)):
        for index, child in enumerate(value):
            assert_no_forbidden_semantic_keys(child, f"{path}[{index}]")


def service_behavior_projection(payload: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "service_actions": copy.deepcopy(payload.get("service_actions", [])),
        "executable_service_action_ids": list(
            payload.get("executable_service_action_ids", [])
        ),
    }


def assert_service_behavior_equal(
    baseline: Mapping[str, Any],
    current: Mapping[str, Any],
) -> None:
    if service_behavior_projection(baseline) != service_behavior_projection(current):
        raise ValueError("A3.6 service behavior drift")


def _transition_by_id(payload: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    output: dict[str, dict[str, Any]] = {}
    for raw in payload.get("transition_validations", []):
        item = dict(raw)
        connector_id = str(item.get("connector_candidate_id", ""))
        if not connector_id or connector_id in output:
            raise ValueError(f"duplicate or empty connector id: {connector_id}")
        output[connector_id] = item
    return output


def compare_transition_outcomes(
    baseline: Mapping[str, Any],
    current: Mapping[str, Any],
) -> dict[str, Any]:
    before = _transition_by_id(baseline)
    after = _transition_by_id(current)
    if set(before) != set(after):
        raise ValueError("A3.6 connector universe drift")

    newly_executable: list[str] = []
    still_rejected: list[str] = []
    regressed: list[str] = []
    for connector_id in sorted(before):
        old_status = str(before[connector_id].get("status", ""))
        new_status = str(after[connector_id].get("status", ""))
        if old_status != EXECUTABLE and new_status == EXECUTABLE:
            newly_executable.append(connector_id)
        if old_status == REJECTED and new_status == REJECTED:
            still_rejected.append(connector_id)
        if old_status == EXECUTABLE and new_status != EXECUTABLE:
            regressed.append(connector_id)
        if old_status == REJECTED and new_status == UNRESOLVED:
            regressed.append(connector_id)

    return {
        "newly_executable_connector_ids": newly_executable,
        "still_rejected_connector_ids": still_rejected,
        "unexpectedly_regressed_connector_ids": sorted(set(regressed)),
    }


def find_a4_minimum_chain(payload: Mapping[str, Any]) -> dict[str, Any]:
    services = {
        str(item["service_state_id"]): dict(item)
        for item in payload.get("service_actions", [])
    }
    transitions = sorted(
        (
            dict(item)
            for item in payload.get("transition_validations", [])
            if item.get("status") == EXECUTABLE
        ),
        key=lambda item: str(item.get("connector_candidate_id", "")),
    )

    for first in transitions:
        for second in transitions:
            if first is second:
                continue
            if first.get("to_service_state_id") != second.get("from_service_state_id"):
                continue
            state_ids = [
                str(first.get("from_service_state_id", "")),
                str(first.get("to_service_state_id", "")),
                str(second.get("to_service_state_id", "")),
            ]
            if any(state_id not in services for state_id in state_ids):
                continue
            segment_ids = [
                str(first.get("from_segment_id", "")),
                str(first.get("to_segment_id", "")),
                str(second.get("to_segment_id", "")),
            ]
            aisle_ids = [
                str(services[state_id].get("aisle_id", "")) for state_id in state_ids
            ]
            if len(set(segment_ids)) < 3 or len(set(aisle_ids)) < 2:
                continue
            return {
                "a4_entry_gate_passed": True,
                "connector_candidate_ids": [
                    str(first["connector_candidate_id"]),
                    str(second["connector_candidate_id"]),
                ],
                "service_state_ids": state_ids,
                "segment_ids": segment_ids,
                "aisle_ids": aisle_ids,
            }

    return {
        "a4_entry_gate_passed": False,
        "connector_candidate_ids": [],
        "service_state_ids": [],
        "segment_ids": [],
        "aisle_ids": [],
    }


def build_expansion_histogram(values: Iterable[int]) -> dict[str, int]:
    histogram = {label: 0 for label, _low, _high in EXPANSION_HISTOGRAM_BINS}
    for raw in values:
        value = int(raw)
        matched = False
        for label, low, high in EXPANSION_HISTOGRAM_BINS:
            if low <= value <= high:
                histogram[label] += 1
                matched = True
                break
        if not matched:
            raise ValueError(f"search expansion count outside frozen histogram: {value}")
    return histogram


def _status_counts(payload: Mapping[str, Any]) -> dict[str, int]:
    counts = Counter(
        str(item.get("status", ""))
        for item in payload.get("transition_validations", [])
    )
    return dict(sorted(counts.items()))


def _r6b_records(payload: Mapping[str, Any]) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for raw in payload.get("transition_validations", []):
        if raw.get("backend") != BOUNDED_REVERSE_PRIMITIVE_SEARCH:
            continue
        diagnostics = dict(raw.get("reverse_backend_diagnostics", {}))
        records.append(
            {
                "connector_candidate_id": str(
                    raw.get("connector_candidate_id", "")
                ),
                "status": str(raw.get("status", "")),
                "backend_status": str(raw.get("backend_status", "")),
                "search_expansions": int(raw.get("search_expansions", 0)),
                "queue_exhausted": bool(diagnostics.get("queue_exhausted", False)),
                "expansion_budget_reached": bool(
                    diagnostics.get("expansion_budget_reached", False)
                ),
                "failure_class": diagnostics.get("failure_class"),
            }
        )
    return sorted(records, key=lambda item: item["connector_candidate_id"])


def _resource_metrics(
    wall_start: float,
    usage_start: resource.struct_rusage,
) -> dict[str, Any]:
    wall_seconds = max(time.perf_counter() - wall_start, 0.0)
    usage_end = resource.getrusage(resource.RUSAGE_SELF)
    user_seconds = max(float(usage_end.ru_utime - usage_start.ru_utime), 0.0)
    system_seconds = max(float(usage_end.ru_stime - usage_start.ru_stime), 0.0)
    cpu_percent = (
        0.0
        if wall_seconds <= 1.0e-12
        else 100.0 * (user_seconds + system_seconds) / wall_seconds
    )
    return {
        "wall_seconds": wall_seconds,
        "user_seconds": user_seconds,
        "system_seconds": system_seconds,
        "cpu_percent_approx": cpu_percent,
        "max_rss_kb": int(usage_end.ru_maxrss),
    }


def main(argv: list[str] | None = None) -> int:
    wall_start = time.perf_counter()
    usage_start = resource.getrusage(resource.RUSAGE_SELF)

    args = build_parser().parse_args(argv)
    paths = _preflight(args)
    hashes_before = _input_hashes(paths)

    baseline_graph = load_vehicle_feasible_motion_graph(paths["baseline_motion_graph"])
    baseline_payload = vehicle_feasible_motion_graph_to_dict(baseline_graph)
    assert_no_forbidden_semantic_keys(baseline_payload)

    if len(baseline_payload.get("service_actions", [])) != EXPECTED_SERVICE_ACTION_COUNT:
        raise ValueError(
            "A3.6 baseline requires exactly "
            f"{EXPECTED_SERVICE_ACTION_COUNT} service actions"
        )
    if len(baseline_payload.get("transition_validations", [])) != EXPECTED_CONNECTOR_COUNT:
        raise ValueError(
            "A3.6 baseline requires exactly "
            f"{EXPECTED_CONNECTOR_COUNT} connector transitions"
        )

    service_graph = load_vehicle_feasible_service_graph(paths["service_graph"])
    turn_zones = load_turn_zones(paths["turn_zones"])
    navigation = load_navigation_grid(paths["navigation_map"])
    boundary = load_site_boundary(paths["site_boundary"])
    vehicle = load_canonical_vehicle_profile(paths["vehicle_profile"])

    current_graph = derive_vehicle_feasible_motion_graph(
        service_graph,
        turn_zones,
        navigation,
        vehicle,
        site_boundary=boundary,
        source={
            "acceptance_stage": "v25_12g_a36_intermediate_curvature",
            "experiment_only": True,
        },
    )
    current_payload = vehicle_feasible_motion_graph_to_dict(current_graph)
    assert_no_forbidden_semantic_keys(current_payload)

    if len(current_payload.get("service_actions", [])) != EXPECTED_SERVICE_ACTION_COUNT:
        raise ValueError(
            "A3.6 amended derivation requires exactly "
            f"{EXPECTED_SERVICE_ACTION_COUNT} service actions"
        )
    if len(current_payload.get("transition_validations", [])) != EXPECTED_CONNECTOR_COUNT:
        raise ValueError(
            "A3.6 amended derivation requires exactly "
            f"{EXPECTED_CONNECTOR_COUNT} connector transitions"
        )

    assert_service_behavior_equal(baseline_payload, current_payload)
    outcomes = compare_transition_outcomes(baseline_payload, current_payload)
    if outcomes["unexpectedly_regressed_connector_ids"]:
        raise ValueError(
            "A3.6 transition regression: "
            + ", ".join(outcomes["unexpectedly_regressed_connector_ids"])
        )

    baseline_ids = set(_transition_by_id(baseline_payload))
    current_ids = set(_transition_by_id(current_payload))
    if baseline_ids != current_ids or len(current_ids) != EXPECTED_CONNECTOR_COUNT:
        raise ValueError("A3.6 connector universe drift")

    r6b_records = _r6b_records(current_payload)
    if len(r6b_records) != EXPECTED_CONNECTOR_COUNT:
        raise ValueError(
            "A3.6 requires all 40 connector transitions to remain in the R6B backend"
        )

    hashes_after = _input_hashes(paths)
    if hashes_before != hashes_after:
        raise ValueError("A3.6 acceptance harness modified a protected input asset")

    a4_chain = find_a4_minimum_chain(current_payload)
    summary = {
        "service_behavior_equal": True,
        "connector_universe_preserved": True,
        "protected_input_hashes_unchanged": True,
        "forbidden_semantic_key_count": 0,
        "baseline_transition_status_counts": _status_counts(baseline_payload),
        "amended_transition_status_counts": _status_counts(current_payload),
        "newly_executable_connector_ids": outcomes[
            "newly_executable_connector_ids"
        ],
        "still_rejected_connector_ids": outcomes["still_rejected_connector_ids"],
        "unexpectedly_regressed_connector_ids": [],
        "queue_exhausted_count": sum(
            bool(item["queue_exhausted"]) for item in r6b_records
        ),
        "expansion_budget_reached_count": sum(
            bool(item["expansion_budget_reached"]) for item in r6b_records
        ),
        "expansion_histogram": build_expansion_histogram(
            int(item["search_expansions"]) for item in r6b_records
        ),
        "a4_minimum_chain": a4_chain,
    }
    summary.update(_resource_metrics(wall_start, usage_start))

    report = {
        "schema": REPORT_SCHEMA,
        "validation_scope": VALIDATION_SCOPE,
        "run_dir": str(paths["run_dir"]),
        "platform_id": current_graph.platform_id,
        "platform_profile_sha256": current_graph.platform_profile_sha256,
        "baseline_motion_graph_asset": str(args.baseline_motion_graph),
        "protected_input_hashes_before": hashes_before,
        "protected_input_hashes_after": hashes_after,
        "r6b_connectors": r6b_records,
        "summary": summary,
    }
    assert_no_forbidden_semantic_keys(report)

    if args.pretty:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(json.dumps(report, separators=(",", ":"), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
