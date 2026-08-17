#!/usr/bin/env python3
"""Read-only V25-12G-A3.5 diagnosis of all R6B connector failures.

The harness re-derives A3 motion evidence in memory, requires exact equality of
all pre-A3.5 behavior fields, and reports only diagnostic evidence.  It does not
write runtime assets and it does not make route-ready or global-infeasibility
claims.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping

from agt_offline_assets.navigation_grid import load_navigation_grid
from agt_offline_assets.site_boundary import load_site_boundary
from agt_offline_assets.turn_zones import load_turn_zones
from agt_offline_assets.vehicle_profile import load_canonical_vehicle_profile
from agt_offline_assets.vehicle_feasible_motion_graph import (
    BOUNDED_REVERSE_PRIMITIVE_SEARCH,
    REJECTED,
    derive_vehicle_feasible_motion_graph,
)
from agt_offline_assets.vehicle_feasible_motion_graph_io import (
    load_vehicle_feasible_motion_graph,
    vehicle_feasible_motion_graph_to_dict,
)
from agt_offline_assets.vehicle_feasible_service_graph_io import (
    load_vehicle_feasible_service_graph,
)
from agt_offline_assets.reverse_primitive_diagnostics import (
    FAILURE_CLASSES,
    R6B_DIAGNOSTIC_SCHEMA,
    reverse_primitive_search_diagnostics_from_dict,
)


REPORT_SCHEMA = "agt_v25_12g_a35_connector_backend_diagnosis/v1"
VALIDATION_SCOPE = (
    "A35_R6B_DIAGNOSTIC_ONLY_NOT_ROUTE_READY_NOT_GLOBAL_INFEASIBILITY_PROOF"
)
BASELINE_MOTION_GRAPH_ASSET = "vehicle_feasible_motion_graph.yaml"
EXPECTED_REAL_R6B_CONNECTOR_COUNT = 40

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
        description="Diagnose V25-12G-A3.5 bounded R6B connector failures"
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
                raise ValueError(f"forbidden A3.5 semantic key {path}.{name}")
            assert_no_forbidden_semantic_keys(child, f"{path}.{name}")
    elif isinstance(value, (list, tuple)):
        for index, child in enumerate(value):
            assert_no_forbidden_semantic_keys(child, f"{path}[{index}]")


def behavior_projection(payload: Mapping[str, Any]) -> dict[str, Any]:
    """Return every A3 behavior field while dropping only A3.5 diagnostics/source."""
    projected = copy.deepcopy(dict(payload))
    projected.pop("source", None)
    transitions = projected.get("transition_validations", ())
    if isinstance(transitions, list):
        for transition in transitions:
            if isinstance(transition, dict):
                transition.pop("reverse_backend_diagnostics", None)
    return projected


def assert_behavior_projection_equal(
    baseline: Mapping[str, Any],
    current: Mapping[str, Any],
) -> None:
    expected = behavior_projection(baseline)
    actual = behavior_projection(current)
    if expected != actual:
        raise ValueError(
            "A3.5 behavior projection drift: fields other than diagnostic/source changed"
        )


def sort_connector_records(records: Iterable[Mapping[str, Any]]) -> list[dict[str, Any]]:
    return sorted(
        (dict(record) for record in records),
        key=lambda item: str(item.get("connector_candidate_id", "")),
    )


def _finite_or_inf(value: Any) -> float:
    if value is None:
        return float("inf")
    result = float(value)
    return result if math.isfinite(result) else float("inf")


def rank_closest_connectors(
    records: Iterable[Mapping[str, Any]],
    *,
    limit: int = 10,
) -> list[dict[str, Any]]:
    ranked = sorted(
        (dict(record) for record in records),
        key=lambda item: (
            _finite_or_inf(item.get("best_goal_position_error_m")),
            _finite_or_inf(item.get("best_goal_yaw_error_rad")),
            str(item.get("connector_candidate_id", "")),
        ),
    )
    return ranked[: max(int(limit), 0)]


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


def _diagnostic_rejection_counts(diagnostics: Mapping[str, Any]) -> dict[str, int]:
    return {
        "SEARCH_ENVELOPE": int(
            diagnostics["primitive_edges_rejected_search_envelope"]
            + diagnostics["goal_shot_rejected_search_envelope"]
        ),
        "SITE_BOUNDARY": int(
            diagnostics["primitive_edges_rejected_site_boundary"]
            + diagnostics["goal_shot_rejected_site_boundary"]
        ),
        "NAVIGATION_GRID": int(
            diagnostics["primitive_edges_rejected_navigation_grid"]
            + diagnostics["goal_shot_rejected_navigation_grid"]
        ),
        "PATH_LENGTH": int(
            diagnostics["primitive_edges_rejected_path_length"]
            + diagnostics["goal_shot_rejected_path_length"]
        ),
        "STATE_DOMINANCE": int(
            diagnostics["primitive_edges_rejected_state_dominance"]
        ),
        "CUSP_STATE_DOMINANCE": int(
            diagnostics["cusp_switches_rejected_state_dominance"]
        ),
    }


def _increment_nested(
    target: dict[str, dict[str, int]],
    outer: str,
    inner: str,
) -> None:
    bucket = target.setdefault(str(outer), {})
    bucket[str(inner)] = int(bucket.get(str(inner), 0)) + 1


def _connector_records(motion_payload: Mapping[str, Any]) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    seen: set[str] = set()
    for transition in motion_payload.get("transition_validations", ()):
        if transition.get("backend") != BOUNDED_REVERSE_PRIMITIVE_SEARCH:
            continue
        connector_id = str(transition.get("connector_candidate_id", ""))
        if not connector_id or connector_id in seen:
            raise ValueError(f"duplicate or empty R6B connector id: {connector_id}")
        seen.add(connector_id)
        raw_diagnostics = transition.get("reverse_backend_diagnostics", {})
        diagnostics = reverse_primitive_search_diagnostics_from_dict(raw_diagnostics)
        if diagnostics.failure_class not in FAILURE_CLASSES:
            raise ValueError(
                f"R6B connector {connector_id} lacks a frozen failure classification"
            )
        diag = dict(raw_diagnostics)
        record = {
            "connector_candidate_id": connector_id,
            "from_segment_id": str(transition.get("from_segment_id", "")),
            "to_segment_id": str(transition.get("to_segment_id", "")),
            "side": str(transition.get("side", "")),
            "turn_zone_id": str(transition.get("turn_zone_id", "")),
            "forward_audit_status": str(
                transition.get("forward_evidence", {}).get("audit_status", "")
            ),
            "r6a_decision": str(
                transition.get("reverse_admission_evidence", {}).get("decision", "")
            ),
            "transition_status": str(transition.get("status", "")),
            "proof_scope": str(transition.get("proof_scope", "")),
            "backend_status": str(transition.get("backend_status", "")),
            "search_expansions": int(transition.get("search_expansions", 0)),
        }
        record.update(diag)
        records.append(record)
    return sort_connector_records(records)


def _summarize(records: list[dict[str, Any]]) -> dict[str, Any]:
    failure_class_counts: dict[str, int] = {}
    classification_by_side: dict[str, dict[str, int]] = {}
    classification_by_forward_audit: dict[str, dict[str, int]] = {}
    rejection_totals = {
        "SEARCH_ENVELOPE": 0,
        "SITE_BOUNDARY": 0,
        "NAVIGATION_GRID": 0,
        "PATH_LENGTH": 0,
        "STATE_DOMINANCE": 0,
        "CUSP_STATE_DOMINANCE": 0,
    }

    for record in records:
        failure_class = str(record["failure_class"])
        failure_class_counts[failure_class] = failure_class_counts.get(failure_class, 0) + 1
        _increment_nested(classification_by_side, str(record["side"]), failure_class)
        _increment_nested(
            classification_by_forward_audit,
            str(record["forward_audit_status"]),
            failure_class,
        )
        for name, value in _diagnostic_rejection_counts(record).items():
            rejection_totals[name] += value

    top_rejection_causes = [
        {"cause": name, "count": int(count)}
        for name, count in sorted(
            rejection_totals.items(), key=lambda item: (-item[1], item[0])
        )
    ]
    closest = rank_closest_connectors(records, limit=10)
    closest_fields = (
        "connector_candidate_id",
        "side",
        "forward_audit_status",
        "search_expansions",
        "failure_class",
        "best_goal_position_error_m",
        "best_goal_yaw_error_rad",
        "best_goal_distance_state_direction",
        "best_goal_distance_cusp_count",
    )
    return {
        "r6b_connector_count": len(records),
        "queue_exhausted_count": sum(bool(item["queue_exhausted"]) for item in records),
        "expansion_budget_reached_count": sum(
            bool(item["expansion_budget_reached"]) for item in records
        ),
        "failure_class_counts": dict(sorted(failure_class_counts.items())),
        "classification_by_side": {
            key: dict(sorted(value.items()))
            for key, value in sorted(classification_by_side.items())
        },
        "classification_by_forward_audit": {
            key: dict(sorted(value.items()))
            for key, value in sorted(classification_by_forward_audit.items())
        },
        "expansion_histogram": build_expansion_histogram(
            int(item["search_expansions"]) for item in records
        ),
        "top_rejection_causes": top_rejection_causes,
        "closest_connectors": [
            {field: item.get(field) for field in closest_fields}
            for item in closest
        ],
    }


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    paths = _preflight(args)
    hashes_before = _input_hashes(paths)

    baseline_graph = load_vehicle_feasible_motion_graph(paths["baseline_motion_graph"])
    baseline_payload = vehicle_feasible_motion_graph_to_dict(baseline_graph)
    assert_no_forbidden_semantic_keys(baseline_payload)

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
            "acceptance_stage": "v25_12g_a35_connector_backend_diagnosis",
            "diagnostic_only": True,
        },
    )
    current_payload = vehicle_feasible_motion_graph_to_dict(current_graph)
    assert_no_forbidden_semantic_keys(current_payload)
    assert_behavior_projection_equal(baseline_payload, current_payload)

    records = _connector_records(current_payload)
    if len(records) != EXPECTED_REAL_R6B_CONNECTOR_COUNT:
        raise ValueError(
            "A3.5 real-data contract requires exactly "
            f"{EXPECTED_REAL_R6B_CONNECTOR_COUNT} R6B connector records, got {len(records)}"
        )
    if any(item["transition_status"] != REJECTED for item in records):
        raise ValueError("A3.5 diagnostic run changed a final transition status")
    if any(
        item["backend_status"] != "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION"
        for item in records
    ):
        raise ValueError("A3.5 diagnostic run changed an R6B backend status")

    hashes_after = _input_hashes(paths)
    hashes_unchanged = hashes_before == hashes_after
    if not hashes_unchanged:
        raise ValueError("A3.5 diagnostic harness modified a protected input asset")

    summary = _summarize(records)
    summary.update(
        {
            "behavior_projection_equal": True,
            "final_transition_statuses_unchanged": True,
            "forbidden_semantic_key_count": 0,
            "protected_input_hashes_unchanged": True,
        }
    )
    report = {
        "schema": REPORT_SCHEMA,
        "validation_scope": VALIDATION_SCOPE,
        "run_dir": str(paths["run_dir"]),
        "platform_id": current_graph.platform_id,
        "platform_profile_sha256": current_graph.platform_profile_sha256,
        "baseline_motion_graph_asset": str(args.baseline_motion_graph),
        "protected_input_hashes_before": hashes_before,
        "protected_input_hashes_after": hashes_after,
        "connectors": records,
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
