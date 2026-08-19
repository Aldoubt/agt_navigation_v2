#!/usr/bin/env python3
"""V25-12G A3.9 successor terminal-closure Pareto diagnosis.

Diagnostic-only harness for exactly two A4-frontier connectors. It temporarily
observes the existing R6B forward goal-shot function, records bounded terminal
entry/candidate evidence, calls the original function unchanged for the actual
search decision, and then restores it. No planner parameter is exposed or
modified.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from contextlib import contextmanager
from dataclasses import replace
from pathlib import Path
from typing import Any, Iterable, Mapping

import agt_offline_assets.reverse_primitive_connector as r6b
from agt_offline_assets.navigation_grid import load_navigation_grid
from agt_offline_assets.site_boundary import load_site_boundary
from agt_offline_assets.turn_zones import load_turn_zones
from agt_offline_assets.vehicle_profile import load_canonical_vehicle_profile
from agt_offline_assets.vehicle_feasible_motion_graph import VehicleFeasibleMotionGraphConfig
from agt_offline_assets.vehicle_feasible_service_graph_io import load_vehicle_feasible_service_graph
from agt_offline_assets.vehicle_feasible_transition_motion import validate_transition_candidates
from agt_offline_assets.reverse_primitive_connector import ReversePrimitiveConnectorConfig

REPORT_SCHEMA = "agt_v25_12g_a39_terminal_closure_pareto_diagnosis/v1"
TERMINAL_DIAGNOSTIC_SCHEMA = "agt_r6b_terminal_closure_diagnostics/v1"
VALIDATION_SCOPE = (
    "A39_TERMINAL_CLOSURE_PARETO_DIAGNOSTIC_ONLY_"
    "NOT_PLANNER_AMENDMENT_NOT_ROUTE_READY"
)
A38_BASELINE_SCHEMA = "agt_v25_12g_a38_operator_terminal_baseline/v1"
A37_MANIFEST_SCHEMA = "agt_v25_12g_a37_operator_frontier_manifest/v1"

PRIMARY_CONNECTOR_ID = (
    "headland.turn_high_u.aisle_017.segment_005.dead_end_forward_in_reverse_out."
    "to.aisle_019.segment_003.dead_end_forward_in_reverse_out"
)
CONTROL_CONNECTOR_ID = (
    "headland.turn_high_u.aisle_017.segment_005.dead_end_forward_in_reverse_out."
    "to.aisle_020.segment_001.service_high_to_low"
)
EXPECTED_CONNECTOR_COUNT = 2
TARGETS = (("PRIMARY", PRIMARY_CONNECTOR_ID), ("CONTROL", CONTROL_CONNECTOR_ID))

REJECTION_REASONS = (
    "PATH_LENGTH",
    "SEARCH_ENVELOPE",
    "SITE_BOUNDARY",
    "NAVIGATION_GRID",
    "FREE",
)
FORBIDDEN_SEMANTIC_KEYS = frozenset(
    {
        "route_ready",
        "reachable_from_start",
        "optimal",
        "root_cause",
        "recommended_amendment",
        "hypothesis_confirmed",
    }
)
A36_OUTCOME_FIELDS = (
    "status",
    "backend_status",
    "search_expansions",
    "queue_exhausted",
    "expansion_budget_reached",
    "failure_class",
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Diagnose terminal-closure Pareto evidence for the A3.9 PRIMARY and "
            "CONTROL connectors without changing R6B behavior"
        )
    )
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--vehicle-profile", required=True)
    parser.add_argument("--a37-manifest", required=True)
    parser.add_argument("--a38-baseline-manifest", required=True)
    parser.add_argument("--service-graph", default="vehicle_feasible_service_graph.yaml")
    parser.add_argument("--turn-zones", default="turn_zones.yaml")
    parser.add_argument("--navigation-map", default="navigation_map.yaml")
    parser.add_argument("--navigation-pgm", default="navigation_map.pgm")
    parser.add_argument("--site-boundary", default="site_boundary.yaml")
    parser.add_argument("--pretty", action="store_true")
    return parser


def assert_no_forbidden_semantic_keys(value: Any, path: str = "root") -> None:
    if isinstance(value, Mapping):
        for key, child in value.items():
            name = str(key)
            if name in FORBIDDEN_SEMANTIC_KEYS:
                raise ValueError(f"forbidden A3.9 semantic key {path}.{name}")
            assert_no_forbidden_semantic_keys(child, f"{path}.{name}")
    elif isinstance(value, (list, tuple)):
        for index, child in enumerate(value):
            assert_no_forbidden_semantic_keys(child, f"{path}[{index}]")


def _require_mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    return value


def _require_list(value: Any, label: str) -> list[Any]:
    if not isinstance(value, list):
        raise ValueError(f"{label} must be a list")
    return value


def validate_a38_baseline_manifest(baseline: Mapping[str, Any]) -> None:
    data = _require_mapping(baseline, "A3.8 terminal baseline")
    if str(data.get("schema", "")) != A38_BASELINE_SCHEMA:
        raise ValueError(
            f"A3.8 terminal baseline schema mismatch: expected {A38_BASELINE_SCHEMA}"
        )
    provenance = _require_mapping(data.get("provenance"), "A3.8 baseline provenance")
    if (
        provenance.get("kind") != "OPERATOR_SUPPLIED_A38_CONSOLE_EVIDENCE"
        or provenance.get("reconstructed_from_console_output") is not True
    ):
        raise ValueError("A3.8 terminal baseline provenance mismatch")

    targets = _require_list(data.get("targets"), "A3.8 baseline targets")
    if len(targets) != EXPECTED_CONNECTOR_COUNT:
        raise ValueError("A3.8 terminal baseline must contain exactly two targets")

    observed: list[tuple[str, str]] = []
    for raw in targets:
        item = _require_mapping(raw, "A3.8 target")
        role = str(item.get("role", ""))
        connector_id = str(item.get("candidate_connector_id", ""))
        observed.append((role, connector_id))
        outcome = _require_mapping(item.get("a36_outcome"), f"{role}.a36_outcome")
        diagnostics = _require_mapping(item.get("r6b_diagnostics"), f"{role}.r6b_diagnostics")
        missing = [field for field in A36_OUTCOME_FIELDS if field not in outcome]
        if missing:
            raise ValueError(f"A3.8 target {role} missing outcome fields: {missing}")
        if int(outcome["search_expansions"]) != int(diagnostics.get("search_expansions", -1)):
            raise ValueError(f"A3.8 target {role} search expansion accounting mismatch")

    if observed != list(TARGETS):
        raise ValueError(f"A3.8 terminal baseline target identity/order mismatch: {observed}")
    assert_no_forbidden_semantic_keys(data)


def select_a39_connector_candidates(all_candidates: Iterable[Any]) -> tuple[Any, ...]:
    index: dict[str, Any] = {}
    for candidate in all_candidates:
        connector_id = str(getattr(candidate, "connector_candidate_id", ""))
        if not connector_id:
            raise ValueError("empty connector id in A2 connector universe")
        if connector_id in index:
            raise ValueError(f"duplicate connector id in A2 connector universe: {connector_id}")
        index[connector_id] = candidate

    wanted = (PRIMARY_CONNECTOR_ID, CONTROL_CONNECTOR_ID)
    missing = [connector_id for connector_id in wanted if connector_id not in index]
    if missing:
        raise ValueError("missing A3.9 target connector(s): " + ", ".join(missing))
    return tuple(index[connector_id] for connector_id in wanted)


def _copy_entry(entry: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "entry_index": int(entry["entry_index"]),
        "position_error_m": float(entry["position_error_m"]),
        "yaw_error_rad": float(entry["yaw_error_rad"]),
        "travel_m": float(entry["travel_m"]),
        "residual_path_budget_m": float(entry["residual_path_budget_m"]),
        "direction": str(entry["direction"]),
        "cusp_count": int(entry["cusp_count"]),
        "forward_goal_shot_eligible": bool(entry["forward_goal_shot_eligible"]),
    }


def _copy_candidate(candidate: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "entry_index": int(candidate["entry_index"]),
        "candidate_index": int(candidate["candidate_index"]),
        "path_type": str(candidate["path_type"]),
        "candidate_length_m": float(candidate["candidate_length_m"]),
        "total_path_length_m": float(candidate["total_path_length_m"]),
        "path_budget_admissible": bool(candidate["path_budget_admissible"]),
        "rejection_reason": str(candidate["rejection_reason"]),
    }


def _min_or_none(items: list[dict[str, Any]], key) -> dict[str, Any] | None:
    if not items:
        return None
    return copy.deepcopy(min(items, key=key))


def aggregate_terminal_trace(
    entry_states: Iterable[Mapping[str, Any]],
    candidates: Iterable[Mapping[str, Any]],
    max_path_length_m: float,
) -> dict[str, Any]:
    max_path_length_m = float(max_path_length_m)
    if not math.isfinite(max_path_length_m) or max_path_length_m <= 0.0:
        raise ValueError("max_path_length_m must be finite and > 0")

    entries = [_copy_entry(_require_mapping(item, "terminal entry")) for item in entry_states]
    shots = [_copy_candidate(_require_mapping(item, "terminal candidate")) for item in candidates]

    entry_indices = [item["entry_index"] for item in entries]
    if len(entry_indices) != len(set(entry_indices)):
        raise ValueError("duplicate terminal entry_index")
    known_entries = set(entry_indices)

    for item in entries:
        expected_residual = max_path_length_m - item["travel_m"]
        if abs(item["residual_path_budget_m"] - expected_residual) > 1.0e-9:
            raise ValueError(f"terminal entry {item['entry_index']} residual path-budget mismatch")
        expected_eligible = item["direction"] == "FORWARD"
        if item["forward_goal_shot_eligible"] != expected_eligible:
            raise ValueError(
                f"terminal entry {item['entry_index']} forward goal-shot eligibility mismatch"
            )

    candidate_keys: set[tuple[int, int]] = set()
    for item in shots:
        if item["entry_index"] not in known_entries:
            raise ValueError(f"terminal candidate references missing entry {item['entry_index']}")
        key = (item["entry_index"], item["candidate_index"])
        if key in candidate_keys:
            raise ValueError(f"duplicate terminal candidate identity: {key}")
        candidate_keys.add(key)
        if item["rejection_reason"] not in REJECTION_REASONS:
            raise ValueError(
                f"unsupported terminal candidate rejection reason: {item['rejection_reason']}"
            )
        expected_admissible = item["total_path_length_m"] <= max_path_length_m
        if item["path_budget_admissible"] != expected_admissible:
            raise ValueError(f"terminal candidate {key} path-budget admissibility mismatch")
        if item["rejection_reason"] == "PATH_LENGTH" and item["path_budget_admissible"]:
            raise ValueError(f"terminal candidate {key} path-budget/reason mismatch")
        if item["rejection_reason"] != "PATH_LENGTH" and not item["path_budget_admissible"]:
            raise ValueError(f"terminal candidate {key} path-budget/reason mismatch")

    rejection_counts = {
        reason: sum(item["rejection_reason"] == reason for item in shots)
        for reason in REJECTION_REASONS
    }
    first_entry = _min_or_none(entries, lambda item: item["entry_index"])
    minimum_travel = _min_or_none(
        entries,
        lambda item: (
            item["travel_m"], item["position_error_m"], item["yaw_error_rad"], item["entry_index"]
        ),
    )
    minimum_yaw = _min_or_none(
        entries,
        lambda item: (
            item["yaw_error_rad"], item["position_error_m"], item["travel_m"], item["entry_index"]
        ),
    )
    candidate_key = lambda item: (
        item["total_path_length_m"],
        item["candidate_length_m"],
        item["entry_index"],
        item["candidate_index"],
        item["path_type"],
    )
    minimum_total = _min_or_none(shots, candidate_key)
    admissible = [item for item in shots if item["path_budget_admissible"]]
    best_admissible = _min_or_none(admissible, candidate_key)
    best_by_outcome: dict[str, dict[str, Any] | None] = {}
    for reason in REJECTION_REASONS:
        subset = [item for item in shots if item["rejection_reason"] == reason]
        best_by_outcome[reason] = _min_or_none(subset, candidate_key)

    return {
        "schema": TERMINAL_DIAGNOSTIC_SCHEMA,
        "max_path_length_m": max_path_length_m,
        "entry_state_count": len(entries),
        "forward_goal_shot_eligible_entry_count": sum(
            item["forward_goal_shot_eligible"] for item in entries
        ),
        "reverse_direction_blocked_entry_count": sum(
            item["direction"] != "FORWARD" for item in entries
        ),
        "candidate_count": len(shots),
        "path_budget_admissible_candidate_count": sum(
            item["path_budget_admissible"] for item in shots
        ),
        "rejection_counts": rejection_counts,
        "entry_state_ranges": {
            "minimum_position_error_m": min(
                (item["position_error_m"] for item in entries), default=None
            ),
            "minimum_yaw_error_rad": min(
                (item["yaw_error_rad"] for item in entries), default=None
            ),
            "minimum_travel_m": min((item["travel_m"] for item in entries), default=None),
            "maximum_residual_path_budget_m": max(
                (item["residual_path_budget_m"] for item in entries), default=None
            ),
        },
        "representatives": {
            "first_entry_state": first_entry,
            "minimum_travel_state": minimum_travel,
            "minimum_yaw_error_state": minimum_yaw,
            "minimum_total_path_candidate": minimum_total,
            "best_path_admissible_candidate": best_admissible,
            "best_candidate_by_outcome": best_by_outcome,
        },
    }


def _baseline_target_index(baseline: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
    validate_a38_baseline_manifest(baseline)
    return {
        str(item["candidate_connector_id"]): _require_mapping(item, "baseline target")
        for item in baseline["targets"]
    }


def _normalize_current_diagnostics_for_baseline(
    current: Mapping[str, Any], baseline_diagnostics: Mapping[str, Any]
) -> dict[str, Any]:
    normalized = dict(current)
    if "schema" not in baseline_diagnostics:
        normalized.pop("schema", None)
    return normalized


def validate_legacy_behavior_preservation(
    baseline: Mapping[str, Any],
    current_records: Iterable[Mapping[str, Any]],
) -> None:
    baseline_index = _baseline_target_index(baseline)
    records = list(current_records)
    if len(records) != EXPECTED_CONNECTOR_COUNT:
        raise ValueError("A3.9 current result must contain exactly two target records")
    observed = [
        (str(item.get("role", "")), str(item.get("candidate_connector_id", "")))
        for item in records
    ]
    if observed != list(TARGETS):
        raise ValueError(f"A3.9 current target identity/order drift: {observed}")

    for record in records:
        connector_id = str(record["candidate_connector_id"])
        frozen = baseline_index[connector_id]
        frozen_outcome = _require_mapping(frozen["a36_outcome"], "frozen outcome")
        current_outcome = {field: record.get(field) for field in A36_OUTCOME_FIELDS}
        expected_outcome = {field: frozen_outcome.get(field) for field in A36_OUTCOME_FIELDS}
        if current_outcome != expected_outcome:
            raise ValueError(
                f"A3.9 legacy R6B outcome drift for {connector_id}: "
                f"frozen={expected_outcome} current={current_outcome}"
            )

        frozen_diag = dict(_require_mapping(frozen["r6b_diagnostics"], "frozen R6B diagnostics"))
        current_diag = _normalize_current_diagnostics_for_baseline(
            _require_mapping(record.get("legacy_r6b_diagnostics"), "current legacy R6B diagnostics"),
            frozen_diag,
        )
        if current_diag != frozen_diag:
            raise ValueError(
                f"A3.9 legacy R6B diagnostic drift for {connector_id}: "
                f"frozen={frozen_diag} current={current_diag}"
            )


def build_a39_report(
    baseline: Mapping[str, Any],
    current_records: Iterable[Mapping[str, Any]],
) -> dict[str, Any]:
    records = [copy.deepcopy(dict(item)) for item in current_records]
    validate_legacy_behavior_preservation(baseline, records)
    report = {
        "schema": REPORT_SCHEMA,
        "validation_scope": VALIDATION_SCOPE,
        "baseline_provenance": copy.deepcopy(baseline["provenance"]),
        "summary": {
            "target_connector_count": len(records),
            "roles": [item["role"] for item in records],
            "legacy_r6b_behavior_preserved": True,
        },
        "connector_records": records,
    }
    assert_no_forbidden_semantic_keys(report)
    return report


def _resolve_run_asset(run_dir: Path, value: str) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (run_dir / path).resolve()


def _require_file(path: Path) -> Path:
    if not path.is_file():
        raise FileNotFoundError(str(path))
    return path


def _preflight(args: argparse.Namespace) -> dict[str, Path]:
    run_dir = Path(args.run_dir).expanduser().resolve()
    if not run_dir.is_dir():
        raise FileNotFoundError(str(run_dir))
    paths = {
        "service_graph": _resolve_run_asset(run_dir, str(args.service_graph)),
        "turn_zones": _resolve_run_asset(run_dir, str(args.turn_zones)),
        "navigation_map": _resolve_run_asset(run_dir, str(args.navigation_map)),
        "navigation_pgm": _resolve_run_asset(run_dir, str(args.navigation_pgm)),
        "site_boundary": _resolve_run_asset(run_dir, str(args.site_boundary)),
        "vehicle_profile": Path(args.vehicle_profile).expanduser().resolve(),
        "a37_manifest": Path(args.a37_manifest).expanduser().resolve(),
        "a38_baseline_manifest": Path(args.a38_baseline_manifest).expanduser().resolve(),
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


def _load_json(path: Path, label: str) -> Mapping[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    return _require_mapping(payload, label)


def _validate_a37_target_membership(manifest: Mapping[str, Any]) -> None:
    if str(manifest.get("schema", "")) != A37_MANIFEST_SCHEMA:
        raise ValueError("A3.7 frontier manifest schema mismatch")
    frontiers = _require_list(manifest.get("frontier_candidates"), "A3.7 frontier_candidates")
    ids = [str(item.get("candidate_connector_id", "")) for item in frontiers]
    if len(ids) != len(set(ids)):
        raise ValueError("duplicate connector id in A3.7 frontier manifest")
    missing = [
        connector_id
        for connector_id in (PRIMARY_CONNECTOR_ID, CONTROL_CONNECTOR_ID)
        if connector_id not in ids
    ]
    if missing:
        raise ValueError("missing A3.9 target from A3.7 frontier manifest: " + ", ".join(missing))
    assert_no_forbidden_semantic_keys(manifest)


class _TerminalTraceObserver:
    def __init__(self, target_ids: Iterable[str]) -> None:
        self._target_ids = frozenset(str(value) for value in target_ids)
        self.entries: dict[str, list[dict[str, Any]]] = {
            connector_id: [] for connector_id in self._target_ids
        }
        self.candidates: dict[str, list[dict[str, Any]]] = {
            connector_id: [] for connector_id in self._target_ids
        }

    def observe(
        self,
        node: Any,
        request: Any,
        radius: float,
        cfg: ReversePrimitiveConnectorConfig,
        bounds: Any,
        navigation: Any,
        local_footprint: Any,
        site_boundary: Any,
    ) -> None:
        connector_id = str(request.connector_id)
        if connector_id not in self._target_ids:
            return
        position_error = math.hypot(
            float(request.goal_pose[0] - node.x),
            float(request.goal_pose[1] - node.y),
        )
        if position_error > float(cfg.goal_shot_distance_m):
            return

        entry_index = len(self.entries[connector_id])
        yaw_error = abs(r6b._wrap_pi(float(request.goal_pose[3] - node.yaw)))
        self.entries[connector_id].append(
            {
                "entry_index": entry_index,
                "position_error_m": position_error,
                "yaw_error_rad": yaw_error,
                "travel_m": float(node.travel_m),
                "residual_path_budget_m": float(cfg.max_path_length_m - node.travel_m),
                "direction": r6b._direction_name(int(node.direction)),
                "cusp_count": int(node.cusp_count),
                "forward_goal_shot_eligible": int(node.direction) == 1,
            }
        )
        if int(node.direction) != 1:
            return

        start = (float(node.x), float(node.y), float(request.start_pose[2]), float(node.yaw))
        candidate_index = 0
        for path_type, normalized_lengths in r6b._dubins_candidates(
            start, request.goal_pose, radius
        ):
            sampled, length_m = r6b._sample_candidate(
                start,
                request.goal_pose,
                path_type,
                normalized_lengths,
                radius,
                cfg.collision_sample_step_m,
            )
            length_m = float(length_m)
            total_path_length_m = float(node.travel_m + length_m)
            admissible = total_path_length_m <= float(cfg.max_path_length_m)
            if not admissible:
                reason = "PATH_LENGTH"
            else:
                converted = tuple(
                    (float(s.x), float(s.y), float(s.yaw), 1, 0.0, False)
                    for s in sampled[1:]
                )
                edge_reason = r6b._edge_rejection_reason(
                    converted,
                    bounds,
                    navigation,
                    local_footprint,
                    site_boundary,
                )
                reason_map = {
                    r6b.EDGE_SEARCH_ENVELOPE: "SEARCH_ENVELOPE",
                    r6b.EDGE_SITE_BOUNDARY: "SITE_BOUNDARY",
                    r6b.EDGE_NAVIGATION_GRID: "NAVIGATION_GRID",
                    r6b.EDGE_FREE: "FREE",
                }
                if edge_reason not in reason_map:
                    raise ValueError(f"unsupported R6B terminal edge reason: {edge_reason}")
                reason = reason_map[edge_reason]

            self.candidates[connector_id].append(
                {
                    "entry_index": entry_index,
                    "candidate_index": candidate_index,
                    "path_type": str(path_type),
                    "candidate_length_m": length_m,
                    "total_path_length_m": total_path_length_m,
                    "path_budget_admissible": admissible,
                    "rejection_reason": reason,
                }
            )
            candidate_index += 1
            if reason == "FREE":
                break


@contextmanager
def _observe_terminal_goal_shots(observer: _TerminalTraceObserver):
    original = r6b._try_forward_goal_shot

    def wrapped(
        node,
        request,
        radius,
        cfg,
        bounds,
        navigation,
        local_footprint,
        site_boundary=None,
        diagnostics=None,
    ):
        observer.observe(
            node,
            request,
            radius,
            cfg,
            bounds,
            navigation,
            local_footprint,
            site_boundary,
        )
        return original(
            node,
            request,
            radius,
            cfg,
            bounds,
            navigation,
            local_footprint,
            site_boundary,
            diagnostics,
        )

    r6b._try_forward_goal_shot = wrapped
    try:
        yield
    finally:
        r6b._try_forward_goal_shot = original


def _current_legacy_diagnostics(validation: Any) -> dict[str, Any]:
    raw = getattr(validation, "reverse_backend_diagnostics", None)
    if not isinstance(raw, Mapping):
        raise ValueError("A3.9 validation lacks legacy R6B diagnostics")
    return copy.deepcopy(dict(raw))


def _validate_trace_accounting(
    connector_id: str,
    diagnostics: Mapping[str, Any],
    trace: Mapping[str, Any],
) -> None:
    expected = {
        "entry_state_count": int(diagnostics.get("goal_shot_attempts", -1)),
        "reverse_direction_blocked_entry_count": int(
            diagnostics.get("goal_shot_reverse_direction_blocked", -1)
        ),
        "candidate_count": int(diagnostics.get("goal_shot_candidates_considered", -1)),
    }
    observed = {name: int(trace.get(name, -2)) for name in expected}
    if observed != expected:
        raise ValueError(
            f"A3.9 terminal trace accounting drift for {connector_id}: "
            f"legacy={expected} trace={observed}"
        )
    rejection_counts = _require_mapping(trace.get("rejection_counts"), "trace rejection counts")
    expected_rejections = {
        "PATH_LENGTH": int(diagnostics.get("goal_shot_rejected_path_length", -1)),
        "SEARCH_ENVELOPE": int(diagnostics.get("goal_shot_rejected_search_envelope", -1)),
        "SITE_BOUNDARY": int(diagnostics.get("goal_shot_rejected_site_boundary", -1)),
        "NAVIGATION_GRID": int(diagnostics.get("goal_shot_rejected_navigation_grid", -1)),
        "FREE": int(diagnostics.get("goal_shot_successes", -1)),
    }
    observed_rejections = {
        reason: int(rejection_counts.get(reason, -2)) for reason in REJECTION_REASONS
    }
    if observed_rejections != expected_rejections:
        raise ValueError(
            f"A3.9 terminal candidate rejection accounting drift for {connector_id}: "
            f"legacy={expected_rejections} trace={observed_rejections}"
        )


def _make_current_records(
    validations: Iterable[Any],
    observer: _TerminalTraceObserver,
    config: ReversePrimitiveConnectorConfig,
) -> tuple[dict[str, Any], ...]:
    validation_index: dict[str, Any] = {}
    for validation in validations:
        connector_id = str(getattr(validation, "connector_candidate_id", ""))
        if not connector_id or connector_id in validation_index:
            raise ValueError(f"duplicate or empty A3.9 validation connector id: {connector_id}")
        validation_index[connector_id] = validation

    records: list[dict[str, Any]] = []
    for role, connector_id in TARGETS:
        validation = validation_index.get(connector_id)
        if validation is None:
            raise ValueError(f"missing A3.9 validation for {connector_id}")
        diagnostics = _current_legacy_diagnostics(validation)
        trace = aggregate_terminal_trace(
            observer.entries[connector_id],
            observer.candidates[connector_id],
            config.max_path_length_m,
        )
        _validate_trace_accounting(connector_id, diagnostics, trace)
        records.append(
            {
                "role": role,
                "candidate_connector_id": connector_id,
                "status": str(getattr(validation, "status", "")),
                "backend_status": str(getattr(validation, "backend_status", "")),
                "search_expansions": int(getattr(validation, "search_expansions", -1)),
                "queue_exhausted": bool(diagnostics.get("queue_exhausted", False)),
                "expansion_budget_reached": bool(
                    diagnostics.get("expansion_budget_reached", False)
                ),
                "failure_class": diagnostics.get("failure_class"),
                "legacy_r6b_diagnostics": diagnostics,
                "terminal_closure_diagnostics": trace,
            }
        )
    if set(validation_index) != {PRIMARY_CONNECTOR_ID, CONTROL_CONNECTOR_ID}:
        raise ValueError(
            "A3.9 selective validation connector universe drift: "
            f"{sorted(validation_index)}"
        )
    return tuple(records)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    paths = _preflight(args)
    hashes_before = _input_hashes(paths)

    a37_manifest = _load_json(paths["a37_manifest"], "A3.7 frontier manifest JSON")
    _validate_a37_target_membership(a37_manifest)
    baseline = _load_json(paths["a38_baseline_manifest"], "A3.8 terminal baseline JSON")
    validate_a38_baseline_manifest(baseline)

    service_graph = load_vehicle_feasible_service_graph(paths["service_graph"])
    selected_candidates = select_a39_connector_candidates(service_graph.connector_candidates)
    selective_service_graph = replace(service_graph, connector_candidates=selected_candidates)

    turn_zones = load_turn_zones(paths["turn_zones"])
    navigation = load_navigation_grid(paths["navigation_map"])
    boundary = load_site_boundary(paths["site_boundary"])
    vehicle = load_canonical_vehicle_profile(paths["vehicle_profile"])
    config = ReversePrimitiveConnectorConfig()
    config.validate()
    motion_config = VehicleFeasibleMotionGraphConfig()

    observer = _TerminalTraceObserver((PRIMARY_CONNECTOR_ID, CONTROL_CONNECTOR_ID))
    with _observe_terminal_goal_shots(observer):
        validations = validate_transition_candidates(
            selective_service_graph,
            turn_zones,
            navigation,
            vehicle,
            motion_config,
            site_boundary=boundary,
        )
    if len(validations) != EXPECTED_CONNECTOR_COUNT:
        raise ValueError(
            f"A3.9 expected exactly {EXPECTED_CONNECTOR_COUNT} validations, got {len(validations)}"
        )

    records = _make_current_records(validations, observer, config)
    report = build_a39_report(baseline, records)

    hashes_after = _input_hashes(paths)
    if hashes_before != hashes_after:
        raise ValueError("A3.9 diagnostic harness modified a protected input asset")
    report.update(
        {
            "run_dir": str(paths["run_dir"]),
            "platform_id": str(service_graph.platform_id),
            "platform_profile_sha256": str(service_graph.platform_profile_sha256),
            "a37_manifest": str(paths["a37_manifest"]),
            "a38_baseline_manifest": str(paths["a38_baseline_manifest"]),
            "selected_connector_count": len(selected_candidates),
            "protected_input_hashes_before": hashes_before,
            "protected_input_hashes_after": hashes_after,
        }
    )
    report["summary"]["protected_input_hashes_unchanged"] = True
    assert_no_forbidden_semantic_keys(report)

    if args.pretty:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(json.dumps(report, separators=(",", ":"), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
