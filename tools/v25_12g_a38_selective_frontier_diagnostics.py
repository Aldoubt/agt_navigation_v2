#!/usr/bin/env python3
"""Selective V25-12G A3.8 diagnostics for the six A4-frontier connectors.

This harness does not amend planner behavior.  It loads the operator-frozen A3.7
frontier manifest, builds an in-memory A2 service-graph view containing exactly
those six connector candidates, and runs the existing A3 transition-validation
chain.  The rerun must reproduce the operator-frozen A3.6 outcome fields before
its detailed current R6B diagnostics are reported.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
from collections import Counter
from dataclasses import replace
from pathlib import Path
from typing import Any, Iterable, Mapping

from agt_offline_assets.navigation_grid import load_navigation_grid
from agt_offline_assets.site_boundary import load_site_boundary
from agt_offline_assets.turn_zones import load_turn_zones
from agt_offline_assets.vehicle_profile import load_canonical_vehicle_profile
from agt_offline_assets.vehicle_feasible_motion_graph import (
    VehicleFeasibleMotionGraphConfig,
)
from agt_offline_assets.vehicle_feasible_service_graph_io import (
    load_vehicle_feasible_service_graph,
)
from agt_offline_assets.vehicle_feasible_transition_motion import (
    validate_transition_candidates,
)
from agt_offline_assets.reverse_primitive_diagnostics import R6B_DIAGNOSTIC_SCHEMA


REPORT_SCHEMA = "agt_v25_12g_a38_selective_frontier_diagnostics/v1"
VALIDATION_SCOPE = (
    "A38_SELECTIVE_A4_FRONTIER_CURRENT_DIAGNOSTIC_ONLY_"
    "NOT_PLANNER_AMENDMENT_NOT_ROUTE_READY"
)
A37_MANIFEST_SCHEMA = "agt_v25_12g_a37_operator_frontier_manifest/v1"
EXPECTED_FRONTIER_COUNT = 6

EXPECTED_PROVENANCE_KIND = "OPERATOR_SUPPLIED_A37_CONSOLE_EVIDENCE"
EXPECTED_RECONSTRUCTION_POLICY = (
    "ONLY_FIELDS_EXPLICITLY_PRESENT_IN_OPERATOR_SUPPLIED_A37_CONSOLE_OUTPUT"
)

FORBIDDEN_SEMANTIC_KEYS = frozenset(
    {
        "route_ready",
        "reachable_from_start",
        "optimal",
        "root_cause",
        "recommended_amendment",
    }
)

A36_EVIDENCE_FIELDS = (
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
            "Run current five-curvature diagnostics for only the six operator-frozen "
            "A4-frontier connectors"
        )
    )
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--vehicle-profile", required=True)
    parser.add_argument("--a37-manifest", required=True)
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
                raise ValueError(f"forbidden A3.8 semantic key {path}.{name}")
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


def _manifest_frontier_index(manifest: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    frontiers = _require_list(manifest.get("frontier_candidates"), "frontier_candidates")
    output: dict[str, dict[str, Any]] = {}
    for raw in frontiers:
        item = dict(_require_mapping(raw, "frontier candidate"))
        connector_id = str(item.get("candidate_connector_id", ""))
        if not connector_id or connector_id in output:
            raise ValueError(f"duplicate or empty frontier connector id: {connector_id}")
        evidence = _require_mapping(item.get("a36_evidence"), f"{connector_id}.a36_evidence")
        missing = [name for name in A36_EVIDENCE_FIELDS if name not in evidence]
        if missing:
            raise ValueError(
                f"frontier connector {connector_id} missing A3.6 evidence fields: {missing}"
            )
        output[connector_id] = item
    return output


def validate_frontier_manifest(manifest: Mapping[str, Any]) -> None:
    data = _require_mapping(manifest, "A3.7 operator frontier manifest")
    if str(data.get("schema", "")) != A37_MANIFEST_SCHEMA:
        raise ValueError(
            f"A3.7 frontier manifest schema mismatch: expected {A37_MANIFEST_SCHEMA}"
        )

    provenance = _require_mapping(data.get("provenance"), "manifest provenance")
    provenance_ok = (
        provenance.get("kind") == EXPECTED_PROVENANCE_KIND
        and provenance.get("original_a35_json_available") is False
        and provenance.get("original_a36_json_available") is False
        and provenance.get("original_a37_json_available") is False
        and provenance.get("reconstructed_from_console_output") is True
        and provenance.get("reconstruction_policy") == EXPECTED_RECONSTRUCTION_POLICY
    )
    if not provenance_ok:
        raise ValueError("A3.7 operator frontier manifest provenance mismatch")

    summary = _require_mapping(data.get("summary"), "manifest summary")
    if summary.get("current_a4_entry_gate_passed") is not False:
        raise ValueError("A3.7 frontier manifest must preserve A4 as blocked")
    if int(summary.get("frontier_candidate_count", -1)) != EXPECTED_FRONTIER_COUNT:
        raise ValueError(
            f"A3.7 frontier count must be exactly {EXPECTED_FRONTIER_COUNT}"
        )

    summary_ids_raw = _require_list(
        summary.get("frontier_candidate_ids"), "summary.frontier_candidate_ids"
    )
    summary_ids = tuple(str(value) for value in summary_ids_raw)
    if len(summary_ids) != EXPECTED_FRONTIER_COUNT or len(set(summary_ids)) != len(summary_ids):
        raise ValueError("A3.7 frontier ids must contain exactly six unique connectors")
    if summary_ids != tuple(sorted(summary_ids)):
        raise ValueError("A3.7 frontier ids must be deterministically sorted")

    frontier_index = _manifest_frontier_index(data)
    if len(frontier_index) != EXPECTED_FRONTIER_COUNT:
        raise ValueError(
            f"A3.7 frontier candidate records must total {EXPECTED_FRONTIER_COUNT}"
        )
    if tuple(sorted(frontier_index)) != summary_ids:
        raise ValueError("A3.7 frontier summary/record connector universe mismatch")

    assert_no_forbidden_semantic_keys(data)


def frontier_connector_ids(manifest: Mapping[str, Any]) -> tuple[str, ...]:
    validate_frontier_manifest(manifest)
    summary = _require_mapping(manifest["summary"], "manifest summary")
    return tuple(str(value) for value in summary["frontier_candidate_ids"])


def select_frontier_connector_candidates(
    all_candidates: Iterable[Any],
    manifest: Mapping[str, Any],
) -> tuple[Any, ...]:
    wanted = frontier_connector_ids(manifest)
    index: dict[str, Any] = {}
    for candidate in all_candidates:
        connector_id = str(getattr(candidate, "connector_candidate_id", ""))
        if not connector_id:
            raise ValueError("empty connector id in full A2 connector universe")
        if connector_id in index:
            raise ValueError(f"duplicate connector id in full A2 connector universe: {connector_id}")
        index[connector_id] = candidate

    missing = [connector_id for connector_id in wanted if connector_id not in index]
    if missing:
        raise ValueError("missing A3.7 frontier connectors from A2 universe: " + ", ".join(missing))
    return tuple(index[connector_id] for connector_id in wanted)


def _diagnostic_mapping(validation: Any) -> dict[str, Any]:
    raw = getattr(validation, "reverse_backend_diagnostics", None)
    if not isinstance(raw, Mapping):
        raise ValueError("selective validation lacks current reverse backend diagnostics")
    diagnostics = dict(raw)
    if diagnostics.get("schema") != R6B_DIAGNOSTIC_SCHEMA:
        raise ValueError("selective validation R6B diagnostic schema mismatch")
    return diagnostics


def _current_evidence(validation: Any) -> dict[str, Any]:
    diagnostics = _diagnostic_mapping(validation)
    validation_expansions = int(getattr(validation, "search_expansions", -1))
    diagnostic_expansions = int(diagnostics.get("search_expansions", -2))
    if validation_expansions != diagnostic_expansions:
        raise ValueError("selective validation search expansion accounting mismatch")
    return {
        "status": str(getattr(validation, "status", "")),
        "backend_status": str(getattr(validation, "backend_status", "")),
        "search_expansions": validation_expansions,
        "queue_exhausted": bool(diagnostics.get("queue_exhausted", False)),
        "expansion_budget_reached": bool(
            diagnostics.get("expansion_budget_reached", False)
        ),
        "failure_class": diagnostics.get("failure_class"),
    }


def compare_selective_outcomes(
    manifest: Mapping[str, Any],
    validations: Iterable[Any],
) -> list[dict[str, Any]]:
    wanted = frontier_connector_ids(manifest)
    frontier_index = _manifest_frontier_index(manifest)

    validation_index: dict[str, Any] = {}
    for validation in validations:
        connector_id = str(getattr(validation, "connector_candidate_id", ""))
        if not connector_id or connector_id in validation_index:
            raise ValueError(f"duplicate or empty selective validation connector id: {connector_id}")
        validation_index[connector_id] = validation

    if set(validation_index) != set(wanted):
        missing = sorted(set(wanted) - set(validation_index))
        extra = sorted(set(validation_index) - set(wanted))
        raise ValueError(
            f"selective validation connector universe drift: missing={missing} extra={extra}"
        )

    records: list[dict[str, Any]] = []
    for connector_id in wanted:
        validation = validation_index[connector_id]
        current = _current_evidence(validation)
        frozen = dict(
            _require_mapping(
                frontier_index[connector_id].get("a36_evidence"),
                f"{connector_id}.a36_evidence",
            )
        )
        frozen_projection = {name: frozen.get(name) for name in A36_EVIDENCE_FIELDS}
        if current != frozen_projection:
            raise ValueError(
                f"A3.8 current outcome drift for {connector_id}: "
                f"frozen={frozen_projection} current={current}"
            )
        records.append(
            {
                "candidate_connector_id": connector_id,
                "a37_frontier": copy.deepcopy(frontier_index[connector_id]),
                "current_evidence": current,
                "current_reverse_backend_diagnostics": copy.deepcopy(
                    _diagnostic_mapping(validation)
                ),
            }
        )
    return records


def build_selective_report(
    manifest: Mapping[str, Any],
    validations: Iterable[Any],
) -> dict[str, Any]:
    validate_frontier_manifest(manifest)
    records = compare_selective_outcomes(manifest, validations)
    failure_counts = Counter(
        str(item["current_evidence"]["failure_class"]) for item in records
    )
    summary = {
        "frontier_candidate_count": len(records),
        "frontier_candidate_ids": [
            str(item["candidate_connector_id"]) for item in records
        ],
        "queue_exhausted_count": sum(
            bool(item["current_evidence"]["queue_exhausted"]) for item in records
        ),
        "expansion_budget_reached_count": sum(
            bool(item["current_evidence"]["expansion_budget_reached"])
            for item in records
        ),
        "failure_class_counts": dict(sorted(failure_counts.items())),
        "current_outcomes_match_operator_frozen_a36_evidence": True,
    }
    report = {
        "schema": REPORT_SCHEMA,
        "validation_scope": VALIDATION_SCOPE,
        "frontier_manifest_provenance": copy.deepcopy(manifest["provenance"]),
        "frontier_records": records,
        "summary": summary,
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


def _load_json(path: Path) -> Mapping[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    return _require_mapping(payload, "A3.7 operator frontier manifest JSON")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    paths = _preflight(args)
    hashes_before = _input_hashes(paths)

    manifest = _load_json(paths["a37_manifest"])
    validate_frontier_manifest(manifest)

    service_graph = load_vehicle_feasible_service_graph(paths["service_graph"])
    selected_candidates = select_frontier_connector_candidates(
        service_graph.connector_candidates,
        manifest,
    )
    if len(selected_candidates) != EXPECTED_FRONTIER_COUNT:
        raise ValueError("A3.8 selective connector count drift")

    selective_service_graph = replace(
        service_graph,
        connector_candidates=selected_candidates,
    )
    turn_zones = load_turn_zones(paths["turn_zones"])
    navigation = load_navigation_grid(paths["navigation_map"])
    boundary = load_site_boundary(paths["site_boundary"])
    vehicle = load_canonical_vehicle_profile(paths["vehicle_profile"])

    validations = validate_transition_candidates(
        selective_service_graph,
        turn_zones,
        navigation,
        vehicle,
        VehicleFeasibleMotionGraphConfig(),
        site_boundary=boundary,
    )
    if len(validations) != EXPECTED_FRONTIER_COUNT:
        raise ValueError(
            f"A3.8 expected {EXPECTED_FRONTIER_COUNT} selective validations, "
            f"got {len(validations)}"
        )

    report = build_selective_report(manifest, validations)

    hashes_after = _input_hashes(paths)
    if hashes_before != hashes_after:
        raise ValueError("A3.8 selective diagnostic harness modified a protected input asset")

    report.update(
        {
            "run_dir": str(paths["run_dir"]),
            "platform_id": str(service_graph.platform_id),
            "platform_profile_sha256": str(service_graph.platform_profile_sha256),
            "a37_manifest": str(paths["a37_manifest"]),
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
