"""A3.9 terminal-closure Pareto diagnosis RED contract.

This stage is diagnostic-only. It must not relax or mutate any frozen R6B
planner parameter and must preserve the legacy R6B outcome/diagnostic counters
for the PRIMARY and CONTROL connectors while adding bounded terminal-closure
aggregates.
"""

import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
HARNESS = ROOT / "tools" / "v25_12g_a39_terminal_closure_pareto_diagnosis.py"
BASELINE = (
    ROOT
    / "docs"
    / "v2.5"
    / "evidence"
    / "V25_12G_A38_OPERATOR_TERMINAL_BASELINE_2026-08-19.json"
)

PRIMARY_CONNECTOR_ID = (
    "headland.turn_high_u.aisle_017.segment_005.dead_end_forward_in_reverse_out."
    "to.aisle_019.segment_003.dead_end_forward_in_reverse_out"
)
CONTROL_CONNECTOR_ID = (
    "headland.turn_high_u.aisle_017.segment_005.dead_end_forward_in_reverse_out."
    "to.aisle_020.segment_001.service_high_to_low"
)


def _load_harness():
    assert HARNESS.is_file(), f"missing A3.9 terminal-closure harness: {HARNESS}"
    spec = importlib.util.spec_from_file_location(
        "v25_12g_a39_terminal_closure_pareto_diagnosis",
        HARNESS,
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _baseline_manifest():
    return json.loads(BASELINE.read_text(encoding="utf-8"))


def _candidate(connector_id: str):
    return SimpleNamespace(connector_candidate_id=connector_id)


def _legacy_diag(expansions: int):
    return {
        "search_started": True,
        "search_expansions": expansions,
        "queue_exhausted": True,
        "expansion_budget_reached": False,
        "failure_class": "MIXED_LIMITATION",
        "start_goal_position_error_m": 4.0,
        "best_goal_position_error_m": 0.30,
        "best_goal_yaw_error_rad": 1.0,
        "best_goal_distance_state_direction": "FORWARD",
        "best_goal_distance_cusp_count": 2,
        "nodes_popped": expansions + 100,
        "stale_nodes_skipped": 100,
        "primitive_edges_considered": 1000,
        "primitive_edges_rejected_search_envelope": 10,
        "primitive_edges_rejected_site_boundary": 100,
        "primitive_edges_rejected_navigation_grid": 200,
        "primitive_edges_rejected_path_length": 0,
        "primitive_edges_rejected_state_dominance": 500,
        "primitive_edges_enqueued": 190,
        "cusp_switches_considered": 50,
        "cusp_switches_rejected_state_dominance": 20,
        "cusp_switches_enqueued": 30,
        "nodes_at_max_cusps": 80,
        "goal_tolerance_checks": expansions,
        "goal_tolerance_successes": 0,
        "goal_shot_attempts": 20,
        "goal_shot_reverse_direction_blocked": 2,
        "goal_shot_candidates_considered": 25,
        "goal_shot_rejected_path_length": 15,
        "goal_shot_rejected_search_envelope": 0,
        "goal_shot_rejected_site_boundary": 4,
        "goal_shot_rejected_navigation_grid": 6,
        "goal_shot_successes": 0,
    }


def _synthetic_baseline():
    return {
        "schema": "agt_v25_12g_a38_operator_terminal_baseline/v1",
        "validation_scope": (
            "A38_OPERATOR_SUPPLIED_TERMINAL_BASELINE_FOR_A39_DIAGNOSTIC_ONLY_"
            "NOT_PLANNER_AMENDMENT_NOT_ROUTE_READY"
        ),
        "provenance": {
            "kind": "OPERATOR_SUPPLIED_A38_CONSOLE_EVIDENCE",
            "reconstructed_from_console_output": True,
        },
        "targets": [
            {
                "role": "PRIMARY",
                "candidate_connector_id": PRIMARY_CONNECTOR_ID,
                "a36_outcome": {
                    "status": "REJECTED",
                    "backend_status": "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION",
                    "search_expansions": 100,
                    "queue_exhausted": True,
                    "expansion_budget_reached": False,
                    "failure_class": "MIXED_LIMITATION",
                },
                "r6b_diagnostics": _legacy_diag(100),
            },
            {
                "role": "CONTROL",
                "candidate_connector_id": CONTROL_CONNECTOR_ID,
                "a36_outcome": {
                    "status": "REJECTED",
                    "backend_status": "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION",
                    "search_expansions": 200,
                    "queue_exhausted": True,
                    "expansion_budget_reached": False,
                    "failure_class": "MIXED_LIMITATION",
                },
                "r6b_diagnostics": _legacy_diag(200),
            },
        ],
    }


def _current_record(role: str, connector_id: str, expansions: int, trace: dict):
    return {
        "role": role,
        "candidate_connector_id": connector_id,
        "status": "REJECTED",
        "backend_status": "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION",
        "search_expansions": expansions,
        "queue_exhausted": True,
        "expansion_budget_reached": False,
        "failure_class": "MIXED_LIMITATION",
        "legacy_r6b_diagnostics": _legacy_diag(expansions),
        "terminal_closure_diagnostics": trace,
    }


def _entry_states():
    return [
        {
            "entry_index": 0,
            "position_error_m": 2.8,
            "yaw_error_rad": 1.2,
            "travel_m": 8.0,
            "residual_path_budget_m": 10.0,
            "direction": "REVERSE",
            "cusp_count": 1,
            "forward_goal_shot_eligible": False,
        },
        {
            "entry_index": 1,
            "position_error_m": 1.5,
            "yaw_error_rad": 0.8,
            "travel_m": 10.0,
            "residual_path_budget_m": 8.0,
            "direction": "FORWARD",
            "cusp_count": 1,
            "forward_goal_shot_eligible": True,
        },
        {
            "entry_index": 2,
            "position_error_m": 0.4,
            "yaw_error_rad": 0.3,
            "travel_m": 15.5,
            "residual_path_budget_m": 2.5,
            "direction": "FORWARD",
            "cusp_count": 2,
            "forward_goal_shot_eligible": True,
        },
        {
            "entry_index": 3,
            "position_error_m": 0.6,
            "yaw_error_rad": 0.1,
            "travel_m": 12.0,
            "residual_path_budget_m": 6.0,
            "direction": "FORWARD",
            "cusp_count": 1,
            "forward_goal_shot_eligible": True,
        },
    ]


def _candidates():
    return [
        {
            "entry_index": 1,
            "candidate_index": 0,
            "path_type": "LSL",
            "candidate_length_m": 9.0,
            "total_path_length_m": 19.0,
            "path_budget_admissible": False,
            "rejection_reason": "PATH_LENGTH",
        },
        {
            "entry_index": 1,
            "candidate_index": 1,
            "path_type": "RSR",
            "candidate_length_m": 5.0,
            "total_path_length_m": 15.0,
            "path_budget_admissible": True,
            "rejection_reason": "SITE_BOUNDARY",
        },
        {
            "entry_index": 2,
            "candidate_index": 0,
            "path_type": "LSL",
            "candidate_length_m": 2.0,
            "total_path_length_m": 17.5,
            "path_budget_admissible": True,
            "rejection_reason": "NAVIGATION_GRID",
        },
        {
            "entry_index": 3,
            "candidate_index": 0,
            "path_type": "RSL",
            "candidate_length_m": 5.5,
            "total_path_length_m": 17.5,
            "path_budget_admissible": True,
            "rejection_reason": "FREE",
        },
        {
            "entry_index": 3,
            "candidate_index": 1,
            "path_type": "LSR",
            "candidate_length_m": 4.0,
            "total_path_length_m": 16.0,
            "path_budget_admissible": True,
            "rejection_reason": "SITE_BOUNDARY",
        },
    ]


def test_a39_constants_parser_and_frozen_r6b_parameters_are_locked():
    module = _load_harness()
    assert module.REPORT_SCHEMA == "agt_v25_12g_a39_terminal_closure_pareto_diagnosis/v1"
    assert module.TERMINAL_DIAGNOSTIC_SCHEMA == "agt_r6b_terminal_closure_diagnostics/v1"
    assert module.VALIDATION_SCOPE == (
        "A39_TERMINAL_CLOSURE_PARETO_DIAGNOSTIC_ONLY_"
        "NOT_PLANNER_AMENDMENT_NOT_ROUTE_READY"
    )
    assert module.PRIMARY_CONNECTOR_ID == PRIMARY_CONNECTOR_ID
    assert module.CONTROL_CONNECTOR_ID == CONTROL_CONNECTOR_ID
    assert module.EXPECTED_CONNECTOR_COUNT == 2

    args = module.build_parser().parse_args(
        [
            "--run-dir",
            "/tmp/run",
            "--vehicle-profile",
            "/tmp/mk_mini.yaml",
            "--a37-manifest",
            "/tmp/a37.json",
            "--a38-baseline-manifest",
            "/tmp/a38.json",
        ]
    )
    assert args.service_graph == "vehicle_feasible_service_graph.yaml"
    assert args.turn_zones == "turn_zones.yaml"
    assert args.navigation_map == "navigation_map.yaml"
    assert args.navigation_pgm == "navigation_map.pgm"
    assert args.site_boundary == "site_boundary.yaml"
    assert args.pretty is False
    assert not hasattr(args, "output")
    for forbidden in (
        "max_path_length_m",
        "max_cusps",
        "max_expansions",
        "goal_shot_distance_m",
        "goal_position_tolerance_m",
        "goal_yaw_tolerance_deg",
        "preview_footprint_padding_m",
    ):
        assert not hasattr(args, forbidden)

    cfg = module.ReversePrimitiveConnectorConfig()
    assert cfg.primitive_length_m == 0.30
    assert cfg.collision_sample_step_m == 0.10
    assert cfg.state_xy_resolution_m == 0.15
    assert cfg.state_yaw_resolution_deg == 15.0
    assert cfg.goal_position_tolerance_m == 0.18
    assert cfg.goal_yaw_tolerance_deg == 12.0
    assert cfg.goal_shot_distance_m == 3.0
    assert cfg.max_cusps == 2
    assert cfg.max_expansions == 30000
    assert cfg.max_path_length_m == 18.0
    assert cfg.longitudinal_zone_padding_m == 0.80
    assert cfg.lateral_pair_padding_m == 1.25
    assert cfg.preview_footprint_padding_m == 0.05


def test_a39_repository_baseline_has_exact_primary_control_provenance():
    module = _load_harness()
    baseline = _baseline_manifest()
    module.validate_a38_baseline_manifest(baseline)

    assert baseline["provenance"]["kind"] == "OPERATOR_SUPPLIED_A38_CONSOLE_EVIDENCE"
    assert baseline["provenance"]["reconstructed_from_console_output"] is True
    assert [item["role"] for item in baseline["targets"]] == ["PRIMARY", "CONTROL"]
    assert [item["candidate_connector_id"] for item in baseline["targets"]] == [
        PRIMARY_CONNECTOR_ID,
        CONTROL_CONNECTOR_ID,
    ]
    assert baseline["targets"][0]["r6b_diagnostics"]["search_expansions"] == 14828
    assert baseline["targets"][0]["r6b_diagnostics"]["goal_shot_attempts"] == 3188
    assert baseline["targets"][1]["r6b_diagnostics"]["search_expansions"] == 14912
    assert baseline["targets"][1]["r6b_diagnostics"]["goal_shot_attempts"] == 818


def test_a39_selects_only_primary_and_control_from_full_connector_universe():
    module = _load_harness()
    all_candidates = tuple(
        [_candidate(PRIMARY_CONNECTOR_ID), _candidate(CONTROL_CONNECTOR_ID)]
        + [_candidate(f"connector.other.{index:02d}") for index in range(38)]
    )

    selected = module.select_a39_connector_candidates(all_candidates)

    assert len(selected) == 2
    assert tuple(item.connector_candidate_id for item in selected) == (
        PRIMARY_CONNECTOR_ID,
        CONTROL_CONNECTOR_ID,
    )


def test_a39_selection_fails_closed_on_missing_or_duplicate_target():
    module = _load_harness()
    with pytest.raises(ValueError, match="missing"):
        module.select_a39_connector_candidates((_candidate(PRIMARY_CONNECTOR_ID),))

    with pytest.raises(ValueError, match="duplicate"):
        module.select_a39_connector_candidates(
            (
                _candidate(PRIMARY_CONNECTOR_ID),
                _candidate(PRIMARY_CONNECTOR_ID),
                _candidate(CONTROL_CONNECTOR_ID),
            )
        )


def test_a39_legacy_r6b_behavior_must_match_a38_baseline_exactly():
    module = _load_harness()
    baseline = _synthetic_baseline()
    trace = module.aggregate_terminal_trace(_entry_states(), _candidates(), 18.0)
    current = (
        _current_record("PRIMARY", PRIMARY_CONNECTOR_ID, 100, trace),
        _current_record("CONTROL", CONTROL_CONNECTOR_ID, 200, trace),
    )

    module.validate_legacy_behavior_preservation(baseline, current)

    drifted = copy.deepcopy(list(current))
    drifted[0]["legacy_r6b_diagnostics"]["goal_shot_attempts"] += 1
    with pytest.raises(ValueError, match="legacy R6B diagnostic drift"):
        module.validate_legacy_behavior_preservation(baseline, tuple(drifted))


def test_a39_terminal_entry_representatives_are_deterministic():
    module = _load_harness()
    trace = module.aggregate_terminal_trace(_entry_states(), _candidates(), 18.0)

    assert trace["schema"] == module.TERMINAL_DIAGNOSTIC_SCHEMA
    assert trace["entry_state_count"] == 4
    assert trace["forward_goal_shot_eligible_entry_count"] == 3
    assert trace["reverse_direction_blocked_entry_count"] == 1
    assert trace["representatives"]["first_entry_state"]["entry_index"] == 0
    assert trace["representatives"]["minimum_travel_state"]["entry_index"] == 0
    assert trace["representatives"]["minimum_yaw_error_state"]["entry_index"] == 3


def test_a39_terminal_candidate_accounting_and_pareto_representatives_are_bounded():
    module = _load_harness()
    trace = module.aggregate_terminal_trace(_entry_states(), _candidates(), 18.0)

    assert trace["candidate_count"] == 5
    assert trace["path_budget_admissible_candidate_count"] == 4
    assert trace["rejection_counts"] == {
        "PATH_LENGTH": 1,
        "SEARCH_ENVELOPE": 0,
        "SITE_BOUNDARY": 2,
        "NAVIGATION_GRID": 1,
        "FREE": 1,
    }
    assert trace["representatives"]["minimum_total_path_candidate"]["total_path_length_m"] == 15.0
    assert trace["representatives"]["best_path_admissible_candidate"]["total_path_length_m"] == 15.0
    assert trace["representatives"]["best_candidate_by_outcome"]["PATH_LENGTH"]["candidate_index"] == 0
    assert trace["representatives"]["best_candidate_by_outcome"]["FREE"]["entry_index"] == 3

    inconsistent = _candidates()
    inconsistent[0]["path_budget_admissible"] = True
    with pytest.raises(ValueError, match="path-budget"):
        module.aggregate_terminal_trace(_entry_states(), inconsistent, 18.0)


def test_a39_report_keeps_primary_control_separate_and_contains_no_causal_claim():
    module = _load_harness()
    baseline = _synthetic_baseline()
    trace = module.aggregate_terminal_trace(_entry_states(), _candidates(), 18.0)
    records = (
        _current_record("PRIMARY", PRIMARY_CONNECTOR_ID, 100, trace),
        _current_record("CONTROL", CONTROL_CONNECTOR_ID, 200, trace),
    )
    report = module.build_a39_report(baseline, records)

    assert report["schema"] == module.REPORT_SCHEMA
    assert report["validation_scope"] == module.VALIDATION_SCOPE
    assert report["summary"]["target_connector_count"] == 2
    assert report["summary"]["roles"] == ["PRIMARY", "CONTROL"]
    assert report["summary"]["legacy_r6b_behavior_preserved"] is True
    assert [item["role"] for item in report["connector_records"]] == [
        "PRIMARY",
        "CONTROL",
    ]
    assert "root_cause" not in report
    assert "recommended_amendment" not in report
    assert "hypothesis_confirmed" not in report


def test_a39_report_is_aggregate_only_and_excludes_unbounded_raw_events():
    module = _load_harness()
    trace = module.aggregate_terminal_trace(_entry_states(), _candidates(), 18.0)
    report = module.build_a39_report(
        _synthetic_baseline(),
        (
            _current_record("PRIMARY", PRIMARY_CONNECTOR_ID, 100, trace),
            _current_record("CONTROL", CONTROL_CONNECTOR_ID, 200, trace),
        ),
    )
    rendered = json.dumps(report, sort_keys=True)

    assert "raw_entry_states" not in rendered
    assert "raw_goal_shot_candidates" not in rendered
    assert "all_entry_states" not in rendered
    assert "all_candidates" not in rendered
    assert len(report["connector_records"][0]["terminal_closure_diagnostics"]["representatives"]) <= 6


def test_a39_forbidden_semantic_keys_fail_closed_recursively():
    module = _load_harness()
    trace = module.aggregate_terminal_trace(_entry_states(), _candidates(), 18.0)
    base_report = module.build_a39_report(
        _synthetic_baseline(),
        (
            _current_record("PRIMARY", PRIMARY_CONNECTOR_ID, 100, trace),
            _current_record("CONTROL", CONTROL_CONNECTOR_ID, 200, trace),
        ),
    )

    for key in ("route_ready", "reachable_from_start", "optimal"):
        report = copy.deepcopy(base_report)
        report["connector_records"][0]["terminal_closure_diagnostics"][key] = True
        with pytest.raises(ValueError, match=key):
            module.assert_no_forbidden_semantic_keys(report)
