import copy
import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
HARNESS = ROOT / "tools" / "v25_12g_a38_frontier_diagnostic_evidence_join.py"


def _load_harness():
    assert HARNESS.is_file(), f"missing A3.8 evidence-join harness: {HARNESS}"
    spec = importlib.util.spec_from_file_location(
        "v25_12g_a38_frontier_diagnostic_evidence_join",
        HARNESS,
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _a35_connector(
    connector_id,
    *,
    failure_class="MIXED_LIMITATION",
    expansions=100,
    best_pos=0.5,
    best_yaw=0.2,
    direction="FORWARD",
    cusp_count=1,
    primitive_considered=1000,
    state_dominance=600,
    primitive_enqueued=100,
    path_rejected=100,
    site_rejected=50,
    grid_rejected=50,
    envelope_rejected=20,
    goal_attempts=20,
    goal_reverse_blocked=5,
    goal_candidates=10,
    goal_successes=0,
    goal_tolerance_checks=100,
    goal_tolerance_successes=0,
):
    return {
        "connector_candidate_id": connector_id,
        "from_segment_id": "segment.a",
        "to_segment_id": "segment.b",
        "side": "HIGH_U",
        "turn_zone_id": "turn.high",
        "forward_audit_status": "LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT",
        "r6a_decision": "ELIGIBLE_REVERSE_FALLBACK",
        "transition_status": "REJECTED",
        "proof_scope": "BOUNDED_SEARCH_NO_SOLUTION",
        "backend_status": "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION",
        "schema": "agt_r6b_connector_diagnostics/v1",
        "failure_class": failure_class,
        "search_started": True,
        "search_expansions": expansions,
        "queue_exhausted": True,
        "expansion_budget_reached": False,
        "start_goal_position_error_m": 5.0,
        "best_goal_position_error_m": best_pos,
        "best_goal_yaw_error_rad": best_yaw,
        "best_goal_distance_state_direction": direction,
        "best_goal_distance_cusp_count": cusp_count,
        "nodes_popped": expansions,
        "stale_nodes_skipped": 0,
        "cusp_switches_considered": 20,
        "cusp_switches_rejected_state_dominance": 10,
        "cusp_switches_enqueued": 10,
        "nodes_at_max_cusps": 20,
        "primitive_edges_considered": primitive_considered,
        "primitive_edges_rejected_search_envelope": envelope_rejected,
        "primitive_edges_rejected_site_boundary": site_rejected,
        "primitive_edges_rejected_navigation_grid": grid_rejected,
        "primitive_edges_rejected_path_length": path_rejected,
        "primitive_edges_rejected_state_dominance": state_dominance,
        "primitive_edges_enqueued": primitive_enqueued,
        "goal_tolerance_checks": goal_tolerance_checks,
        "goal_tolerance_successes": goal_tolerance_successes,
        "goal_shot_attempts": goal_attempts,
        "goal_shot_reverse_direction_blocked": goal_reverse_blocked,
        "goal_shot_candidates_considered": goal_candidates,
        "goal_shot_rejected_path_length": 2,
        "goal_shot_rejected_search_envelope": 2,
        "goal_shot_rejected_site_boundary": 2,
        "goal_shot_rejected_navigation_grid": 4,
        "goal_shot_successes": goal_successes,
    }


def _a35_report():
    connectors = [
        _a35_connector(
            "connector.frontier.a",
            failure_class="GOAL_CONNECTION_LIMITED",
            expansions=8001,
            best_pos=0.09,
            best_yaw=0.05,
            goal_reverse_blocked=25,
            goal_attempts=40,
        ),
        _a35_connector(
            "connector.frontier.b",
            failure_class="MIXED_LIMITATION",
            expansions=162,
            best_pos=0.45,
            best_yaw=0.30,
            state_dominance=700,
            goal_reverse_blocked=0,
            goal_attempts=3,
        ),
    ]
    for index in range(38):
        connectors.append(
            _a35_connector(
                f"connector.other.{index:02d}",
                expansions=10 + index,
                best_pos=1.0 + index,
            )
        )
    return {
        "schema": "agt_v25_12g_a35_connector_backend_diagnosis/v1",
        "validation_scope": (
            "A35_R6B_DIAGNOSTIC_ONLY_NOT_ROUTE_READY_NOT_GLOBAL_INFEASIBILITY_PROOF"
        ),
        "connectors": connectors,
        "summary": {
            "r6b_connector_count": 40,
            "queue_exhausted_count": 40,
            "expansion_budget_reached_count": 0,
            "behavior_projection_equal": True,
            "final_transition_statuses_unchanged": True,
            "forbidden_semantic_key_count": 0,
            "protected_input_hashes_unchanged": True,
        },
    }


def _frontier(connector_id, *, expansions, failure_class, position):
    return {
        "candidate_connector_id": connector_id,
        "from_service_state_id": "service.a",
        "to_service_state_id": "service.b",
        "from_segment_id": "segment.a",
        "to_segment_id": "segment.b",
        "side": "HIGH_U",
        "turn_zone_id": "turn.high",
        "a36_evidence": {
            "status": "REJECTED",
            "backend_status": "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION",
            "search_expansions": expansions,
            "queue_exhausted": True,
            "expansion_budget_reached": False,
            "failure_class": failure_class,
        },
        "a35_reference": {
            "connector_candidate_id": connector_id,
            "search_expansions": 1,
            "reverse_backend_diagnostics": {},
        },
        "hypothetical_witnesses": [
            {
                "position": position,
                "connector_candidate_ids": ["connector.existing", connector_id],
                "service_state_ids": ["service.x", "service.a", "service.b"],
                "segment_ids": ["segment.x", "segment.a", "segment.b"],
                "aisle_ids": ["aisle.1", "aisle.2", "aisle.3"],
            }
        ],
    }


def _a37_report():
    frontiers = [
        _frontier(
            "connector.frontier.a",
            expansions=14828,
            failure_class="MIXED_LIMITATION",
            position="SUCCESSOR_FROM_EXECUTABLE",
        ),
        _frontier(
            "connector.frontier.b",
            expansions=301,
            failure_class="INCONCLUSIVE",
            position="PREDECESSOR_TO_EXECUTABLE",
        ),
    ]
    return {
        "schema": "agt_v25_12g_a37_a4_chain_frontier_diagnosis/v1",
        "validation_scope": (
            "A37_A4_CHAIN_FRONTIER_DIAGNOSTIC_ONLY_NOT_PLANNER_AMENDMENT_NOT_ROUTE_READY"
        ),
        "summary": {
            "a36_executable_connector_ids": ["connector.existing"],
            "a36_rejected_connector_count": 38,
            "frontier_candidate_ids": [
                "connector.frontier.a",
                "connector.frontier.b",
            ],
            "frontier_candidate_count": 2,
            "current_a4_entry_gate_passed": False,
        },
        "frontier_candidates": frontiers,
    }


def test_a38_constants_parser_and_source_are_read_only():
    module = _load_harness()
    assert module.REPORT_SCHEMA == (
        "agt_v25_12g_a38_frontier_diagnostic_evidence_join/v1"
    )
    assert module.VALIDATION_SCOPE == (
        "A38_FRONTIER_DIAGNOSTIC_EVIDENCE_JOIN_ONLY_NOT_PLANNER_AMENDMENT_NOT_ROUTE_READY"
    )
    assert module.A35_REPORT_SCHEMA == (
        "agt_v25_12g_a35_connector_backend_diagnosis/v1"
    )
    assert module.A37_REPORT_SCHEMA == (
        "agt_v25_12g_a37_a4_chain_frontier_diagnosis/v1"
    )
    assert module.EXPECTED_A35_CONNECTOR_COUNT == 40

    args = module.build_parser().parse_args(
        [
            "--a35-report",
            "/tmp/a35.json",
            "--a37-report",
            "/tmp/a37.json",
        ]
    )
    assert args.pretty is False
    assert not hasattr(args, "run_dir")
    assert not hasattr(args, "vehicle_profile")
    assert not hasattr(args, "output")

    source = HARNESS.read_text(encoding="utf-8")
    assert "derive_vehicle_feasible_motion_graph" not in source
    assert "derive_reverse_primitive_connector_plan" not in source


def test_a38_join_uses_a35_g3_report_not_a37_baseline_placeholder():
    module = _load_harness()
    a35 = _a35_report()
    a37 = _a37_report()
    before35 = copy.deepcopy(a35)
    before37 = copy.deepcopy(a37)

    records = module.join_frontier_evidence(a35, a37)

    assert a35 == before35
    assert a37 == before37
    assert [item["candidate_connector_id"] for item in records] == [
        "connector.frontier.a",
        "connector.frontier.b",
    ]
    first = records[0]
    assert first["a37_frontier"]["a35_reference"]["search_expansions"] == 1
    assert first["a35_g3_evidence"]["search_expansions"] == 8001
    assert first["a35_g3_evidence"]["failure_class"] == "GOAL_CONNECTION_LIMITED"
    assert first["a35_g3_evidence"]["best_goal_position_error_m"] == 0.09
    assert first["a35_g3_evidence"]["goal_shot_reverse_direction_blocked"] == 25


def test_a38_join_keeps_a36_a37_and_a35_provenance_separate():
    module = _load_harness()
    first = module.join_frontier_evidence(_a35_report(), _a37_report())[0]

    assert first["a37_frontier"]["a36_evidence"]["search_expansions"] == 14828
    assert first["a35_g3_evidence"]["search_expansions"] == 8001
    assert "best_goal_position_error_m" not in first["a37_frontier"]["a36_evidence"]
    assert first["a35_g3_evidence"]["schema"] == "agt_r6b_connector_diagnostics/v1"


def test_a38_report_exposes_deterministic_frontier_diagnostic_summary():
    module = _load_harness()
    report = module.build_evidence_join_report(_a35_report(), _a37_report())
    summary = report["summary"]

    assert report["schema"] == module.REPORT_SCHEMA
    assert summary["frontier_candidate_count"] == 2
    assert summary["frontier_candidate_ids"] == [
        "connector.frontier.a",
        "connector.frontier.b",
    ]
    assert summary["a35_failure_class_counts"] == {
        "GOAL_CONNECTION_LIMITED": 1,
        "MIXED_LIMITATION": 1,
    }
    assert summary["a36_failure_class_counts"] == {
        "INCONCLUSIVE": 1,
        "MIXED_LIMITATION": 1,
    }
    assert summary["a35_closest_goal_connector_ids"] == [
        "connector.frontier.a",
        "connector.frontier.b",
    ]
    assert summary["a35_goal_shot_reverse_blocked_connector_ids"] == [
        "connector.frontier.a"
    ]
    assert summary["a35_goal_shot_attempted_connector_ids"] == [
        "connector.frontier.a",
        "connector.frontier.b",
    ]
    assert summary["a35_goal_tolerance_success_connector_ids"] == []


def test_a38_report_computes_descriptive_fractions_without_causal_label():
    module = _load_harness()
    report = module.build_evidence_join_report(_a35_report(), _a37_report())
    by_id = {
        item["candidate_connector_id"]: item
        for item in report["frontier_records"]
    }
    row = by_id["connector.frontier.a"]
    ratios = row["a35_descriptive_fractions"]

    assert ratios["state_dominance_rejection_fraction"] == pytest.approx(0.6)
    assert ratios["primitive_enqueued_fraction"] == pytest.approx(0.1)
    assert ratios["path_length_rejection_fraction"] == pytest.approx(0.1)
    assert "root_cause" not in row
    assert "recommended_amendment" not in row


def test_a38_rejects_missing_frontier_connector_in_a35_g3_report():
    module = _load_harness()
    a35 = _a35_report()
    a35["connectors"] = [
        item
        for item in a35["connectors"]
        if item["connector_candidate_id"] != "connector.frontier.b"
    ]
    a35["summary"]["r6b_connector_count"] = 39

    with pytest.raises(ValueError, match="40"):
        module.join_frontier_evidence(a35, _a37_report())


def test_a38_rejects_a35_or_a37_integrity_drift():
    module = _load_harness()

    bad35 = _a35_report()
    bad35["summary"]["behavior_projection_equal"] = False
    with pytest.raises(ValueError, match="behavior projection"):
        module.join_frontier_evidence(bad35, _a37_report())

    bad37 = _a37_report()
    bad37["summary"]["frontier_candidate_count"] = 3
    with pytest.raises(ValueError, match="frontier"):
        module.join_frontier_evidence(_a35_report(), bad37)


@pytest.mark.parametrize("key", ["route_ready", "reachable_from_start", "optimal"])
def test_a38_forbidden_semantic_keys_fail_closed_recursively(key):
    module = _load_harness()
    report = module.build_evidence_join_report(_a35_report(), _a37_report())
    report["frontier_records"][0]["a35_g3_evidence"][key] = True
    with pytest.raises(ValueError, match=key):
        module.assert_no_forbidden_semantic_keys(report)
