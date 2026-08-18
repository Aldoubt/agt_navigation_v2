import copy
import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
HARNESS = ROOT / "tools" / "v25_12g_a37_chain_frontier_diagnosis.py"


def _load_harness():
    assert HARNESS.is_file(), f"missing A3.7 chain-frontier harness: {HARNESS}"
    spec = importlib.util.spec_from_file_location(
        "v25_12g_a37_chain_frontier_diagnosis",
        HARNESS,
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _service(service_id, segment_id, aisle_id):
    return {
        "service_state_id": service_id,
        "segment_id": segment_id,
        "aisle_id": aisle_id,
        "status": "EXECUTABLE",
    }


def _transition(
    connector_id,
    from_state,
    to_state,
    from_segment,
    to_segment,
    *,
    status="REJECTED",
    failure_class="MIXED_LIMITATION",
    expansions=100,
):
    return {
        "connector_candidate_id": connector_id,
        "from_service_state_id": from_state,
        "to_service_state_id": to_state,
        "from_segment_id": from_segment,
        "to_segment_id": to_segment,
        "side": "HIGH_U",
        "turn_zone_id": "turn.high",
        "status": status,
        "backend": "BOUNDED_REVERSE_PRIMITIVE_SEARCH",
        "backend_status": (
            "REVERSE_PRIMITIVE_PREVIEW_FREE"
            if status == "EXECUTABLE"
            else "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION"
        ),
        "search_expansions": expansions,
        "forward_evidence": {"audit_status": "LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT"},
        "reverse_admission_evidence": {"decision": "ADMIT_REVERSE_FALLBACK"},
        "reverse_backend_diagnostics": {
            "failure_class": failure_class,
            "search_expansions": expansions,
            "queue_exhausted": status != "EXECUTABLE",
            "expansion_budget_reached": False,
            "primitive_edges_rejected_state_dominance": 11,
            "goal_shot_reverse_direction_blocked": 7,
        },
    }


def _baseline_payload():
    return {
        "schema": "agt_vehicle_feasible_motion_graph/v1",
        "status": "MOTION_EVIDENCE_ONLY",
        "service_actions": [
            _service("service.x", "segment.x", "aisle.1"),
            _service("service.a", "segment.a", "aisle.1"),
            _service("service.b", "segment.b", "aisle.2"),
            _service("service.c", "segment.c", "aisle.2"),
        ],
        "executable_service_action_ids": [
            "service.x",
            "service.a",
            "service.b",
            "service.c",
        ],
        "transition_validations": [
            _transition(
                "connector.xa",
                "service.x",
                "service.a",
                "segment.x",
                "segment.a",
                failure_class="GOAL_CONNECTION_LIMITED",
                expansions=121,
            ),
            _transition(
                "connector.ab",
                "service.a",
                "service.b",
                "segment.a",
                "segment.b",
                expansions=600,
            ),
            _transition(
                "connector.bc",
                "service.b",
                "service.c",
                "segment.b",
                "segment.c",
                failure_class="STATE_DISCRETIZATION_OR_DOMINANCE_LIMITED",
                expansions=30000,
            ),
        ],
        "executable_transition_ids": [],
    }


def _a36_report():
    return {
        "schema": "agt_v25_12g_a36_intermediate_curvature_acceptance/v1",
        "validation_scope": (
            "A36_INTERMEDIATE_CURVATURE_EXPERIMENT_NOT_ROUTE_READY_"
            "NOT_GLOBAL_INFEASIBILITY_PROOF"
        ),
        "r6b_connectors": [
            {
                "connector_candidate_id": "connector.xa",
                "status": "REJECTED",
                "backend_status": "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION",
                "search_expansions": 240,
                "queue_exhausted": True,
                "expansion_budget_reached": False,
                "failure_class": "GOAL_CONNECTION_LIMITED",
            },
            {
                "connector_candidate_id": "connector.ab",
                "status": "EXECUTABLE",
                "backend_status": "REVERSE_PRIMITIVE_PREVIEW_FREE",
                "search_expansions": 880,
                "queue_exhausted": False,
                "expansion_budget_reached": False,
                "failure_class": None,
            },
            {
                "connector_candidate_id": "connector.bc",
                "status": "REJECTED",
                "backend_status": "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION",
                "search_expansions": 30000,
                "queue_exhausted": False,
                "expansion_budget_reached": True,
                "failure_class": "MIXED_LIMITATION",
            },
        ],
        "summary": {
            "service_behavior_equal": True,
            "connector_universe_preserved": True,
            "protected_input_hashes_unchanged": True,
            "forbidden_semantic_key_count": 0,
            "unexpectedly_regressed_connector_ids": [],
            "a4_minimum_chain": {
                "a4_entry_gate_passed": False,
                "connector_candidate_ids": [],
                "service_state_ids": [],
                "segment_ids": [],
                "aisle_ids": [],
            },
        },
    }


def test_a37_constants_parser_and_source_are_strictly_read_only():
    module = _load_harness()
    assert module.REPORT_SCHEMA == "agt_v25_12g_a37_a4_chain_frontier_diagnosis/v1"
    assert module.VALIDATION_SCOPE == (
        "A37_A4_CHAIN_FRONTIER_DIAGNOSTIC_ONLY_NOT_PLANNER_AMENDMENT_NOT_ROUTE_READY"
    )
    assert module.A36_REPORT_SCHEMA == (
        "agt_v25_12g_a36_intermediate_curvature_acceptance/v1"
    )
    assert module.EXPECTED_CONNECTOR_COUNT == 40

    args = module.build_parser().parse_args(
        [
            "--run-dir",
            "/tmp/run",
            "--a36-report",
            "/tmp/a36.json",
        ]
    )
    assert args.baseline_motion_graph == "vehicle_feasible_motion_graph.yaml"
    assert args.pretty is False
    assert not hasattr(args, "vehicle_profile")
    assert not hasattr(args, "write_motion_graph")
    assert not hasattr(args, "output")

    source = HARNESS.read_text(encoding="utf-8")
    assert "derive_vehicle_feasible_motion_graph" not in source
    assert "derive_reverse_primitive_connector_plan" not in source


def test_a37_transition_view_overlays_a36_outcomes_without_mutating_baseline():
    module = _load_harness()
    baseline = _baseline_payload()
    baseline_before = copy.deepcopy(baseline)
    view = module.build_transition_view(baseline, _a36_report())

    assert baseline == baseline_before
    assert sorted(view) == ["connector.ab", "connector.bc", "connector.xa"]
    assert view["connector.ab"]["a36_evidence"]["status"] == "EXECUTABLE"
    assert view["connector.ab"]["from_service_state_id"] == "service.a"
    assert view["connector.ab"]["to_service_state_id"] == "service.b"


def test_a37_transition_view_keeps_a36_and_a35_diagnostics_provenance_separate():
    module = _load_harness()
    view = module.build_transition_view(_baseline_payload(), _a36_report())
    row = view["connector.bc"]

    assert row["a36_evidence"] == {
        "status": "REJECTED",
        "backend_status": "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION",
        "search_expansions": 30000,
        "queue_exhausted": False,
        "expansion_budget_reached": True,
        "failure_class": "MIXED_LIMITATION",
    }
    assert row["a35_reference"]["search_expansions"] == 30000
    assert (
        row["a35_reference"]["reverse_backend_diagnostics"]
        ["primitive_edges_rejected_state_dominance"]
        == 11
    )
    assert "primitive_edges_rejected_state_dominance" not in row["a36_evidence"]


def test_a37_finds_predecessor_and_successor_frontiers_around_existing_executable():
    module = _load_harness()
    candidates = module.find_a4_frontier_candidates(
        _baseline_payload(),
        _a36_report(),
    )
    by_id = {item["candidate_connector_id"]: item for item in candidates}

    assert sorted(by_id) == ["connector.bc", "connector.xa"]
    assert by_id["connector.xa"]["hypothetical_witnesses"][0]["position"] == (
        "PREDECESSOR_TO_EXECUTABLE"
    )
    assert by_id["connector.xa"]["hypothetical_witnesses"][0][
        "connector_candidate_ids"
    ] == ["connector.xa", "connector.ab"]
    assert by_id["connector.bc"]["hypothetical_witnesses"][0]["position"] == (
        "SUCCESSOR_FROM_EXECUTABLE"
    )
    assert by_id["connector.bc"]["hypothetical_witnesses"][0][
        "connector_candidate_ids"
    ] == ["connector.ab", "connector.bc"]


def test_a37_frontier_requires_three_distinct_segments_and_two_aisles():
    module = _load_harness()

    same_segment = _baseline_payload()
    same_segment["transition_validations"][2]["to_segment_id"] = "segment.a"
    same_segment["service_actions"][3]["segment_id"] = "segment.a"
    ids = {
        item["candidate_connector_id"]
        for item in module.find_a4_frontier_candidates(same_segment, _a36_report())
    }
    assert "connector.bc" not in ids

    one_aisle = _baseline_payload()
    for service in one_aisle["service_actions"]:
        service["aisle_id"] = "aisle.1"
    assert module.find_a4_frontier_candidates(one_aisle, _a36_report()) == []


def test_a37_frontier_report_is_deterministic_and_counts_current_outcomes():
    module = _load_harness()
    report = module.build_frontier_report(_baseline_payload(), _a36_report())
    summary = report["summary"]

    assert report["schema"] == module.REPORT_SCHEMA
    assert summary["a36_executable_connector_ids"] == ["connector.ab"]
    assert summary["a36_rejected_connector_count"] == 2
    assert summary["frontier_candidate_ids"] == ["connector.bc", "connector.xa"]
    assert summary["frontier_candidate_count"] == 2
    assert summary["frontier_expansion_budget_reached_count"] == 1
    assert summary["frontier_queue_exhausted_count"] == 1
    assert summary["current_a4_entry_gate_passed"] is False


def test_a37_rejects_a36_integrity_or_connector_universe_drift():
    module = _load_harness()
    baseline = _baseline_payload()

    bad_integrity = _a36_report()
    bad_integrity["summary"]["connector_universe_preserved"] = False
    with pytest.raises(ValueError, match="connector universe"):
        module.build_transition_view(baseline, bad_integrity)

    missing = _a36_report()
    missing["r6b_connectors"] = missing["r6b_connectors"][:-1]
    with pytest.raises(ValueError, match="connector universe"):
        module.build_transition_view(baseline, missing)


@pytest.mark.parametrize("key", ["route_ready", "reachable_from_start", "optimal"])
def test_a37_forbidden_semantic_keys_fail_closed_recursively(key):
    module = _load_harness()
    report = module.build_frontier_report(_baseline_payload(), _a36_report())
    report["frontier_candidates"][0]["a35_reference"][key] = True
    with pytest.raises(ValueError, match=key):
        module.assert_no_forbidden_semantic_keys(report)
