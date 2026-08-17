import copy
import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
HARNESS = ROOT / "tools" / "v25_12g_a36_intermediate_curvature_acceptance.py"


def _load_harness():
    assert HARNESS.is_file(), f"missing A3.6 acceptance harness: {HARNESS}"
    spec = importlib.util.spec_from_file_location(
        "v25_12g_a36_intermediate_curvature_acceptance",
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


def _transition(connector_id, from_state, to_state, from_segment, to_segment, status):
    return {
        "connector_candidate_id": connector_id,
        "from_service_state_id": from_state,
        "to_service_state_id": to_state,
        "from_segment_id": from_segment,
        "to_segment_id": to_segment,
        "status": status,
        "backend_status": (
            "REVERSE_PRIMITIVE_PREVIEW_FREE"
            if status == "EXECUTABLE"
            else "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION"
        ),
        "search_expansions": 10,
        "reverse_backend_diagnostics": {},
    }


def _payload():
    return {
        "schema": "agt_vehicle_feasible_motion_graph/v1",
        "status": "MOTION_EVIDENCE_ONLY",
        "source": {"fixture": True},
        "service_actions": [
            _service("service.a", "segment.a", "aisle.1"),
            _service("service.b", "segment.b", "aisle.2"),
            _service("service.c", "segment.c", "aisle.2"),
        ],
        "executable_service_action_ids": ["service.a", "service.b", "service.c"],
        "transition_validations": [
            _transition(
                "connector.ab",
                "service.a",
                "service.b",
                "segment.a",
                "segment.b",
                "REJECTED",
            ),
            _transition(
                "connector.bc",
                "service.b",
                "service.c",
                "segment.b",
                "segment.c",
                "REJECTED",
            ),
        ],
        "executable_transition_ids": [],
    }


def test_a36_harness_constants_and_parser_are_read_only():
    module = _load_harness()
    assert module.REPORT_SCHEMA == "agt_v25_12g_a36_intermediate_curvature_acceptance/v1"
    assert module.VALIDATION_SCOPE == (
        "A36_INTERMEDIATE_CURVATURE_EXPERIMENT_NOT_ROUTE_READY_NOT_GLOBAL_INFEASIBILITY_PROOF"
    )
    assert module.EXPECTED_CONNECTOR_COUNT == 40
    assert module.EXPECTED_SERVICE_ACTION_COUNT == 32
    args = module.build_parser().parse_args(
        ["--run-dir", "/tmp/run", "--vehicle-profile", "/tmp/mk_mini.yaml"]
    )
    assert args.baseline_motion_graph == "vehicle_feasible_motion_graph.yaml"
    assert args.pretty is False
    assert not hasattr(args, "write_motion_graph")
    assert not hasattr(args, "overwrite_motion_graph")


def test_a36_service_behavior_projection_must_remain_exact():
    module = _load_harness()
    baseline = _payload()
    current = copy.deepcopy(baseline)
    current["source"] = {"fixture": "amended"}
    current["transition_validations"][0]["status"] = "EXECUTABLE"
    current["executable_transition_ids"] = ["connector.ab"]
    module.assert_service_behavior_equal(baseline, current)

    current["service_actions"][0]["status"] = "REJECTED"
    with pytest.raises(ValueError, match="service"):
        module.assert_service_behavior_equal(baseline, current)


def test_a36_transition_comparison_is_deterministic_and_detects_changes():
    module = _load_harness()
    baseline = _payload()
    current = copy.deepcopy(baseline)
    current["transition_validations"][0]["status"] = "EXECUTABLE"
    current["transition_validations"][0]["backend_status"] = (
        "REVERSE_PRIMITIVE_PREVIEW_FREE"
    )
    current["executable_transition_ids"] = ["connector.ab"]
    result = module.compare_transition_outcomes(baseline, current)
    assert result["newly_executable_connector_ids"] == ["connector.ab"]
    assert result["still_rejected_connector_ids"] == ["connector.bc"]
    assert result["unexpectedly_regressed_connector_ids"] == []


def test_a36_a4_chain_requires_two_continuous_transitions_three_segments_two_aisles():
    module = _load_harness()
    payload = _payload()
    for transition in payload["transition_validations"]:
        transition["status"] = "EXECUTABLE"
        transition["backend_status"] = "REVERSE_PRIMITIVE_PREVIEW_FREE"
    payload["executable_transition_ids"] = ["connector.ab", "connector.bc"]
    result = module.find_a4_minimum_chain(payload)
    assert result["a4_entry_gate_passed"] is True
    assert result["connector_candidate_ids"] == ["connector.ab", "connector.bc"]
    assert result["segment_ids"] == ["segment.a", "segment.b", "segment.c"]
    assert set(result["aisle_ids"]) == {"aisle.1", "aisle.2"}


def test_a36_a4_chain_rejects_disconnected_or_single_aisle_pairs():
    module = _load_harness()
    disconnected = _payload()
    disconnected["transition_validations"][0]["status"] = "EXECUTABLE"
    disconnected["transition_validations"][1]["status"] = "EXECUTABLE"
    disconnected["transition_validations"][1]["from_service_state_id"] = "service.other"
    assert module.find_a4_minimum_chain(disconnected)["a4_entry_gate_passed"] is False

    one_aisle = _payload()
    for service in one_aisle["service_actions"]:
        service["aisle_id"] = "aisle.1"
    for transition in one_aisle["transition_validations"]:
        transition["status"] = "EXECUTABLE"
    assert module.find_a4_minimum_chain(one_aisle)["a4_entry_gate_passed"] is False


@pytest.mark.parametrize("key", ["route_ready", "reachable_from_start", "optimal"])
def test_a36_forbidden_semantic_keys_fail_closed_recursively(key):
    module = _load_harness()
    payload = _payload()
    payload["transition_validations"][0]["reverse_backend_diagnostics"][key] = True
    with pytest.raises(ValueError, match=key):
        module.assert_no_forbidden_semantic_keys(payload)
