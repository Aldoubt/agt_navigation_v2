"""A3.8 selective-six current-diagnostic RED contract.

The filename is retained because the earlier evidence-join RED was never executed.
Its contents are superseded by the approved selective A4-frontier design.
"""

import copy
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
HARNESS = ROOT / "tools" / "v25_12g_a38_selective_frontier_diagnostics.py"


def _load_harness():
    assert HARNESS.is_file(), f"missing A3.8 selective-frontier harness: {HARNESS}"
    spec = importlib.util.spec_from_file_location(
        "v25_12g_a38_selective_frontier_diagnostics",
        HARNESS,
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _connector_id(index: int) -> str:
    return f"connector.frontier.{index}"


def _a36_evidence(index: int):
    return {
        "status": "REJECTED",
        "backend_status": "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION",
        "search_expansions": 100 + index,
        "queue_exhausted": True,
        "expansion_budget_reached": False,
        "failure_class": "MIXED_LIMITATION" if index < 4 else "INCONCLUSIVE",
    }


def _frontier(index: int):
    connector_id = _connector_id(index)
    return {
        "candidate_connector_id": connector_id,
        "from_service_state_id": f"service.{index}",
        "to_service_state_id": f"service.{index + 1}",
        "from_segment_id": f"segment.{index}",
        "to_segment_id": f"segment.{index + 1}",
        "side": "HIGH_U",
        "turn_zone_id": "turn_high_u",
        "a36_evidence": _a36_evidence(index),
        "a35_reference": {
            "connector_candidate_id": connector_id,
            "search_expansions": 1,
            "reverse_backend_diagnostics": {},
        },
        "hypothetical_witnesses": [
            {
                "position": (
                    "SUCCESSOR_FROM_EXECUTABLE"
                    if index < 3
                    else "PREDECESSOR_TO_EXECUTABLE"
                ),
                "connector_candidate_ids": ["connector.existing", connector_id],
                "service_state_ids": ["service.x", f"service.{index}", f"service.{index + 1}"],
                "segment_ids": ["segment.x", f"segment.{index}", f"segment.{index + 1}"],
                "aisle_ids": ["aisle.016", "aisle.017", f"aisle.{19 + index:03d}"],
            }
        ],
    }


def _a37_report():
    frontiers = [_frontier(index) for index in range(6)]
    return {
        "schema": "agt_v25_12g_a37_a4_chain_frontier_diagnosis/v1",
        "validation_scope": (
            "A37_A4_CHAIN_FRONTIER_DIAGNOSTIC_ONLY_NOT_PLANNER_AMENDMENT_NOT_ROUTE_READY"
        ),
        "summary": {
            "a36_executable_connector_ids": [
                "connector.existing.a",
                "connector.existing.b",
            ],
            "a36_rejected_connector_count": 38,
            "frontier_candidate_ids": [_connector_id(index) for index in range(6)],
            "frontier_candidate_count": 6,
            "frontier_queue_exhausted_count": 6,
            "frontier_expansion_budget_reached_count": 0,
            "current_a4_entry_gate_passed": False,
        },
        "frontier_candidates": frontiers,
    }


def _candidate(connector_id: str):
    return SimpleNamespace(connector_candidate_id=connector_id)


def _diagnostics(index: int):
    expansions = 100 + index
    primitive_considered = 1000 + index
    state_dominance = 600 + index
    primitive_enqueued = 100
    return {
        "schema": "agt_r6b_connector_diagnostics/v1",
        "failure_class": "MIXED_LIMITATION" if index < 4 else "INCONCLUSIVE",
        "search_started": True,
        "search_expansions": expansions,
        "queue_exhausted": True,
        "expansion_budget_reached": False,
        "start_goal_position_error_m": 4.0,
        "best_goal_position_error_m": 0.10 + index * 0.01,
        "best_goal_yaw_error_rad": 0.05 + index * 0.01,
        "best_goal_distance_state_direction": "FORWARD",
        "best_goal_distance_cusp_count": 1,
        "nodes_popped": expansions,
        "stale_nodes_skipped": 0,
        "cusp_switches_considered": 20,
        "cusp_switches_rejected_state_dominance": 10,
        "cusp_switches_enqueued": 10,
        "nodes_at_max_cusps": 10,
        "primitive_edges_considered": primitive_considered,
        "primitive_edges_rejected_search_envelope": 20,
        "primitive_edges_rejected_site_boundary": 50,
        "primitive_edges_rejected_navigation_grid": 50,
        "primitive_edges_rejected_path_length": 100,
        "primitive_edges_rejected_state_dominance": state_dominance,
        "primitive_edges_enqueued": primitive_enqueued,
        "goal_tolerance_checks": 50,
        "goal_tolerance_successes": 0,
        "goal_shot_attempts": 20,
        "goal_shot_reverse_direction_blocked": 5,
        "goal_shot_candidates_considered": 10,
        "goal_shot_rejected_path_length": 2,
        "goal_shot_rejected_search_envelope": 2,
        "goal_shot_rejected_site_boundary": 2,
        "goal_shot_rejected_navigation_grid": 4,
        "goal_shot_successes": 0,
    }


def _validation(index: int):
    evidence = _a36_evidence(index)
    return SimpleNamespace(
        connector_candidate_id=_connector_id(index),
        status=evidence["status"],
        backend_status=evidence["backend_status"],
        search_expansions=evidence["search_expansions"],
        reverse_backend_diagnostics=_diagnostics(index),
    )


def _validations():
    return tuple(_validation(index) for index in range(6))


def test_a38_constants_parser_and_source_lock_selective_current_diagnostics():
    module = _load_harness()
    assert module.REPORT_SCHEMA == (
        "agt_v25_12g_a38_selective_frontier_diagnostics/v1"
    )
    assert module.VALIDATION_SCOPE == (
        "A38_SELECTIVE_A4_FRONTIER_CURRENT_DIAGNOSTIC_ONLY_NOT_PLANNER_AMENDMENT_NOT_ROUTE_READY"
    )
    assert module.A37_REPORT_SCHEMA == (
        "agt_v25_12g_a37_a4_chain_frontier_diagnosis/v1"
    )
    assert module.EXPECTED_FRONTIER_COUNT == 6

    args = module.build_parser().parse_args(
        [
            "--run-dir",
            "/tmp/run",
            "--vehicle-profile",
            "/tmp/mk_mini.yaml",
            "--a37-report",
            "/tmp/a37.json",
        ]
    )
    assert args.service_graph == "vehicle_feasible_service_graph.yaml"
    assert args.turn_zones == "turn_zones.yaml"
    assert args.navigation_map == "navigation_map.yaml"
    assert args.navigation_pgm == "navigation_map.pgm"
    assert args.site_boundary == "site_boundary.yaml"
    assert args.pretty is False
    assert not hasattr(args, "output")
    assert not hasattr(args, "connector_ids")

    source = HARNESS.read_text(encoding="utf-8")
    assert "derive_vehicle_feasible_motion_graph" not in source
    assert "derive_reverse_primitive_connector_plan" not in source
    assert "validate_transition_candidates" in source
    assert "replace(" in source


def test_a38_frontier_ids_are_exactly_six_sorted_and_a4_is_still_blocked():
    module = _load_harness()
    report = _a37_report()
    before = copy.deepcopy(report)

    ids = module.frontier_connector_ids(report)

    assert report == before
    assert ids == tuple(_connector_id(index) for index in range(6))


def test_a38_selects_only_frontier_candidates_from_full_connector_universe():
    module = _load_harness()
    all_candidates = tuple(
        [_candidate(_connector_id(index)) for index in range(6)]
        + [_candidate(f"connector.other.{index:02d}") for index in range(34)]
    )

    selected = module.select_frontier_connector_candidates(
        all_candidates,
        _a37_report(),
    )

    assert len(selected) == 6
    assert tuple(item.connector_candidate_id for item in selected) == tuple(
        _connector_id(index) for index in range(6)
    )


def test_a38_current_outcomes_must_match_saved_a36_frontier_evidence_exactly():
    module = _load_harness()
    records = module.compare_selective_outcomes(_a37_report(), _validations())

    assert len(records) == 6
    assert records[0]["candidate_connector_id"] == "connector.frontier.0"
    assert records[0]["a37_frontier"]["a36_evidence"]["search_expansions"] == 100
    assert records[0]["current_evidence"] == _a36_evidence(0)

    drifted = list(_validations())
    drifted[0] = SimpleNamespace(
        connector_candidate_id="connector.frontier.0",
        status="EXECUTABLE",
        backend_status="REVERSE_PRIMITIVE_PREVIEW_FREE",
        search_expansions=100,
        reverse_backend_diagnostics=_diagnostics(0),
    )
    with pytest.raises(ValueError, match="outcome drift"):
        module.compare_selective_outcomes(_a37_report(), tuple(drifted))


def test_a38_report_exposes_full_current_diagnostics_separate_from_a37_frontier():
    module = _load_harness()
    report = module.build_selective_report(_a37_report(), _validations())
    first = report["frontier_records"][0]

    assert first["a37_frontier"]["a36_evidence"]["search_expansions"] == 100
    assert first["current_evidence"]["search_expansions"] == 100
    assert first["current_reverse_backend_diagnostics"]["schema"] == (
        "agt_r6b_connector_diagnostics/v1"
    )
    assert first["current_reverse_backend_diagnostics"]["best_goal_position_error_m"] == 0.10
    assert first["current_reverse_backend_diagnostics"]["primitive_edges_rejected_state_dominance"] == 600
    assert first["current_reverse_backend_diagnostics"]["goal_shot_reverse_direction_blocked"] == 5
    assert "best_goal_position_error_m" not in first["a37_frontier"]["a36_evidence"]


def test_a38_report_summary_is_descriptive_and_contains_no_causal_amendment_claim():
    module = _load_harness()
    report = module.build_selective_report(_a37_report(), _validations())
    summary = report["summary"]

    assert report["schema"] == module.REPORT_SCHEMA
    assert summary["frontier_candidate_count"] == 6
    assert summary["frontier_candidate_ids"] == [
        _connector_id(index) for index in range(6)
    ]
    assert summary["queue_exhausted_count"] == 6
    assert summary["expansion_budget_reached_count"] == 0
    assert summary["failure_class_counts"] == {
        "INCONCLUSIVE": 2,
        "MIXED_LIMITATION": 4,
    }
    assert "root_cause" not in report
    assert "recommended_amendment" not in report


def test_a38_fails_closed_on_frontier_or_connector_universe_integrity_drift():
    module = _load_harness()

    bad_frontier = _a37_report()
    bad_frontier["summary"]["frontier_candidate_count"] = 5
    with pytest.raises(ValueError, match="frontier"):
        module.frontier_connector_ids(bad_frontier)

    missing_candidate = tuple(
        _candidate(_connector_id(index)) for index in range(5)
    )
    with pytest.raises(ValueError, match="missing"):
        module.select_frontier_connector_candidates(missing_candidate, _a37_report())

    duplicate_candidate = tuple(
        [_candidate(_connector_id(index)) for index in range(6)]
        + [_candidate("connector.frontier.0")]
    )
    with pytest.raises(ValueError, match="duplicate"):
        module.select_frontier_connector_candidates(duplicate_candidate, _a37_report())


@pytest.mark.parametrize("key", ["route_ready", "reachable_from_start", "optimal"])
def test_a38_forbidden_semantic_keys_fail_closed_recursively(key):
    module = _load_harness()
    report = module.build_selective_report(_a37_report(), _validations())
    report["frontier_records"][0]["current_reverse_backend_diagnostics"][key] = True
    with pytest.raises(ValueError, match=key):
        module.assert_no_forbidden_semantic_keys(report)
