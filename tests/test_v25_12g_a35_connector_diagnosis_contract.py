import copy
import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
HARNESS = ROOT / "tools" / "v25_12g_a35_connector_diagnosis.py"


def _load_harness():
    assert HARNESS.is_file(), f"missing A3.5 diagnosis harness: {HARNESS}"
    spec = importlib.util.spec_from_file_location(
        "v25_12g_a35_connector_diagnosis",
        HARNESS,
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _payload():
    return {
        "schema": "agt_vehicle_feasible_motion_graph/v1",
        "status": "MOTION_EVIDENCE_ONLY",
        "frame_id": "map",
        "platform_id": "mk_mini",
        "platform_profile_sha256": "fixture-sha256",
        "row_direction_xy": [1.0, 0.0],
        "source": {"run": "baseline"},
        "diagnostics": {
            "a2_service_state_count": 2,
            "service_validation_count": 2,
            "executable_service_action_count": 2,
            "rejected_service_action_count": 0,
            "unresolved_service_action_count": 0,
            "ordinary_service_action_count": 2,
            "dead_end_service_action_count": 0,
            "executable_dead_end_service_action_count": 0,
            "a2_connector_candidate_count": 1,
            "transition_validation_count": 1,
            "executable_transition_count": 0,
            "forward_executable_transition_count": 0,
            "reverse_executable_transition_count": 0,
            "rejected_transition_count": 1,
            "unresolved_transition_count": 0,
            "locally_validated_segment_count": 2,
            "locally_validated_unique_coverage_length_m": 2.0,
            "distinct_locally_validated_aisle_count": 2,
        },
        "service_actions": [
            {
                "service_state_id": "service.a",
                "segment_id": "segment.a",
                "status": "EXECUTABLE",
            },
            {
                "service_state_id": "service.b",
                "segment_id": "segment.b",
                "status": "EXECUTABLE",
            },
        ],
        "transition_validations": [
            {
                "connector_candidate_id": "connector.a_to_b",
                "from_service_state_id": "service.a",
                "to_service_state_id": "service.b",
                "from_segment_id": "segment.a",
                "to_segment_id": "segment.b",
                "side": "HIGH_U",
                "turn_zone_id": "turn_high_u",
                "start_pose": [1.0, 0.0, 0.0, 0.0],
                "goal_pose": [1.0, 1.0, 0.0, 3.141592653589793],
                "status": "REJECTED",
                "proof_scope": "BOUNDED_SEARCH_NO_SOLUTION",
                "backend": "BOUNDED_REVERSE_PRIMITIVE_SEARCH",
                "backend_status": "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION",
                "reason": "bounded fixture",
                "path_length_m": 0.0,
                "forward_distance_m": 0.0,
                "reverse_distance_m": 0.0,
                "cusp_count": 0,
                "search_expansions": 321,
                "samples": [],
                "forward_evidence": {
                    "audit_status": "LOCAL_FORWARD_OCCUPANCY_BLOCKED"
                },
                "reverse_admission_evidence": {
                    "decision": "ELIGIBLE_REVERSE_FALLBACK"
                },
                "reverse_backend_diagnostics": {
                    "schema": "agt_r6b_connector_diagnostics/v1",
                    "failure_class": "SEARCH_ENVELOPE_LIMITED",
                },
            }
        ],
        "executable_service_action_ids": ["service.a", "service.b"],
        "executable_transition_ids": [],
    }


def test_a35_harness_constants_and_parser_defaults():
    module = _load_harness()
    assert module.REPORT_SCHEMA == "agt_v25_12g_a35_connector_backend_diagnosis/v1"
    assert (
        module.VALIDATION_SCOPE
        == "A35_R6B_DIAGNOSTIC_ONLY_NOT_ROUTE_READY_NOT_GLOBAL_INFEASIBILITY_PROOF"
    )
    assert module.BASELINE_MOTION_GRAPH_ASSET == "vehicle_feasible_motion_graph.yaml"
    assert module.EXPECTED_REAL_R6B_CONNECTOR_COUNT == 40

    args = module.build_parser().parse_args(
        ["--run-dir", "/tmp/run", "--vehicle-profile", "/tmp/mk_mini.yaml"]
    )
    assert args.baseline_motion_graph == "vehicle_feasible_motion_graph.yaml"
    assert args.service_graph == "vehicle_feasible_service_graph.yaml"
    assert args.turn_zones == "turn_zones.yaml"
    assert args.navigation_map == "navigation_map.yaml"
    assert args.site_boundary == "site_boundary.yaml"
    assert args.pretty is False
    assert not hasattr(args, "write_motion_graph")
    assert not hasattr(args, "overwrite_motion_graph")


def test_a35_harness_requires_run_dir_and_vehicle_profile():
    module = _load_harness()
    with pytest.raises(SystemExit):
        module.build_parser().parse_args([])
    with pytest.raises(SystemExit):
        module.build_parser().parse_args(["--run-dir", "/tmp/run"])


def test_a35_missing_baseline_fails_before_derivation(tmp_path, monkeypatch):
    module = _load_harness()
    profile = tmp_path / "mk_mini.yaml"
    profile.write_text("fixture\n", encoding="utf-8")
    for name in (
        "vehicle_feasible_service_graph.yaml",
        "vehicle_feasible_segments.yaml",
        "turn_zones.yaml",
        "navigation_map.yaml",
        "navigation_map.pgm",
        "site_boundary.yaml",
        "derivation.yaml",
        "aisle_graph.yaml",
    ):
        (tmp_path / name).write_text("fixture\n", encoding="utf-8")
    monkeypatch.setattr(
        module,
        "derive_vehicle_feasible_motion_graph",
        lambda *args, **kwargs: pytest.fail("derivation must not run before preflight"),
    )
    with pytest.raises(FileNotFoundError, match="vehicle_feasible_motion_graph.yaml"):
        module.main(
            [
                "--run-dir",
                str(tmp_path),
                "--vehicle-profile",
                str(profile),
            ]
        )


def test_a35_behavior_projection_ignores_only_new_diagnostics_and_top_level_source():
    module = _load_harness()
    baseline = _payload()
    current = copy.deepcopy(baseline)
    current["source"] = {"run": "diagnosis"}
    current["transition_validations"][0]["reverse_backend_diagnostics"] = {
        "schema": "agt_r6b_connector_diagnostics/v1",
        "failure_class": "MIXED_LIMITATION",
        "search_expansions": 321,
    }
    assert module.behavior_projection(baseline) == module.behavior_projection(current)
    module.assert_behavior_projection_equal(baseline, current)


@pytest.mark.parametrize(
    "path,value",
    [
        (("transition_validations", 0, "status"), "EXECUTABLE"),
        (("transition_validations", 0, "backend_status"), "OTHER_STATUS"),
        (("transition_validations", 0, "search_expansions"), 322),
        (("transition_validations", 0, "path_length_m"), 1.0),
        (
            ("transition_validations", 0, "samples"),
            [
                {
                    "x": 1.0,
                    "y": 0.0,
                    "z": 0.0,
                    "yaw": 0.0,
                    "motion_direction": "FORWARD",
                    "segment_index": 0,
                    "is_cusp": False,
                }
            ],
        ),
    ],
)
def test_a35_behavior_projection_rejects_preexisting_behavior_drift(path, value):
    module = _load_harness()
    baseline = _payload()
    current = copy.deepcopy(baseline)
    target = current
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value
    with pytest.raises(ValueError, match="behavior|projection|drift"):
        module.assert_behavior_projection_equal(baseline, current)


def test_a35_connector_records_sort_deterministically():
    module = _load_harness()
    records = [
        {"connector_candidate_id": "connector.c"},
        {"connector_candidate_id": "connector.a"},
        {"connector_candidate_id": "connector.b"},
    ]
    sorted_records = module.sort_connector_records(records)
    assert [item["connector_candidate_id"] for item in sorted_records] == [
        "connector.a",
        "connector.b",
        "connector.c",
    ]


def test_a35_closest_connectors_rank_by_position_yaw_then_id_and_missing_last():
    module = _load_harness()
    records = [
        {
            "connector_candidate_id": "connector.missing",
            "best_goal_position_error_m": None,
            "best_goal_yaw_error_rad": None,
        },
        {
            "connector_candidate_id": "connector.c",
            "best_goal_position_error_m": 0.5,
            "best_goal_yaw_error_rad": 0.1,
        },
        {
            "connector_candidate_id": "connector.b",
            "best_goal_position_error_m": 0.5,
            "best_goal_yaw_error_rad": 0.1,
        },
        {
            "connector_candidate_id": "connector.a",
            "best_goal_position_error_m": 0.4,
            "best_goal_yaw_error_rad": 0.9,
        },
    ]
    ranked = module.rank_closest_connectors(records, limit=10)
    assert [item["connector_candidate_id"] for item in ranked] == [
        "connector.a",
        "connector.b",
        "connector.c",
        "connector.missing",
    ]


def test_a35_expansion_histogram_uses_frozen_bins():
    module = _load_harness()
    values = [
        0,
        9,
        10,
        99,
        100,
        499,
        500,
        999,
        1000,
        4999,
        5000,
        9999,
        10000,
        19999,
        20000,
        30000,
    ]
    assert module.build_expansion_histogram(values) == {
        "0-9": 2,
        "10-99": 2,
        "100-499": 2,
        "500-999": 2,
        "1000-4999": 2,
        "5000-9999": 2,
        "10000-19999": 2,
        "20000-30000": 2,
    }


@pytest.mark.parametrize("forbidden", ["route_ready", "reachable_from_start", "optimal"])
def test_a35_forbidden_semantics_fail_closed_at_any_depth(forbidden):
    module = _load_harness()
    payload = _payload()
    payload["transition_validations"][0]["reverse_backend_diagnostics"][forbidden] = True
    with pytest.raises(ValueError, match=forbidden):
        module.assert_no_forbidden_semantic_keys(payload)
