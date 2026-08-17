import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

from agt_offline_assets.agricultural_coverage_ordering import ConnectorRequest
from agt_offline_assets.vehicle_feasible_motion_graph import (
    BOUNDED_REVERSE_PRIMITIVE_SEARCH,
    BOUNDED_SEARCH_NO_SOLUTION,
    EXECUTABLE,
    FORWARD_DUBINS_NAVIGATION_GATE,
    LOCAL_MOTION_EXECUTABLE,
    PROVEN_HARD_CONSTRAINT_REJECTION,
    REJECTED,
    VehicleFeasibleMotionGraphConfig,
)


CONNECTOR_ID = "headland.turn_high_u.a_to_b"
ROOT = Path(__file__).resolve().parents[1]
HARNESS = ROOT / "tools" / "v25_12g_a3_acceptance.py"


def _load_harness():
    spec = importlib.util.spec_from_file_location(
        "v25_12g_a3_acceptance_amendment_test",
        HARNESS,
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _candidate():
    return SimpleNamespace(
        connector_candidate_id=CONNECTOR_ID,
        start_pose=(4.0, 0.0, 0.0, 0.0),
        goal_pose=(4.0, 2.0, 0.0, 3.141592653589793),
    )


def _graph():
    return SimpleNamespace(
        frame_id="map",
        platform_id="mk_mini",
        platform_profile_sha256="fixture-sha256",
        connector_candidates=(_candidate(),),
    )


def _turn_zones():
    return SimpleNamespace(frame_id="map")


def _navigation():
    return SimpleNamespace(frame_id="map")


def _vehicle():
    return SimpleNamespace(
        profile_id="mk_mini",
        profile_sha256="fixture-sha256",
    )


def _request():
    return ConnectorRequest(
        connector_id=CONNECTOR_ID,
        from_aisle_id="aisle_001",
        to_aisle_id="aisle_002",
        turn_zone_id="turn_high_u",
        side="HIGH_U",
        start_pose=(4.0, 0.0, 0.0, 0.0),
        goal_pose=(4.0, 2.0, 0.0, 3.141592653589793),
    )


def _binding(api):
    return api.ConnectorBinding(
        connector_candidate_id=CONNECTOR_ID,
        from_service_state_id="aisle_001.segment_001.service_low_to_high",
        to_service_state_id="aisle_002.segment_001.service_high_to_low",
        from_segment_id="aisle_001.segment_001",
        to_segment_id="aisle_002.segment_001",
        side="HIGH_U",
        turn_zone_id="turn_high_u",
    )


def _gate_plan():
    return SimpleNamespace(
        connectors=(
            SimpleNamespace(
                connector_id=CONNECTOR_ID,
                status="NO_FORWARD_PREVIEW_FREE_CANDIDATE",
                reason="fixture gate miss",
            ),
        )
    )


def _audit_plan(
    status,
    *,
    start_boundary_free=True,
    goal_boundary_free=True,
):
    result = SimpleNamespace(
        connector_id=CONNECTOR_ID,
        status=status,
        reason=status,
        start_endpoint_site_boundary_free=start_boundary_free,
        goal_endpoint_site_boundary_free=goal_boundary_free,
    )
    return SimpleNamespace(connectors=(result,))


def _admission(decision="ELIGIBLE_REVERSE_FALLBACK"):
    item = SimpleNamespace(
        connector_id=CONNECTOR_ID,
        decision=decision,
        reason=decision,
    )
    return SimpleNamespace(
        items=(item,),
        eligible_connector_ids=(
            (CONNECTOR_ID,)
            if decision == "ELIGIBLE_REVERSE_FALLBACK"
            else ()
        ),
    )


def _reverse_plan(status):
    solved = status == "REVERSE_PRIMITIVE_PREVIEW_FREE"
    samples = (
        SimpleNamespace(
            x=4.0,
            y=0.0,
            z=0.0,
            yaw=0.0,
            motion_direction="FORWARD",
            segment_index=0,
            is_cusp=False,
        ),
        SimpleNamespace(
            x=4.5,
            y=0.5,
            z=0.0,
            yaw=0.5,
            motion_direction="REVERSE",
            segment_index=1,
            is_cusp=True,
        ),
        SimpleNamespace(
            x=4.0,
            y=2.0,
            z=0.0,
            yaw=3.141592653589793,
            motion_direction="FORWARD",
            segment_index=2,
            is_cusp=True,
        ),
    )
    result = SimpleNamespace(
        connector_id=CONNECTOR_ID,
        status=status,
        path_length_m=5.5 if solved else None,
        forward_distance_m=3.0 if solved else 0.0,
        reverse_distance_m=2.5 if solved else 0.0,
        cusp_count=2 if solved else 0,
        search_expansions=321,
        samples=samples if solved else (),
        reason=status,
    )
    return SimpleNamespace(connectors=(result,))


def _install_transition_pipeline(
    api,
    monkeypatch,
    *,
    audit_status,
    admission_decision,
    reverse_status=None,
    start_boundary_free=True,
    goal_boundary_free=True,
):
    request = _request()
    binding = _binding(api)
    monkeypatch.setattr(
        api,
        "adapt_connector_candidates",
        lambda *args, **kwargs: ((request,), {CONNECTOR_ID: binding}),
    )
    monkeypatch.setattr(
        api,
        "derive_forward_connector_navigation_gate",
        lambda *args, **kwargs: _gate_plan(),
    )
    monkeypatch.setattr(
        api,
        "derive_forward_connector_candidate_audit",
        lambda *args, **kwargs: _audit_plan(
            audit_status,
            start_boundary_free=start_boundary_free,
            goal_boundary_free=goal_boundary_free,
        ),
    )
    monkeypatch.setattr(
        api,
        "derive_reverse_fallback_admission",
        lambda *args, **kwargs: _admission(admission_decision),
    )
    if reverse_status is None:
        monkeypatch.setattr(
            api,
            "derive_reverse_primitive_connector_plan",
            lambda *args, **kwargs: pytest.fail(
                "endpoint hard conflict must not invoke R6B"
            ),
        )
    else:
        monkeypatch.setattr(
            api,
            "derive_reverse_primitive_connector_plan",
            lambda *args, **kwargs: _reverse_plan(reverse_status),
        )


def test_transition_endpoint_boundary_conflict_is_hard_rejected_without_r6b(
    monkeypatch,
):
    import agt_offline_assets.vehicle_feasible_transition_motion as api

    _install_transition_pipeline(
        api,
        monkeypatch,
        audit_status="CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT",
        admission_decision="REJECT_HARD_CONSTRAINT",
        start_boundary_free=False,
        goal_boundary_free=True,
    )
    result = api.validate_transition_candidates(
        _graph(),
        _turn_zones(),
        _navigation(),
        _vehicle(),
        VehicleFeasibleMotionGraphConfig(),
    )[0]
    assert result.status == REJECTED
    assert result.proof_scope == PROVEN_HARD_CONSTRAINT_REJECTION
    assert result.reverse_admission_evidence["decision"] == "REJECT_HARD_CONSTRAINT"
    assert result.forward_evidence["start_endpoint_site_boundary_free"] is False
    assert result.forward_evidence["goal_endpoint_site_boundary_free"] is True


def test_transition_forward_path_boundary_conflict_can_be_executable_via_r6b(
    monkeypatch,
):
    import agt_offline_assets.vehicle_feasible_transition_motion as api

    _install_transition_pipeline(
        api,
        monkeypatch,
        audit_status="LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT",
        admission_decision="ELIGIBLE_REVERSE_FALLBACK",
        reverse_status="REVERSE_PRIMITIVE_PREVIEW_FREE",
    )
    result = api.validate_transition_candidates(
        _graph(),
        _turn_zones(),
        _navigation(),
        _vehicle(),
        VehicleFeasibleMotionGraphConfig(),
    )[0]
    assert result.status == EXECUTABLE
    assert result.proof_scope == LOCAL_MOTION_EXECUTABLE
    assert result.backend == BOUNDED_REVERSE_PRIMITIVE_SEARCH
    assert result.path_length_m == pytest.approx(5.5)
    assert result.reverse_distance_m == pytest.approx(2.5)
    assert result.forward_evidence["audit_status"] == (
        "LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT"
    )
    assert result.forward_evidence["start_endpoint_site_boundary_free"] is True
    assert result.forward_evidence["goal_endpoint_site_boundary_free"] is True
    assert result.reverse_admission_evidence["decision"] == (
        "ELIGIBLE_REVERSE_FALLBACK"
    )


def test_transition_forward_path_boundary_bounded_failure_keeps_bounded_scope(
    monkeypatch,
):
    import agt_offline_assets.vehicle_feasible_transition_motion as api

    _install_transition_pipeline(
        api,
        monkeypatch,
        audit_status="LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT",
        admission_decision="ELIGIBLE_REVERSE_FALLBACK",
        reverse_status="NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION",
    )
    result = api.validate_transition_candidates(
        _graph(),
        _turn_zones(),
        _navigation(),
        _vehicle(),
        VehicleFeasibleMotionGraphConfig(),
    )[0]
    assert result.status == REJECTED
    assert result.proof_scope == BOUNDED_SEARCH_NO_SOLUTION
    assert result.backend == BOUNDED_REVERSE_PRIMITIVE_SEARCH
    assert result.backend_status == "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION"


def _diagnostics():
    return SimpleNamespace(
        a2_service_state_count=0,
        service_validation_count=0,
        executable_service_action_count=0,
        rejected_service_action_count=0,
        unresolved_service_action_count=0,
        ordinary_service_action_count=0,
        dead_end_service_action_count=0,
        executable_dead_end_service_action_count=0,
        a2_connector_candidate_count=3,
        transition_validation_count=3,
        executable_transition_count=1,
        forward_executable_transition_count=0,
        reverse_executable_transition_count=1,
        rejected_transition_count=2,
        unresolved_transition_count=0,
        locally_validated_segment_count=0,
        locally_validated_unique_coverage_length_m=0.0,
        distinct_locally_validated_aisle_count=0,
    )


def _summary_fixture(endpoint_decision="REJECT_HARD_CONSTRAINT"):
    connector_ids = ("endpoint", "path_exec", "path_fail")
    service_graph = SimpleNamespace(
        service_states=(),
        connector_candidates=tuple(
            SimpleNamespace(connector_candidate_id=value)
            for value in connector_ids
        ),
    )
    transitions = (
        SimpleNamespace(
            connector_candidate_id="endpoint",
            backend=FORWARD_DUBINS_NAVIGATION_GATE,
            backend_status="CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT",
            status=REJECTED,
            proof_scope=PROVEN_HARD_CONSTRAINT_REJECTION,
            forward_evidence={
                "audit_status": "CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT",
                "start_endpoint_site_boundary_free": False,
                "goal_endpoint_site_boundary_free": True,
            },
            reverse_admission_evidence={"decision": endpoint_decision},
        ),
        SimpleNamespace(
            connector_candidate_id="path_exec",
            backend=BOUNDED_REVERSE_PRIMITIVE_SEARCH,
            backend_status="REVERSE_PRIMITIVE_PREVIEW_FREE",
            status=EXECUTABLE,
            proof_scope=LOCAL_MOTION_EXECUTABLE,
            forward_evidence={
                "audit_status": "LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT",
                "start_endpoint_site_boundary_free": True,
                "goal_endpoint_site_boundary_free": True,
            },
            reverse_admission_evidence={
                "decision": "ELIGIBLE_REVERSE_FALLBACK"
            },
        ),
        SimpleNamespace(
            connector_candidate_id="path_fail",
            backend=BOUNDED_REVERSE_PRIMITIVE_SEARCH,
            backend_status="NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION",
            status=REJECTED,
            proof_scope=BOUNDED_SEARCH_NO_SOLUTION,
            forward_evidence={
                "audit_status": "LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT",
                "start_endpoint_site_boundary_free": True,
                "goal_endpoint_site_boundary_free": True,
            },
            reverse_admission_evidence={
                "decision": "ELIGIBLE_REVERSE_FALLBACK"
            },
        ),
    )
    motion_graph = SimpleNamespace(
        service_actions=(),
        transition_validations=transitions,
        diagnostics=_diagnostics(),
    )
    transition_records = [
        {
            "connector_candidate_id": item.connector_candidate_id,
            "status": item.status,
            "proof_scope": item.proof_scope,
            "backend": item.backend,
            "backend_status": item.backend_status,
            "forward_evidence": dict(item.forward_evidence),
            "reverse_admission_evidence": dict(item.reverse_admission_evidence),
        }
        for item in transitions
    ]
    return service_graph, motion_graph, transition_records


def test_acceptance_summary_reports_amended_boundary_admission_funnel():
    harness = _load_harness()
    service_graph, motion_graph, transition_records = _summary_fixture()
    summary = harness._summary(
        service_graph,
        motion_graph,
        [],
        transition_records,
    )
    assert summary["endpoint_boundary_conflict_count"] == 1
    assert summary["forward_path_boundary_conflict_count"] == 2
    assert summary["forward_path_boundary_reverse_admitted_count"] == 2
    assert summary["forward_path_boundary_reverse_executable_count"] == 1
    assert summary[
        "forward_path_boundary_reverse_bounded_no_solution_count"
    ] == 1
    assert summary["endpoint_boundary_conflict_reverse_admission_count"] == 0


def test_acceptance_summary_rejects_endpoint_boundary_reverse_admission():
    harness = _load_harness()
    service_graph, motion_graph, transition_records = _summary_fixture(
        endpoint_decision="ELIGIBLE_REVERSE_FALLBACK"
    )
    with pytest.raises(ValueError, match="endpoint.*boundary|boundary.*endpoint"):
        harness._summary(
            service_graph,
            motion_graph,
            [],
            transition_records,
        )