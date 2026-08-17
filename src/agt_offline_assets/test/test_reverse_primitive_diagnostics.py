import importlib
import math
from dataclasses import fields, replace
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

import agt_offline_assets.reverse_primitive_connector as r6b
from agt_offline_assets.forward_connector_navigation_gate import GridPathEvidence
from agt_offline_assets.navigation_grid import NavigationGridEvidence
from agt_offline_assets.navigation_map_derivation import FREE
from agt_offline_assets.turn_zones import TurnZone, TurnZoneSet
from agt_offline_assets.vehicle_profile import CanonicalVehicleProfile
from agt_offline_assets.vehicle_feasible_motion_graph import (
    A1_CENTERLINE_DIRECTIONAL_REVALIDATION,
    BOUNDED_REVERSE_PRIMITIVE_SEARCH,
    BOUNDED_SEARCH_NO_SOLUTION,
    EXECUTABLE,
    LOCAL_MOTION_EXECUTABLE,
    MOTION_EVIDENCE_ONLY,
    REJECTED,
    MotionGraphDiagnostics,
    MotionSample,
    ServiceActionValidation,
    TransitionValidation,
    VehicleFeasibleMotionGraph,
    VehicleFeasibleMotionGraphConfig,
)
from agt_offline_assets.vehicle_feasible_service_graph import (
    HEADLAND_TOPOLOGY_CANDIDATE,
    REQUIRES_A3_CONNECTOR_VALIDATION,
    SERVICE_HIGH_TO_LOW,
    SERVICE_LOW_TO_HIGH,
    TOPOLOGY_CANDIDATE_ONLY,
    TOPOLOGY_SERVICE_CANDIDATE,
    ConnectorCandidate,
    ServiceGraphDiagnostics,
    ServiceResource,
    ServiceState,
    VehicleFeasibleServiceGraph,
)


EXPECTED_FAILURE_CLASSES = {
    "SEARCH_ENVELOPE_LIMITED",
    "SITE_BOUNDARY_LIMITED",
    "FOOTPRINT_GRID_LIMITED",
    "STATE_DISCRETIZATION_OR_DOMINANCE_LIMITED",
    "GOAL_CONNECTION_LIMITED",
    "MOTION_PRIMITIVE_LIMITED",
    "PATH_OR_CUSP_ENVELOPE_LIMITED",
    "MIXED_LIMITATION",
    "INCONCLUSIVE",
}


def _a35_api():
    diagnostics_type = getattr(r6b, "ReversePrimitiveSearchDiagnostics", None)
    classifier = getattr(r6b, "classify_reverse_primitive_failure", None)
    to_dict = getattr(r6b, "reverse_primitive_search_diagnostics_to_dict", None)
    from_dict = getattr(r6b, "reverse_primitive_search_diagnostics_from_dict", None)
    assert diagnostics_type is not None, "missing ReversePrimitiveSearchDiagnostics"
    assert classifier is not None, "missing classify_reverse_primitive_failure"
    assert to_dict is not None, "missing diagnostic serializer"
    assert from_dict is not None, "missing diagnostic strict loader"
    return diagnostics_type, classifier, to_dict, from_dict


def _diag(**overrides):
    diagnostics_type, _classifier, _to_dict, _from_dict = _a35_api()
    return diagnostics_type(**overrides)


def _valid_diag(**overrides):
    values = dict(
        failure_class="SEARCH_ENVELOPE_LIMITED",
        search_started=True,
        search_expansions=2,
        queue_exhausted=True,
        expansion_budget_reached=False,
        start_goal_position_error_m=2.0,
        best_goal_position_error_m=1.0,
        best_goal_yaw_error_rad=0.2,
        best_goal_distance_state_direction="FORWARD",
        best_goal_distance_cusp_count=1,
        nodes_popped=2,
        stale_nodes_skipped=0,
        cusp_switches_considered=1,
        cusp_switches_rejected_state_dominance=0,
        cusp_switches_enqueued=1,
        nodes_at_max_cusps=0,
        primitive_edges_considered=10,
        primitive_edges_rejected_search_envelope=6,
        primitive_edges_rejected_site_boundary=0,
        primitive_edges_rejected_navigation_grid=0,
        primitive_edges_rejected_path_length=0,
        primitive_edges_rejected_state_dominance=0,
        primitive_edges_enqueued=4,
        goal_tolerance_checks=2,
        goal_tolerance_successes=0,
        goal_shot_attempts=0,
        goal_shot_reverse_direction_blocked=0,
        goal_shot_candidates_considered=0,
        goal_shot_rejected_path_length=0,
        goal_shot_rejected_search_envelope=0,
        goal_shot_rejected_site_boundary=0,
        goal_shot_rejected_navigation_grid=0,
        goal_shot_successes=0,
    )
    values.update(overrides)
    return _diag(**values)


def test_a35_diagnostic_schema_and_failure_classes_exist():
    item = _diag()
    assert item.schema == "agt_r6b_connector_diagnostics/v1"
    actual = {
        getattr(r6b, "SEARCH_ENVELOPE_LIMITED", None),
        getattr(r6b, "SITE_BOUNDARY_LIMITED", None),
        getattr(r6b, "FOOTPRINT_GRID_LIMITED", None),
        getattr(r6b, "STATE_DISCRETIZATION_OR_DOMINANCE_LIMITED", None),
        getattr(r6b, "GOAL_CONNECTION_LIMITED", None),
        getattr(r6b, "MOTION_PRIMITIVE_LIMITED", None),
        getattr(r6b, "PATH_OR_CUSP_ENVELOPE_LIMITED", None),
        getattr(r6b, "MIXED_LIMITATION", None),
        getattr(r6b, "INCONCLUSIVE", None),
    }
    assert actual == EXPECTED_FAILURE_CLASSES


@pytest.mark.parametrize(
    "diagnostics,expected",
    [
        (
            dict(
                primitive_edges_considered=100,
                primitive_edges_rejected_search_envelope=60,
                primitive_edges_enqueued=40,
            ),
            "SEARCH_ENVELOPE_LIMITED",
        ),
        (
            dict(
                primitive_edges_considered=100,
                primitive_edges_rejected_site_boundary=60,
                primitive_edges_enqueued=40,
            ),
            "SITE_BOUNDARY_LIMITED",
        ),
        (
            dict(
                primitive_edges_considered=100,
                primitive_edges_rejected_navigation_grid=60,
                primitive_edges_enqueued=40,
            ),
            "FOOTPRINT_GRID_LIMITED",
        ),
        (
            dict(
                primitive_edges_considered=100,
                primitive_edges_rejected_path_length=60,
                primitive_edges_enqueued=40,
            ),
            "PATH_OR_CUSP_ENVELOPE_LIMITED",
        ),
        (
            dict(
                primitive_edges_considered=100,
                primitive_edges_rejected_state_dominance=70,
                primitive_edges_rejected_navigation_grid=20,
                primitive_edges_enqueued=10,
            ),
            "STATE_DISCRETIZATION_OR_DOMINANCE_LIMITED",
        ),
        (
            dict(
                best_goal_position_error_m=0.50,
                best_goal_yaw_error_rad=0.30,
                goal_shot_reverse_direction_blocked=4,
            ),
            "GOAL_CONNECTION_LIMITED",
        ),
        (
            dict(
                search_started=True,
                queue_exhausted=True,
                start_goal_position_error_m=10.0,
                best_goal_position_error_m=8.5,
                primitive_edges_considered=100,
                primitive_edges_rejected_state_dominance=20,
                primitive_edges_enqueued=80,
            ),
            "MOTION_PRIMITIVE_LIMITED",
        ),
        (
            dict(
                primitive_edges_considered=100,
                primitive_edges_rejected_search_envelope=30,
                primitive_edges_rejected_site_boundary=30,
                primitive_edges_enqueued=40,
            ),
            "MIXED_LIMITATION",
        ),
        (
            dict(
                search_started=True,
                queue_exhausted=True,
                start_goal_position_error_m=10.0,
                best_goal_position_error_m=5.0,
                primitive_edges_considered=100,
                primitive_edges_rejected_state_dominance=50,
                primitive_edges_enqueued=50,
            ),
            "INCONCLUSIVE",
        ),
    ],
)
def test_a35_classifier_uses_all_frozen_failure_classes(diagnostics, expected):
    diagnostics_type, classifier, _to_dict, _from_dict = _a35_api()
    item = diagnostics_type(**diagnostics)
    assert classifier(item, max_cusps=2) == expected


def test_a35_dominant_fraction_is_inclusive_at_half():
    diagnostics_type, classifier, _to_dict, _from_dict = _a35_api()
    item = diagnostics_type(
        primitive_edges_considered=100,
        primitive_edges_rejected_search_envelope=50,
        primitive_edges_enqueued=50,
    )
    assert classifier(item, max_cusps=2) == "SEARCH_ENVELOPE_LIMITED"


def test_a35_lone_moderate_fraction_at_quarter_is_not_sufficient():
    diagnostics_type, classifier, _to_dict, _from_dict = _a35_api()
    item = diagnostics_type(
        primitive_edges_considered=100,
        primitive_edges_rejected_search_envelope=25,
        primitive_edges_enqueued=75,
    )
    assert classifier(item, max_cusps=2) == "INCONCLUSIVE"


def test_a35_state_dominance_thresholds_are_inclusive():
    diagnostics_type, classifier, _to_dict, _from_dict = _a35_api()
    item = diagnostics_type(
        primitive_edges_considered=100,
        primitive_edges_rejected_state_dominance=60,
        primitive_edges_enqueued=15,
    )
    assert classifier(item, max_cusps=2) == "STATE_DISCRETIZATION_OR_DOMINANCE_LIMITED"


def test_a35_cusp_saturation_threshold_is_inclusive():
    diagnostics_type, classifier, _to_dict, _from_dict = _a35_api()
    item = diagnostics_type(
        search_expansions=10,
        nodes_at_max_cusps=5,
        best_goal_distance_cusp_count=2,
    )
    assert classifier(item, max_cusps=2) == "PATH_OR_CUSP_ENVELOPE_LIMITED"


def test_a35_little_progress_threshold_is_inclusive():
    diagnostics_type, classifier, _to_dict, _from_dict = _a35_api()
    item = diagnostics_type(
        search_started=True,
        queue_exhausted=True,
        start_goal_position_error_m=10.0,
        best_goal_position_error_m=8.0,
        primitive_edges_considered=100,
        primitive_edges_rejected_state_dominance=20,
        primitive_edges_enqueued=80,
    )
    assert classifier(item, max_cusps=2) == "MOTION_PRIMITIVE_LIMITED"


def test_a35_diagnostic_round_trip_is_exact():
    _diagnostics_type, _classifier, to_dict, from_dict = _a35_api()
    item = _valid_diag()
    assert from_dict(to_dict(item)) == item


@pytest.mark.parametrize(
    "field_name,bad_value",
    [
        ("nodes_popped", -1),
        ("primitive_edges_considered", -1),
        ("goal_shot_candidates_considered", -1),
    ],
)
def test_a35_diagnostic_loader_rejects_negative_counters(field_name, bad_value):
    _diagnostics_type, _classifier, to_dict, from_dict = _a35_api()
    raw = to_dict(_valid_diag())
    raw[field_name] = bad_value
    with pytest.raises(ValueError):
        from_dict(raw)


def test_a35_diagnostic_loader_rejects_wrong_schema():
    _diagnostics_type, _classifier, to_dict, from_dict = _a35_api()
    raw = to_dict(_valid_diag())
    raw["schema"] = "wrong/v0"
    with pytest.raises(ValueError, match="schema"):
        from_dict(raw)


def test_a35_diagnostic_loader_rejects_unknown_failure_class():
    _diagnostics_type, _classifier, to_dict, from_dict = _a35_api()
    raw = to_dict(_valid_diag())
    raw["failure_class"] = "NOT_A_REAL_CLASS"
    with pytest.raises(ValueError, match="failure"):
        from_dict(raw)


def test_a35_diagnostic_loader_rejects_unknown_direction():
    _diagnostics_type, _classifier, to_dict, from_dict = _a35_api()
    raw = to_dict(_valid_diag())
    raw["best_goal_distance_state_direction"] = "SIDEWAYS"
    with pytest.raises(ValueError, match="direction"):
        from_dict(raw)


@pytest.mark.parametrize("bad_value", [float("nan"), float("inf"), -0.1])
def test_a35_diagnostic_loader_rejects_invalid_goal_errors(bad_value):
    _diagnostics_type, _classifier, to_dict, from_dict = _a35_api()
    raw = to_dict(_valid_diag())
    raw["best_goal_position_error_m"] = bad_value
    with pytest.raises(ValueError):
        from_dict(raw)


def test_a35_diagnostic_loader_rejects_both_termination_flags():
    _diagnostics_type, _classifier, to_dict, from_dict = _a35_api()
    raw = to_dict(_valid_diag())
    raw["expansion_budget_reached"] = True
    with pytest.raises(ValueError, match="termination|queue|budget"):
        from_dict(raw)


@pytest.mark.parametrize(
    "field_name",
    [
        "nodes_popped",
        "cusp_switches_considered",
        "primitive_edges_considered",
        "goal_shot_candidates_considered",
    ],
)
def test_a35_diagnostic_loader_rejects_each_broken_accounting_identity(field_name):
    _diagnostics_type, _classifier, to_dict, from_dict = _a35_api()
    raw = to_dict(_valid_diag())
    raw[field_name] += 1
    with pytest.raises(ValueError, match="account|mismatch|identity"):
        from_dict(raw)


def _vehicle():
    footprint = ((0.30, 0.20), (0.30, -0.20), (-0.30, -0.20), (-0.30, 0.20))
    return CanonicalVehicleProfile(
        profile_id="mk_mini",
        profile_path="fixture.yaml",
        profile_sha256="fixture-sha256",
        kinematics="ackermann",
        footprint_frame="base_link",
        base_frame="base_link",
        physical_length_m=0.60,
        physical_width_m=0.40,
        footprint_xy=footprint,
        navigation_footprint_xy=footprint,
        navigation_width_m=0.40,
        navigation_length_m=0.60,
        wheel_base_m=0.60,
        track_width_m=0.40,
        wheel_diameter_m=0.24,
        ground_clearance_m=0.11,
        minimum_turning_radius_m=1.00,
        minimum_turning_radius_verified=True,
        maximum_steering_angle_rad=0.54,
        maximum_steering_angle_deg=31.0,
        allow_in_place_rotation=False,
        max_forward_velocity_mps=1.0,
        max_reverse_velocity_mps=0.5,
        max_angular_velocity_rps=1.0,
        manufacturer_maximum_speed_mps=1.0,
        route_acceptance_enabled=False,
        preview_planning_enabled=True,
        blocked_reason="",
    )


def _navigation():
    return NavigationGridEvidence(
        resolution_m=0.25,
        origin_x_m=-2.0,
        origin_y_m=-4.0,
        width=40,
        height=32,
        occupancy=np.full((32, 40), FREE, dtype=np.uint8),
        frame_id="map",
        source={},
    )


def _resource(segment_id, y):
    aisle_id = segment_id.split(".")[0]
    return ServiceResource(
        segment_id=segment_id,
        aisle_id=aisle_id,
        ordinal_in_aisle=1,
        coverage_length_m=4.0,
        coverage_fraction_of_aisle=1.0,
        low_endpoint_type="LOW_U_HEADLAND",
        high_endpoint_type="HIGH_U_HEADLAND",
        low_endpoint_pose=(0.0, y, 0.0, 0.0),
        high_endpoint_pose=(4.0, y, 0.0, 0.0),
        centerline_xyz=((0.0, y, 0.0), (2.0, y, 0.0), (4.0, y, 0.0)),
    )


def _service_state(resource, service_type):
    if service_type == SERVICE_LOW_TO_HIGH:
        return ServiceState(
            service_state_id=f"{resource.segment_id}.service_low_to_high",
            segment_id=resource.segment_id,
            aisle_id=resource.aisle_id,
            service_type=service_type,
            entry_endpoint_type=resource.low_endpoint_type,
            exit_endpoint_type=resource.high_endpoint_type,
            entry_pose=resource.low_endpoint_pose,
            exit_pose=resource.high_endpoint_pose,
            service_motion_direction="FORWARD",
            forward_service_distance_m=4.0,
            reverse_service_distance_m=0.0,
            coverage_segment_id=resource.segment_id,
            coverage_reward_length_m=4.0,
            external_reachability_status=HEADLAND_TOPOLOGY_CANDIDATE,
            validation_status=TOPOLOGY_SERVICE_CANDIDATE,
        )
    return ServiceState(
        service_state_id=f"{resource.segment_id}.service_high_to_low",
        segment_id=resource.segment_id,
        aisle_id=resource.aisle_id,
        service_type=service_type,
        entry_endpoint_type=resource.high_endpoint_type,
        exit_endpoint_type=resource.low_endpoint_type,
        entry_pose=(4.0, resource.high_endpoint_pose[1], 0.0, -math.pi),
        exit_pose=(0.0, resource.low_endpoint_pose[1], 0.0, -math.pi),
        service_motion_direction="FORWARD",
        forward_service_distance_m=4.0,
        reverse_service_distance_m=0.0,
        coverage_segment_id=resource.segment_id,
        coverage_reward_length_m=4.0,
        external_reachability_status=HEADLAND_TOPOLOGY_CANDIDATE,
        validation_status=TOPOLOGY_SERVICE_CANDIDATE,
    )


def _service_graph():
    a = _resource("aisle_001.segment_001", 0.0)
    b = _resource("aisle_002.segment_001", 2.0)
    a_state = _service_state(a, SERVICE_LOW_TO_HIGH)
    b_state = _service_state(b, SERVICE_HIGH_TO_LOW)
    connector = ConnectorCandidate(
        connector_candidate_id="headland.turn_high_u.a_to_b",
        from_service_state_id=a_state.service_state_id,
        to_service_state_id=b_state.service_state_id,
        from_segment_id=a.segment_id,
        to_segment_id=b.segment_id,
        side="HIGH_U",
        turn_zone_id="turn_high_u",
        start_pose=a_state.exit_pose,
        goal_pose=b_state.entry_pose,
        validation_status=REQUIRES_A3_CONNECTOR_VALIDATION,
    )
    diagnostics = ServiceGraphDiagnostics(
        resource_count=2,
        ordinary_service_state_count=2,
        dead_end_candidate_count=0,
        total_service_state_count=2,
        connector_candidate_count=1,
        low_u_connector_candidate_count=0,
        high_u_connector_candidate_count=1,
        candidate_component_count=0,
        isolated_component_count=0,
        externally_unproven_state_count=0,
        unique_coverage_length_m=8.0,
        distinct_aisle_count=2,
    )
    return VehicleFeasibleServiceGraph(
        frame_id="map",
        platform_id="mk_mini",
        platform_profile_sha256="fixture-sha256",
        row_direction_xy=(1.0, 0.0),
        service_resources=(a, b),
        service_states=(a_state, b_state),
        connector_candidates=(connector,),
        candidate_components=(),
        diagnostics=diagnostics,
        source={},
        status=TOPOLOGY_CANDIDATE_ONLY,
    )


def _zones():
    return TurnZoneSet(
        frame_id="map",
        row_direction_xy=(1.0, 0.0),
        zones=(
            TurnZone(
                zone_id="turn_high_u",
                side="HIGH_U",
                polygon_xy=((3.0, -2.5), (6.0, -2.5), (6.0, 2.5), (3.0, 2.5)),
                supported_aisle_ids=("aisle_001", "aisle_002"),
                endpoint_count=2,
                free_fraction=1.0,
                allow_turn=True,
            ),
        ),
        source={},
    )


def _forward_gate_plan(status):
    result = SimpleNamespace(
        connector_id="headland.turn_high_u.a_to_b",
        status=status,
        path_type=None,
        length_m=None,
        samples=(),
        centerline_evidence=SimpleNamespace(),
        footprint_evidence=SimpleNamespace(),
        reason=status,
    )
    return SimpleNamespace(connectors=(result,))


def _audit_plan(status="LOCAL_FORWARD_OCCUPANCY_BLOCKED"):
    result = SimpleNamespace(
        connector_id="headland.turn_high_u.a_to_b",
        status=status,
        reason=status,
        start_endpoint_site_boundary_free=True,
        goal_endpoint_site_boundary_free=True,
    )
    return SimpleNamespace(connectors=(result,))


def _admission():
    item = SimpleNamespace(
        connector_id="headland.turn_high_u.a_to_b",
        decision="ELIGIBLE_REVERSE_FALLBACK",
        reason="fixture admission",
    )
    return SimpleNamespace(
        items=(item,),
        eligible_connector_ids=("headland.turn_high_u.a_to_b",),
    )


def _reverse_plan(status, diagnostics):
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
            x=4.0,
            y=2.0,
            z=0.0,
            yaw=math.pi,
            motion_direction="FORWARD",
            segment_index=1,
            is_cusp=False,
        ),
    )
    solved = status == "REVERSE_PRIMITIVE_PREVIEW_FREE"
    result = SimpleNamespace(
        connector_id="headland.turn_high_u.a_to_b",
        status=status,
        path_length_m=5.5 if solved else None,
        forward_distance_m=3.0 if solved else 0.0,
        reverse_distance_m=2.5 if solved else 0.0,
        cusp_count=0,
        search_expansions=321,
        samples=samples if solved else (),
        centerline_evidence=SimpleNamespace(),
        footprint_evidence=SimpleNamespace(
            occupied_fraction=0.0,
            unknown_fraction=0.0,
            grid_coverage_fraction=1.0,
        ),
        reason=status,
        diagnostics=diagnostics,
    )
    return SimpleNamespace(connectors=(result,))


def _install_transition_pipeline(api, monkeypatch, reverse_status, diagnostics):
    monkeypatch.setattr(
        api,
        "derive_forward_connector_navigation_gate",
        lambda *args, **kwargs: _forward_gate_plan("NO_FORWARD_PREVIEW_FREE_CANDIDATE"),
    )
    monkeypatch.setattr(
        api,
        "derive_forward_connector_candidate_audit",
        lambda *args, **kwargs: _audit_plan(),
    )
    monkeypatch.setattr(
        api,
        "derive_reverse_fallback_admission",
        lambda *args, **kwargs: _admission(),
    )
    monkeypatch.setattr(
        api,
        "derive_reverse_primitive_connector_plan",
        lambda *args, **kwargs: _reverse_plan(reverse_status, diagnostics),
    )


def test_a35_transition_preserves_r6b_diagnostics_without_changing_failure(monkeypatch):
    api = importlib.import_module("agt_offline_assets.vehicle_feasible_transition_motion")
    diagnostics = _valid_diag(search_expansions=321, nodes_popped=321, goal_tolerance_checks=321)
    _install_transition_pipeline(
        api,
        monkeypatch,
        "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION",
        diagnostics,
    )
    result = api.validate_transition_candidates(
        _service_graph(),
        _zones(),
        _navigation(),
        _vehicle(),
        VehicleFeasibleMotionGraphConfig(),
    )[0]
    assert result.status == REJECTED
    assert result.proof_scope == BOUNDED_SEARCH_NO_SOLUTION
    assert result.backend == BOUNDED_REVERSE_PRIMITIVE_SEARCH
    assert result.backend_status == "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION"
    assert result.search_expansions == 321
    assert result.reverse_backend_diagnostics["queue_exhausted"] is True
    assert result.reverse_backend_diagnostics["failure_class"] == "SEARCH_ENVELOPE_LIMITED"


def test_a35_transition_preserves_diagnostics_on_r6b_success(monkeypatch):
    api = importlib.import_module("agt_offline_assets.vehicle_feasible_transition_motion")
    diagnostics = _valid_diag(
        failure_class=None,
        queue_exhausted=False,
        search_expansions=321,
        nodes_popped=321,
        goal_tolerance_checks=321,
    )
    _install_transition_pipeline(
        api,
        monkeypatch,
        "REVERSE_PRIMITIVE_PREVIEW_FREE",
        diagnostics,
    )
    result = api.validate_transition_candidates(
        _service_graph(),
        _zones(),
        _navigation(),
        _vehicle(),
        VehicleFeasibleMotionGraphConfig(),
    )[0]
    assert result.status == EXECUTABLE
    assert result.proof_scope == LOCAL_MOTION_EXECUTABLE
    assert result.backend == BOUNDED_REVERSE_PRIMITIVE_SEARCH
    assert result.backend_status == "REVERSE_PRIMITIVE_PREVIEW_FREE"
    assert result.search_expansions == 321
    assert result.reverse_backend_diagnostics["failure_class"] is None


def _grid_evidence():
    return GridPathEvidence(
        cell_count=1,
        free_count=1,
        occupied_count=0,
        unknown_count=0,
        free_fraction=1.0,
        occupied_fraction=0.0,
        unknown_fraction=0.0,
        grid_coverage_fraction=1.0,
    )


def _motion_service(state_id, segment_id, aisle_id):
    return ServiceActionValidation(
        service_state_id=state_id,
        segment_id=segment_id,
        aisle_id=aisle_id,
        service_type=SERVICE_LOW_TO_HIGH,
        entry_pose=(0.0, 0.0, 0.0, 0.0),
        exit_pose=(1.0, 0.0, 0.0, 0.0),
        status=EXECUTABLE,
        proof_scope=LOCAL_MOTION_EXECUTABLE,
        backend=A1_CENTERLINE_DIRECTIONAL_REVALIDATION,
        backend_status="CENTERLINE_DIRECTIONAL_PREVIEW_FREE",
        reason="fixture",
        coverage_segment_id=segment_id,
        coverage_reward_length_m=1.0,
        path_length_m=1.0,
        forward_distance_m=1.0,
        reverse_distance_m=0.0,
        cusp_count=0,
        samples=(
            MotionSample(
                x=0.0,
                y=0.0,
                z=0.0,
                yaw=0.0,
                motion_direction="FORWARD",
                segment_index=0,
                is_cusp=False,
            ),
        ),
        footprint_evidence=_grid_evidence(),
    )


def _old_transition():
    return TransitionValidation(
        connector_candidate_id="connector.a_to_b",
        from_service_state_id="service.a",
        to_service_state_id="service.b",
        from_segment_id="segment.a",
        to_segment_id="segment.b",
        side="HIGH_U",
        turn_zone_id="turn_high_u",
        start_pose=(1.0, 0.0, 0.0, 0.0),
        goal_pose=(1.0, 1.0, 0.0, math.pi),
        status=REJECTED,
        proof_scope=BOUNDED_SEARCH_NO_SOLUTION,
        backend=BOUNDED_REVERSE_PRIMITIVE_SEARCH,
        backend_status="NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION",
        reason="bounded fixture",
        path_length_m=0.0,
        forward_distance_m=0.0,
        reverse_distance_m=0.0,
        cusp_count=0,
        search_expansions=321,
        samples=(),
        forward_evidence={"audit_status": "LOCAL_FORWARD_OCCUPANCY_BLOCKED"},
        reverse_admission_evidence={"decision": "ELIGIBLE_REVERSE_FALLBACK"},
    )


def _motion_graph(transition):
    services = (
        _motion_service("service.a", "segment.a", "aisle.a"),
        _motion_service("service.b", "segment.b", "aisle.b"),
    )
    diagnostics = MotionGraphDiagnostics(
        a2_service_state_count=2,
        service_validation_count=2,
        executable_service_action_count=2,
        rejected_service_action_count=0,
        unresolved_service_action_count=0,
        ordinary_service_action_count=2,
        dead_end_service_action_count=0,
        executable_dead_end_service_action_count=0,
        a2_connector_candidate_count=1,
        transition_validation_count=1,
        executable_transition_count=0,
        forward_executable_transition_count=0,
        reverse_executable_transition_count=0,
        rejected_transition_count=1,
        unresolved_transition_count=0,
        locally_validated_segment_count=2,
        locally_validated_unique_coverage_length_m=2.0,
        distinct_locally_validated_aisle_count=2,
    )
    return VehicleFeasibleMotionGraph(
        frame_id="map",
        platform_id="mk_mini",
        platform_profile_sha256="fixture-sha256",
        row_direction_xy=(1.0, 0.0),
        service_actions=services,
        transition_validations=(transition,),
        executable_service_action_ids=("service.a", "service.b"),
        executable_transition_ids=(),
        diagnostics=diagnostics,
        source={"fixture": "a35"},
        status=MOTION_EVIDENCE_ONLY,
    )


def _diagnostic_mapping():
    _diagnostics_type, _classifier, to_dict, _from_dict = _a35_api()
    return to_dict(_valid_diag(search_expansions=321, nodes_popped=321, goal_tolerance_checks=321))


def test_a35_transition_contract_appends_optional_diagnostic_mapping():
    names = {item.name for item in fields(TransitionValidation)}
    assert "reverse_backend_diagnostics" in names
    transition = replace(
        _old_transition(),
        reverse_backend_diagnostics=_diagnostic_mapping(),
    )
    assert transition.status == REJECTED
    assert transition.search_expansions == 321
    assert transition.reverse_backend_diagnostics["schema"] == "agt_r6b_connector_diagnostics/v1"


def test_a35_motion_graph_io_round_trips_diagnostics_byte_stably(tmp_path):
    io = importlib.import_module("agt_offline_assets.vehicle_feasible_motion_graph_io")
    transition = replace(
        _old_transition(),
        reverse_backend_diagnostics=_diagnostic_mapping(),
    )
    graph = _motion_graph(transition)
    first = tmp_path / "first.yaml"
    second = tmp_path / "second.yaml"
    io.write_vehicle_feasible_motion_graph(graph, first)
    loaded = io.load_vehicle_feasible_motion_graph(first)
    assert loaded.transition_validations[0].reverse_backend_diagnostics == _diagnostic_mapping()
    io.write_vehicle_feasible_motion_graph(loaded, second)
    assert first.read_bytes() == second.read_bytes()


def test_a35_motion_graph_loader_keeps_pre_a35_v1_yaml_backward_compatible(tmp_path):
    io = importlib.import_module("agt_offline_assets.vehicle_feasible_motion_graph_io")
    graph = _motion_graph(_old_transition())
    payload = io.vehicle_feasible_motion_graph_to_dict(graph)
    payload["transition_validations"][0].pop("reverse_backend_diagnostics", None)
    path = tmp_path / "old-v1.yaml"
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    loaded = io.load_vehicle_feasible_motion_graph(path)
    assert loaded.transition_validations[0].reverse_backend_diagnostics == {}


def test_a35_motion_graph_loader_rejects_forbidden_key_inside_diagnostics(tmp_path):
    io = importlib.import_module("agt_offline_assets.vehicle_feasible_motion_graph_io")
    payload = io.vehicle_feasible_motion_graph_to_dict(_motion_graph(_old_transition()))
    payload["transition_validations"][0]["reverse_backend_diagnostics"] = {
        "route_ready": True
    }
    path = tmp_path / "forbidden.yaml"
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    with pytest.raises(ValueError, match="route_ready"):
        io.load_vehicle_feasible_motion_graph(path)


def test_a35_motion_graph_loader_rejects_malformed_diagnostic_accounting(tmp_path):
    io = importlib.import_module("agt_offline_assets.vehicle_feasible_motion_graph_io")
    payload = io.vehicle_feasible_motion_graph_to_dict(_motion_graph(_old_transition()))
    diagnostics = _diagnostic_mapping()
    diagnostics["nodes_popped"] += 1
    payload["transition_validations"][0]["reverse_backend_diagnostics"] = diagnostics
    path = tmp_path / "bad-accounting.yaml"
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    with pytest.raises(ValueError, match="account|mismatch|identity"):
        io.load_vehicle_feasible_motion_graph(path)
