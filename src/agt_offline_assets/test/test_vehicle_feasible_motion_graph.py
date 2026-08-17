import importlib
import math
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from agt_offline_assets.navigation_grid import NavigationGridEvidence
from agt_offline_assets.navigation_map_derivation import FREE, OCCUPIED, UNKNOWN
from agt_offline_assets.site_boundary import SiteBoundary
from agt_offline_assets.turn_zones import TurnZone, TurnZoneSet
from agt_offline_assets.vehicle_profile import CanonicalVehicleProfile
from agt_offline_assets.vehicle_feasible_motion_graph import (
    A1_CENTERLINE_DIRECTIONAL_REVALIDATION,
    A1_CENTERLINE_EXACT_REVERSE_RETRACE,
    BOUNDED_REVERSE_PRIMITIVE_SEARCH,
    BOUNDED_SEARCH_NO_SOLUTION,
    EXECUTABLE,
    FORWARD_DUBINS_NAVIGATION_GATE,
    LOCAL_MOTION_EXECUTABLE,
    MAP_EVIDENCE_INSUFFICIENT,
    MIXED_EVIDENCE_REQUIRES_REVIEW,
    MOTION_EVIDENCE_ONLY,
    PROVEN_HARD_CONSTRAINT_REJECTION,
    REJECTED,
    UNRESOLVED,
    VEHICLE_FEASIBLE_MOTION_GRAPH_SCHEMA,
    VehicleFeasibleMotionGraphConfig,
)
from agt_offline_assets.vehicle_feasible_service_graph import (
    DEAD_END_FORWARD_IN_REVERSE_OUT,
    HEADLAND_TOPOLOGY_CANDIDATE,
    REQUIRES_A3_CONNECTOR_VALIDATION,
    REQUIRES_A3_REVERSE_SERVICE_VALIDATION,
    SERVICE_HIGH_TO_LOW,
    SERVICE_LOW_TO_HIGH,
    TOPOLOGY_SERVICE_CANDIDATE,
    ConnectorCandidate,
    ServiceGraphDiagnostics,
    ServiceResource,
    ServiceState,
    VehicleFeasibleServiceGraph,
)


def _vehicle():
    footprint = (
        (0.30, 0.20),
        (0.30, -0.20),
        (-0.30, -0.20),
        (-0.30, 0.20),
    )
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


def _navigation(fill=FREE):
    return NavigationGridEvidence(
        resolution_m=0.25,
        origin_x_m=-2.0,
        origin_y_m=-4.0,
        width=40,
        height=32,
        occupancy=np.full((32, 40), fill, dtype=np.uint8),
        frame_id="map",
        source={},
    )


def _boundary(x_max=8.0):
    return SiteBoundary(
        frame_id="map",
        outer_boundary_xy=(
            (-1.5, -3.5),
            (x_max, -3.5),
            (x_max, 3.5),
            (-1.5, 3.5),
        ),
        source={},
    )


def _resource(
    segment_id="aisle_001.segment_001",
    y=0.0,
    low="LOW_U_HEADLAND",
    high="HIGH_U_HEADLAND",
):
    aisle_id = segment_id.split(".")[0]
    return ServiceResource(
        segment_id=segment_id,
        aisle_id=aisle_id,
        ordinal_in_aisle=1,
        coverage_length_m=4.0,
        coverage_fraction_of_aisle=1.0,
        low_endpoint_type=low,
        high_endpoint_type=high,
        low_endpoint_pose=(0.0, y, 0.0, 0.0),
        high_endpoint_pose=(4.0, y, 0.0, 0.0),
        centerline_xyz=((0.0, y, 0.0), (2.0, y, 0.0), (4.0, y, 0.0)),
    )


def _ordinary_state(resource, service_type):
    if service_type == SERVICE_LOW_TO_HIGH:
        return ServiceState(
            service_state_id=f"{resource.segment_id}.service_low_to_high",
            segment_id=resource.segment_id,
            aisle_id=resource.aisle_id,
            service_type=SERVICE_LOW_TO_HIGH,
            entry_endpoint_type=resource.low_endpoint_type,
            exit_endpoint_type=resource.high_endpoint_type,
            entry_pose=resource.low_endpoint_pose,
            exit_pose=resource.high_endpoint_pose,
            service_motion_direction="FORWARD",
            forward_service_distance_m=resource.coverage_length_m,
            reverse_service_distance_m=0.0,
            coverage_segment_id=resource.segment_id,
            coverage_reward_length_m=resource.coverage_length_m,
            external_reachability_status=HEADLAND_TOPOLOGY_CANDIDATE,
            validation_status=TOPOLOGY_SERVICE_CANDIDATE,
        )
    return ServiceState(
        service_state_id=f"{resource.segment_id}.service_high_to_low",
        segment_id=resource.segment_id,
        aisle_id=resource.aisle_id,
        service_type=SERVICE_HIGH_TO_LOW,
        entry_endpoint_type=resource.high_endpoint_type,
        exit_endpoint_type=resource.low_endpoint_type,
        entry_pose=(
            resource.high_endpoint_pose[0],
            resource.high_endpoint_pose[1],
            resource.high_endpoint_pose[2],
            -math.pi,
        ),
        exit_pose=(
            resource.low_endpoint_pose[0],
            resource.low_endpoint_pose[1],
            resource.low_endpoint_pose[2],
            -math.pi,
        ),
        service_motion_direction="FORWARD",
        forward_service_distance_m=resource.coverage_length_m,
        reverse_service_distance_m=0.0,
        coverage_segment_id=resource.segment_id,
        coverage_reward_length_m=resource.coverage_length_m,
        external_reachability_status=HEADLAND_TOPOLOGY_CANDIDATE,
        validation_status=TOPOLOGY_SERVICE_CANDIDATE,
    )


def _dead_end_state(resource):
    low_headland = resource.low_endpoint_type in {
        "LOW_U_HEADLAND",
        "HIGH_U_HEADLAND",
    }
    if low_headland:
        endpoint_type = resource.low_endpoint_type
        endpoint_pose = resource.low_endpoint_pose
    else:
        endpoint_type = resource.high_endpoint_type
        endpoint_pose = (
            resource.high_endpoint_pose[0],
            resource.high_endpoint_pose[1],
            resource.high_endpoint_pose[2],
            -math.pi,
        )
    return ServiceState(
        service_state_id=f"{resource.segment_id}.dead_end_forward_in_reverse_out",
        segment_id=resource.segment_id,
        aisle_id=resource.aisle_id,
        service_type=DEAD_END_FORWARD_IN_REVERSE_OUT,
        entry_endpoint_type=endpoint_type,
        exit_endpoint_type=endpoint_type,
        entry_pose=endpoint_pose,
        exit_pose=endpoint_pose,
        service_motion_direction="FORWARD",
        forward_service_distance_m=resource.coverage_length_m,
        reverse_service_distance_m=resource.coverage_length_m,
        coverage_segment_id=resource.segment_id,
        coverage_reward_length_m=resource.coverage_length_m,
        external_reachability_status=HEADLAND_TOPOLOGY_CANDIDATE,
        validation_status=REQUIRES_A3_REVERSE_SERVICE_VALIDATION,
    )


def _diagnostics(resources, states, connectors):
    return ServiceGraphDiagnostics(
        resource_count=len(resources),
        ordinary_service_state_count=sum(
            state.service_type != DEAD_END_FORWARD_IN_REVERSE_OUT
            for state in states
        ),
        dead_end_candidate_count=sum(
            state.service_type == DEAD_END_FORWARD_IN_REVERSE_OUT
            for state in states
        ),
        total_service_state_count=len(states),
        connector_candidate_count=len(connectors),
        low_u_connector_candidate_count=sum(
            candidate.side == "LOW_U" for candidate in connectors
        ),
        high_u_connector_candidate_count=sum(
            candidate.side == "HIGH_U" for candidate in connectors
        ),
        candidate_component_count=0,
        isolated_component_count=0,
        externally_unproven_state_count=0,
        unique_coverage_length_m=sum(
            resource.coverage_length_m for resource in resources
        ),
        distinct_aisle_count=len({resource.aisle_id for resource in resources}),
    )


def _graph(resources, states, connectors=()):
    resources = tuple(resources)
    states = tuple(states)
    connectors = tuple(connectors)
    return VehicleFeasibleServiceGraph(
        frame_id="map",
        platform_id="mk_mini",
        platform_profile_sha256="fixture-sha256",
        row_direction_xy=(1.0, 0.0),
        service_resources=resources,
        service_states=states,
        connector_candidates=connectors,
        candidate_components=(),
        diagnostics=_diagnostics(resources, states, connectors),
        source={},
    )


def _zones():
    return TurnZoneSet(
        frame_id="map",
        row_direction_xy=(1.0, 0.0),
        zones=(
            TurnZone(
                zone_id="turn_high_u",
                side="HIGH_U",
                polygon_xy=(
                    (3.0, -2.5),
                    (6.0, -2.5),
                    (6.0, 2.5),
                    (3.0, 2.5),
                ),
                supported_aisle_ids=("aisle_001", "aisle_002"),
                endpoint_count=2,
                free_fraction=1.0,
                allow_turn=True,
            ),
        ),
        source={},
    )


def _two_resource_connector_graph():
    a = _resource("aisle_001.segment_001", y=0.0)
    b = _resource("aisle_002.segment_001", y=2.0)
    a_state = _ordinary_state(a, SERVICE_LOW_TO_HIGH)
    b_state = _ordinary_state(b, SERVICE_HIGH_TO_LOW)
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
    return _graph((a, b), (a_state, b_state), (connector,))


def _service_api():
    return importlib.import_module(
        "agt_offline_assets.vehicle_feasible_service_motion"
    )


def _transition_api():
    return importlib.import_module(
        "agt_offline_assets.vehicle_feasible_transition_motion"
    )


def _core_api():
    return importlib.import_module("agt_offline_assets.vehicle_feasible_motion_graph")


def _io_api():
    return importlib.import_module(
        "agt_offline_assets.vehicle_feasible_motion_graph_io"
    )


def test_a3_public_module_exists_with_frozen_schema():
    module = _core_api()
    assert (
        module.VEHICLE_FEASIBLE_MOTION_GRAPH_SCHEMA
        == "agt_vehicle_feasible_motion_graph/v1"
    )
    assert module.MOTION_EVIDENCE_ONLY == "MOTION_EVIDENCE_ONLY"


def test_low_to_high_is_forward_and_uses_polyline_length():
    validate_service_actions = _service_api().validate_service_actions
    resource = _resource()
    graph = _graph(
        (resource,),
        (_ordinary_state(resource, SERVICE_LOW_TO_HIGH),),
    )
    action = validate_service_actions(
        graph,
        _navigation(),
        _vehicle(),
        VehicleFeasibleMotionGraphConfig(),
    )[0]
    assert action.status == EXECUTABLE
    assert [sample.x for sample in action.samples] == [0.0, 2.0, 4.0]
    assert all(sample.motion_direction == "FORWARD" for sample in action.samples)
    assert action.path_length_m == pytest.approx(4.0)
    assert action.forward_distance_m == pytest.approx(4.0)
    assert action.reverse_distance_m == pytest.approx(0.0)
    assert action.cusp_count == 0
    assert action.backend == A1_CENTERLINE_DIRECTIONAL_REVALIDATION


def test_high_to_low_reverses_points_but_stays_forward_gear():
    validate_service_actions = _service_api().validate_service_actions
    resource = _resource()
    graph = _graph(
        (resource,),
        (_ordinary_state(resource, SERVICE_HIGH_TO_LOW),),
    )
    action = validate_service_actions(
        graph,
        _navigation(),
        _vehicle(),
        VehicleFeasibleMotionGraphConfig(),
    )[0]
    assert [sample.x for sample in action.samples] == [4.0, 2.0, 0.0]
    assert all(sample.motion_direction == "FORWARD" for sample in action.samples)
    assert all(
        abs(abs(sample.yaw) - math.pi) <= 1.0e-9
        for sample in action.samples
    )


def test_service_occupied_unknown_and_boundary_keep_distinct_semantics():
    validate_service_actions = _service_api().validate_service_actions
    resource = _resource()
    graph = _graph(
        (resource,),
        (_ordinary_state(resource, SERVICE_LOW_TO_HIGH),),
    )
    config = VehicleFeasibleMotionGraphConfig()

    occupied = validate_service_actions(
        graph,
        _navigation(OCCUPIED),
        _vehicle(),
        config,
    )[0]
    assert occupied.status == REJECTED
    assert occupied.proof_scope == PROVEN_HARD_CONSTRAINT_REJECTION

    unknown = validate_service_actions(
        graph,
        _navigation(UNKNOWN),
        _vehicle(),
        config,
    )[0]
    assert unknown.status == UNRESOLVED
    assert unknown.proof_scope == MAP_EVIDENCE_INSUFFICIENT

    clipped = validate_service_actions(
        graph,
        _navigation(),
        _vehicle(),
        config,
        site_boundary=_boundary(x_max=3.9),
    )[0]
    assert clipped.status == REJECTED
    assert clipped.backend_status == "SITE_BOUNDARY_CONFLICT"


def test_dead_end_exact_retrace_has_one_cusp_and_single_coverage_reward():
    validate_service_actions = _service_api().validate_service_actions
    resource = _resource(high="INTERIOR_BLOCKED_END")
    graph = _graph((resource,), (_dead_end_state(resource),))
    action = validate_service_actions(
        graph,
        _navigation(),
        _vehicle(),
        VehicleFeasibleMotionGraphConfig(),
    )[0]
    assert action.backend == A1_CENTERLINE_EXACT_REVERSE_RETRACE
    assert action.coverage_reward_length_m == pytest.approx(4.0)
    assert action.path_length_m == pytest.approx(8.0)
    assert action.forward_distance_m == pytest.approx(4.0)
    assert action.reverse_distance_m == pytest.approx(4.0)
    assert action.cusp_count == 1
    cusp = next(
        index for index, sample in enumerate(action.samples) if sample.is_cusp
    )
    assert action.samples[cusp - 1].x == pytest.approx(4.0)
    assert action.samples[cusp].x == pytest.approx(4.0)
    assert action.samples[cusp].motion_direction == "REVERSE"
    assert action.samples[cusp].yaw == pytest.approx(
        action.samples[cusp - 1].yaw
    )
    forward_xyz = [
        (sample.x, sample.y, sample.z) for sample in action.samples[:cusp]
    ]
    reverse_xyz = [
        (sample.x, sample.y, sample.z)
        for sample in action.samples[cusp + 1 :]
    ]
    assert reverse_xyz == list(reversed(forward_xyz[:-1]))


def test_high_headland_dead_end_orients_forward_in_then_exactly_retraces():
    validate_service_actions = _service_api().validate_service_actions
    resource = _resource(low="INTERIOR_BLOCKED_END", high="HIGH_U_HEADLAND")
    graph = _graph((resource,), (_dead_end_state(resource),))
    action = validate_service_actions(
        graph,
        _navigation(),
        _vehicle(),
        VehicleFeasibleMotionGraphConfig(),
    )[0]
    cusp = next(
        index for index, sample in enumerate(action.samples) if sample.is_cusp
    )
    assert [sample.x for sample in action.samples[:cusp]] == [4.0, 2.0, 0.0]
    assert action.cusp_count == 1
    assert all(
        abs(abs(sample.yaw) - math.pi) <= 1.0e-9
        for sample in action.samples[: cusp + 1]
    )
    assert [sample.x for sample in action.samples[cusp + 1 :]] == [2.0, 4.0]


def test_service_validation_fails_closed_on_entry_and_exit_identity():
    validate_service_actions = _service_api().validate_service_actions
    resource = _resource()
    low = _ordinary_state(resource, SERVICE_LOW_TO_HIGH)
    bad_entry = replace(
        low,
        entry_pose=(0.01, *low.entry_pose[1:]),
    )
    with pytest.raises(ValueError, match="entry pose"):
        validate_service_actions(
            _graph((resource,), (bad_entry,)),
            _navigation(),
            _vehicle(),
            VehicleFeasibleMotionGraphConfig(),
        )

    bad_exit = replace(
        low,
        exit_pose=(3.99, *low.exit_pose[1:]),
    )
    with pytest.raises(ValueError, match="exit pose"):
        validate_service_actions(
            _graph((resource,), (bad_exit,)),
            _navigation(),
            _vehicle(),
            VehicleFeasibleMotionGraphConfig(),
        )


def test_service_validation_fails_closed_on_resource_and_graph_contracts():
    validate_service_actions = _service_api().validate_service_actions
    resource = _resource()
    state = _ordinary_state(resource, SERVICE_LOW_TO_HIGH)
    config = VehicleFeasibleMotionGraphConfig()

    missing = replace(state, segment_id="aisle_999.segment_999")
    with pytest.raises(ValueError, match="resource"):
        validate_service_actions(
            _graph((resource,), (missing,)),
            _navigation(),
            _vehicle(),
            config,
        )

    invalid_resource = replace(
        resource,
        centerline_xyz=((0.0, 0.0, 0.0), (float("nan"), 0.0, 0.0)),
    )
    with pytest.raises(ValueError, match="centerline"):
        validate_service_actions(
            _graph((invalid_resource,), (state,)),
            _navigation(),
            _vehicle(),
            config,
        )

    with pytest.raises(ValueError, match="duplicate"):
        validate_service_actions(
            _graph((resource,), (state, state)),
            _navigation(),
            _vehicle(),
            config,
        )

    wrong_frame = replace(_navigation(), frame_id="odom")
    with pytest.raises(ValueError, match="frame"):
        validate_service_actions(
            _graph((resource,), (state,)),
            wrong_frame,
            _vehicle(),
            config,
        )

    wrong_platform = replace(
        _graph((resource,), (state,)),
        platform_id="other_vehicle",
    )
    with pytest.raises(ValueError, match="platform"):
        validate_service_actions(
            wrong_platform,
            _navigation(),
            _vehicle(),
            config,
        )

    wrong_hash = replace(
        _graph((resource,), (state,)),
        platform_profile_sha256="other-sha256",
    )
    with pytest.raises(ValueError, match="profile"):
        validate_service_actions(
            wrong_hash,
            _navigation(),
            _vehicle(),
            config,
        )


def test_connector_adapter_is_one_to_one_and_checks_exit_entry_identity():
    api = _transition_api()
    graph = _two_resource_connector_graph()
    requests, bindings = api.adapt_connector_candidates(
        graph,
        _zones(),
        VehicleFeasibleMotionGraphConfig(),
    )
    assert len(requests) == 1
    request = requests[0]
    binding = bindings[request.connector_id]
    assert request.connector_id == "headland.turn_high_u.a_to_b"
    assert request.start_pose == graph.connector_candidates[0].start_pose
    assert request.goal_pose == graph.connector_candidates[0].goal_pose
    assert (
        binding.from_service_state_id
        == graph.connector_candidates[0].from_service_state_id
    )

    bad = replace(
        graph.connector_candidates[0],
        start_pose=(3.99, *graph.connector_candidates[0].start_pose[1:]),
    )
    with pytest.raises(ValueError, match="start pose"):
        api.adapt_connector_candidates(
            replace(graph, connector_candidates=(bad,)),
            _zones(),
            VehicleFeasibleMotionGraphConfig(),
        )


def test_connector_adapter_fails_closed_on_metadata_and_identity_errors():
    api = _transition_api()
    graph = _two_resource_connector_graph()
    candidate = graph.connector_candidates[0]
    config = VehicleFeasibleMotionGraphConfig()

    for modified, match in (
        (replace(candidate, side="LOW_U"), "side"),
        (replace(candidate, turn_zone_id="missing_zone"), "Turn Zone"),
        (replace(candidate, from_service_state_id="missing_source"), "source"),
        (replace(candidate, to_service_state_id="missing_target"), "target"),
    ):
        with pytest.raises(ValueError, match=match):
            api.adapt_connector_candidates(
                replace(graph, connector_candidates=(modified,)),
                _zones(),
                config,
            )

    with pytest.raises(ValueError, match="duplicate"):
        api.adapt_connector_candidates(
            replace(graph, connector_candidates=(candidate, candidate)),
            _zones(),
            config,
        )


def _forward_gate_plan(status, *, length=5.0):
    samples = (
        SimpleNamespace(
            x=4.0,
            y=0.0,
            z=0.0,
            yaw=0.0,
            motion_direction="FORWARD",
        ),
        SimpleNamespace(
            x=4.0,
            y=2.0,
            z=0.0,
            yaw=math.pi,
            motion_direction="FORWARD",
        ),
    )
    result = SimpleNamespace(
        connector_id="headland.turn_high_u.a_to_b",
        status=status,
        path_type="LSL",
        length_m=length,
        samples=samples,
        centerline_evidence=SimpleNamespace(),
        footprint_evidence=SimpleNamespace(),
        reason=status,
    )
    return SimpleNamespace(connectors=(result,))


def _audit_plan(status):
    result = SimpleNamespace(
        connector_id="headland.turn_high_u.a_to_b",
        status=status,
        reason=status,
    )
    return SimpleNamespace(connectors=(result,))


def _admission(decision="ELIGIBLE_REVERSE_FALLBACK"):
    item = SimpleNamespace(
        connector_id="headland.turn_high_u.a_to_b",
        decision=decision,
        reason=decision,
    )
    return SimpleNamespace(
        items=(item,),
        eligible_connector_ids=(
            ("headland.turn_high_u.a_to_b",)
            if decision == "ELIGIBLE_REVERSE_FALLBACK"
            else ()
        ),
    )


def _reverse_plan(status):
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
            yaw=math.pi,
            motion_direction="FORWARD",
            segment_index=2,
            is_cusp=True,
        ),
    )
    result = SimpleNamespace(
        connector_id="headland.turn_high_u.a_to_b",
        status=status,
        path_length_m=5.5 if "FREE" in status else None,
        forward_distance_m=3.0 if "FREE" in status else 0.0,
        reverse_distance_m=2.5 if "FREE" in status else 0.0,
        cusp_count=2 if "FREE" in status else 0,
        search_expansions=321,
        samples=samples if "FREE" in status else (),
        centerline_evidence=SimpleNamespace(),
        footprint_evidence=SimpleNamespace(
            occupied_fraction=0.0,
            unknown_fraction=0.0,
            grid_coverage_fraction=1.0,
        ),
        reason=status,
    )
    return SimpleNamespace(connectors=(result,))


def test_transition_forward_preview_free_maps_directly_to_executable(monkeypatch):
    api = _transition_api()
    monkeypatch.setattr(
        api,
        "derive_forward_connector_navigation_gate",
        lambda *args, **kwargs: _forward_gate_plan("PREVIEW_FOOTPRINT_FREE"),
    )
    monkeypatch.setattr(
        api,
        "derive_forward_connector_candidate_audit",
        lambda *args, **kwargs: pytest.fail("audit must not run for forward success"),
    )
    result = api.validate_transition_candidates(
        _two_resource_connector_graph(),
        _zones(),
        _navigation(),
        _vehicle(),
        VehicleFeasibleMotionGraphConfig(),
    )[0]
    assert result.status == EXECUTABLE
    assert result.backend == FORWARD_DUBINS_NAVIGATION_GATE
    assert result.reverse_distance_m == 0.0
    assert result.cusp_count == 0


@pytest.mark.parametrize(
    "audit_status,expected_status,expected_scope",
    [
        (
            "LOCAL_FORWARD_SITE_BOUNDARY_CONFLICT",
            REJECTED,
            PROVEN_HARD_CONSTRAINT_REJECTION,
        ),
        (
            "LOCAL_FORWARD_MAP_EVIDENCE_INSUFFICIENT",
            UNRESOLVED,
            MAP_EVIDENCE_INSUFFICIENT,
        ),
        (
            "LOCAL_FORWARD_MIXED_EVIDENCE",
            UNRESOLVED,
            MIXED_EVIDENCE_REQUIRES_REVIEW,
        ),
    ],
)
def test_transition_forward_audit_holds_or_rejects_without_reverse(
    monkeypatch,
    audit_status,
    expected_status,
    expected_scope,
):
    api = _transition_api()
    monkeypatch.setattr(
        api,
        "derive_forward_connector_navigation_gate",
        lambda *args, **kwargs: _forward_gate_plan(
            "NO_FORWARD_PREVIEW_FREE_CANDIDATE"
        ),
    )
    monkeypatch.setattr(
        api,
        "derive_forward_connector_candidate_audit",
        lambda *args, **kwargs: _audit_plan(audit_status),
    )
    monkeypatch.setattr(
        api,
        "derive_reverse_fallback_admission",
        lambda *args, **kwargs: pytest.fail("R6A must not run"),
    )
    monkeypatch.setattr(
        api,
        "derive_reverse_primitive_connector_plan",
        lambda *args, **kwargs: pytest.fail("R6B must not run"),
    )
    result = api.validate_transition_candidates(
        _two_resource_connector_graph(),
        _zones(),
        _navigation(),
        _vehicle(),
        VehicleFeasibleMotionGraphConfig(),
        site_boundary=_boundary(),
    )[0]
    assert result.status == expected_status
    assert result.proof_scope == expected_scope


def test_transition_reverse_success_preserves_r6b_metrics(monkeypatch):
    api = _transition_api()
    monkeypatch.setattr(
        api,
        "derive_forward_connector_navigation_gate",
        lambda *args, **kwargs: _forward_gate_plan(
            "NO_FORWARD_PREVIEW_FREE_CANDIDATE"
        ),
    )
    monkeypatch.setattr(
        api,
        "derive_forward_connector_candidate_audit",
        lambda *args, **kwargs: _audit_plan("LOCAL_FORWARD_OCCUPANCY_BLOCKED"),
    )
    monkeypatch.setattr(
        api,
        "derive_reverse_fallback_admission",
        lambda *args, **kwargs: _admission(),
    )
    monkeypatch.setattr(
        api,
        "derive_reverse_primitive_connector_plan",
        lambda *args, **kwargs: _reverse_plan("REVERSE_PRIMITIVE_PREVIEW_FREE"),
    )
    result = api.validate_transition_candidates(
        _two_resource_connector_graph(),
        _zones(),
        _navigation(),
        _vehicle(),
        VehicleFeasibleMotionGraphConfig(),
    )[0]
    assert result.status == EXECUTABLE
    assert result.proof_scope == LOCAL_MOTION_EXECUTABLE
    assert result.backend == BOUNDED_REVERSE_PRIMITIVE_SEARCH
    assert result.path_length_m == pytest.approx(5.5)
    assert result.forward_distance_m == pytest.approx(3.0)
    assert result.reverse_distance_m == pytest.approx(2.5)
    assert result.cusp_count == 2
    assert result.search_expansions == 321


def test_transition_bounded_reverse_failure_is_not_global_infeasibility(monkeypatch):
    api = _transition_api()
    monkeypatch.setattr(
        api,
        "derive_forward_connector_navigation_gate",
        lambda *args, **kwargs: _forward_gate_plan(
            "NO_FORWARD_PREVIEW_FREE_CANDIDATE"
        ),
    )
    monkeypatch.setattr(
        api,
        "derive_forward_connector_candidate_audit",
        lambda *args, **kwargs: _audit_plan("LOCAL_FORWARD_OCCUPANCY_BLOCKED"),
    )
    monkeypatch.setattr(
        api,
        "derive_reverse_fallback_admission",
        lambda *args, **kwargs: _admission(),
    )
    monkeypatch.setattr(
        api,
        "derive_reverse_primitive_connector_plan",
        lambda *args, **kwargs: _reverse_plan(
            "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION"
        ),
    )
    result = api.validate_transition_candidates(
        _two_resource_connector_graph(),
        _zones(),
        _navigation(),
        _vehicle(),
        VehicleFeasibleMotionGraphConfig(),
    )[0]
    assert result.status == REJECTED
    assert result.proof_scope == BOUNDED_SEARCH_NO_SOLUTION
    assert "global" not in result.reason.lower()


def test_graph_deduplicates_two_executable_directions_to_one_physical_coverage():
    derive = getattr(_core_api(), "derive_vehicle_feasible_motion_graph")
    resource = _resource()
    graph = _graph(
        (resource,),
        (
            _ordinary_state(resource, SERVICE_LOW_TO_HIGH),
            _ordinary_state(resource, SERVICE_HIGH_TO_LOW),
        ),
    )
    motion = derive(graph, _zones(), _navigation(), _vehicle())
    assert motion.schema == VEHICLE_FEASIBLE_MOTION_GRAPH_SCHEMA
    assert motion.status == MOTION_EVIDENCE_ONLY
    assert motion.diagnostics.service_validation_count == 2
    assert motion.diagnostics.locally_validated_segment_count == 1
    assert (
        motion.diagnostics.locally_validated_unique_coverage_length_m
        == pytest.approx(4.0)
    )


def _contains_forbidden_key(value):
    forbidden = {"route_ready", "reachable_from_start", "optimal"}
    if isinstance(value, dict):
        if forbidden & set(value):
            return True
        return any(_contains_forbidden_key(item) for item in value.values())
    if isinstance(value, (list, tuple)):
        return any(_contains_forbidden_key(item) for item in value)
    return False


def test_motion_graph_io_is_deterministic_strict_and_refuses_overwrite(tmp_path):
    derive = getattr(_core_api(), "derive_vehicle_feasible_motion_graph")
    io = _io_api()
    resource = _resource()
    graph = _graph(
        (resource,),
        (_ordinary_state(resource, SERVICE_LOW_TO_HIGH),),
    )
    motion = derive(graph, _zones(), _navigation(), _vehicle())
    payload = io.vehicle_feasible_motion_graph_to_dict(motion)
    assert not _contains_forbidden_key(payload)

    output = tmp_path / "vehicle_feasible_motion_graph.yaml"
    io.write_vehicle_feasible_motion_graph(motion, output)
    original = output.read_bytes()
    with pytest.raises(FileExistsError):
        io.write_vehicle_feasible_motion_graph(motion, output)

    loaded = io.load_vehicle_feasible_motion_graph(output)
    rewrite = tmp_path / "rewrite.yaml"
    io.write_vehicle_feasible_motion_graph(loaded, rewrite)
    assert rewrite.read_bytes() == original


def test_motion_graph_strict_loader_rejects_bad_cross_reference(tmp_path):
    derive = getattr(_core_api(), "derive_vehicle_feasible_motion_graph")
    io = _io_api()
    resource = _resource()
    graph = _graph(
        (resource,),
        (_ordinary_state(resource, SERVICE_LOW_TO_HIGH),),
    )
    motion = derive(graph, _zones(), _navigation(), _vehicle())
    payload = io.vehicle_feasible_motion_graph_to_dict(motion)
    payload["executable_service_action_ids"] = ["missing.service.state"]

    import yaml

    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    with pytest.raises(ValueError, match="executable_service_action_ids"):
        io.load_vehicle_feasible_motion_graph(path)
