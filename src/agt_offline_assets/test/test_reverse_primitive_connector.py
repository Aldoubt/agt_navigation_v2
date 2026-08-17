import math

import numpy as np

import agt_offline_assets.reverse_primitive_connector as r6b
from agt_offline_assets.agricultural_coverage_ordering import ConnectorRequest
from agt_offline_assets.navigation_grid import NavigationGridEvidence
from agt_offline_assets.navigation_map_derivation import FREE, OCCUPIED
from agt_offline_assets.reverse_fallback_admission import (
    ReverseFallbackAdmissionItem,
    ReverseFallbackAdmissionPlan,
)
from agt_offline_assets.reverse_primitive_connector import (
    REVERSE_PRIMITIVE_BACKEND,
    ReversePrimitiveConnectorConfig,
    derive_reverse_primitive_connector_plan,
    reverse_primitive_connector_plan_to_dict,
)
from agt_offline_assets.site_boundary import SiteBoundary
from agt_offline_assets.turn_zones import TurnZone, TurnZoneSet
from agt_offline_assets.vehicle_profile import CanonicalVehicleProfile


def _vehicle() -> CanonicalVehicleProfile:
    return CanonicalVehicleProfile(
        profile_id="mk_mini",
        profile_path="profiles/platforms/mk_mini.yaml",
        profile_sha256="deadbeef",
        kinematics="ackermann",
        footprint_frame="base_footprint",
        base_frame="base_link",
        physical_length_m=0.84,
        physical_width_m=0.60,
        footprint_xy=((0.42, 0.30), (0.42, -0.30), (-0.42, -0.30), (-0.42, 0.30)),
        navigation_footprint_xy=((0.42, 0.30), (0.42, -0.30), (-0.42, -0.30), (-0.42, 0.30)),
        navigation_width_m=0.60,
        navigation_length_m=0.84,
        wheel_base_m=0.60,
        track_width_m=0.517,
        wheel_diameter_m=0.240,
        ground_clearance_m=0.111,
        minimum_turning_radius_m=1.50,
        minimum_turning_radius_verified=True,
        maximum_steering_angle_rad=math.radians(34.0),
        maximum_steering_angle_deg=34.0,
        allow_in_place_rotation=False,
        max_forward_velocity_mps=1.0,
        max_reverse_velocity_mps=0.5,
        max_angular_velocity_rps=None,
        manufacturer_maximum_speed_mps=9.7 / 3.6,
        route_acceptance_enabled=False,
        preview_planning_enabled=True,
        blocked_reason="preview-only fixture",
    )


def _request(connector_id="connector_002") -> ConnectorRequest:
    return ConnectorRequest(
        connector_id=connector_id,
        from_aisle_id="aisle_002",
        to_aisle_id="aisle_003",
        turn_zone_id="turn_low_u",
        side="LOW_U",
        start_pose=(0.0, 0.0, -1.6, 0.0),
        goal_pose=(-0.9, 0.0, -1.6, 0.0),
    )


def _far_request(connector_id="connector_far") -> ConnectorRequest:
    return ConnectorRequest(
        connector_id=connector_id,
        from_aisle_id="aisle_002",
        to_aisle_id="aisle_003",
        turn_zone_id="turn_low_u",
        side="LOW_U",
        start_pose=(0.0, 0.0, -1.6, 0.0),
        goal_pose=(-4.0, 0.0, -1.6, 0.0),
    )


def _zones() -> TurnZoneSet:
    zone = TurnZone(
        zone_id="turn_low_u",
        side="LOW_U",
        polygon_xy=((-1.5, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.5, 1.0)),
        supported_aisle_ids=("aisle_002", "aisle_003"),
        endpoint_count=2,
        free_fraction=float("nan"),
    )
    return TurnZoneSet(frame_id="map", row_direction_xy=(1.0, 0.0), zones=(zone,))


def _grid(fill=FREE) -> NavigationGridEvidence:
    occupancy = np.full((100, 100), fill, dtype=np.uint8)
    return NavigationGridEvidence(
        resolution_m=0.10,
        origin_x_m=-5.0,
        origin_y_m=-5.0,
        width=100,
        height=100,
        occupancy=occupancy,
        source={"fixture": "reverse_grid"},
    )


def _admission(*ids) -> ReverseFallbackAdmissionPlan:
    items = tuple(
        ReverseFallbackAdmissionItem(
            connector_id=connector_id,
            from_aisle_id="aisle_002",
            to_aisle_id="aisle_003",
            turn_zone_id="turn_low_u",
            forward_audit_status="LOCAL_FORWARD_OCCUPANCY_BLOCKED",
            decision="ELIGIBLE_REVERSE_FALLBACK",
            reason="fixture",
        )
        for connector_id in ids
    )
    return ReverseFallbackAdmissionPlan(
        frame_id="map",
        platform_id="mk_mini",
        platform_profile_sha256="deadbeef",
        items=items,
    )


def _a35_api():
    diagnostics_type = getattr(r6b, "ReversePrimitiveSearchDiagnostics", None)
    edge_reason = getattr(r6b, "_edge_rejection_reason", None)
    assert diagnostics_type is not None, "missing A3.5 diagnostics type"
    assert edge_reason is not None, "missing A3.5 edge rejection reason API"
    return diagnostics_type, edge_reason


def _assert_a35_accounting(diagnostics):
    assert diagnostics.nodes_popped == (
        diagnostics.search_expansions + diagnostics.stale_nodes_skipped
    )
    assert diagnostics.cusp_switches_considered == (
        diagnostics.cusp_switches_rejected_state_dominance
        + diagnostics.cusp_switches_enqueued
    )
    assert diagnostics.primitive_edges_considered == (
        diagnostics.primitive_edges_rejected_search_envelope
        + diagnostics.primitive_edges_rejected_site_boundary
        + diagnostics.primitive_edges_rejected_navigation_grid
        + diagnostics.primitive_edges_rejected_path_length
        + diagnostics.primitive_edges_rejected_state_dominance
        + diagnostics.primitive_edges_enqueued
    )
    assert diagnostics.goal_shot_candidates_considered == (
        diagnostics.goal_shot_rejected_path_length
        + diagnostics.goal_shot_rejected_search_envelope
        + diagnostics.goal_shot_rejected_site_boundary
        + diagnostics.goal_shot_rejected_navigation_grid
        + diagnostics.goal_shot_successes
    )


def test_tight_behind_goal_uses_explicit_reverse_and_cusps():
    plan = derive_reverse_primitive_connector_plan(
        (_request(),),
        _admission("connector_002"),
        _zones(),
        _grid(),
        _vehicle(),
        ReversePrimitiveConnectorConfig(
            primitive_length_m=0.30,
            collision_sample_step_m=0.10,
            max_expansions=8000,
            max_path_length_m=8.0,
            goal_shot_distance_m=2.5,
        ),
    )
    assert len(plan.connectors) == 1
    result = plan.connectors[0]
    assert result.status == "REVERSE_PRIMITIVE_PREVIEW_FREE"
    assert result.backend == REVERSE_PRIMITIVE_BACKEND
    assert result.reverse_distance_m > 0.5
    assert result.cusp_count >= 2
    assert any(sample.motion_direction == "REVERSE" for sample in result.samples)
    assert any(sample.is_cusp for sample in result.samples)
    assert result.footprint_evidence.occupied_count == 0
    assert result.footprint_evidence.unknown_count == 0


def test_r6b_processes_only_r6a_eligible_connector_ids():
    requests = (_request("connector_002"), _request("connector_003"))
    plan = derive_reverse_primitive_connector_plan(
        requests,
        _admission("connector_003"),
        _zones(),
        _grid(),
        _vehicle(),
        ReversePrimitiveConnectorConfig(max_expansions=8000, max_path_length_m=8.0),
    )
    assert [item.connector_id for item in plan.connectors] == ["connector_003"]


def test_occupied_start_fails_closed_and_serialization_keeps_preview_boundary():
    plan = derive_reverse_primitive_connector_plan(
        (_request(),),
        _admission("connector_002"),
        _zones(),
        _grid(OCCUPIED),
        _vehicle(),
    )
    result = plan.connectors[0]
    assert result.status == "R6B_START_FOOTPRINT_NOT_FREE"
    payload = reverse_primitive_connector_plan_to_dict(plan)
    assert payload["schema"] == "agt_reverse_primitive_connector_plan/v1"
    assert payload["status"] == "DRAFT"
    assert payload["connectors"][0]["validation_scope"] == "PREVIEW_ONLY_NOT_R8_VEHICLE_READY"
    assert "NOT_ANALYTIC_REEDS_SHEPP" in payload["connectors"][0]["backend"]


def test_start_center_inside_but_footprint_touching_boundary_fails_before_search():
    boundary = SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((-0.10, -1.0), (1.0, -1.0), (1.0, 1.0), (-0.10, 1.0)),
    )
    plan = derive_reverse_primitive_connector_plan(
        (_request(),),
        _admission("connector_002"),
        _zones(),
        _grid(),
        _vehicle(),
        site_boundary=boundary,
    )
    result = plan.connectors[0]
    assert result.status == "SITE_BOUNDARY_CONFLICT"
    assert result.search_expansions == 0
    assert result.samples == ()
    assert "site boundary" in result.reason.lower()


def test_a35_edge_reason_separates_envelope_boundary_grid_and_free():
    _diagnostics_type, edge_reason = _a35_api()
    cfg = ReversePrimitiveConnectorConfig()
    zones = _zones()
    request = _request()
    bounds = r6b._row_frame_bounds(request, zones.zones[0], zones.row_direction_xy, cfg)
    local_footprint = r6b._preview_local_footprint(
        _vehicle(), cfg.preview_footprint_padding_m
    )
    outside = ((2.0, 0.0, 0.0, 1, 0.0, False),)
    inside = ((0.0, 0.0, 0.0, 1, 0.0, False),)
    boundary = SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((-0.10, -1.0), (1.0, -1.0), (1.0, 1.0), (-0.10, 1.0)),
    )

    assert edge_reason(outside, bounds, _grid(), local_footprint, None) == getattr(
        r6b, "EDGE_SEARCH_ENVELOPE"
    )
    assert edge_reason(inside, bounds, _grid(), local_footprint, boundary) == getattr(
        r6b, "EDGE_SITE_BOUNDARY"
    )
    assert edge_reason(inside, bounds, _grid(OCCUPIED), local_footprint, None) == getattr(
        r6b, "EDGE_NAVIGATION_GRID"
    )
    assert edge_reason(inside, bounds, _grid(), local_footprint, None) == getattr(
        r6b, "EDGE_FREE"
    )


def test_a35_edge_reason_is_boolean_equivalent_to_existing_gate():
    _diagnostics_type, edge_reason = _a35_api()
    cfg = ReversePrimitiveConnectorConfig()
    zones = _zones()
    bounds = r6b._row_frame_bounds(_request(), zones.zones[0], zones.row_direction_xy, cfg)
    local_footprint = r6b._preview_local_footprint(
        _vehicle(), cfg.preview_footprint_padding_m
    )
    boundary = SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((-0.10, -1.0), (1.0, -1.0), (1.0, 1.0), (-0.10, 1.0)),
    )
    cases = (
        (((2.0, 0.0, 0.0, 1, 0.0, False),), _grid(), None),
        (((0.0, 0.0, 0.0, 1, 0.0, False),), _grid(), boundary),
        (((0.0, 0.0, 0.0, 1, 0.0, False),), _grid(OCCUPIED), None),
        (((0.0, 0.0, 0.0, 1, 0.0, False),), _grid(), None),
    )
    for samples, navigation, site_boundary in cases:
        assert r6b._edge_is_free(
            samples,
            bounds,
            navigation,
            local_footprint,
            site_boundary,
        ) == (
            edge_reason(
                samples,
                bounds,
                navigation,
                local_footprint,
                site_boundary,
            )
            == getattr(r6b, "EDGE_FREE")
        )


def test_a35_queue_exhaustion_and_path_length_funnel_are_distinct():
    _a35_api()
    plan = derive_reverse_primitive_connector_plan(
        (_request(),),
        _admission("connector_002"),
        _zones(),
        _grid(),
        _vehicle(),
        ReversePrimitiveConnectorConfig(
            max_path_length_m=0.10,
            max_expansions=50,
        ),
    )
    result = plan.connectors[0]
    diagnostics = result.diagnostics
    assert result.status == "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION"
    assert diagnostics.queue_exhausted is True
    assert diagnostics.expansion_budget_reached is False
    assert diagnostics.primitive_edges_rejected_path_length > 0
    _assert_a35_accounting(diagnostics)


def test_a35_expansion_budget_termination_is_reported_without_status_change():
    _a35_api()
    request = _far_request()
    plan = derive_reverse_primitive_connector_plan(
        (request,),
        _admission(request.connector_id),
        _zones(),
        _grid(),
        _vehicle(),
        ReversePrimitiveConnectorConfig(max_expansions=1),
    )
    result = plan.connectors[0]
    diagnostics = result.diagnostics
    assert result.status == "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION"
    assert result.search_expansions == 1
    assert diagnostics.search_expansions == 1
    assert diagnostics.queue_exhausted is False
    assert diagnostics.expansion_budget_reached is True
    _assert_a35_accounting(diagnostics)


def test_a35_diagnostics_are_deterministic_and_best_goal_state_is_coherent():
    _a35_api()
    request = _far_request()
    cfg = ReversePrimitiveConnectorConfig(max_expansions=20)
    first = derive_reverse_primitive_connector_plan(
        (request,),
        _admission(request.connector_id),
        _zones(),
        _grid(),
        _vehicle(),
        cfg,
    ).connectors[0]
    second = derive_reverse_primitive_connector_plan(
        (request,),
        _admission(request.connector_id),
        _zones(),
        _grid(),
        _vehicle(),
        cfg,
    ).connectors[0]
    assert first.status == second.status
    assert first.search_expansions == second.search_expansions
    assert first.samples == second.samples
    assert first.diagnostics == second.diagnostics
    diagnostics = first.diagnostics
    assert diagnostics.best_goal_position_error_m is not None
    assert diagnostics.best_goal_yaw_error_rad is not None
    assert diagnostics.best_goal_distance_state_direction in {"FORWARD", "REVERSE"}
    assert diagnostics.best_goal_distance_cusp_count is not None
    assert 0 <= diagnostics.best_goal_distance_cusp_count <= cfg.max_cusps
    _assert_a35_accounting(diagnostics)


def test_a35_accounting_holds_on_existing_executable_reverse_fixture():
    _a35_api()
    plan = derive_reverse_primitive_connector_plan(
        (_request(),),
        _admission("connector_002"),
        _zones(),
        _grid(),
        _vehicle(),
        ReversePrimitiveConnectorConfig(
            primitive_length_m=0.30,
            collision_sample_step_m=0.10,
            max_expansions=8000,
            max_path_length_m=8.0,
            goal_shot_distance_m=2.5,
        ),
    )
    result = plan.connectors[0]
    assert result.status == "REVERSE_PRIMITIVE_PREVIEW_FREE"
    assert result.backend == REVERSE_PRIMITIVE_BACKEND
    assert result.samples
    _assert_a35_accounting(result.diagnostics)
