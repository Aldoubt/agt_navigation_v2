import math

import numpy as np

from agt_offline_assets import (
    CanonicalVehicleProfile,
    ConnectorRequest,
    ForwardConnectorConfig,
    TurnZone,
    TurnZoneSet,
    derive_forward_connector_plan,
    forward_connector_plan_to_dict,
)


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
        footprint_xy=((-0.42, -0.30), (0.42, -0.30), (0.42, 0.30), (-0.42, 0.30)),
        navigation_footprint_xy=((-0.42, -0.30), (0.42, -0.30), (0.42, 0.30), (-0.42, 0.30)),
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
        blocked_reason="base_footprint reference measurement pending",
    )


def _request() -> ConnectorRequest:
    return ConnectorRequest(
        connector_id="connector_001",
        from_aisle_id="aisle_001",
        to_aisle_id="aisle_002",
        turn_zone_id="turn_high_u",
        side="HIGH_U",
        start_pose=(0.0, 0.0, -1.6, 0.0),
        goal_pose=(0.0, 2.0, -1.5, math.pi),
    )


def _zones(polygon) -> TurnZoneSet:
    zone = TurnZone(
        zone_id="turn_high_u",
        side="HIGH_U",
        polygon_xy=tuple(polygon),
        supported_aisle_ids=("aisle_001", "aisle_002"),
        endpoint_count=2,
        free_fraction=float("nan"),
    )
    return TurnZoneSet(
        frame_id="map",
        row_direction_xy=(1.0, 0.0),
        zones=(zone,),
    )


def test_forward_dubins_accepts_shortest_candidate_inside_broad_turn_zone():
    zones = _zones(((-10.0, -10.0), (10.0, -10.0), (10.0, 10.0), (-10.0, 10.0)))
    plan = derive_forward_connector_plan(
        (_request(),),
        zones,
        _vehicle(),
        ForwardConnectorConfig(sample_step_m=0.05),
    )
    assert plan.accepted_count == 1
    assert plan.rejected_count == 0
    result = plan.connectors[0]
    assert result.status == "ACCEPTED_CENTERLINE"
    assert result.backend == "ANALYTIC_DUBINS_FORWARD_ONLY"
    assert result.path_type in {"LSL", "RSR", "LSR", "RSL", "RLR", "LRL"}
    assert result.minimum_turning_radius_m == 1.5
    assert result.inside_turn_zone_fraction == 1.0
    assert result.length_m is not None and result.length_m > 0.0
    assert len(result.samples) > 2
    assert np.allclose(
        [result.samples[0].x, result.samples[0].y, result.samples[0].z],
        _request().start_pose[:3],
    )
    assert np.allclose(
        [result.samples[-1].x, result.samples[-1].y, result.samples[-1].z],
        _request().goal_pose[:3],
    )
    assert all(sample.motion_direction == "FORWARD" for sample in result.samples)


def test_forward_dubins_rejects_candidates_that_leave_turn_zone():
    zones = _zones(((-0.40, -0.10), (0.40, -0.10), (0.40, 2.10), (-0.40, 2.10)))
    plan = derive_forward_connector_plan((_request(),), zones, _vehicle())
    result = plan.connectors[0]
    assert result.status == "NO_FORWARD_DUBINS_IN_TURN_ZONE"
    assert result.samples == ()
    assert result.candidate_count > 0
    assert 0.0 <= result.inside_turn_zone_fraction < 1.0


def test_forward_connector_reports_missing_zone_without_fabricating_path():
    plan = derive_forward_connector_plan(
        (_request(),),
        TurnZoneSet(frame_id="map", row_direction_xy=(1.0, 0.0), zones=()),
        _vehicle(),
    )
    result = plan.connectors[0]
    assert result.status == "TURN_ZONE_NOT_FOUND"
    assert result.path_type is None
    assert result.samples == ()


def test_forward_connector_serialization_preserves_preview_only_scope():
    zones = _zones(((-10.0, -10.0), (10.0, -10.0), (10.0, 10.0), (-10.0, 10.0)))
    plan = derive_forward_connector_plan((_request(),), zones, _vehicle())
    payload = forward_connector_plan_to_dict(plan)
    assert payload["schema"] == "agt_forward_connector_plan/v1"
    assert payload["status"] == "DRAFT"
    assert payload["connectors"][0]["motion_direction"] == "FORWARD"
    assert payload["connectors"][0]["validation_scope"] == "CENTERLINE_KINEMATICS_AND_TURN_ZONE_ONLY"
