import math

import numpy as np

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
