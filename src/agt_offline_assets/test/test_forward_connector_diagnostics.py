import math

from agt_offline_assets import (
    CanonicalVehicleProfile,
    ConnectorRequest,
    ForwardConnectorConfig,
    TurnZone,
    TurnZoneSet,
    diagnose_forward_connector_zone_fit,
    forward_connector_zone_fit_to_dict,
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
    return TurnZoneSet(
        frame_id="map",
        row_direction_xy=(1.0, 0.0),
        zones=(
            TurnZone(
                zone_id="turn_high_u",
                side="HIGH_U",
                polygon_xy=tuple(polygon),
                supported_aisle_ids=("aisle_001", "aisle_002"),
                endpoint_count=2,
                free_fraction=float("nan"),
            ),
        ),
    )


def test_zone_fit_diagnostic_reports_zero_deficit_for_broad_zone():
    report = diagnose_forward_connector_zone_fit(
        (_request(),),
        _zones(((-10.0, -10.0), (10.0, -10.0), (10.0, 10.0), (-10.0, 10.0))),
        _vehicle(),
        ForwardConnectorConfig(sample_step_m=0.05),
    )
    item = report.diagnostics[0]
    assert item.status == "FITS_CURRENT_ZONE"
    assert item.max_required_extension_m == 0.0
    assert item.inside_turn_zone_fraction == 1.0
    assert item.best_path_type is not None


def test_zone_fit_diagnostic_quantifies_small_turn_zone_deficit():
    report = diagnose_forward_connector_zone_fit(
        (_request(),),
        _zones(((-0.40, -0.10), (0.40, -0.10), (0.40, 2.10), (-0.40, 2.10))),
        _vehicle(),
        ForwardConnectorConfig(sample_step_m=0.05),
    )
    item = report.diagnostics[0]
    assert item.status == "ZONE_EXPANSION_REQUIRED"
    assert item.candidate_count > 0
    assert item.best_path_type is not None
    assert item.best_candidate_length_m is not None and item.best_candidate_length_m > 0.0
    assert item.max_required_extension_m > 0.0
    assert (
        item.required_outward_extension_m > 0.0
        or item.required_inward_extension_m > 0.0
        or item.required_lateral_low_extension_m > 0.0
        or item.required_lateral_high_extension_m > 0.0
    )


def test_zone_fit_serialization_keeps_diagnostic_scope_explicit():
    report = diagnose_forward_connector_zone_fit(
        (_request(),),
        _zones(((-0.40, -0.10), (0.40, -0.10), (0.40, 2.10), (-0.40, 2.10))),
        _vehicle(),
    )
    payload = forward_connector_zone_fit_to_dict(report)
    assert payload["schema"] == "agt_forward_connector_zone_fit_diagnostic/v1"
    assert payload["status"] == "DRAFT"
    assert payload["diagnostics"][0]["diagnostic_scope"] == "ROW_FRAME_ENVELOPE_ONLY_NOT_COLLISION_TRUTH"
