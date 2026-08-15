import math

import numpy as np

from agt_offline_assets.agricultural_aisle_graph import AislePrimitive, AgriculturalAisleGraph
from agt_offline_assets.navigation_grid import NavigationGridEvidence
from agt_offline_assets.navigation_map_derivation import FREE, OCCUPIED
from agt_offline_assets.vehicle_profile import CanonicalVehicleProfile
from agt_offline_assets.vehicle_safe_lane_diagnostics import (
    VehicleSafeLaneDiagnosticConfig,
    derive_vehicle_safe_lane_diagnostics,
)


def _vehicle():
    footprint = ((0.42, 0.30), (0.42, -0.30), (-0.42, -0.30), (-0.42, 0.30))
    return CanonicalVehicleProfile(
        profile_id="mk_mini",
        profile_path="mk_mini.yaml",
        profile_sha256="abc",
        kinematics="ackermann",
        footprint_frame="base_footprint",
        base_frame="base_link",
        physical_length_m=0.84,
        physical_width_m=0.60,
        footprint_xy=footprint,
        navigation_footprint_xy=footprint,
        navigation_width_m=0.60,
        navigation_length_m=0.84,
        wheel_base_m=0.60,
        track_width_m=0.517,
        wheel_diameter_m=0.24,
        ground_clearance_m=0.111,
        minimum_turning_radius_m=1.5,
        minimum_turning_radius_verified=True,
        maximum_steering_angle_rad=math.radians(34.0),
        maximum_steering_angle_deg=34.0,
        allow_in_place_rotation=False,
        max_forward_velocity_mps=1.0,
        max_reverse_velocity_mps=0.5,
        max_angular_velocity_rps=None,
        manufacturer_maximum_speed_mps=None,
        route_acceptance_enabled=False,
        preview_planning_enabled=True,
        blocked_reason="test",
    )


def _aisle(width=1.2):
    points = tuple((float(x), 0.0, 0.0) for x in np.linspace(1.0, 5.0, 41))
    return AislePrimitive(
        aisle_id="aisle_001",
        kind="interior",
        pair_kind="ROW_ROW",
        left_structure_ref="row_01",
        right_structure_ref="row_02",
        centerline_xyz=points,
        start_pose=(1.0, 0.0, 0.0, 0.0),
        end_pose=(5.0, 0.0, 0.0, 0.0),
        length_m=4.0,
        geometric_width_m=width,
        minimum_required_width_m=0.45,
        center_distance_m=1.6,
        longitudinal_overlap_m=4.0,
        safe_cell_count=100,
        centerline_cell_count=41,
        diagnostic_status="ACCEPTED",
    )


def _graph(width=1.2):
    return AgriculturalAisleGraph(
        frame_id="map",
        row_direction_xy=(1.0, 0.0),
        nominal_row_spacing_m=1.6,
        aisles=(_aisle(width),),
    )


def _navigation():
    resolution = 0.05
    return NavigationGridEvidence(
        resolution_m=resolution,
        origin_x_m=0.0,
        origin_y_m=-1.0,
        width=140,
        height=40,
        occupancy=np.full((40, 140), FREE, dtype=np.uint8),
        frame_id="map",
    )


def _diagnose(navigation, width=1.2):
    return derive_vehicle_safe_lane_diagnostics(
        _graph(width),
        navigation,
        _vehicle(),
        VehicleSafeLaneDiagnosticConfig(
            sample_spacing_m=0.10,
            lateral_search_step_m=0.05,
            maximum_lateral_shift_m=0.25,
            preview_footprint_padding_m=0.05,
        ),
    ).aisles[0]


def test_all_free_map_is_mostly_configuration_space_free():
    item = _diagnose(_navigation())
    assert item.classification == "MOSTLY_CONFIGURATION_SPACE_FREE"
    assert item.reference_free_fraction == 1.0
    assert item.any_lateral_footprint_free_fraction == 1.0


def test_centerline_occupied_is_distinguished_from_footprint_edge_conflict():
    navigation = _navigation()
    occupancy = navigation.occupancy.copy()
    row = int((0.0 - navigation.origin_y_m) / navigation.resolution_m)
    occupancy[row, :] = OCCUPIED
    navigation = NavigationGridEvidence(**{**navigation.__dict__, "occupancy": occupancy})
    item = _diagnose(navigation)
    assert item.classification == "CENTER_REFERENCE_OCCUPIED_DOMINANT"
    assert item.reference_occupied_count > item.reference_free_count


def test_structural_width_gate_remains_explicit():
    item = _diagnose(_navigation(), width=0.65)
    assert item.classification == "STRUCTURAL_WIDTH_BLOCKED"
    assert item.required_preview_width_m == 0.70
