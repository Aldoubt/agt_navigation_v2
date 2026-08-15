import math

import numpy as np

from agt_offline_assets.agricultural_aisle_graph import AislePrimitive, AgriculturalAisleGraph
from agt_offline_assets.navigation_grid import NavigationGridEvidence
from agt_offline_assets.navigation_map_derivation import FREE
from agt_offline_assets.vehicle_profile import CanonicalVehicleProfile
from agt_offline_assets.vehicle_safe_lane_direct_source_sensitivity import (
    DirectSourceSensitivityConfig,
    derive_vehicle_safe_lane_direct_source_sensitivity_from_masks,
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


def _graph():
    points = tuple((float(x), 0.0, 0.0) for x in np.linspace(1.0, 5.0, 41))
    aisle = AislePrimitive(
        aisle_id="aisle_001",
        kind="interior",
        pair_kind="ROW_ROW",
        left_structure_ref="row_01",
        right_structure_ref="row_02",
        centerline_xyz=points,
        start_pose=(1.0, 0.0, 0.0, 0.0),
        end_pose=(5.0, 0.0, 0.0, 0.0),
        length_m=4.0,
        geometric_width_m=1.2,
        minimum_required_width_m=0.45,
        center_distance_m=1.6,
        longitudinal_overlap_m=4.0,
        safe_cell_count=100,
        centerline_cell_count=41,
        diagnostic_status="ACCEPTED",
    )
    return AgriculturalAisleGraph(
        frame_id="map",
        row_direction_xy=(1.0, 0.0),
        nominal_row_spacing_m=1.6,
        aisles=(aisle,),
    )


def _navigation():
    return NavigationGridEvidence(
        resolution_m=0.05,
        origin_x_m=0.0,
        origin_y_m=-1.0,
        width=140,
        height=40,
        occupancy=np.full((40, 140), FREE, dtype=np.uint8),
        frame_id="map",
    )


def _run(raw_mask, geometry_mask):
    navigation = _navigation()
    shape = navigation.occupancy.shape
    return derive_vehicle_safe_lane_direct_source_sensitivity_from_masks(
        _graph(),
        navigation,
        _vehicle(),
        raw_mask,
        geometry_mask,
        np.ones(shape, dtype=bool),
        np.full(shape, 3, dtype=np.int32),
        minimum_ground_support_points=2,
        config=DirectSourceSensitivityConfig(
            sample_spacing_m=0.10,
            lateral_search_step_m=0.05,
            maximum_lateral_shift_m=0.25,
            preview_footprint_padding_m=0.0,
        ),
    )


def _fraction(plan, case_id):
    case = next(case for case in plan.cases if case.case_id == case_id)
    return case.aisles[0].any_lateral_footprint_free_fraction


def test_raw_obstacle_only_case_isolated_from_geometry_case():
    navigation = _navigation()
    shape = navigation.occupancy.shape
    raw = np.zeros(shape, dtype=bool)
    geometry = np.zeros(shape, dtype=bool)

    center_row = int((0.0 - navigation.origin_y_m) / navigation.resolution_m)
    raw[center_row, :] = True

    plan = _run(raw, geometry)

    assert _fraction(plan, "NO_DIRECT_OCCUPIED") == 1.0
    assert _fraction(plan, "GEOMETRY_ONLY") == 1.0
    assert _fraction(plan, "RAW_OBSTACLE_ONLY") == 0.0
    assert _fraction(plan, "RAW_PLUS_GEOMETRY") == 0.0


def test_geometry_only_case_isolated_from_raw_case():
    navigation = _navigation()
    shape = navigation.occupancy.shape
    raw = np.zeros(shape, dtype=bool)
    geometry = np.zeros(shape, dtype=bool)

    center_row = int((0.0 - navigation.origin_y_m) / navigation.resolution_m)
    geometry[center_row, :] = True

    plan = _run(raw, geometry)

    assert _fraction(plan, "NO_DIRECT_OCCUPIED") == 1.0
    assert _fraction(plan, "RAW_OBSTACLE_ONLY") == 1.0
    assert _fraction(plan, "GEOMETRY_ONLY") == 0.0
    assert _fraction(plan, "RAW_PLUS_GEOMETRY") == 0.0


def test_source_masks_and_production_navigation_are_not_mutated():
    navigation = _navigation()
    shape = navigation.occupancy.shape
    raw = np.zeros(shape, dtype=bool)
    geometry = np.zeros(shape, dtype=bool)
    raw_before = raw.copy()
    geometry_before = geometry.copy()
    occupancy_before = navigation.occupancy.copy()

    derive_vehicle_safe_lane_direct_source_sensitivity_from_masks(
        _graph(),
        navigation,
        _vehicle(),
        raw,
        geometry,
        np.ones(shape, dtype=bool),
        np.full(shape, 3, dtype=np.int32),
        minimum_ground_support_points=2,
    )

    assert np.array_equal(raw, raw_before)
    assert np.array_equal(geometry, geometry_before)
    assert np.array_equal(navigation.occupancy, occupancy_before)
