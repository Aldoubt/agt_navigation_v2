import numpy as np

from agt_offline_assets import (
    CorridorRefinementConfig,
    CorridorRefinementResult,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    VehicleCorridorConfig,
    derive_vehicle_corridor,
)


def _fixture():
    shape = (30, 40)
    navigation = NavigationMapResult(
        resolution_m=0.10,
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=shape[1],
        height=shape[0],
        ground_height_m=np.zeros(shape, dtype=np.float64),
        ground_valid=np.ones(shape, dtype=bool),
        point_count=np.ones(shape, dtype=np.int32),
        ground_support_count=np.ones(shape, dtype=np.int32),
        obstacle_count=np.zeros(shape, dtype=np.int32),
        slope_deg=np.zeros(shape, dtype=np.float64),
        step_m=np.zeros(shape, dtype=np.float64),
        occupancy=np.zeros(shape, dtype=np.uint8),
        config=GroundRelativeNavigationConfig(resolution_m=0.10),
    )
    aisle = np.zeros(shape, dtype=bool)
    aisle[8:22, 3:37] = True
    centerline = np.zeros(shape, dtype=bool)
    centerline[15, 3:37] = True
    corridor = CorridorRefinementResult(
        row_centerline=np.zeros(shape, dtype=bool),
        row_structural_band=np.zeros(shape, dtype=bool),
        vegetation_envelope=np.zeros(shape, dtype=bool),
        boundary_exclusion=np.zeros(shape, dtype=bool),
        aisle_candidate=aisle,
        aisle_centerline=centerline,
        boundary_aisle_candidate=np.zeros(shape, dtype=bool),
        boundary_aisle_centerline=np.zeros(shape, dtype=bool),
        accepted_row_centers_v_m=(0.8, 2.2),
        rejected_row_centers_v_m=(),
        nominal_row_spacing_m=1.4,
        aisle_pair_diagnostics=(),
        config=CorridorRefinementConfig(),
    )
    return navigation, corridor


def test_vehicle_corridor_is_clipped_to_refined_aisle():
    navigation, corridor = _fixture()
    result = derive_vehicle_corridor(
        navigation,
        corridor,
        VehicleCorridorConfig(vehicle_width_m=0.60, lateral_safety_margin_m=0.10),
    )
    assert np.all(~result.corridor_mask | corridor.aisle_candidate)
    assert np.count_nonzero(result.corridor_mask) > np.count_nonzero(corridor.aisle_centerline)
    assert np.isclose(result.required_width_m, 0.80)
    assert np.isclose(result.required_half_width_m, 0.40)
    assert result.covered_centerline_cells == np.count_nonzero(corridor.aisle_centerline)


def test_zero_centerline_produces_empty_review_ribbon():
    navigation, corridor = _fixture()
    corridor = CorridorRefinementResult(
        **{
            **corridor.__dict__,
            "aisle_centerline": np.zeros_like(corridor.aisle_centerline),
        }
    )
    result = derive_vehicle_corridor(navigation, corridor)
    assert np.count_nonzero(result.corridor_mask) == 0
    assert result.covered_centerline_cells == 0


def test_vehicle_corridor_config_validates_width_and_margin():
    for cfg in (
        VehicleCorridorConfig(vehicle_width_m=0.0),
        VehicleCorridorConfig(lateral_safety_margin_m=-0.01),
    ):
        try:
            cfg.validate()
        except ValueError:
            pass
        else:
            raise AssertionError("invalid vehicle corridor config must fail validation")
