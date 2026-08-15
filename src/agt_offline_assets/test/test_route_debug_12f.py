from pathlib import Path

import numpy as np

from agt_offline_assets import (
    FREE,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    SiteBoundary,
    TraversabilityConfig,
    TraversabilityEvidence,
    write_site_boundary,
    write_traversability_candidate,
)
from agt_offline_assets.route_debug_12f import (
    build_route_debug_12f_features,
    load_route_debug_12f,
)


def _candidate_fixture(run_dir: Path):
    shape = (3, 6)
    navigation = NavigationMapResult(
        resolution_m=0.10,
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=6,
        height=3,
        ground_height_m=np.zeros(shape, dtype=np.float64),
        ground_valid=np.ones(shape, dtype=bool),
        point_count=np.ones(shape, dtype=np.int32),
        ground_support_count=np.ones(shape, dtype=np.int32),
        obstacle_count=np.zeros(shape, dtype=np.int32),
        slope_deg=np.zeros(shape, dtype=np.float64),
        step_m=np.zeros(shape, dtype=np.float64),
        occupancy=np.full(shape, FREE, dtype=np.uint8),
        config=GroundRelativeNavigationConfig(resolution_m=0.10),
    )
    boundary = SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((0.0, 0.0), (0.6, 0.0), (0.6, 0.3), (0.0, 0.3)),
        source={"authoring_mode": "WORKBENCH_MANUAL_POLYGON"},
    )
    observed = np.ones(shape, dtype=bool)
    inferred = np.zeros(shape, dtype=bool)
    inferred[1, 2] = True
    observed[1, 2] = False
    evidence = TraversabilityEvidence(
        frame_id="map",
        observed_free_mask=observed,
        inferred_traversable_mask=inferred,
        hard_blocked_mask=np.zeros(shape, dtype=bool),
        sensor_obstacle_mask=np.zeros(shape, dtype=bool),
        unknown_mask=np.zeros(shape, dtype=bool),
        semantic_no_go_mask=np.zeros(shape, dtype=bool),
        aisle_geometric_envelope_mask=np.ones(shape, dtype=bool),
        config=TraversabilityConfig(maximum_inferred_gap_m=0.60),
    )
    write_site_boundary(boundary, run_dir / "site_boundary.yaml")
    write_traversability_candidate(
        evidence,
        navigation,
        boundary,
        run_dir,
        source_navigation_asset="navigation_map.yaml",
    )


def test_missing_12f_assets_degrade_without_error(tmp_path: Path):
    bundle = load_route_debug_12f(tmp_path, expected_frame_id="map")
    assert bundle.candidate_navigation is None
    assert bundle.site_boundary is None
    assert bundle.traversability is None
    assert {state.availability for state in bundle.asset_states} == {"MISSING"}


def test_12f_bundle_loads_candidate_boundary_and_rich_masks(tmp_path: Path):
    _candidate_fixture(tmp_path)
    bundle = load_route_debug_12f(tmp_path, expected_frame_id="map")
    assert bundle.candidate_navigation is not None
    assert bundle.site_boundary is not None
    assert bundle.traversability is not None
    assert bundle.candidate_navigation.occupancy[1, 2] == FREE
    assert bundle.traversability.inferred_traversable_mask[1, 2]
    assert all(state.availability == "LOADED" for state in bundle.asset_states)


def test_site_boundary_feature_uses_dedicated_semantic_layer(tmp_path: Path):
    _candidate_fixture(tmp_path)
    bundle = load_route_debug_12f(tmp_path, expected_frame_id="map")
    features = build_route_debug_12f_features(bundle)
    assert len(features) == 1
    feature = features[0]
    props = feature["properties"]
    assert props["layer_key"] == "semantics.site_boundary"
    assert props["feature_kind"] == "SITE_BOUNDARY"
    assert props["status"] == "READY"
    assert props["boundary_semantics"] == "VEHICLE_PERMITTED_INNER_BOUNDARY"
    assert feature["geometry"]["type"] == "Polygon"
