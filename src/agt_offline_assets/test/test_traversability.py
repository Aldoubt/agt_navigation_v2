from pathlib import Path
from types import SimpleNamespace

import numpy as np

from agt_offline_assets import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    SiteBoundary,
    TraversabilityConfig,
    derive_traversability_evidence,
    load_navigation_grid,
    load_traversability_evidence,
    write_site_boundary,
    write_traversability_candidate,
)


def _fixture(*, gap_columns=range(6, 10), boundary_max_x=2.0, overrides=()):
    resolution = 0.10
    height, width = 3, 20
    shape = (height, width)
    occupancy = np.full(shape, FREE, dtype=np.uint8)
    ground_valid = np.ones(shape, dtype=bool)
    ground_height = np.zeros(shape, dtype=np.float64)

    for col in gap_columns:
        occupancy[1, col] = UNKNOWN
        ground_valid[1, col] = False
        ground_height[1, col] = np.nan

    navigation = NavigationMapResult(
        resolution_m=resolution,
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=width,
        height=height,
        ground_height_m=ground_height,
        ground_valid=ground_valid,
        point_count=np.full(shape, 6, dtype=np.int32),
        ground_support_count=np.full(shape, 4, dtype=np.int32),
        obstacle_count=np.zeros(shape, dtype=np.int32),
        slope_deg=np.zeros(shape, dtype=np.float64),
        step_m=np.zeros(shape, dtype=np.float64),
        occupancy=occupancy,
        config=GroundRelativeNavigationConfig(
            resolution_m=resolution,
            maximum_slope_deg=15.0,
            maximum_step_m=0.12,
        ),
    )
    structure = SimpleNamespace(
        row_model=SimpleNamespace(
            direction_xy=np.array([1.0, 0.0], dtype=np.float64)
        )
    )
    geometric = np.zeros(shape, dtype=bool)
    geometric[1, 1:19] = True
    corridor = SimpleNamespace(
        aisle_geometric_envelope=geometric,
        row_structural_band=np.zeros(shape, dtype=bool),
    )
    boundary = SiteBoundary(
        frame_id="map",
        outer_boundary_xy=(
            (0.0, 0.0),
            (boundary_max_x, 0.0),
            (boundary_max_x, 0.30),
            (0.0, 0.30),
        ),
        source={"authoring_mode": "WORKBENCH_MANUAL_POLYGON"},
    )
    evidence = derive_traversability_evidence(
        navigation,
        structure,
        corridor,
        boundary,
        TraversabilityConfig(maximum_inferred_gap_m=0.60),
        overrides=overrides,
        frame_id="map",
    )
    return navigation, corridor, evidence


def _full_boundary():
    return SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((0.0, 0.0), (2.0, 0.0), (2.0, 0.30), (0.0, 0.30)),
        source={"authoring_mode": "WORKBENCH_MANUAL_POLYGON"},
    )


def test_short_longitudinal_unknown_gap_is_inferred():
    _navigation, _corridor, evidence = _fixture(gap_columns=range(6, 10))
    assert np.all(evidence.inferred_traversable_mask[1, 6:10])
    assert np.all(evidence.candidate_occupancy()[1, 6:10] == FREE)


def test_long_unknown_gap_remains_unknown():
    _navigation, _corridor, evidence = _fixture(gap_columns=range(5, 14))
    assert not np.any(evidence.inferred_traversable_mask[1, 5:14])
    assert np.all(evidence.candidate_occupancy()[1, 5:14] == UNKNOWN)


def test_row_structural_band_is_not_recovered():
    navigation, corridor, _evidence = _fixture(gap_columns=range(6, 10))
    corridor.row_structural_band[1, 7] = True
    structure = SimpleNamespace(
        row_model=SimpleNamespace(direction_xy=np.array([1.0, 0.0]))
    )
    evidence = derive_traversability_evidence(
        navigation,
        structure,
        corridor,
        _full_boundary(),
        frame_id="map",
    )
    assert not evidence.inferred_traversable_mask[1, 7]


def test_current_occupied_cell_is_never_recovered():
    navigation, corridor, _evidence = _fixture(gap_columns=range(6, 10))
    navigation.occupancy[1, 8] = OCCUPIED
    structure = SimpleNamespace(
        row_model=SimpleNamespace(direction_xy=np.array([1.0, 0.0]))
    )
    evidence = derive_traversability_evidence(
        navigation,
        structure,
        corridor,
        _full_boundary(),
        frame_id="map",
    )
    assert evidence.sensor_obstacle_mask[1, 8]
    assert not evidence.inferred_traversable_mask[1, 8]
    assert evidence.candidate_occupancy()[1, 8] == OCCUPIED


def test_no_go_is_not_recovered():
    no_go = {
        "mode": "no_go",
        "polygon_xy": [[0.60, 0.05], [1.00, 0.05], [1.00, 0.25], [0.60, 0.25]],
    }
    _navigation, _corridor, evidence = _fixture(
        gap_columns=range(6, 10), overrides=(no_go,)
    )
    assert np.any(evidence.semantic_no_go_mask[1, 6:10])
    assert not np.any(
        evidence.inferred_traversable_mask & evidence.semantic_no_go_mask
    )


def test_outside_site_boundary_is_hard_blocked():
    _navigation, _corridor, evidence = _fixture(
        gap_columns=range(6, 10), boundary_max_x=1.50
    )
    assert np.all(evidence.hard_blocked_mask[:, 15:])
    assert np.all(evidence.candidate_occupancy()[:, 15:] == OCCUPIED)


def test_state_precedence_masks_do_not_overlap_inference():
    no_go = {
        "mode": "no_go",
        "polygon_xy": [[0.60, 0.05], [0.80, 0.05], [0.80, 0.25], [0.60, 0.25]],
    }
    navigation, corridor, _evidence = _fixture(
        gap_columns=range(6, 10), overrides=(no_go,)
    )
    navigation.occupancy[1, 8] = OCCUPIED
    boundary = SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((0.0, 0.0), (1.50, 0.0), (1.50, 0.30), (0.0, 0.30)),
    )
    structure = SimpleNamespace(
        row_model=SimpleNamespace(direction_xy=np.array([1.0, 0.0]))
    )
    evidence = derive_traversability_evidence(
        navigation,
        structure,
        corridor,
        boundary,
        overrides=(no_go,),
        frame_id="map",
    )
    assert not np.any(evidence.inferred_traversable_mask & evidence.hard_blocked_mask)
    assert not np.any(
        evidence.inferred_traversable_mask & evidence.semantic_no_go_mask
    )
    assert not np.any(
        evidence.inferred_traversable_mask & evidence.sensor_obstacle_mask
    )


def test_candidate_serialization_round_trip_preserves_canonical_files(tmp_path: Path):
    navigation, _corridor, evidence = _fixture(gap_columns=range(6, 10))
    run_dir = tmp_path / "run"
    run_dir.mkdir()

    canonical = {
        "navigation_map.yaml": b"canonical-yaml\n",
        "navigation_map.pgm": b"canonical-pgm\n",
        "derivation.yaml": b"canonical-derivation\n",
    }
    for name, payload in canonical.items():
        (run_dir / name).write_bytes(payload)

    boundary = _full_boundary()
    write_site_boundary(boundary, run_dir / "site_boundary.yaml")
    before = {name: (run_dir / name).read_bytes() for name in canonical}

    write_traversability_candidate(
        evidence,
        navigation,
        boundary,
        run_dir,
        source_navigation_asset="navigation_map.yaml",
    )

    expected = {
        "traversability_evidence.yaml",
        "traversability_evidence.npz",
        "navigation_map_12f.yaml",
        "navigation_map_12f.pgm",
        "navigation_map_12f_derivation.yaml",
    }
    assert expected.issubset({path.name for path in run_dir.iterdir()})
    assert before == {name: (run_dir / name).read_bytes() for name in canonical}

    loaded = load_traversability_evidence(run_dir, expected_frame_id="map")
    assert np.array_equal(
        loaded.inferred_traversable_mask,
        evidence.inferred_traversable_mask,
    )
    candidate = load_navigation_grid(run_dir / "navigation_map_12f.yaml")
    assert np.array_equal(candidate.occupancy, evidence.candidate_occupancy())
