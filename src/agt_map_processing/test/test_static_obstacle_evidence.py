from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageDraw


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from static_obstacle_evidence import (  # noqa: E402
    inside_or_near_polygon,
    interpolate_pose,
    rasterize_footprint_cells,
    select_unique_cells,
)
from apply_swept_footprint_to_map import apply_swept_cells, read_p5, write_p5  # noqa: E402
from generate_traversability_variants import (  # noqa: E402
    expand_baseline,
    fit_ground_plane,
    raytrace_free_scan,
    select_cells,
    update_cell_statistics,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_relative_height_filter_tracks_base_z_and_rejects_nonfinite():
    points = np.asarray(
        [
            [2.0, 0.0, -0.96],  # 0.04 m: below obstacle band
            [2.0, 0.0, -0.90],  # 0.10 m: obstacle
            [3.0, 0.0, 1.10],   # 2.10 m: above obstacle band
            [4.0, 0.0, np.nan],
        ],
        dtype=np.float32,
    )
    cells = select_unique_cells(
        points,
        base_x=0.0,
        base_y=0.0,
        base_z=-1.0,
        base_yaw=0.0,
        min_relative_height=0.05,
        max_relative_height=2.0,
        footprint=[[-0.6, -0.47], [0.6, -0.47], [0.6, 0.47], [-0.6, 0.47]],
        self_filter_padding=0.12,
        resolution=0.05,
    )
    assert cells.tolist() == [[40, 0]]


def test_self_filter_and_per_cloud_cell_deduplication():
    points = np.asarray(
        [
            [0.2, 0.0, 0.5],
            [1.01, 1.01, 0.5],
            [1.02, 1.02, 0.6],
        ],
        dtype=np.float32,
    )
    cells = select_unique_cells(
        points,
        base_x=0.0,
        base_y=0.0,
        base_z=0.0,
        base_yaw=0.0,
        min_relative_height=0.05,
        max_relative_height=2.0,
        footprint=[[-0.6, -0.47], [0.6, -0.47], [0.6, 0.47], [-0.6, 0.47]],
        self_filter_padding=0.12,
        resolution=0.05,
    )
    assert cells.tolist() == [[20, 20]]


def test_polygon_filter_uses_vehicle_yaw_and_explicit_padding():
    polygon = np.asarray([[-0.6, -0.47], [0.6, -0.47], [0.6, 0.47], [-0.6, 0.47]])
    local = np.asarray([[0.0, 0.0], [0.0, 0.55], [0.0, 0.70]])
    assert inside_or_near_polygon(local, polygon, 0.12).tolist() == [True, True, False]
    points = np.asarray([[0.0, 0.5, 0.4], [0.7, 0.0, 0.4]], dtype=np.float32)
    cells = select_unique_cells(
        points,
        base_x=0.0,
        base_y=0.0,
        base_z=0.0,
        base_yaw=np.pi / 2.0,
        min_relative_height=0.05,
        max_relative_height=2.0,
        footprint=polygon,
        self_filter_padding=0.12,
        resolution=0.05,
    )
    assert cells.tolist() == [[14, 0]]


def test_pose_interpolation_handles_translation_and_wrapped_yaw():
    samples = [
        (10.0, 0.0, 0.0, -1.0, np.deg2rad(170.0)),
        (10.2, 2.0, 4.0, -0.8, np.deg2rad(-170.0)),
    ]
    pose = interpolate_pose(samples, 10.1, 0.15)
    assert np.allclose(pose[:3], [1.0, 2.0, -0.9])
    assert abs(abs(pose[3]) - np.pi) < 1e-6
    assert interpolate_pose(samples, 9.0, 0.25) is None


def test_swept_footprint_rasterization_rotates_canonical_polygon():
    footprint = [[-0.6, -0.4], [0.6, -0.4], [0.6, 0.4], [-0.6, 0.4]]
    cells = rasterize_footprint_cells(
        base_x=2.0,
        base_y=3.0,
        base_yaw=np.pi / 2.0,
        footprint=footprint,
        padding=0.0,
        resolution=0.1,
    )
    centers = (cells.astype(float) + 0.5) * 0.1
    assert centers[:, 0].min() >= 1.6
    assert centers[:, 0].max() <= 2.4
    assert centers[:, 1].min() >= 2.4
    assert centers[:, 1].max() <= 3.6
    assert len(cells) == 96


def test_swept_cells_clear_pgm_rows_without_overwriting_input(tmp_path):
    image = np.zeros((4, 4), dtype=np.uint8)
    output, changed = apply_swept_cells(
        image,
        {(0, 0), (3, 3)},
        origin_x=0.0,
        origin_y=0.0,
        resolution=1.0,
    )
    assert changed == 2
    assert output[3, 0] == 254
    assert output[0, 3] == 254
    assert image[3, 0] == 0
    path = tmp_path / "swept.pgm"
    write_p5(path, output, 1.0)
    assert np.array_equal(read_p5(path), output)


def test_ground_plane_ransac_rejects_obstacle_outliers():
    x, y = np.meshgrid(np.linspace(-5.0, 5.0, 30), np.linspace(-4.0, 4.0, 25))
    z = 0.02 * x - 0.01 * y - 0.04
    ground = np.column_stack((x.reshape(-1), y.reshape(-1), z.reshape(-1)))
    obstacles = ground[:80].copy()
    obstacles[:, 2] += 0.6
    fitted = fit_ground_plane(np.vstack((ground, obstacles)), seed=9)
    assert fitted is not None
    coefficients, ratio = fitted
    assert np.allclose(coefficients, [0.02, -0.01, -0.04], atol=0.01)
    assert ratio > 0.8


def test_temporal_cell_filter_requires_observation_span():
    statistics = {}
    cells = np.asarray([[2, 3], [4, 5]], dtype=np.int64)
    update_cell_statistics(statistics, cells, 10.0)
    update_cell_statistics(statistics, cells, 10.1)
    update_cell_statistics(statistics, np.asarray([[2, 3]]), 10.8)
    assert select_cells(
        statistics, minimum_observations=3, minimum_span=0.5
    ) == {(2, 3)}


def test_expanded_baseline_preserves_pixels_and_adds_only_unknown_space():
    baseline = np.asarray([[0, 205], [254, 205]], dtype=np.uint8)
    expanded, origin_x, origin_y, report = expand_baseline(
        baseline,
        origin_x=0.0,
        origin_y=0.0,
        resolution=1.0,
        odometry=[(0.0, 0.5, 0.5, 0.0, 0.0)],
        evidence_range=2.0,
        padding=1.0,
    )

    assert (origin_x, origin_y) == (-3.0, -3.0)
    assert report["metric_bounds"] == [-3.0, -3.0, 4.0, 4.0]
    assert expanded.shape == (7, 7)
    assert np.array_equal(expanded[2:4, 3:5], baseline)
    preserved = np.zeros_like(expanded, dtype=bool)
    preserved[2:4, 3:5] = True
    assert np.all(expanded[~preserved] == 205)


def test_raytrace_free_scan_marks_only_observed_wedges():
    raster = np.full((21, 21), 205, dtype=np.uint8)
    image = Image.fromarray(raster)
    statistics = raytrace_free_scan(
        ImageDraw.Draw(image),
        np.asarray([[4.0, 0.0, 0.0], [4.0, 0.2, 0.0]], dtype=np.float64),
        pose=(0.0, 0.0, 0.0, 0.0),
        sensor_offset_xy=(0.0, 0.0),
        origin_x=-10.0,
        origin_y=-10.0,
        resolution=1.0,
        width=21,
        height=21,
        maximum_range=5.0,
        angular_resolution=0.1,
        minimum_relative_height=-1.0,
        maximum_relative_height=1.0,
        maximum_gap_bins=2,
    )
    output = np.asarray(image)

    assert statistics["rays"] == 2
    assert np.count_nonzero(output == 254) > 0
    assert output[10, 10] == 254
    assert output[0, 0] == 205


def test_offline_launch_preserves_recorded_raytraced_baseline_by_default():
    source = (PACKAGE_ROOT / "launch" / "offline_static_obstacle_map.launch.py").read_text(
        encoding="utf-8"
    )
    assert '"rebuild_raytraced_baseline"' in source
    assert 'default_value="false"' in source
    assert 'IfCondition(LaunchConfiguration("rebuild_raytraced_baseline"))' in source
    assert '"/agt/map/octomap_occupancy"' in source
    assert '"navigation_footprint"' in source
    assert '"footprint_json": json.dumps(footprint)' in source
    assert '"self_filter_padding": 0.12' in source
    assert '"clear_swept_footprint": True' in source
    assert '"sweep_clearance": 0.05' in source
