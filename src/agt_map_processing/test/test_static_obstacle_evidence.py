from pathlib import Path
import sys

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from static_obstacle_evidence import (  # noqa: E402
    inside_or_near_polygon,
    interpolate_pose,
    rasterize_footprint_cells,
    select_unique_cells,
)
from apply_swept_footprint_to_map import apply_swept_cells, read_p5, write_p5  # noqa: E402


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
