from pathlib import Path
import sys

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from static_obstacle_evidence import select_unique_cells  # noqa: E402


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
        min_relative_height=0.05,
        max_relative_height=2.0,
        self_filter_radius=0.75,
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
        min_relative_height=0.05,
        max_relative_height=2.0,
        self_filter_radius=0.75,
        resolution=0.05,
    )
    assert cells.tolist() == [[20, 20]]


def test_offline_launch_preserves_recorded_raytraced_baseline_by_default():
    source = (PACKAGE_ROOT / "launch" / "offline_static_obstacle_map.launch.py").read_text(
        encoding="utf-8"
    )
    assert '"rebuild_raytraced_baseline"' in source
    assert 'default_value="false"' in source
    assert 'IfCondition(LaunchConfiguration("rebuild_raytraced_baseline"))' in source
    assert '"/agt/map/octomap_occupancy"' in source
    assert '"navigation_footprint"' in source
