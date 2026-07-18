import importlib.util
from pathlib import Path
import sys

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "apply_pcd_viewpoint.py"
sys.path.insert(0, str(SCRIPT.parent))
SPEC = importlib.util.spec_from_file_location("apply_pcd_viewpoint", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_viewpoint_is_applied_to_xyz_and_normals():
    half_sqrt_two = np.sqrt(0.5)
    matrix = MODULE.viewpoint_matrix(
        ["1", "2", "3", str(half_sqrt_two), "0", "0", str(half_sqrt_two)]
    )
    columns = {
        "x": np.array([1.0], dtype=np.float32),
        "y": np.array([0.0], dtype=np.float32),
        "z": np.array([4.0], dtype=np.float32),
        "Coord._Z": np.array([4.0], dtype=np.float32),
        "normal_x": np.array([1.0], dtype=np.float32),
        "normal_y": np.array([0.0], dtype=np.float32),
        "normal_z": np.array([0.0], dtype=np.float32),
    }

    MODULE.transform_columns(columns, matrix)

    assert np.allclose([columns["x"][0], columns["y"][0], columns["z"][0]], [1, 3, 7])
    assert np.isclose(columns["Coord._Z"][0], 7)
    assert np.allclose(
        [columns["normal_x"][0], columns["normal_y"][0], columns["normal_z"][0]],
        [0, 1, 0],
        atol=1e-6,
    )
