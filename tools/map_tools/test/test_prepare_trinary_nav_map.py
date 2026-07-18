import importlib.util
from pathlib import Path

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "prepare_trinary_nav_map.py"
SPEC = importlib.util.spec_from_file_location("prepare_trinary_nav_map", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_explicit_alpha_point_mask_preserves_points_and_frees_enclosed_space():
    points = np.zeros((15, 15), dtype=bool)
    points[2, 2:13] = True
    points[12, 2:13] = True
    points[2:13, 2] = True
    points[2:13, 12] = True
    points[7, 7] = True

    result = MODULE.classify_point_mask(points, closure_size=1)

    assert result[7, 7] == MODULE.OCCUPIED_PIXEL
    assert result[6, 6] == MODULE.FREE_PIXEL
    assert result[0, 0] == MODULE.UNKNOWN_PIXEL
