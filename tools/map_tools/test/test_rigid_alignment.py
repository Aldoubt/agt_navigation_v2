import math
import sys
from pathlib import Path

import numpy as np
import pytest


MODULE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MODULE_DIR))

from rigid_alignment import solve_rigid_alignment  # noqa: E402


def test_four_points_recover_rotation_and_translation_without_scale():
    source = np.array([[-4.0, -2.0], [5.0, -1.0], [4.0, 7.0], [-3.0, 5.0]])
    angle = math.radians(7.5)
    rotation = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
    translation = np.array([1.25, -0.8])
    target = (rotation @ source.T).T + translation

    result = solve_rigid_alignment(source, target)

    np.testing.assert_allclose(result.rotation, rotation, atol=1e-12)
    np.testing.assert_allclose(result.translation, translation, atol=1e-12)
    assert result.rms_error < 1e-12
    assert result.diagnostic_scale == pytest.approx(1.0)
    np.testing.assert_allclose(result.matrix_4x4[:2, 3], translation)


def test_scale_mismatch_is_diagnostic_but_not_applied():
    source = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 4.0], [0.0, 4.0]])
    target = source * 1.1 + np.array([2.0, 3.0])

    result = solve_rigid_alignment(source, target)

    assert result.diagnostic_scale == pytest.approx(1.1)
    assert result.rms_error > 0.1
    assert np.linalg.det(result.rotation) == pytest.approx(1.0)


def test_collinear_landmarks_are_rejected():
    points = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]])
    with pytest.raises(ValueError, match="collinear"):
        solve_rigid_alignment(points, points)
