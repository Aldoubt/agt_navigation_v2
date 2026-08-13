import numpy as np
import pytest

from agt_map_workbench import (
    MAP_FRAME_SCHEMA,
    fit_horizontal_axis_from_corridor,
    fit_vertical_axis_from_cylinder,
    nearest_xyz_in_window,
    solve_map_frame,
)


def test_nearest_xyz_respects_active_z_window():
    x = np.array([0.0, 0.02, 2.0])
    y = np.array([0.0, 0.01, 2.0])
    z = np.array([3.0, 0.1, 0.0])
    point = nearest_xyz_in_window(x, y, z, (0.0, 0.0), z_min=-0.5, z_max=1.0)
    assert np.allclose(point, [0.02, 0.01, 0.1])


def test_horizontal_corridor_fits_wall_direction():
    rng = np.random.default_rng(7)
    x = np.linspace(0.0, 8.0, 500)
    y = 2.0 + rng.normal(0.0, 0.015, x.size)
    z = rng.uniform(-0.5, 1.2, x.size)
    fit = fit_horizontal_axis_from_corridor(
        x,
        y,
        z,
        (0.0, 2.0),
        (8.0, 2.0),
        z_min=-1.0,
        z_max=1.5,
        half_width_m=0.1,
    )
    assert fit.point_count >= 450
    assert fit.direction[0] > 0.99
    assert abs(fit.direction[1]) < 0.03
    assert fit.rms_residual_m < 0.03
    assert fit.linearity_ratio > 50.0


def test_vertical_cylinder_fits_pillar_direction():
    rng = np.random.default_rng(9)
    z = np.linspace(-1.0, 3.0, 600)
    x = 4.0 + rng.normal(0.0, 0.01, z.size)
    y = -2.0 + rng.normal(0.0, 0.01, z.size)
    fit = fit_vertical_axis_from_cylinder(
        x,
        y,
        z,
        (4.0, -2.0),
        z_min=-1.2,
        z_max=3.2,
        radius_m=0.08,
    )
    assert fit.point_count >= 550
    assert fit.direction[2] > 0.99
    assert abs(fit.direction[0]) < 0.03
    assert abs(fit.direction[1]) < 0.03
    assert fit.rms_residual_m < 0.03
    assert fit.linearity_ratio > 50.0


def test_solve_map_frame_is_orthonormal_right_handed_and_transforms_origin():
    calibration = solve_map_frame(
        [10.0, 20.0, 2.0],
        [1.0, 0.1, 0.05],
        [0.02, -0.01, 1.0],
    )
    basis = calibration.basis_source_from_map
    assert np.allclose(basis.T @ basis, np.eye(3), atol=1e-10)
    assert np.linalg.det(basis) == pytest.approx(1.0, abs=1e-10)
    transformed = calibration.transform_points(np.array([[10.0, 20.0, 2.0]]))
    assert np.allclose(transformed, [[0.0, 0.0, 0.0]], atol=1e-10)


def test_flip_x_and_z_preserve_right_handed_frame():
    calibration = solve_map_frame([0, 0, 0], [1, 0, 0], [0, 0, 1])
    flipped_x = calibration.flipped_x()
    flipped_z = calibration.flipped_z()
    assert np.allclose(flipped_x.x_axis_in_source, [-1, 0, 0])
    assert np.linalg.det(flipped_x.basis_source_from_map) == pytest.approx(1.0)
    assert np.allclose(flipped_z.z_axis_in_source, [0, 0, -1])
    assert np.linalg.det(flipped_z.basis_source_from_map) == pytest.approx(1.0)


def test_map_frame_yaml_keeps_georeference_separate(tmp_path):
    calibration = solve_map_frame([1, 2, 3], [1, 0, 0], [0, 0, 1])
    data = calibration.to_dict(source_asset="green house full.pcd")
    assert data["schema"] == MAP_FRAME_SCHEMA
    assert data["target_frame_id"] == "map"
    assert data["georeference"]["status"] == "UNBOUND"
    assert "ENU/UTM" in data["georeference"]["note"]
    path = calibration.write_yaml(tmp_path / "map_frame.yaml")
    assert path.exists()
