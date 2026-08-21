import json
from pathlib import Path

import numpy as np
import pytest

from agt_map_pipeline.canonical_frame import (
    NavigationGridSpec,
    load_alignment_spec,
    load_nav2_grid_spec,
    regrid_navigation_result,
    transform_cloud_to_map,
)
from agt_offline_assets.navigation_map_derivation import (
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    UNKNOWN,
)
from agt_offline_assets.pcd_io import PcdCloud, PcdSchema


def _cloud(points):
    schema = PcdSchema(("x", "y", "z"), (4, 4, 4), ("F", "F", "F"), (1, 1, 1), (0, 0, 0, 1, 0, 0, 0))
    data = np.zeros(len(points), dtype=schema.dtype)
    for index, (x, y, z) in enumerate(points):
        data[index]["x"] = x
        data[index]["y"] = y
        data[index]["z"] = z
    return PcdCloud(schema, data, "ascii")


def _write_pgm(path: Path, width: int, height: int):
    path.write_bytes(f"P5\n{width} {height}\n255\n".encode("ascii") + bytes([205]) * width * height)


def test_load_alignment_and_grid_contracts(tmp_path: Path):
    alignment = tmp_path / "alignment.json"
    alignment.write_text(json.dumps({
        "status": "PASS",
        "method": "SITE_CONTROL_POINTS",
        "source_frame": "mapping_session",
        "map_frame": "map",
        "transform": {"translation_xyz_m": [1.0, 2.0, 0.0], "yaw_rad": 0.0},
        "control_points": {"rmse_m": 0.02, "max_residual_m": 0.04},
    }))
    spec = load_alignment_spec(alignment)
    assert spec.source_frame_id == "mapping_session"
    assert spec.target_frame_id == "map"
    assert spec.translation_xyz_m == (1.0, 2.0, 0.0)
    assert len(spec.source_sha256) == 64

    image = tmp_path / "map.pgm"
    _write_pgm(image, 8, 6)
    map_yaml = tmp_path / "map.yaml"
    map_yaml.write_text("image: map.pgm\nresolution: 0.1\norigin: [-0.4, -0.3, 0.0]\n")
    grid = load_nav2_grid_spec(map_yaml)
    assert grid == NavigationGridSpec("map", 0.1, -0.4, -0.3, 8, 6)


def test_alignment_rejects_unverified_or_non_map_target(tmp_path: Path):
    path = tmp_path / "alignment.yaml"
    path.write_text("status: PENDING\nsource_frame: mapping_session\nmap_frame: map\ntransform:\n  yaw_rad: 0.0\n  translation_xyz_m: [0, 0, 0]\n")
    with pytest.raises(ValueError, match="PASS"):
        load_alignment_spec(path)


def test_transform_cloud_applies_rigid_transform_and_crops_to_grid(tmp_path: Path):
    alignment = tmp_path / "alignment.yaml"
    alignment.write_text("status: PASS\nmethod: TEST\nsource_frame: session\nmap_frame: map\ntransform:\n  yaw_rad: 0.0\n  translation_xyz_m: [1.0, 2.0, 0.0]\n")
    spec = load_alignment_spec(alignment)
    grid = NavigationGridSpec("map", 0.1, 1.0, 2.0, 10, 10)
    transformed = transform_cloud_to_map(_cloud([(0.1, 0.1, 0.0), (10.0, 10.0, 0.0)]), spec, grid)
    xyz = transformed.xyz()
    assert xyz.shape == (1, 3)
    assert np.allclose(xyz[0], [1.1, 2.1, 0.0])


def test_regrid_navigation_result_preserves_cells_and_fills_uncovered_unknown():
    cfg = GroundRelativeNavigationConfig(resolution_m=0.1)
    shape = (2, 2)
    result = NavigationMapResult(
        resolution_m=0.1,
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=2,
        height=2,
        ground_height_m=np.array([[1.0, 2.0], [3.0, 4.0]]),
        ground_valid=np.ones(shape, dtype=bool),
        point_count=np.ones(shape, dtype=np.int32),
        ground_support_count=np.ones(shape, dtype=np.int32),
        obstacle_count=np.zeros(shape, dtype=np.int32),
        slope_deg=np.zeros(shape),
        step_m=np.zeros(shape),
        occupancy=np.array([[254, 0], [205, 254]], dtype=np.uint8),
        config=cfg,
    )
    target = NavigationGridSpec("map", 0.1, -0.1, -0.1, 4, 4)
    regridded = regrid_navigation_result(result, target)
    assert (regridded.origin_x_m, regridded.origin_y_m, regridded.width, regridded.height) == (-0.1, -0.1, 4, 4)
    assert np.array_equal(regridded.occupancy[1:3, 1:3], result.occupancy)
    assert regridded.occupancy[0, 0] == UNKNOWN
    assert np.isnan(regridded.ground_height_m[0, 0])
    assert regridded.point_count[0, 0] == 0
