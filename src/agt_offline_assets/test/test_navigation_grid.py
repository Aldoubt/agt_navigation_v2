from pathlib import Path

import numpy as np
import yaml

from agt_offline_assets.navigation_grid import load_navigation_grid
from agt_offline_assets.navigation_map_derivation import FREE, OCCUPIED, UNKNOWN


def _write_fixture(tmp_path: Path, world_occupancy: np.ndarray) -> Path:
    pgm_image = np.flipud(np.asarray(world_occupancy, dtype=np.uint8))
    pgm = tmp_path / "navigation_map.pgm"
    header = f"P5\n# fixture\n{pgm_image.shape[1]} {pgm_image.shape[0]}\n255\n".encode("ascii")
    pgm.write_bytes(header + pgm_image.tobytes(order="C"))
    nav = {
        "image": pgm.name,
        "mode": "trinary",
        "resolution": 0.1,
        "origin": [-1.0, -2.0, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    (tmp_path / "navigation_map.yaml").write_text(
        yaml.safe_dump(nav, sort_keys=False), encoding="utf-8"
    )
    return tmp_path


def test_load_navigation_grid_restores_world_row_order(tmp_path):
    world = np.array(
        [
            [FREE, OCCUPIED, UNKNOWN],
            [UNKNOWN, FREE, OCCUPIED],
        ],
        dtype=np.uint8,
    )
    directory = _write_fixture(tmp_path, world)
    grid = load_navigation_grid(directory)
    assert grid.resolution_m == 0.1
    assert grid.origin_x_m == -1.0
    assert grid.origin_y_m == -2.0
    assert grid.width == 3
    assert grid.height == 2
    assert np.array_equal(grid.occupancy, world)
    assert grid.counts() == {"free": 2, "occupied": 2, "unknown": 2}
    assert "navigation_pgm_sha256" in grid.source


def test_load_navigation_grid_accepts_yaml_path(tmp_path):
    world = np.full((3, 4), FREE, dtype=np.uint8)
    directory = _write_fixture(tmp_path, world)
    grid = load_navigation_grid(directory / "navigation_map.yaml")
    assert grid.bounds_m() == (-1.0, -2.0, -0.6, -1.7)
