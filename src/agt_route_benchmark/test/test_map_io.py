from pathlib import Path
import numpy as np
import pytest

from agt_route_benchmark.map_io import load_nav2_map


def test_nav2_map_loads_image_and_world_extent(tmp_path: Path):
    image = tmp_path / "map.pgm"
    image.write_text("P2\n3 2\n255\n0 127 255\n255 127 0\n", encoding="ascii")
    yaml_path = tmp_path / "map.yaml"
    yaml_path.write_text(
        "image: map.pgm\nresolution: 0.5\norigin: [10.0, -2.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="utf-8",
    )
    loaded = load_nav2_map(yaml_path)
    assert loaded.resolution_m == 0.5
    assert loaded.origin == (10.0, -2.0, 0.0)
    assert loaded.image.shape == (2, 3)
    assert loaded.extent == (10.0, 11.5, -2.0, -1.0)
    assert np.asarray(loaded.image)[0, 0] == 0
    assert np.asarray(loaded.image)[1, 0] == 255


def test_nav2_map_rejects_rotated_origin_for_axis_aligned_paper_renderer(tmp_path: Path):
    image = tmp_path / "map.pgm"
    image.write_text("P2\n1 1\n255\n255\n", encoding="ascii")
    yaml_path = tmp_path / "map.yaml"
    yaml_path.write_text("image: map.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.2]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n", encoding="utf-8")
    with pytest.raises(ValueError, match="rotated map origin"):
        load_nav2_map(yaml_path)


def test_renderer_can_use_raw_nav2_image_with_upper_origin(tmp_path: Path):
    from agt_route_benchmark.contracts import PathPoint
    from agt_route_benchmark.renderer import render_route

    image = tmp_path / "map.pgm"
    image.write_text("P2\n2 2\n255\n0 255\n255 0\n", encoding="ascii")
    yaml_path = tmp_path / "map.yaml"
    yaml_path.write_text("image: map.pgm\nresolution: 1.0\norigin: [0.0, 0.0, 0.0]\n", encoding="utf-8")
    nav_map = load_nav2_map(yaml_path)
    outputs = render_route(
        [PathPoint(0.5, 0.5, 0.0, "F", "P2P", ""), PathPoint(1.5, 1.5, 0.0, "F", "P2P", "")],
        tmp_path / "map_route",
        title="map overlay",
        map_extent=nav_map.extent,
        occupancy_image=nav_map.image,
        image_origin="upper",
    )
    assert all(p.exists() and p.stat().st_size > 0 for p in outputs)
