import argparse
import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "create_cloudcompare_runtime_map.py"
SPEC = importlib.util.spec_from_file_location("create_cloudcompare_runtime_map", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def arguments(**overrides):
    values = {
        "source_image": Path("map.png"),
        "source_pcd": None,
        "resolution": 0.05,
        "map_id": "test",
        "origin_x": 99.0,
        "origin_y": 88.0,
        "origin_yaw": 0.0,
        "min_center_x": -2.45,
        "min_center_y": -30.45,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


def test_minimum_cell_centers_are_the_only_origin_source_when_present():
    assert MODULE.resolve_nav_origin(arguments()) == pytest.approx([-2.475, -30.475, 0.0])


def test_minimum_cell_centers_must_be_provided_as_a_pair(tmp_path):
    image = tmp_path / "map.png"
    image.write_bytes(b"placeholder")
    args = arguments(source_image=image, min_center_y=None)
    with pytest.raises(ValueError, match="provided together"):
        MODULE.validate_args(args)
