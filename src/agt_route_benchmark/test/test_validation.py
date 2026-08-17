from pathlib import Path
from types import ModuleType, SimpleNamespace
import sys

from agt_route_benchmark.contracts import PathPoint
from agt_route_benchmark.map_io import load_nav2_map
from agt_route_benchmark.validation import evaluate_normalized_path


def test_validation_bridge_reuses_map_profile_and_validator_contract(tmp_path: Path, monkeypatch):
    image = tmp_path / "map.pgm"
    image.write_text("P2\n2 2\n255\n255 255\n0 255\n", encoding="ascii")
    yaml_path = tmp_path / "map.yaml"
    yaml_path.write_text(
        "image: map.pgm\nresolution: 0.5\norigin: [1.0, 2.0, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="utf-8",
    )
    nav_map = load_nav2_map(yaml_path)
    profile = SimpleNamespace(
        navigation_footprint=((0.42, 0.30), (0.42, -0.30), (-0.42, -0.30), (-0.42, 0.30)),
        min_turning_radius_m=1.5,
    )
    captured = {}

    fake_module = ModuleType("agt_coverage_planning.path_validator")

    class GridMap:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    class Pose2D:
        def __init__(self, x, y, yaw):
            self.x, self.y, self.yaw = x, y, yaw

    class ValidatorConfig:
        pass

    def validate_path(poses, frame, grid, footprint, min_turning_radius, config=None):
        captured.update(
            poses=poses,
            frame=frame,
            grid=grid,
            footprint=footprint,
            min_turning_radius=min_turning_radius,
        )
        report = SimpleNamespace(
            valid=False,
            collision_pose_count=2,
            minimum_clearance=0.08,
            maximum_curvature=0.8,
            in_place_rotation_count=0,
            out_of_bounds_pose_count=0,
            unknown_collision_pose_count=0,
            sample_count=17,
            error_codes=["footprint_collision", "minimum_turning_radius_violation"],
        )
        return SimpleNamespace(report=report)

    fake_module.GridMap = GridMap
    fake_module.Pose2D = Pose2D
    fake_module.ValidatorConfig = ValidatorConfig
    fake_module.validate_path = validate_path
    monkeypatch.setitem(sys.modules, "agt_coverage_planning.path_validator", fake_module)

    metrics = evaluate_normalized_path(
        [
            PathPoint(1.0, 2.0, 0.0, "F", "P2P", ""),
            PathPoint(2.0, 2.0, 0.0, "F", "P2P", ""),
        ],
        nav_map,
        profile,
    )

    assert captured["frame"] == "map"
    assert captured["grid"].width == 2
    assert captured["grid"].height == 2
    assert captured["grid"].resolution == 0.5
    assert captured["grid"].origin_x == 1.0
    assert captured["grid"].origin_y == 2.0
    assert captured["grid"].data == (100, 0, 0, 0)
    assert captured["footprint"] == profile.navigation_footprint
    assert captured["min_turning_radius"] == 1.5
    assert metrics["execution_feasible"] is False
    assert metrics["collision_free"] is False
    assert metrics["kinematic_feasible"] is False
    assert metrics["footprint_collision_count"] == 2
    assert metrics["min_clearance_m"] == 0.08
    assert metrics["required_max_curvature_1pm"] == 1.0 / 1.5
    assert metrics["curvature_excess_1pm"] > 0.0
    assert metrics["max_abs_curvature_1pm"] == metrics["validated_max_abs_curvature_1pm"]
    assert "worst_curvature_segment_index" in metrics
    assert "worst_curvature_ratio_to_limit" in metrics
