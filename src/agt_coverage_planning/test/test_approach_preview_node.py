import importlib.util
import json
from pathlib import Path
import sys

import pytest
import rclpy
from rclpy.parameter import Parameter


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPOSITORY_ROOT / "src/agt_ui_bridge"))
SCRIPT = PACKAGE_ROOT / "scripts/coverage_approach_preview.py"
SPEC = importlib.util.spec_from_file_location("coverage_approach_preview", SCRIPT)
APPROACH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(APPROACH)
PROFILE = REPOSITORY_ROOT / "profiles/platforms/greenhouse_ackermann.yaml"


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_approach_preview_loads_entry_and_exposes_preview_only_topics(tmp_path):
    semantic_path = tmp_path / "semantic_map.geojson"
    semantic_path.write_text(
        json.dumps(
            {
                "type": "FeatureCollection",
                "schema_version": "1.0",
                "map_id": "test",
                "frame_id": "map",
                "features": [
                    {
                        "type": "Feature",
                        "geometry": {"type": "Point", "coordinates": [1.0, 2.0]},
                        "properties": {
                            "id": "entry_01",
                            "feature_type": "entry_pose",
                            "name": "entry",
                            "enabled": True,
                            "frame_id": "map",
                            "yaw": 0.25,
                        },
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    parameters = [
        Parameter("semantic_map", value=str(semantic_path)),
        Parameter("platform_profile", value=str(PROFILE)),
        Parameter("planner_action", value="/agt/test/unavailable_approach_planner"),
    ]
    node = APPROACH.CoverageApproachPreview(parameter_overrides=parameters)
    try:
        assert node.entry_pose == APPROACH.Pose2D(1.0, 2.0, 0.25)
        assert node.repair_policy.planner_id == "CoverageRepairHybrid"
        topics = dict(node.get_topic_names_and_types())
        assert topics["/agt/coverage/path_approach_preview"] == ["nav_msgs/msg/Path"]
        assert topics["/agt/coverage/approach_report"] == ["std_msgs/msg/String"]
    finally:
        node.destroy_node()
