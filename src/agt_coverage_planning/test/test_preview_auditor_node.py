import importlib.util
from pathlib import Path
import sys

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rclpy.parameter import Parameter
import pytest
import rclpy


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPOSITORY_ROOT / "src/agt_ui_bridge"))
SCRIPT = PACKAGE_ROOT / "scripts/coverage_preview_auditor.py"
SPEC = importlib.util.spec_from_file_location("coverage_preview_auditor", SCRIPT)
AUDITOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDITOR)
PROFILE = REPOSITORY_ROOT / "profiles/platforms/greenhouse_ackermann.yaml"


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _grid(cells=None):
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.info.width = 100
    message.info.height = 30
    message.info.resolution = 0.1
    message.info.origin.orientation.w = 1.0
    data = [0] * 3000
    for column, row, value in cells or []:
        data[row * 100 + column] = value
    message.data = data
    return message


def _path():
    message = NavPath()
    message.header.frame_id = "map"
    for x_value in (1.0, 8.0):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x_value
        pose.pose.position.y = 1.0
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
    return message


def test_node_publishes_advisory_report_and_collision_poses():
    node = AUDITOR.CoveragePreviewAuditor(
        parameter_overrides=[Parameter("platform_profile", value=str(PROFILE))]
    )
    try:
        node._base_map_callback(_grid(cells=[(30, 10, 100)]))
        node._keepout_callback(_grid(cells=[(60, 10, 100)]))
        node._path_callback(_path())

        assert node.last_report["status"] == "CONFLICT"
        assert node.last_report["eligible_for_execution"] is False
        assert node.last_report["base_collision_pose_count"] > 0
        assert node.last_report["keepout_collision_pose_count"] > 0
        assert node.last_report["path_stamp_ns"] == 0
        assert node.last_collisions.poses
        topics = dict(node.get_topic_names_and_types())
        assert topics["/agt/coverage/preview_audit"] == ["std_msgs/msg/String"]
        assert topics["/agt/coverage/preview_collision_poses"] == [
            "geometry_msgs/msg/PoseArray"
        ]
    finally:
        node.destroy_node()
