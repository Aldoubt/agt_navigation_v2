import ast
from pathlib import Path
import sys

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))

from octomap_cloud_throttle import LatestCloudSlot, ProjectionGate  # noqa: E402


def test_launch_files_parse():
    for launch_file in (PACKAGE_ROOT / "launch").glob("*.launch.py"):
        ast.parse(launch_file.read_text(encoding="utf-8"), filename=str(launch_file))


def test_octomap_consumes_lidar_frame_cloud_for_dynamic_sensor_origin():
    launch_source = (
        PACKAGE_ROOT / "launch" / "octomap_projection.launch.py"
    ).read_text(encoding="utf-8")
    assert 'default_value="/agt/mapping/registered_points_lidar"' in launch_source
    assert 'default_value="/agt/map/mapping_occupancy"' in launch_source
    assert 'default_value="/agt/map/mapping_occupancy_raw"' in launch_source
    assert 'default_value="/agt/mapping/octomap_points"' in launch_source
    assert 'default_value="0.2"' in launch_source
    assert 'default_value="0.10"' in launch_source
    assert 'default_value="8000"' in launch_source
    assert 'default_value="60.0"' in launch_source
    assert 'executable="octomap_cloud_throttle.py"' in launch_source
    assert '("projected_map", LaunchConfiguration("raw_map_topic"))' in launch_source


def test_octomap_throttle_replaces_pending_cloud_with_the_latest_message():
    slot = LatestCloudSlot()
    old = object()
    latest = object()

    slot.update(old)
    slot.update(latest)

    assert slot.take() is latest
    assert slot.take() is None
    assert slot.received == 2
    assert slot.superseded == 1


def test_octomap_throttle_uses_a_steady_timer_and_preserves_message_headers():
    source = (PACKAGE_ROOT / "scripts" / "octomap_cloud_throttle.py").read_text(
        encoding="utf-8"
    )
    assert "ClockType.STEADY_TIME" in source
    assert "self._pending.update(message)" in source
    assert "point_cloud2.create_cloud_xyz32(message.header" in source
    assert "self._projection_gate.acknowledge()" in source
    assert "self._map_publisher.publish(message)" in source
    assert "OctoMap projection acknowledgement timed out" in source


def test_octomap_projection_gate_waits_for_grid_ack_with_bounded_recovery():
    gate = ProjectionGate()

    assert gate.ready(10.0, 60.0) is True
    gate.mark_published(10.0)
    assert gate.ready(69.9, 60.0) is False
    gate.acknowledge()
    assert gate.ready(20.0, 60.0) is True

    gate.mark_published(20.0)
    assert gate.ready(80.0, 60.0) is True
    assert gate.timeouts == 1


def test_octomap_projection_uses_v2_frame_contract():
    parameters = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "octomap_projection.yaml").read_text(
            encoding="utf-8"
        )
    )["/**"]["ros__parameters"]

    assert parameters["frame_id"] == "odom"
    assert parameters["base_frame_id"] == "base_footprint"
    assert parameters["resolution"] > 0.0
    assert parameters["point_cloud_min_z"] < parameters["point_cloud_max_z"]
    assert "pointcloud_min_z" not in parameters
    assert "pointcloud_max_z" not in parameters
    assert "filter_ground" not in parameters
    assert parameters["filter_ground_plane"] is False
    assert parameters["occupancy_min_z"] < parameters["occupancy_max_z"]
    assert parameters["sensor_model.max_range"] <= 15.0
    assert parameters["incremental_2D_projection"] is False
    assert parameters["latch"] is False


def test_map_saver_preserves_pgm_unknown_on_nav2_reload():
    launch_source = (
        PACKAGE_ROOT / "launch" / "save_occupancy_map.launch.py"
    ).read_text(encoding="utf-8")
    assert "free_thresh_default:=0.196" in launch_source
    assert "occupied_thresh_default:=0.65" in launch_source
    assert "refusing to overwrite existing map output" in launch_source


def test_mapping_adapter_output_frame_matches_octomap_frame():
    adapter_config = yaml.safe_load(
        (
            PACKAGE_ROOT.parent
            / "agt_mapping"
            / "config"
            / "fast_livo2_adapter.yaml"
        ).read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    map_config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "octomap_projection.yaml").read_text(
            encoding="utf-8"
        )
    )["/**"]["ros__parameters"]

    assert adapter_config["registered_points_frame"] == map_config["frame_id"]
