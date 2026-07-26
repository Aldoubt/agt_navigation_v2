from pathlib import Path

from nav_msgs.msg import Odometry
import pytest
from rclpy.serialization import serialize_message
import rosbag2_py

from agt_teach_repeat.bag_path_extractor import (
    MISSING_ODOMETRY_MESSAGE,
    extract_demo,
    read_odometry_bag,
)
from agt_teach_repeat.path_io import load_manifest, load_reference_path, read_raw_path
from agt_teach_repeat.path_types import ProcessingConfig, TeachRepeatError, TransformSE2


def write_bag(path, topic="/agt/mapping/odometry"):
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name=topic,
            type="nav_msgs/msg/Odometry",
            serialization_format="cdr",
        )
    )
    for index in range(3):
        message = Odometry()
        message.header.frame_id = "odom"
        message.child_frame_id = "base_footprint"
        message.header.stamp.sec = index + 1
        message.pose.pose.position.x = float(index)
        message.pose.pose.orientation.w = 2.0
        message.twist.twist.linear.x = 0.2
        writer.write(topic, serialize_message(message), (index + 1) * 1_000_000_000)
    del writer


def map_assets(tmp_path):
    image = tmp_path / "map.pgm"
    image.write_bytes(b"P5\n1 1\n255\n\xfe")
    map_yaml = tmp_path / "map.yaml"
    map_yaml.write_text("image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n", encoding="utf-8")
    pcd = tmp_path / "localization_map.pcd"
    pcd.write_bytes(b"pcd-data")
    from agt_teach_repeat.path_io import sha256_file

    record = tmp_path / "localization_map.processing.yaml"
    record.write_text(
        f"state: ready\nmap_file: localization_map.pcd\npcd_sha256: {sha256_file(pcd)}\n",
        encoding="utf-8",
    )
    return map_yaml, pcd, record


def test_direct_rosbag_extraction_builds_bound_asset(tmp_path):
    bag = tmp_path / "bag"
    write_bag(bag)
    map_yaml, pcd, record = map_assets(tmp_path)
    output = tmp_path / "runtime" / "teach_repeat" / "route_01"
    manifest = extract_demo(
        bag_path=bag,
        output_demo_dir=output,
        demo_id="route_01",
        odometry_topic="/agt/mapping/odometry",
        platform_profile=Path(__file__).resolve().parents[3] / "profiles/platforms/bunker.yaml",
        map_id="map_01",
        map_yaml=map_yaml,
        localization_pcd=pcd,
        processing_record=record,
        config=ProcessingConfig(resample_distance_m=0.25),
        map_transform=TransformSE2(x=1.0, y=2.0),
    )
    assert len(read_raw_path(output / "raw/raw_path.csv")) == 3
    manifest_path, loaded = load_manifest(output / "manifest.yaml")
    reference = load_reference_path(output / loaded["assets"]["reference_path"])
    assert reference[0].x == pytest.approx(1.0)
    assert reference[0].y == pytest.approx(2.0)
    assert loaded["map"]["localization_pcd_sha256"] == manifest["map"]["localization_pcd_sha256"]
    assert (output / "processed/task_control_points.json").is_file()
    assert (output / "processed/route_annotations.json").is_file()
    assert loaded["assets"]["route_annotations_sha256"].startswith("sha256:")


def test_missing_mapping_odometry_has_actionable_error(tmp_path):
    bag = tmp_path / "bag"
    write_bag(bag, topic="/other/odometry")
    with pytest.raises(TeachRepeatError, match=MISSING_ODOMETRY_MESSAGE):
        read_odometry_bag(bag)


def test_output_cannot_be_nested_inside_source_bag(tmp_path):
    bag = tmp_path / "bag"
    write_bag(bag)
    map_yaml, pcd, record = map_assets(tmp_path)
    with pytest.raises(TeachRepeatError) as error:
        extract_demo(
            bag_path=bag,
            output_demo_dir=bag / "generated_demo",
            demo_id="route_01",
            odometry_topic="/agt/mapping/odometry",
            platform_profile=(
                Path(__file__).resolve().parents[3]
                / "profiles/platforms/bunker.yaml"
            ),
            map_id="map_01",
            map_yaml=map_yaml,
            localization_pcd=pcd,
            processing_record=record,
            config=ProcessingConfig(),
            map_transform=TransformSE2(),
        )
    assert error.value.code == "output_inside_source_bag"
