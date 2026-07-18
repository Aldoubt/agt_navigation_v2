import importlib.util
from pathlib import Path

import yaml


SCRIPT = Path(__file__).resolve().parents[1] / "init_map_production.py"
SPEC = importlib.util.spec_from_file_location("init_map_production", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_reads_pcd_identity_and_bag_topics(tmp_path):
    pcd = tmp_path / "raw.pcd"
    pcd.write_bytes(
        b"VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        b"WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary\n" + b"\0" * 12
    )
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text(
        yaml.safe_dump({"rosbag2_bagfile_information": {
            "duration": {"nanoseconds": 10},
            "message_count": 2,
            "topics_with_message_count": [{
                "topic_metadata": {"name": "/lidar", "type": "PointCloud2"},
                "message_count": 2,
            }],
        }}),
        encoding="utf-8",
    )

    assert MODULE.read_pcd_header(pcd)["points"] == 1
    assert MODULE.read_bag_metadata(bag)["topics"][0]["name"] == "/lidar"
