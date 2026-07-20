#!/usr/bin/env python3

import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml

from static_obstacle_evidence import rasterize_footprint_cells


def read_p5(path):
    with Path(path).open("rb") as stream:
        if stream.readline().strip() != b"P5":
            raise ValueError("swept-footprint post-processing requires a P5 PGM")
        line = stream.readline()
        while line.startswith(b"#"):
            line = stream.readline()
        width, height = map(int, line.split())
        if int(stream.readline()) != 255:
            raise ValueError("PGM max value must be 255")
        image = np.frombuffer(stream.read(), dtype=np.uint8)
    if image.size != width * height:
        raise ValueError("PGM payload size does not match its dimensions")
    return image.reshape((height, width)).copy()


def apply_swept_cells(image, cells, *, origin_x, origin_y, resolution):
    if not cells:
        return image.copy(), 0
    output = image.copy()
    height, width = output.shape
    cells_array = np.asarray(tuple(cells), dtype=np.int64)
    world_x = (cells_array[:, 0] + 0.5) * resolution
    world_y = (cells_array[:, 1] + 0.5) * resolution
    columns = np.floor((world_x - origin_x) / resolution).astype(np.int64)
    map_rows = np.floor((world_y - origin_y) / resolution).astype(np.int64)
    rows = height - 1 - map_rows
    valid = (columns >= 0) & (columns < width) & (rows >= 0) & (rows < height)
    changed = int(np.count_nonzero(output[rows[valid], columns[valid]] != 254))
    output[rows[valid], columns[valid]] = 254
    return output, changed


def write_p5(path, image, resolution):
    with Path(path).open("wb") as stream:
        stream.write(
            f"P5\n# CREATOR: agt swept footprint {resolution:.3f} m/pix\n"
            f"{image.shape[1]} {image.shape[0]}\n255\n".encode("ascii")
        )
        stream.write(image.tobytes())


def main():
    parser = argparse.ArgumentParser(
        description="Clear the complete recorded robot footprint sweep from a P5 map."
    )
    parser.add_argument("--bag", required=True)
    parser.add_argument("--input-yaml", required=True)
    parser.add_argument("--output-prefix", required=True)
    parser.add_argument("--platform-profile", required=True)
    parser.add_argument("--clearance", type=float, default=0.05)
    args = parser.parse_args()

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    input_yaml = Path(args.input_yaml).resolve()
    output_prefix = Path(args.output_prefix).resolve()
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    metadata = yaml.safe_load(input_yaml.read_text(encoding="utf-8"))
    image_path = (input_yaml.parent / metadata["image"]).resolve()
    output_pgm = output_prefix.with_suffix(".pgm")
    output_yaml = output_prefix.with_suffix(".yaml")
    if output_pgm == image_path or output_yaml == input_yaml:
        raise ValueError("output must not overwrite the input map")
    image = read_p5(image_path)
    resolution = float(metadata["resolution"])
    origin_x, origin_y = map(float, metadata["origin"][:2])
    profile = yaml.safe_load(Path(args.platform_profile).read_text(encoding="utf-8"))
    footprint = profile["platform"]["geometry"]["navigation_footprint"]

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(Path(args.bag).resolve()), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic = "/agt/mapping/odometry"
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise ValueError(f"bag does not contain {topic}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    cells = set()
    poses = 0
    while reader.has_next():
        _, serialized, _ = reader.read_next()
        message = deserialize_message(serialized, get_message(topic_types[topic]))
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        rasterized = rasterize_footprint_cells(
            base_x=position.x,
            base_y=position.y,
            base_yaw=yaw,
            footprint=footprint,
            padding=float(args.clearance),
            resolution=resolution,
        )
        cells.update((int(x), int(y)) for x, y in rasterized)
        poses += 1

    output, changed = apply_swept_cells(
        image,
        cells,
        origin_x=origin_x,
        origin_y=origin_y,
        resolution=resolution,
    )
    write_p5(output_pgm, output, resolution)
    output_metadata = dict(metadata)
    output_metadata["image"] = output_pgm.name
    output_yaml.write_text(
        yaml.safe_dump(output_metadata, sort_keys=False), encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "odometry_poses": poses,
                "swept_cells": len(cells),
                "changed_pixels": changed,
                "output_yaml": str(output_yaml),
            },
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
