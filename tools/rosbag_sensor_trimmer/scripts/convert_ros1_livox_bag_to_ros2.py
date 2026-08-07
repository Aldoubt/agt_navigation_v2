#!/usr/bin/env python3
"""Convert ROS 1 Livox split bags to a ROS 2 rosbag2 dataset.

This converter is intentionally narrow: it reads ROS 1 bag v2 files directly,
decodes Livox CustomMsg and sensor_msgs/Imu payloads, then writes ROS 2 CDR
messages with rosbag2_py. It does not require ROS 1 to be installed.
"""

from __future__ import annotations

import argparse
import bz2
import json
import re
import shutil
import struct
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import BinaryIO, Dict, Iterable, Iterator, List, Optional, Sequence, Tuple

import rosbag2_py
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu


ROS1_LIVOX_CUSTOM = "livox_ros_driver2/CustomMsg"
ROS1_IMU = "sensor_msgs/Imu"
ROS2_LIVOX_CUSTOM = "livox_ros_driver2/msg/CustomMsg"
ROS2_IMU = "sensor_msgs/msg/Imu"

DEFAULT_INPUT_LIDAR_TOPIC = "/livox/lidar"
DEFAULT_INPUT_IMU_TOPIC = "/livox/imu"
DEFAULT_OUTPUT_LIDAR_TOPIC = "/agt/sensors/lidar/custom"
DEFAULT_OUTPUT_IMU_TOPIC = "/agt/sensors/imu/data"

OP_MSG_DATA = 0x02
OP_FILE_HEADER = 0x03
OP_CHUNK = 0x05
OP_CONNECTION = 0x07


class Ros1BagError(RuntimeError):
    """Raised when a ROS 1 bag cannot be decoded."""


@dataclass(frozen=True)
class ConnectionInfo:
    conn_id: int
    topic: str
    datatype: str
    md5sum: str = ""


@dataclass(frozen=True)
class Ros1Message:
    path: Path
    topic: str
    datatype: str
    timestamp_ns: int
    payload: bytes


@dataclass
class TopicStats:
    topic: str
    datatype: str
    count: int = 0
    first_timestamp_ns: Optional[int] = None
    last_timestamp_ns: Optional[int] = None

    def add(self, timestamp_ns: int) -> None:
        self.count += 1
        if self.first_timestamp_ns is None or timestamp_ns < self.first_timestamp_ns:
            self.first_timestamp_ns = timestamp_ns
        if self.last_timestamp_ns is None or timestamp_ns > self.last_timestamp_ns:
            self.last_timestamp_ns = timestamp_ns


@dataclass
class ConversionStats:
    input_files: List[str] = field(default_factory=list)
    output_uri: str = ""
    start_timestamp_ns: Optional[int] = None
    end_timestamp_ns: Optional[int] = None
    read_messages: int = 0
    skipped_messages: int = 0
    written_lidar: int = 0
    written_imu: int = 0
    topic_stats: Dict[str, TopicStats] = field(default_factory=dict)
    warnings: List[str] = field(default_factory=list)

    def add_seen(self, topic: str, datatype: str, timestamp_ns: int) -> None:
        self.read_messages += 1
        key = f"{topic}|{datatype}"
        if key not in self.topic_stats:
            self.topic_stats[key] = TopicStats(topic=topic, datatype=datatype)
        self.topic_stats[key].add(timestamp_ns)
        if self.start_timestamp_ns is None or timestamp_ns < self.start_timestamp_ns:
            self.start_timestamp_ns = timestamp_ns
        if self.end_timestamp_ns is None or timestamp_ns > self.end_timestamp_ns:
            self.end_timestamp_ns = timestamp_ns


class Ros1Buffer:
    def __init__(self, data: bytes):
        self.data = data
        self.offset = 0

    def remaining(self) -> int:
        return len(self.data) - self.offset

    def read(self, size: int) -> bytes:
        if self.offset + size > len(self.data):
            raise Ros1BagError(
                f"truncated ROS 1 message: need {size} bytes, "
                f"remaining {self.remaining()}"
            )
        value = self.data[self.offset:self.offset + size]
        self.offset += size
        return value

    def read_u8(self) -> int:
        return self.read(1)[0]

    def read_u32(self) -> int:
        return struct.unpack("<I", self.read(4))[0]

    def read_u64(self) -> int:
        return struct.unpack("<Q", self.read(8))[0]

    def read_f32(self) -> float:
        return struct.unpack("<f", self.read(4))[0]

    def read_f64(self) -> float:
        return struct.unpack("<d", self.read(8))[0]

    def read_string(self) -> str:
        length = self.read_u32()
        return self.read(length).decode("utf-8", "replace")

    def read_time(self) -> Tuple[int, int]:
        sec, nsec = struct.unpack("<II", self.read(8))
        return int(sec), int(nsec)

    def read_header(self) -> Tuple[int, int, str]:
        _seq = self.read_u32()
        sec, nsec = self.read_time()
        frame_id = self.read_string()
        return sec, nsec, frame_id


def parse_fields(data: bytes) -> Dict[str, bytes]:
    fields: Dict[str, bytes] = {}
    offset = 0
    while offset + 4 <= len(data):
        field_len = struct.unpack_from("<I", data, offset)[0]
        offset += 4
        if offset + field_len > len(data):
            raise Ros1BagError("truncated ROS 1 record header field")
        raw = data[offset:offset + field_len]
        offset += field_len
        if b"=" in raw:
            key, value = raw.split(b"=", 1)
            fields[key.decode("utf-8", "replace")] = value
    return fields


def read_record(stream: BinaryIO) -> Optional[Tuple[Dict[str, bytes], bytes]]:
    header_len_bytes = stream.read(4)
    if not header_len_bytes:
        return None
    if len(header_len_bytes) != 4:
        raise Ros1BagError("truncated ROS 1 record header length")
    header_len = struct.unpack("<I", header_len_bytes)[0]
    header = stream.read(header_len)
    if len(header) != header_len:
        raise Ros1BagError("truncated ROS 1 record header")
    data_len_bytes = stream.read(4)
    if len(data_len_bytes) != 4:
        raise Ros1BagError("truncated ROS 1 record data length")
    data_len = struct.unpack("<I", data_len_bytes)[0]
    data = stream.read(data_len)
    if len(data) != data_len:
        raise Ros1BagError("truncated ROS 1 record data")
    return parse_fields(header), data


def record_op(header: Dict[str, bytes]) -> Optional[int]:
    value = header.get("op")
    return value[0] if value else None


def read_u32_field(header: Dict[str, bytes], name: str) -> Optional[int]:
    value = header.get(name)
    if value is None or len(value) < 4:
        return None
    return struct.unpack("<I", value[:4])[0]


def read_time_field_ns(header: Dict[str, bytes], name: str) -> Optional[int]:
    value = header.get(name)
    if value is None or len(value) < 8:
        return None
    sec, nsec = struct.unpack("<II", value[:8])
    return int(sec) * 1_000_000_000 + int(nsec)


def field_text(header: Dict[str, bytes], name: str, default: str = "") -> str:
    value = header.get(name)
    if value is None:
        return default
    return value.decode("utf-8", "replace")


def decompress_chunk(header: Dict[str, bytes], data: bytes, path: Path) -> bytes:
    compression = field_text(header, "compression", "none")
    if compression in ("none", "NONE", ""):
        return data
    if compression in ("bz2", "BZ2"):
        return bz2.decompress(data)
    if compression.lower() == "lz4":
        raise Ros1BagError(
            f"{path}: lz4-compressed ROS 1 bags are not supported by this "
            "standalone converter; reindex/decompress with ROS 1 first"
        )
    raise Ros1BagError(f"{path}: unsupported ROS 1 chunk compression: {compression}")


def iter_chunk_records(data: bytes, path: Path) -> Iterator[Tuple[Dict[str, bytes], bytes]]:
    offset = 0
    while offset < len(data):
        if offset + 4 > len(data):
            raise Ros1BagError(f"{path}: truncated nested record header length")
        header_len = struct.unpack_from("<I", data, offset)[0]
        offset += 4
        if offset + header_len + 4 > len(data):
            raise Ros1BagError(f"{path}: truncated nested record header")
        header = parse_fields(data[offset:offset + header_len])
        offset += header_len
        data_len = struct.unpack_from("<I", data, offset)[0]
        offset += 4
        if offset + data_len > len(data):
            raise Ros1BagError(f"{path}: truncated nested record data")
        payload = data[offset:offset + data_len]
        offset += data_len
        yield header, payload


def parse_connection(
    header: Dict[str, bytes],
    data: bytes,
    fallback_conn: Optional[int] = None,
) -> Optional[ConnectionInfo]:
    conn_id = read_u32_field(header, "conn")
    if conn_id is None:
        conn_id = fallback_conn
    if conn_id is None:
        return None
    fields = parse_fields(data)
    topic = field_text(header, "topic")
    if not topic:
        topic_value = fields.get("topic")
        topic = topic_value.decode("utf-8", "replace") if topic_value else ""
    datatype_value = fields.get("type")
    datatype = datatype_value.decode("utf-8", "replace") if datatype_value else ""
    md5_value = fields.get("md5sum")
    md5sum = md5_value.decode("utf-8", "replace") if md5_value else ""
    return ConnectionInfo(conn_id=conn_id, topic=topic, datatype=datatype, md5sum=md5sum)


def iter_ros1_messages(
    path: Path,
    wanted_topics: Sequence[str],
    warnings: List[str],
) -> Iterator[Ros1Message]:
    wanted = set(wanted_topics)
    connections: Dict[int, ConnectionInfo] = {}
    with path.open("rb") as stream:
        magic = stream.readline().rstrip(b"\n")
        if magic != b"#ROSBAG V2.0":
            raise Ros1BagError(f"{path}: unsupported bag magic: {magic!r}")
        first = read_record(stream)
        if first is None:
            raise Ros1BagError(f"{path}: missing file header")
        first_header, _first_data = first
        if record_op(first_header) != OP_FILE_HEADER:
            raise Ros1BagError(f"{path}: first record is not a ROS 1 file header")

        while True:
            record = read_record(stream)
            if record is None:
                break
            header, data = record
            op = record_op(header)
            if op == OP_CONNECTION:
                connection = parse_connection(header, data)
                if connection:
                    connections[connection.conn_id] = connection
                continue
            if op != OP_CHUNK:
                continue
            try:
                chunk_data = decompress_chunk(header, data, path)
                for nested_header, nested_data in iter_chunk_records(chunk_data, path):
                    nested_op = record_op(nested_header)
                    if nested_op == OP_CONNECTION:
                        connection = parse_connection(nested_header, nested_data)
                        if connection:
                            connections[connection.conn_id] = connection
                    elif nested_op == OP_MSG_DATA:
                        conn_id = read_u32_field(nested_header, "conn")
                        timestamp_ns = read_time_field_ns(nested_header, "time")
                        if conn_id is None or timestamp_ns is None:
                            continue
                        connection = connections.get(conn_id)
                        if connection is None:
                            continue
                        if connection.topic not in wanted:
                            continue
                        yield Ros1Message(
                            path=path,
                            topic=connection.topic,
                            datatype=connection.datatype,
                            timestamp_ns=timestamp_ns,
                            payload=nested_data,
                        )
            except Ros1BagError as exc:
                warnings.append(str(exc))
                if not path.name.endswith(".active"):
                    raise
                warnings.append(f"{path}: stopped at a truncated active chunk")
                break


def parse_ros1_custom_msg(data: bytes) -> CustomMsg:
    source = Ros1Buffer(data)
    sec, nsec, frame_id = source.read_header()
    output = CustomMsg()
    output.header.stamp.sec = sec
    output.header.stamp.nanosec = nsec
    output.header.frame_id = frame_id
    output.timebase = source.read_u64()
    ros1_point_num = source.read_u32()
    output.lidar_id = source.read_u8()
    output.rsvd = [source.read_u8(), source.read_u8(), source.read_u8()]
    point_count = source.read_u32()
    points: List[CustomPoint] = []
    for _ in range(point_count):
        point = CustomPoint()
        point.offset_time = source.read_u32()
        point.x = source.read_f32()
        point.y = source.read_f32()
        point.z = source.read_f32()
        point.reflectivity = source.read_u8()
        point.tag = source.read_u8()
        point.line = source.read_u8()
        points.append(point)
    output.points = points
    output.point_num = len(points)
    if ros1_point_num != point_count:
        output.point_num = len(points)
    return output


def parse_ros1_imu(data: bytes) -> Imu:
    source = Ros1Buffer(data)
    sec, nsec, frame_id = source.read_header()
    output = Imu()
    output.header.stamp.sec = sec
    output.header.stamp.nanosec = nsec
    output.header.frame_id = frame_id
    output.orientation.x = source.read_f64()
    output.orientation.y = source.read_f64()
    output.orientation.z = source.read_f64()
    output.orientation.w = source.read_f64()
    output.orientation_covariance = [source.read_f64() for _ in range(9)]
    output.angular_velocity.x = source.read_f64()
    output.angular_velocity.y = source.read_f64()
    output.angular_velocity.z = source.read_f64()
    output.angular_velocity_covariance = [source.read_f64() for _ in range(9)]
    output.linear_acceleration.x = source.read_f64()
    output.linear_acceleration.y = source.read_f64()
    output.linear_acceleration.z = source.read_f64()
    output.linear_acceleration_covariance = [source.read_f64() for _ in range(9)]
    return output


def natural_bag_key(path: Path) -> Tuple[int, str]:
    match = re.search(r"data_(\d+)\.bag(?:\.active)?$", path.name)
    if match:
        return int(match.group(1)), path.name
    return sys.maxsize, path.name


def resolve_input_files(inputs: Sequence[Path]) -> List[Path]:
    files: List[Path] = []
    for input_path in inputs:
        path = input_path.expanduser()
        if path.is_dir():
            matches = list(path.glob("*.bag")) + list(path.glob("*.bag.active"))
            if not matches:
                raise Ros1BagError(f"no ROS 1 .bag files found in directory: {path}")
            files.extend(sorted(matches, key=natural_bag_key))
        elif path.is_file():
            files.append(path)
        else:
            raise Ros1BagError(f"input path does not exist: {path}")
    unique: List[Path] = []
    seen = set()
    for path in files:
        resolved = path.resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        unique.append(path)
    return unique


def selected_topics(args: argparse.Namespace) -> Tuple[str, str]:
    return args.input_lidar_topic, args.input_imu_topic


def scan_statistics(
    input_files: Sequence[Path],
    args: argparse.Namespace,
) -> ConversionStats:
    stats = ConversionStats(input_files=[str(path) for path in input_files])
    for path in input_files:
        for message in iter_ros1_messages(path, selected_topics(args), stats.warnings):
            stats.add_seen(message.topic, message.datatype, message.timestamp_ns)
    return stats


def resolve_window_ns(
    args: argparse.Namespace,
    input_files: Sequence[Path],
) -> Tuple[Optional[int], Optional[int], Optional[ConversionStats]]:
    if args.start_ns is not None or args.end_ns is not None:
        return args.start_ns, args.end_ns, None
    needs_relative_scan = (
        args.start is not None or args.end is not None or args.duration is not None
    )
    if not needs_relative_scan:
        return None, None, None
    scanned = scan_statistics(input_files, args)
    if scanned.start_timestamp_ns is None:
        raise Ros1BagError("no selected Livox/IMU messages found while resolving time window")
    start_sec = args.start if args.start is not None else 0.0
    start_ns = scanned.start_timestamp_ns + int(start_sec * 1_000_000_000)
    if args.duration is not None:
        end_ns = start_ns + int(args.duration * 1_000_000_000)
    elif args.end is not None:
        end_ns = scanned.start_timestamp_ns + int(args.end * 1_000_000_000)
    else:
        end_ns = None
    return start_ns, end_ns, scanned


def in_time_window(timestamp_ns: int, start_ns: Optional[int], end_ns: Optional[int]) -> bool:
    if start_ns is not None and timestamp_ns < start_ns:
        return False
    if end_ns is not None and timestamp_ns >= end_ns:
        return False
    return True


def create_writer(args: argparse.Namespace) -> rosbag2_py.SequentialWriter:
    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(
        uri=str(args.output),
        storage_id=args.output_storage,
    )
    converter_options = rosbag2_py.ConverterOptions("cdr", "cdr")
    writer.open(storage_options, converter_options)
    writer.create_topic(rosbag2_py.TopicMetadata(
        name=args.output_lidar_topic,
        type=ROS2_LIVOX_CUSTOM,
        serialization_format="cdr",
    ))
    writer.create_topic(rosbag2_py.TopicMetadata(
        name=args.output_imu_topic,
        type=ROS2_IMU,
        serialization_format="cdr",
    ))
    return writer


def remove_output_if_allowed(output: Path, overwrite: bool) -> None:
    if not output.exists():
        return
    if not overwrite:
        raise Ros1BagError(f"output already exists: {output}")
    if output.is_dir():
        shutil.rmtree(output)
    else:
        output.unlink()


def convert(args: argparse.Namespace, input_files: Sequence[Path]) -> ConversionStats:
    start_ns, end_ns, pre_scan = resolve_window_ns(args, input_files)
    stats = ConversionStats(input_files=[str(path) for path in input_files], output_uri=str(args.output))
    if pre_scan is not None:
        stats.warnings.extend(pre_scan.warnings)
    if args.dry_run:
        source = pre_scan if pre_scan is not None else scan_statistics(input_files, args)
        source.output_uri = str(args.output) if args.output else ""
        return source

    remove_output_if_allowed(args.output, args.overwrite)
    writer = create_writer(args)

    last_timestamp_ns: Optional[int] = None
    stop_after_ns: Optional[int] = None
    started = time.monotonic()
    last_progress = started

    for path in input_files:
        for message in iter_ros1_messages(path, selected_topics(args), stats.warnings):
            if stop_after_ns is not None and message.timestamp_ns > stop_after_ns:
                return stats
            stats.add_seen(message.topic, message.datatype, message.timestamp_ns)
            if not in_time_window(message.timestamp_ns, start_ns, end_ns):
                stats.skipped_messages += 1
                continue
            if last_timestamp_ns is not None and message.timestamp_ns < last_timestamp_ns:
                stats.warnings.append(
                    "non-monotonic bag timestamp: "
                    f"{message.timestamp_ns} after {last_timestamp_ns} in {message.path}"
                )
            last_timestamp_ns = message.timestamp_ns

            if message.topic == args.input_lidar_topic:
                if message.datatype != ROS1_LIVOX_CUSTOM:
                    stats.warnings.append(
                        f"skip lidar topic with unexpected type {message.datatype}"
                    )
                    stats.skipped_messages += 1
                    continue
                ros2_msg = parse_ros1_custom_msg(message.payload)
                writer.write(
                    args.output_lidar_topic,
                    serialize_message(ros2_msg),
                    message.timestamp_ns,
                )
                stats.written_lidar += 1
                if (
                    args.max_lidar_messages is not None and
                    stats.written_lidar >= args.max_lidar_messages and
                    stop_after_ns is None
                ):
                    stop_after_ns = message.timestamp_ns
            elif message.topic == args.input_imu_topic:
                if message.datatype != ROS1_IMU:
                    stats.warnings.append(
                        f"skip IMU topic with unexpected type {message.datatype}"
                    )
                    stats.skipped_messages += 1
                    continue
                ros2_msg = parse_ros1_imu(message.payload)
                writer.write(
                    args.output_imu_topic,
                    serialize_message(ros2_msg),
                    message.timestamp_ns,
                )
                stats.written_imu += 1
            else:
                stats.skipped_messages += 1

            now = time.monotonic()
            if not args.quiet and now - last_progress >= 2.0:
                elapsed = now - started
                print(
                    "converted "
                    f"lidar={stats.written_lidar} imu={stats.written_imu} "
                    f"read={stats.read_messages} elapsed={elapsed:.1f}s",
                    file=sys.stderr,
                )
                last_progress = now
    return stats


def stats_to_dict(stats: ConversionStats) -> Dict[str, object]:
    return {
        "input_files": stats.input_files,
        "output_uri": stats.output_uri,
        "start_timestamp_ns": stats.start_timestamp_ns,
        "end_timestamp_ns": stats.end_timestamp_ns,
        "read_messages": stats.read_messages,
        "skipped_messages": stats.skipped_messages,
        "written_lidar": stats.written_lidar,
        "written_imu": stats.written_imu,
        "topics": [
            {
                "topic": topic.topic,
                "datatype": topic.datatype,
                "count": topic.count,
                "first_timestamp_ns": topic.first_timestamp_ns,
                "last_timestamp_ns": topic.last_timestamp_ns,
            }
            for topic in sorted(stats.topic_stats.values(), key=lambda item: item.topic)
        ],
        "warnings": stats.warnings,
    }


def write_report(path: Path, stats: ConversionStats) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(stats_to_dict(stats), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        type=Path,
        nargs="+",
        required=True,
        help="ROS 1 .bag files or directories containing split data_*.bag files",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Output rosbag2 directory. Required unless --dry-run is used.",
    )
    parser.add_argument("--output-storage", default="sqlite3")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--report", type=Path)
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--start", type=float, help="Relative start time in seconds")
    parser.add_argument("--end", type=float, help="Relative end time in seconds")
    parser.add_argument("--duration", type=float, help="Duration in seconds from --start")
    parser.add_argument("--start-ns", type=int, help="Absolute bag receive timestamp start")
    parser.add_argument("--end-ns", type=int, help="Absolute bag receive timestamp end")
    parser.add_argument(
        "--max-lidar-messages",
        type=int,
        help="Stop after writing this many lidar messages, keeping IMU up to that stamp",
    )
    parser.add_argument("--input-lidar-topic", default=DEFAULT_INPUT_LIDAR_TOPIC)
    parser.add_argument("--input-imu-topic", default=DEFAULT_INPUT_IMU_TOPIC)
    parser.add_argument("--output-lidar-topic", default=DEFAULT_OUTPUT_LIDAR_TOPIC)
    parser.add_argument("--output-imu-topic", default=DEFAULT_OUTPUT_IMU_TOPIC)
    args = parser.parse_args(argv)

    if not args.dry_run and args.output is None:
        parser.error("--output is required unless --dry-run is used")
    if args.output is not None:
        args.output = args.output.expanduser()
    if args.start is not None and args.start < 0:
        parser.error("--start must be non-negative")
    if args.end is not None and args.end < 0:
        parser.error("--end must be non-negative")
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be positive")
    if args.max_lidar_messages is not None and args.max_lidar_messages <= 0:
        parser.error("--max-lidar-messages must be positive")
    if (args.start_ns is None) != (args.end_ns is None):
        parser.error("--start-ns and --end-ns must be specified together")
    if (args.start_ns is not None or args.end_ns is not None) and (
        args.start is not None or args.end is not None or args.duration is not None
    ):
        parser.error("absolute ns and relative seconds windows cannot be mixed")
    if args.end is not None and args.duration is not None:
        parser.error("--end and --duration cannot be used together")
    if args.start is not None and args.end is not None and args.end <= args.start:
        parser.error("--end must be greater than --start")
    if args.start_ns is not None and args.end_ns <= args.start_ns:
        parser.error("--end-ns must be greater than --start-ns")
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    try:
        args = parse_args(argv)
        input_files = resolve_input_files(args.input)
        stats = convert(args, input_files)
        if args.report:
            write_report(args.report, stats)
        print(json.dumps(stats_to_dict(stats), indent=2, sort_keys=True))
        return 0
    except (OSError, Ros1BagError, struct.error) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
