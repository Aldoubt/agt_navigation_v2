#!/usr/bin/env python3
"""Small, dependency-light PCD reader for map alignment diagnostics."""

from __future__ import annotations

from dataclasses import dataclass
import math
import mmap
from pathlib import Path
import struct
from typing import BinaryIO, Dict, List, Tuple

import numpy as np


@dataclass(frozen=True)
class PcdSample:
    x: np.ndarray
    y: np.ndarray
    z: np.ndarray
    point_count: int
    sampled_count: int
    bounds_xyz: Tuple[float, float, float, float, float, float]
    data_format: str
    bounds_are_sampled: bool = False


def _read_header(stream: BinaryIO) -> Tuple[Dict[str, List[str]], str]:
    header: Dict[str, List[str]] = {}
    while True:
        line = stream.readline()
        if not line:
            raise ValueError("PCD header is missing a DATA line")
        text = line.decode("ascii", errors="strict").strip()
        if not text or text.startswith("#"):
            continue
        parts = text.split()
        key = parts[0].upper()
        header[key] = parts[1:]
        if key == "DATA":
            if not parts[1:]:
                raise ValueError("PCD DATA format is missing")
            return header, parts[1].lower()


def _lzf_decompress(payload: bytes, expected_size: int) -> bytes:
    """Decode the LZF stream used by PCD binary_compressed files."""
    source_index = 0
    output = bytearray()
    payload_size = len(payload)
    while source_index < payload_size:
        control = payload[source_index]
        source_index += 1
        if control < 32:
            literal_length = control + 1
            end = source_index + literal_length
            if end > payload_size:
                raise ValueError("Truncated LZF literal run")
            output.extend(payload[source_index:end])
            source_index = end
            continue

        match_length = control >> 5
        reference = len(output) - ((control & 0x1F) << 8) - 1
        if match_length == 7:
            if source_index >= payload_size:
                raise ValueError("Truncated LZF extended match")
            match_length += payload[source_index]
            source_index += 1
        if source_index >= payload_size:
            raise ValueError("Truncated LZF match offset")
        reference -= payload[source_index]
        source_index += 1
        match_length += 2
        if reference < 0:
            raise ValueError("Invalid LZF back-reference")
        for _ in range(match_length):
            if reference >= len(output):
                raise ValueError("Invalid overlapping LZF back-reference")
            output.append(output[reference])
            reference += 1

    if len(output) != expected_size:
        raise ValueError(
            f"LZF size mismatch: expected {expected_size}, decoded {len(output)}"
        )
    return bytes(output)


def _numpy_dtype(type_code: str, size: int) -> np.dtype:
    kinds = {"F": "f", "I": "i", "U": "u"}
    try:
        return np.dtype(f"<{kinds[type_code]}{size}")
    except (KeyError, TypeError) as exc:
        raise ValueError(f"Unsupported PCD scalar type: TYPE={type_code} SIZE={size}") from exc


def _header_layout(header: Dict[str, List[str]]) -> Tuple[List[str], List[int], List[str], List[int], int]:
    fields = header.get("FIELDS") or header.get("FIELD")
    if not fields:
        raise ValueError("PCD FIELDS is missing")
    sizes = [int(value) for value in header.get("SIZE", [])]
    types = [value.upper() for value in header.get("TYPE", [])]
    counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError("PCD FIELDS/SIZE/TYPE/COUNT lengths differ")
    points_values = header.get("POINTS")
    if points_values:
        point_count = int(points_values[0])
    else:
        point_count = int(header["WIDTH"][0]) * int(header.get("HEIGHT", ["1"])[0])
    if point_count <= 0:
        raise ValueError("PCD contains no points")
    return fields, sizes, types, counts, point_count


def _compressed_columns(
    raw: bytes,
    fields: List[str],
    sizes: List[int],
    types: List[str],
    counts: List[int],
    point_count: int,
) -> Dict[str, np.ndarray]:
    columns: Dict[str, np.ndarray] = {}
    offset = 0
    for name, size, type_code, count in zip(fields, sizes, types, counts):
        byte_count = point_count * size * count
        if offset + byte_count > len(raw):
            raise ValueError(f"PCD field {name!r} exceeds decoded payload")
        values = np.frombuffer(
            raw, dtype=_numpy_dtype(type_code, size), count=point_count * count, offset=offset
        )
        columns[name] = values.reshape(point_count, count)[:, 0]
        offset += byte_count
    return columns


def _binary_columns(
    raw: bytes,
    fields: List[str],
    sizes: List[int],
    types: List[str],
    counts: List[int],
    point_count: int,
) -> Dict[str, np.ndarray]:
    descriptors = []
    for name, size, type_code, count in zip(fields, sizes, types, counts):
        dtype = _numpy_dtype(type_code, size)
        descriptors.append((name, dtype, (count,)) if count > 1 else (name, dtype))
    records = np.frombuffer(raw, dtype=np.dtype(descriptors), count=point_count)
    return {
        name: records[name][:, 0] if counts[index] > 1 else records[name]
        for index, name in enumerate(fields)
    }


def _binary_record_dtype(
    fields: List[str], sizes: List[int], types: List[str], counts: List[int]
) -> np.dtype:
    descriptors = []
    for name, size, type_code, count in zip(fields, sizes, types, counts):
        dtype = _numpy_dtype(type_code, size)
        descriptors.append((name, dtype, (count,)) if count > 1 else (name, dtype))
    return np.dtype(descriptors)


def _load_binary_sample(
    pcd_path: Path,
    data_offset: int,
    fields: List[str],
    sizes: List[int],
    types: List[str],
    counts: List[int],
    point_count: int,
    max_points: int,
) -> PcdSample:
    bounds_are_sampled = point_count > max(max_points * 4, 5_000_000)
    records = np.memmap(
        pcd_path,
        dtype=_binary_record_dtype(fields, sizes, types, counts),
        mode="r",
        offset=data_offset,
        shape=(point_count,),
    )
    if bounds_are_sampled and hasattr(records, "_mmap") and hasattr(mmap, "MADV_RANDOM"):
        records._mmap.madvise(mmap.MADV_RANDOM)
    if bounds_are_sampled:
        desired = min(point_count, max(1, max_points))
        block_count = min(128, desired)
        block_size = max(1, math.ceil(desired / block_count))
        starts = np.linspace(
            0, max(0, point_count - block_size), num=block_count, dtype=np.int64
        )
        sampled = [
            np.concatenate([
                np.asarray(records[axis][start:min(start + block_size, point_count)])
                for start in starts
            ]).astype(np.float64, copy=False)[:desired]
            for axis in ("x", "y", "z")
        ]
    else:
        stride = max(1, math.ceil(point_count / max(1, max_points)))
        sampled = [
            np.asarray(records[axis][::stride], dtype=np.float64) for axis in ("x", "y", "z")
        ]
    finite_sample = np.isfinite(sampled[0]) & np.isfinite(sampled[1]) & np.isfinite(sampled[2])
    if not finite_sample.any():
        raise ValueError("PCD has no finite sampled XYZ points")
    sampled = [values[finite_sample].copy() for values in sampled]

    minimum = np.array([values.min() for values in sampled], dtype=np.float64)
    maximum = np.array([values.max() for values in sampled], dtype=np.float64)
    if not bounds_are_sampled:
        chunk_size = 2_000_000
        for start in range(0, point_count, chunk_size):
            stop = min(start + chunk_size, point_count)
            chunk = [np.asarray(records[axis][start:stop]) for axis in ("x", "y", "z")]
            finite = np.isfinite(chunk[0]) & np.isfinite(chunk[1]) & np.isfinite(chunk[2])
            if not finite.any():
                continue
            for index, values in enumerate(chunk):
                minimum[index] = min(minimum[index], float(values[finite].min()))
                maximum[index] = max(maximum[index], float(values[finite].max()))
    if not np.isfinite(minimum).all():
        raise ValueError("PCD has no finite XYZ points")
    return PcdSample(
        x=sampled[0],
        y=sampled[1],
        z=sampled[2],
        point_count=point_count,
        sampled_count=sampled[0].size,
        bounds_xyz=(
            float(minimum[0]), float(minimum[1]), float(minimum[2]),
            float(maximum[0]), float(maximum[1]), float(maximum[2]),
        ),
        data_format="binary",
        bounds_are_sampled=bounds_are_sampled,
    )


def load_pcd_xyz(path: Path | str, max_points: int = 750_000) -> PcdSample:
    """Load XYZ and retain an evenly spaced sample while measuring full bounds."""
    pcd_path = Path(path).expanduser().resolve()
    with pcd_path.open("rb") as stream:
        header, data_format = _read_header(stream)
        data_offset = stream.tell()
        fields, sizes, types, counts, point_count = _header_layout(header)
        missing = {"x", "y", "z"}.difference(fields)
        if missing:
            raise ValueError(f"PCD is missing coordinate fields: {sorted(missing)}")

        if data_format == "binary_compressed":
            size_header = stream.read(8)
            if len(size_header) != 8:
                raise ValueError("PCD compressed size header is truncated")
            compressed_size, uncompressed_size = struct.unpack("<II", size_header)
            payload = stream.read(compressed_size)
            if len(payload) != compressed_size:
                raise ValueError("PCD compressed payload is truncated")
            decoded = _lzf_decompress(payload, uncompressed_size)
            columns = _compressed_columns(decoded, fields, sizes, types, counts, point_count)
        elif data_format == "binary":
            return _load_binary_sample(
                pcd_path, data_offset, fields, sizes, types, counts, point_count, max_points
            )
        elif data_format == "ascii":
            values = np.loadtxt(stream, dtype=np.float64, ndmin=2)
            scalar_offsets = np.cumsum([0] + counts[:-1]).tolist()
            columns = {name: values[:, scalar_offsets[index]] for index, name in enumerate(fields)}
            point_count = values.shape[0]
        else:
            raise ValueError(f"Unsupported PCD DATA format: {data_format}")

        x = np.asarray(columns["x"], dtype=np.float64)
        y = np.asarray(columns["y"], dtype=np.float64)
        z = np.asarray(columns["z"], dtype=np.float64)
        finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        if not finite.any():
            raise ValueError("PCD has no finite XYZ points")
        x_valid, y_valid, z_valid = x[finite], y[finite], z[finite]
        bounds = (
            float(x_valid.min()), float(y_valid.min()), float(z_valid.min()),
            float(x_valid.max()), float(y_valid.max()), float(z_valid.max()),
        )
        stride = max(1, math.ceil(x_valid.size / max(1, max_points)))
        return PcdSample(
            x=x_valid[::stride].copy(),
            y=y_valid[::stride].copy(),
            z=z_valid[::stride].copy(),
            point_count=point_count,
            sampled_count=x_valid[::stride].size,
            bounds_xyz=bounds,
            data_format=data_format,
        )
