#!/usr/bin/env python3
"""Bake a PCD VIEWPOINT pose into XYZ/normals and write an identity-viewpoint PCD."""

from __future__ import annotations

import argparse
from pathlib import Path
import struct

import numpy as np

from pcd_projection import (
    _binary_columns,
    _binary_record_dtype,
    _compressed_columns,
    _header_layout,
    _lzf_decompress,
    _read_header,
)


def viewpoint_matrix(values: list[str]) -> np.ndarray:
    if len(values) != 7:
        raise ValueError("PCD VIEWPOINT must contain tx ty tz qw qx qy qz")
    tx, ty, tz, qw, qx, qy, qz = (float(value) for value in values)
    quaternion = np.array([qw, qx, qy, qz], dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm <= 0.0:
        raise ValueError("PCD VIEWPOINT quaternion has zero length")
    qw, qx, qy, qz = quaternion / norm
    rotation = np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = [tx, ty, tz]
    return matrix


def load_columns(path: Path):
    with path.open("rb") as stream:
        header, data_format = _read_header(stream)
        fields, sizes, types, counts, point_count = _header_layout(header)
        if any(count != 1 for count in counts):
            raise ValueError("multi-count PCD fields are not supported")
        if data_format == "binary_compressed":
            compressed_size, uncompressed_size = struct.unpack("<II", stream.read(8))
            decoded = _lzf_decompress(stream.read(compressed_size), uncompressed_size)
            columns = _compressed_columns(decoded, fields, sizes, types, counts, point_count)
        elif data_format == "binary":
            columns = _binary_columns(
                stream.read(), fields, sizes, types, counts, point_count
            )
        else:
            raise ValueError(f"unsupported PCD DATA format: {data_format}")
    return header, fields, sizes, types, counts, point_count, columns


def transform_columns(columns: dict[str, np.ndarray], matrix: np.ndarray) -> None:
    missing = {"x", "y", "z"}.difference(columns)
    if missing:
        raise ValueError(f"PCD is missing coordinate fields: {sorted(missing)}")
    xyz = np.column_stack((columns["x"], columns["y"], columns["z"]))
    transformed = xyz @ matrix[:3, :3].T + matrix[:3, 3]
    for index, name in enumerate(("x", "y", "z")):
        columns[name] = transformed[:, index].astype(columns[name].dtype)
    if "Coord._Z" in columns:
        columns["Coord._Z"] = transformed[:, 2].astype(columns["Coord._Z"].dtype)

    normal_names = ("normal_x", "normal_y", "normal_z")
    if all(name in columns for name in normal_names):
        normals = np.column_stack(tuple(columns[name] for name in normal_names))
        rotated = normals @ matrix[:3, :3].T
        for index, name in enumerate(normal_names):
            columns[name] = rotated[:, index].astype(columns[name].dtype)


def write_binary_pcd(
    path: Path,
    fields: list[str],
    sizes: list[int],
    types: list[str],
    counts: list[int],
    point_count: int,
    columns: dict[str, np.ndarray],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    header = "\n".join(
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            f"FIELDS {' '.join(fields)}",
            f"SIZE {' '.join(str(value) for value in sizes)}",
            f"TYPE {' '.join(types)}",
            f"COUNT {' '.join(str(value) for value in counts)}",
            f"WIDTH {point_count}",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {point_count}",
            "DATA binary",
            "",
        ]
    ).encode("ascii")
    dtype = _binary_record_dtype(fields, sizes, types, counts)
    chunk_size = 500_000
    with path.open("wb") as stream:
        stream.write(header)
        for start in range(0, point_count, chunk_size):
            stop = min(start + chunk_size, point_count)
            records = np.empty(stop - start, dtype=dtype)
            for index, name in enumerate(fields):
                values = columns[name][start:stop]
                records[name] = values if counts[index] == 1 else values.reshape(-1, counts[index])
            stream.write(records.tobytes())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--matrix-output", type=Path)
    args = parser.parse_args()
    source = args.input.expanduser().resolve()
    destination = args.output.expanduser().resolve()
    header, fields, sizes, types, counts, point_count, columns = load_columns(source)
    matrix = viewpoint_matrix(header.get("VIEWPOINT", ["0", "0", "0", "1", "0", "0", "0"]))
    transform_columns(columns, matrix)
    write_binary_pcd(destination, fields, sizes, types, counts, point_count, columns)
    if args.matrix_output:
        matrix_path = args.matrix_output.expanduser().resolve()
        matrix_path.parent.mkdir(parents=True, exist_ok=True)
        np.savetxt(matrix_path, matrix, fmt="%.12f")
    print(f"Input points : {point_count}")
    print(f"Output       : {destination}")
    print("Applied VIEWPOINT matrix:")
    print(np.array2string(matrix, precision=12, suppress_small=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
