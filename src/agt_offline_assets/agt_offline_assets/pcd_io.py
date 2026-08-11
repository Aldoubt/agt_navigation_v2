"""Minimal deterministic PCD I/O used by the offline processing pipeline.

The reader accepts the three PCD v0.7 storage modes used by PCL: ASCII,
uncompressed binary, and ``binary_compressed``.  Compressed input is decoded
according to PCL's LZF + structure-of-arrays representation, then normalized
into the same structured NumPy array used by the other modes.  The writer
intentionally emits only ASCII or uncompressed binary so processing artifacts
have one simple deterministic output representation.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import struct
from typing import Any

import numpy as np

from .contracts import AssetContractError


_TYPE_TO_DTYPE = {
    ("F", 4): np.dtype("<f4"),
    ("F", 8): np.dtype("<f8"),
    ("I", 1): np.dtype("i1"),
    ("I", 2): np.dtype("<i2"),
    ("I", 4): np.dtype("<i4"),
    ("I", 8): np.dtype("<i8"),
    ("U", 1): np.dtype("u1"),
    ("U", 2): np.dtype("<u2"),
    ("U", 4): np.dtype("<u4"),
    ("U", 8): np.dtype("<u8"),
}


@dataclass(frozen=True)
class PcdSchema:
    fields: tuple[str, ...]
    sizes: tuple[int, ...]
    types: tuple[str, ...]
    counts: tuple[int, ...]
    viewpoint: tuple[float, ...]

    @property
    def scalar_count(self) -> int:
        return sum(self.counts)

    @property
    def packed_point_size(self) -> int:
        return sum(size * count for size, count in zip(self.sizes, self.counts))

    @property
    def dtype(self) -> np.dtype:
        formats: list[Any] = []
        for field, size, type_code, count in zip(
            self.fields, self.sizes, self.types, self.counts
        ):
            base = _TYPE_TO_DTYPE.get((type_code, size))
            if base is None:
                raise AssetContractError(
                    "pcd_field_type_unsupported",
                    f"unsupported PCD field type: {field} TYPE={type_code} SIZE={size}",
                )
            formats.append(base if count == 1 else (base, (count,)))
        return np.dtype({"names": self.fields, "formats": formats}, align=False)


@dataclass
class PcdCloud:
    schema: PcdSchema
    points: np.ndarray
    data_mode: str

    def xyz(self) -> np.ndarray:
        missing = [name for name in ("x", "y", "z") if name not in self.schema.fields]
        if missing:
            raise AssetContractError(
                "pcd_xyz_missing", "PCD requires scalar x/y/z fields"
            )
        for name in ("x", "y", "z"):
            index = self.schema.fields.index(name)
            if self.schema.counts[index] != 1:
                raise AssetContractError(
                    "pcd_xyz_not_scalar", f"PCD field {name} must have COUNT 1"
                )
        return np.column_stack(
            [
                np.asarray(self.points["x"], dtype=np.float64),
                np.asarray(self.points["y"], dtype=np.float64),
                np.asarray(self.points["z"], dtype=np.float64),
            ]
        )

    def subset(self, indices_or_mask: np.ndarray) -> "PcdCloud":
        return PcdCloud(
            self.schema,
            np.asarray(self.points[indices_or_mask]).copy(),
            self.data_mode,
        )


def _tokens(mapping: dict[str, list[str]], key: str, *, required: bool = True) -> list[str]:
    values = mapping.get(key)
    if values is None and required:
        raise AssetContractError("pcd_header_missing", f"PCD header missing {key}")
    return values or []


def _lzf_decompress(payload: bytes, expected_size: int) -> bytes:
    """Decode the LZF stream used by PCL binary_compressed PCD bodies.

    This is a small bounds-checked implementation of the public LZF format.  It
    supports overlapping back references, which are required by valid LZF
    streams, and fails closed on malformed or truncated input.
    """
    if expected_size < 0:
        raise AssetContractError("pcd_compressed_size_invalid", "negative LZF output size")
    if expected_size == 0:
        if payload:
            raise AssetContractError(
                "pcd_compressed_size_mismatch", "empty compressed cloud contains payload bytes"
            )
        return b""

    source = memoryview(payload)
    output = bytearray(expected_size)
    ip = 0
    op = 0
    while ip < len(source):
        ctrl = int(source[ip])
        ip += 1
        if ctrl < 32:
            length = ctrl + 1
            if ip + length > len(source) or op + length > expected_size:
                raise AssetContractError(
                    "pcd_lzf_invalid", "LZF literal run exceeds input or output bounds"
                )
            output[op : op + length] = source[ip : ip + length]
            ip += length
            op += length
            continue

        length = ctrl >> 5
        reference = op - ((ctrl & 0x1F) << 8) - 1
        if length == 7:
            if ip >= len(source):
                raise AssetContractError("pcd_lzf_invalid", "LZF length extension is truncated")
            length += int(source[ip])
            ip += 1
        if ip >= len(source):
            raise AssetContractError("pcd_lzf_invalid", "LZF back reference is truncated")
        reference -= int(source[ip])
        ip += 1
        length += 2
        if reference < 0 or op + length > expected_size:
            raise AssetContractError(
                "pcd_lzf_invalid", "LZF back reference exceeds output bounds"
            )
        # Copy byte-by-byte so overlapping references behave like the LZF C
        # decoder rather than Python slice assignment with a temporary copy.
        for _ in range(length):
            output[op] = output[reference]
            op += 1
            reference += 1

    if op != expected_size:
        raise AssetContractError(
            "pcd_lzf_size_mismatch",
            f"LZF decompressed {op} bytes, expected {expected_size}",
        )
    return bytes(output)


def _read_binary_compressed(
    payload: bytes, schema: PcdSchema, point_count: int
) -> np.ndarray:
    if len(payload) < 8:
        raise AssetContractError(
            "pcd_compressed_truncated", "binary_compressed PCD body needs an 8-byte size header"
        )
    compressed_size, uncompressed_size = struct.unpack_from("<II", payload, 0)
    if len(payload) < 8 + compressed_size:
        raise AssetContractError(
            "pcd_compressed_truncated",
            f"compressed PCD payload is truncated: expected {compressed_size} bytes, "
            f"got {max(0, len(payload) - 8)}",
        )
    expected_uncompressed = point_count * schema.packed_point_size
    if uncompressed_size != expected_uncompressed:
        raise AssetContractError(
            "pcd_compressed_uncompressed_size_mismatch",
            f"binary_compressed PCD declares {uncompressed_size} uncompressed bytes, "
            f"expected {expected_uncompressed} from POINTS/FIELDS",
        )
    compressed = payload[8 : 8 + compressed_size]
    unpacked = _lzf_decompress(compressed, uncompressed_size)

    # PCL reorders AoS point records to one contiguous plane per field before
    # LZF compression: xyzxyz -> xxxyyyzzz.  Restore each field independently.
    points = np.empty(point_count, dtype=schema.dtype)
    offset = 0
    for field, size, type_code, count in zip(
        schema.fields, schema.sizes, schema.types, schema.counts
    ):
        base = _TYPE_TO_DTYPE.get((type_code, size))
        if base is None:
            raise AssetContractError(
                "pcd_field_type_unsupported",
                f"unsupported PCD field type: {field} TYPE={type_code} SIZE={size}",
            )
        field_bytes = point_count * size * count
        end = offset + field_bytes
        if end > len(unpacked):
            raise AssetContractError(
                "pcd_compressed_layout_invalid", f"field plane is truncated: {field}"
            )
        values = np.frombuffer(unpacked[offset:end], dtype=base)
        expected_values = point_count * count
        if values.size != expected_values:
            raise AssetContractError(
                "pcd_compressed_layout_invalid", f"field plane has wrong size: {field}"
            )
        if count == 1:
            points[field] = values
        else:
            points[field] = values.reshape(point_count, count)
        offset = end
    if offset != len(unpacked):
        raise AssetContractError(
            "pcd_compressed_layout_invalid", "compressed PCD contains unclaimed field bytes"
        )
    return points


def read_pcd(path: str | Path) -> PcdCloud:
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise AssetContractError("pcd_missing", f"PCD does not exist: {path}")

    header_lines: list[str] = []
    with path.open("rb") as stream:
        while True:
            raw = stream.readline()
            if not raw:
                raise AssetContractError("pcd_header_invalid", "PCD has no DATA line")
            try:
                line = raw.decode("ascii").strip()
            except UnicodeDecodeError as exc:
                raise AssetContractError(
                    "pcd_header_encoding", "PCD header must be ASCII"
                ) from exc
            header_lines.append(line)
            if line.upper().startswith("DATA "):
                break
        payload = stream.read()

    mapping: dict[str, list[str]] = {}
    for line in header_lines:
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        mapping[parts[0].upper()] = parts[1:]

    fields = tuple(_tokens(mapping, "FIELDS") or _tokens(mapping, "FIELD"))
    sizes = tuple(int(value) for value in _tokens(mapping, "SIZE"))
    types = tuple(value.upper() for value in _tokens(mapping, "TYPE"))
    counts_raw = _tokens(mapping, "COUNT", required=False)
    counts = tuple(int(value) for value in counts_raw) if counts_raw else (1,) * len(fields)
    if not fields or not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise AssetContractError(
            "pcd_schema_invalid", "FIELDS/SIZE/TYPE/COUNT lengths must match"
        )
    if len(set(fields)) != len(fields):
        raise AssetContractError("pcd_field_duplicate", "PCD field names must be unique")
    if any(count <= 0 for count in counts):
        raise AssetContractError("pcd_count_invalid", "PCD COUNT values must be positive")

    viewpoint_values = _tokens(mapping, "VIEWPOINT", required=False)
    viewpoint = tuple(float(value) for value in viewpoint_values) if viewpoint_values else (
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
    )
    points_tokens = _tokens(mapping, "POINTS", required=False)
    width_tokens = _tokens(mapping, "WIDTH", required=False)
    height_tokens = _tokens(mapping, "HEIGHT", required=False)
    if points_tokens:
        point_count = int(points_tokens[0])
    elif width_tokens and height_tokens:
        point_count = int(width_tokens[0]) * int(height_tokens[0])
    else:
        raise AssetContractError("pcd_point_count_missing", "PCD needs POINTS or WIDTH+HEIGHT")
    if point_count < 0:
        raise AssetContractError("pcd_point_count_invalid", "PCD point count must be non-negative")

    data_values = _tokens(mapping, "DATA")
    if len(data_values) != 1:
        raise AssetContractError("pcd_data_invalid", "PCD DATA line is invalid")
    mode = data_values[0].lower()
    if mode not in {"ascii", "binary", "binary_compressed"}:
        raise AssetContractError("pcd_data_unsupported", f"unsupported PCD DATA mode: {mode}")

    schema = PcdSchema(fields, sizes, types, counts, viewpoint)
    dtype = schema.dtype
    if mode == "binary":
        expected = point_count * dtype.itemsize
        if len(payload) < expected:
            raise AssetContractError(
                "pcd_binary_truncated",
                f"binary PCD payload is truncated: expected {expected} bytes, got {len(payload)}",
            )
        points = np.frombuffer(payload[:expected], dtype=dtype, count=point_count).copy()
    elif mode == "binary_compressed":
        points = _read_binary_compressed(payload, schema, point_count)
    else:
        if point_count == 0:
            points = np.empty(0, dtype=dtype)
        else:
            try:
                scalars = np.fromstring(payload.decode("ascii"), sep=" ")
            except UnicodeDecodeError as exc:
                raise AssetContractError("pcd_ascii_encoding", "ASCII PCD data is not ASCII") from exc
            expected_scalars = point_count * schema.scalar_count
            if scalars.size != expected_scalars:
                raise AssetContractError(
                    "pcd_ascii_value_count",
                    f"ASCII PCD expected {expected_scalars} scalar values, got {scalars.size}",
                )
            matrix = scalars.reshape(point_count, schema.scalar_count)
            points = np.empty(point_count, dtype=dtype)
            cursor = 0
            for field, count in zip(fields, counts):
                values = matrix[:, cursor : cursor + count]
                points[field] = values[:, 0] if count == 1 else values
                cursor += count

    cloud = PcdCloud(schema, points, mode)
    cloud.xyz()  # fail early when a non-spatial PCD is supplied
    return cloud


def _format_ascii_scalar(value: Any, type_code: str) -> str:
    if type_code in {"I", "U"}:
        return str(int(value))
    return format(float(value), ".17g")


def write_pcd(path: str | Path, cloud: PcdCloud, *, data_mode: str = "binary") -> Path:
    path = Path(path)
    mode = str(data_mode).lower()
    if mode not in {"ascii", "binary"}:
        raise AssetContractError("pcd_output_mode_invalid", "output DATA mode must be ascii or binary")
    points = np.asarray(cloud.points, dtype=cloud.schema.dtype)
    count = int(points.shape[0])
    viewpoint = " ".join(format(float(value), ".17g") for value in cloud.schema.viewpoint)
    header = "\n".join(
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            "FIELDS " + " ".join(cloud.schema.fields),
            "SIZE " + " ".join(str(value) for value in cloud.schema.sizes),
            "TYPE " + " ".join(cloud.schema.types),
            "COUNT " + " ".join(str(value) for value in cloud.schema.counts),
            f"WIDTH {count}",
            "HEIGHT 1",
            f"VIEWPOINT {viewpoint}",
            f"POINTS {count}",
            f"DATA {mode}",
            "",
        ]
    ).encode("ascii")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as stream:
        stream.write(header)
        if mode == "binary":
            stream.write(points.tobytes(order="C"))
        else:
            for row in points:
                values: list[str] = []
                for field, type_code, field_count in zip(
                    cloud.schema.fields, cloud.schema.types, cloud.schema.counts
                ):
                    item = row[field]
                    if field_count == 1:
                        values.append(_format_ascii_scalar(item, type_code))
                    else:
                        values.extend(
                            _format_ascii_scalar(value, type_code)
                            for value in np.asarray(item).reshape(-1)
                        )
                stream.write((" ".join(values) + "\n").encode("ascii"))
    return path
