import struct
import sys
from pathlib import Path

import numpy as np


MODULE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MODULE_DIR))

from pcd_projection import _lzf_decompress, load_pcd_xyz  # noqa: E402


def _literal_lzf(payload: bytes) -> bytes:
    encoded = bytearray()
    for start in range(0, len(payload), 32):
        chunk = payload[start:start + 32]
        encoded.append(len(chunk) - 1)
        encoded.extend(chunk)
    return bytes(encoded)


def test_lzf_literal_stream_round_trip():
    payload = bytes(range(100))
    assert _lzf_decompress(_literal_lzf(payload), len(payload)) == payload


def test_load_binary_compressed_field_major_xyz(tmp_path):
    x = np.array([-2.0, 1.0, 4.0], dtype="<f4")
    y = np.array([-3.0, 2.0, 5.0], dtype="<f4")
    z = np.array([0.1, 0.2, 0.3], dtype="<f4")
    decoded = x.tobytes() + y.tobytes() + z.tobytes()
    compressed = _literal_lzf(decoded)
    header = (
        "VERSION .7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        "WIDTH 3\nHEIGHT 1\nPOINTS 3\n"
        "DATA binary_compressed\n"
    ).encode("ascii")
    path = tmp_path / "sample.pcd"
    path.write_bytes(header + struct.pack("<II", len(compressed), len(decoded)) + compressed)

    sample = load_pcd_xyz(path, max_points=2)

    assert sample.point_count == 3
    assert sample.sampled_count == 2
    np.testing.assert_allclose(sample.x, [-2.0, 4.0])
    np.testing.assert_allclose(sample.y, [-3.0, 5.0])
    np.testing.assert_allclose(sample.z, [0.1, 0.3])
    assert sample.bounds_xyz == (
        -2.0, -3.0, 0.10000000149011612, 4.0, 5.0, 0.30000001192092896
    )


def test_load_binary_interleaved_xyz_uses_sample_and_full_bounds(tmp_path):
    records = np.array(
        [(-5.0, 2.0, 0.0), (1.0, -7.0, 1.0), (8.0, 3.0, 2.0), (4.0, 9.0, 3.0)],
        dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4")],
    )
    header = (
        "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        "WIDTH 4\nHEIGHT 1\nPOINTS 4\nDATA binary\n"
    ).encode("ascii")
    path = tmp_path / "binary.pcd"
    path.write_bytes(header + records.tobytes())

    sample = load_pcd_xyz(path, max_points=2)

    np.testing.assert_allclose(sample.x, [-5.0, 8.0])
    assert sample.bounds_xyz == (-5.0, -7.0, 0.0, 8.0, 9.0, 3.0)
