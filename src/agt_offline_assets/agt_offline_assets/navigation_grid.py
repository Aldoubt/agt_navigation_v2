"""Lightweight loading of frozen Navigation Map PGM/YAML evidence.

This module deliberately does not reconstruct ``NavigationMapResult`` because
that in-memory derivation object contains arrays that are not all materialized
in the exported navigation asset.  Route production only needs the frozen
occupancy grid geometry and trinary cell values for later Turn Zone and swept
footprint gates.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from .contracts import AssetContractError, sha256_file
from .navigation_map_derivation import FREE, OCCUPIED, UNKNOWN


@dataclass(frozen=True)
class NavigationGridEvidence:
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    width: int
    height: int
    occupancy: np.ndarray
    frame_id: str = "map"
    source: Mapping[str, Any] = field(default_factory=dict)

    def bounds_m(self) -> tuple[float, float, float, float]:
        return (
            float(self.origin_x_m),
            float(self.origin_y_m),
            float(self.origin_x_m + self.width * self.resolution_m),
            float(self.origin_y_m + self.height * self.resolution_m),
        )

    def counts(self) -> dict[str, int]:
        return {
            "free": int(np.count_nonzero(self.occupancy == FREE)),
            "occupied": int(np.count_nonzero(self.occupancy == OCCUPIED)),
            "unknown": int(np.count_nonzero(self.occupancy == UNKNOWN)),
        }


def _read_p5_pgm(path: Path) -> np.ndarray:
    """Read one 8-bit binary P5 PGM, including comment-bearing AGT outputs."""
    data = path.read_bytes()
    index = 0

    def token() -> bytes:
        nonlocal index
        while True:
            while index < len(data) and data[index] in b" \t\r\n":
                index += 1
            if index < len(data) and data[index] == ord("#"):
                while index < len(data) and data[index] not in b"\r\n":
                    index += 1
                continue
            break
        start = index
        while index < len(data) and data[index] not in b" \t\r\n#":
            index += 1
        if start == index:
            raise AssetContractError("navigation_pgm_invalid", f"invalid PGM header: {path}")
        return data[start:index]

    magic = token()
    if magic != b"P5":
        raise AssetContractError(
            "navigation_pgm_mode_unsupported",
            f"expected binary P5 PGM, got {magic.decode('ascii', errors='replace')}",
        )
    try:
        width = int(token())
        height = int(token())
        max_value = int(token())
    except ValueError as exc:
        raise AssetContractError("navigation_pgm_invalid", f"invalid PGM dimensions: {path}") from exc
    if width <= 0 or height <= 0 or max_value != 255:
        raise AssetContractError(
            "navigation_pgm_invalid",
            f"expected positive dimensions and maxval=255, got {width}x{height} max={max_value}",
        )

    # Exactly one or more whitespace bytes separate maxval from binary payload.
    while index < len(data) and data[index] in b" \t\r\n":
        index += 1
    expected = width * height
    payload = data[index:]
    if len(payload) != expected:
        raise AssetContractError(
            "navigation_pgm_size_mismatch",
            f"expected {expected} payload bytes, got {len(payload)}: {path}",
        )
    return np.frombuffer(payload, dtype=np.uint8).reshape(height, width).copy()


def load_navigation_grid(path: str | Path) -> NavigationGridEvidence:
    """Load frozen ``navigation_map.pgm/yaml`` without rerunning PCD derivation.

    ``path`` may be the derivation directory or the ``navigation_map.yaml`` file.
    The returned occupancy uses world-grid row order (minimum Y first), matching
    ``NavigationMapResult.occupancy`` rather than PGM display row order.
    """
    input_path = Path(path).expanduser().resolve()
    if input_path.is_dir():
        yaml_path = input_path / "navigation_map.yaml"
    else:
        yaml_path = input_path
    if not yaml_path.is_file():
        raise FileNotFoundError(f"navigation map YAML not found: {yaml_path}")

    payload = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
    if not isinstance(payload, Mapping):
        raise AssetContractError("navigation_yaml_invalid", "navigation_map.yaml must be a mapping")
    image_name = payload.get("image")
    if not isinstance(image_name, str) or not image_name:
        raise AssetContractError("navigation_yaml_invalid", "navigation_map.yaml image must be a filename")
    pgm_path = (yaml_path.parent / image_name).resolve()
    if not pgm_path.is_file():
        raise FileNotFoundError(f"navigation PGM not found: {pgm_path}")

    resolution = float(payload.get("resolution", 0.0))
    origin = payload.get("origin")
    if resolution <= 0.0:
        raise AssetContractError("navigation_yaml_invalid", "resolution must be > 0")
    if not isinstance(origin, (list, tuple)) or len(origin) < 2:
        raise AssetContractError("navigation_yaml_invalid", "origin must contain x and y")

    pgm_image = _read_p5_pgm(pgm_path)
    occupancy = np.flipud(pgm_image).copy()
    allowed = np.isin(occupancy, np.array([FREE, OCCUPIED, UNKNOWN], dtype=np.uint8))
    if not bool(np.all(allowed)):
        values = np.unique(occupancy[~allowed]).tolist()
        raise AssetContractError(
            "navigation_grid_not_trinary",
            f"expected AGT trinary values 0/205/254, got unexpected values {values[:8]}",
        )

    return NavigationGridEvidence(
        resolution_m=resolution,
        origin_x_m=float(origin[0]),
        origin_y_m=float(origin[1]),
        width=int(occupancy.shape[1]),
        height=int(occupancy.shape[0]),
        occupancy=occupancy,
        frame_id="map",
        source={
            "navigation_yaml": yaml_path.name,
            "navigation_pgm": pgm_path.name,
            "navigation_yaml_sha256": sha256_file(yaml_path),
            "navigation_pgm_sha256": sha256_file(pgm_path),
        },
    )
