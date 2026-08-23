"""Diagnostic 3D height-profile audit for critical aisle blockers.

This module never mutates Navigation Map occupancy. It re-reads raw point
returns around the minimum-blocker cells already identified by
``aisle_blocker_audit`` and measures their height relative to the same ground
surface used by the Navigation Map.

The implementation is intentionally chunked so an 80M+ point PCD does not
require another full Nx3 float64 copy just for review evidence.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterator

import numpy as np


@dataclass
class CriticalBarrierHeightAuditResult:
    """Serializable records plus compact 2D histograms for optional plots."""

    records: list[dict[str, object]]
    plot_histograms: dict[str, dict[str, np.ndarray]]
    u_edges_m: np.ndarray
    v_edges_normalized: np.ndarray
    height_edges_m: np.ndarray


def _normalized_direction(structure: Any) -> tuple[np.ndarray, np.ndarray]:
    direction = np.asarray(structure.row_model.direction_xy, dtype=np.float64).reshape(2)
    norm = float(np.linalg.norm(direction))
    if not np.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("critical barrier 3D audit row direction must be finite and non-zero")
    direction = direction / norm
    perpendicular = np.array([-direction[1], direction[0]], dtype=np.float64)
    return direction, perpendicular


def _iter_xyz_chunks(cloud: Any, chunk_size: int) -> Iterator[np.ndarray]:
    if chunk_size < 1:
        raise ValueError("critical barrier 3D audit chunk_size must be >= 1")

    points = getattr(cloud, "points", None)
    names = getattr(getattr(points, "dtype", None), "names", None)
    if points is not None and names and all(name in names for name in ("x", "y", "z")):
        total = len(points)
        for start in range(0, total, chunk_size):
            block = points[start : start + chunk_size]
            yield np.column_stack(
                [
                    np.asarray(block["x"], dtype=np.float64),
                    np.asarray(block["y"], dtype=np.float64),
                    np.asarray(block["z"], dtype=np.float64),
                ]
            )
        return

    xyz = np.asarray(cloud.xyz(), dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] < 3:
        raise ValueError("critical barrier 3D audit requires Nx3 XYZ points")
    for start in range(0, xyz.shape[0], chunk_size):
        yield xyz[start : start + chunk_size, :3]


def _diagnostic_by_pair(corridor: Any) -> dict[int, Any]:
    result: dict[int, Any] = {}
    for diagnostic in corridor.aisle_pair_diagnostics:
        if str(getattr(diagnostic, "pair_kind", "ROW_ROW")) != "ROW_ROW":
            continue
        result[int(diagnostic.pair_index)] = diagnostic
    return result


def _plot_key(aisle_id: str, row: int, col: int) -> str:
    return f"{aisle_id}_r{row:04d}_c{col:04d}"


def build_critical_barrier_height_audit(
    cloud: Any,
    navigation: Any,
    structure: Any,
    corridor: Any,
    blocker_reports: list[dict[str, object]],
    *,
    longitudinal_half_window_m: float = 0.40,
    chunk_size: int = 1_000_000,
    rotation_map_from_source: np.ndarray | None = None,
    translation_map_from_source_m: np.ndarray | None = None,
) -> CriticalBarrierHeightAuditResult:
    """Measure ground-relative 3D evidence around every critical blocker cell.

    The local review prism spans ``±longitudinal_half_window_m`` along the
    agricultural row direction and the full geometric aisle width transversely.
    Height bands are fixed at 0.12/0.25/0.50/1.00 m so repeated greenhouse runs
    are directly comparable. Obstacle-evidence quantiles use the active
    navigation configuration's obstacle min/max heights.
    """

    if longitudinal_half_window_m <= 0.0:
        raise ValueError("longitudinal_half_window_m must be > 0")

    occupancy = np.asarray(navigation.occupancy)
    ground_height = np.asarray(navigation.ground_height_m, dtype=np.float64)
    if ground_height.shape != occupancy.shape:
        raise ValueError("critical barrier 3D audit ground/grid shape mismatch")

    resolution = float(navigation.resolution_m)
    if resolution <= 0.0:
        raise ValueError("critical barrier 3D audit resolution must be > 0")
    origin_x = float(navigation.origin_x_m)
    origin_y = float(navigation.origin_y_m)
    height, width = occupancy.shape

    cfg = navigation.config
    obstacle_min = float(getattr(cfg, "obstacle_min_height_m", 0.12))
    obstacle_max = float(getattr(cfg, "obstacle_max_height_m", 1.00))
    if obstacle_max <= obstacle_min:
        raise ValueError("critical barrier 3D audit obstacle height range is invalid")

    direction, perpendicular = _normalized_direction(structure)
    diagnostics = _diagnostic_by_pair(corridor)

    accumulators: list[dict[str, object]] = []
    for report in blocker_reports:
        pair_index = int(report.get("pair_index", -1))
        diagnostic = diagnostics.get(pair_index)
        if diagnostic is None:
            continue
        v_low = min(
            float(diagnostic.left_row_center_v_m),
            float(diagnostic.right_row_center_v_m),
        )
        v_high = max(
            float(diagnostic.left_row_center_v_m),
            float(diagnostic.right_row_center_v_m),
        )
        aisle_width = max(v_high - v_low, 1.0e-9)
        aisle_id = str(report.get("aisle_id", f"aisle_{pair_index:03d}"))
        for blocker in report.get("critical_blocker_cells", []):
            row = int(blocker["row"])
            col = int(blocker["col"])
            x_m = float(blocker.get("x_m", origin_x + (col + 0.5) * resolution))
            y_m = float(blocker.get("y_m", origin_y + (row + 0.5) * resolution))
            u0 = float(blocker.get("u_m", x_m * direction[0] + y_m * direction[1]))
            v0 = float(blocker.get("v_m", x_m * perpendicular[0] + y_m * perpendicular[1]))
            accumulators.append(
                {
                    "aisle_id": aisle_id,
                    "pair_index": pair_index,
                    "row": row,
                    "col": col,
                    "cause": str(blocker.get("cause", "UNKNOWN")),
                    "x_m": x_m,
                    "y_m": y_m,
                    "u0": u0,
                    "v0": v0,
                    "v_low": v_low,
                    "v_high": v_high,
                    "aisle_width": aisle_width,
                    "classifiable": 0,
                    "bands": np.zeros(5, dtype=np.int64),
                    "obstacle_heights": [],
                }
            )

    u_edges = np.linspace(
        -float(longitudinal_half_window_m),
        float(longitudinal_half_window_m),
        33,
        dtype=np.float64,
    )
    v_edges = np.linspace(0.0, 1.0, 33, dtype=np.float64)
    height_top = max(1.5, obstacle_max + 0.5)
    height_edges = np.linspace(-0.10, height_top, 65, dtype=np.float64)
    plot_histograms: dict[str, dict[str, np.ndarray]] = {}
    for acc in accumulators:
        key = _plot_key(str(acc["aisle_id"]), int(acc["row"]), int(acc["col"]))
        acc["plot_key"] = key
        plot_histograms[key] = {
            "uv": np.zeros((len(u_edges) - 1, len(v_edges) - 1), dtype=np.int64),
            "u_height": np.zeros(
                (len(u_edges) - 1, len(height_edges) - 1), dtype=np.int64
            ),
            "v_height": np.zeros(
                (len(v_edges) - 1, len(height_edges) - 1), dtype=np.int64
            ),
        }

    rotation = None
    translation = None
    if rotation_map_from_source is not None or translation_map_from_source_m is not None:
        rotation = (
            np.eye(3, dtype=np.float64)
            if rotation_map_from_source is None
            else np.asarray(rotation_map_from_source, dtype=np.float64).reshape(3, 3)
        )
        translation = (
            np.zeros(3, dtype=np.float64)
            if translation_map_from_source_m is None
            else np.asarray(translation_map_from_source_m, dtype=np.float64).reshape(3)
        )

    for xyz in _iter_xyz_chunks(cloud, chunk_size):
        xyz = np.asarray(xyz, dtype=np.float64)
        finite = np.all(np.isfinite(xyz), axis=1)
        if not np.any(finite):
            continue
        xyz = xyz[finite]
        if rotation is not None:
            xyz = xyz @ rotation.T + translation
        x = xyz[:, 0]
        y = xyz[:, 1]
        z = xyz[:, 2]

        cols = np.floor((x - origin_x) / resolution).astype(np.int64)
        rows = np.floor((y - origin_y) / resolution).astype(np.int64)
        in_grid = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
        if not np.any(in_grid):
            continue

        safe_rows = np.clip(rows, 0, height - 1)
        safe_cols = np.clip(cols, 0, width - 1)
        ground = ground_height[safe_rows, safe_cols]
        classifiable = in_grid & np.isfinite(ground)
        relative_height = z - ground
        u = x * direction[0] + y * direction[1]
        v = x * perpendicular[0] + y * perpendicular[1]

        for acc in accumulators:
            selected = (
                classifiable
                & (np.abs(u - float(acc["u0"])) <= longitudinal_half_window_m)
                & (v >= float(acc["v_low"]) - 1.0e-9)
                & (v <= float(acc["v_high"]) + 1.0e-9)
            )
            if not np.any(selected):
                continue
            h = relative_height[selected]
            uu = u[selected] - float(acc["u0"])
            vv = (v[selected] - float(acc["v_low"])) / float(acc["aisle_width"])
            acc["classifiable"] = int(acc["classifiable"]) + int(h.size)

            bands = np.asarray(acc["bands"], dtype=np.int64)
            bands[0] += int(np.count_nonzero(h < 0.12))
            bands[1] += int(np.count_nonzero((h >= 0.12) & (h < 0.25)))
            bands[2] += int(np.count_nonzero((h >= 0.25) & (h < 0.50)))
            bands[3] += int(np.count_nonzero((h >= 0.50) & (h <= 1.00)))
            bands[4] += int(np.count_nonzero(h > 1.00))
            acc["bands"] = bands

            obstacle = (h >= obstacle_min) & (h <= obstacle_max)
            if np.any(obstacle):
                cast_list = acc["obstacle_heights"]
                assert isinstance(cast_list, list)
                cast_list.append(np.asarray(h[obstacle], dtype=np.float32))

            hist = plot_histograms[str(acc["plot_key"])]
            hist["uv"] += np.histogram2d(uu, vv, bins=(u_edges, v_edges))[0].astype(
                np.int64
            )
            hist["u_height"] += np.histogram2d(
                uu, h, bins=(u_edges, height_edges)
            )[0].astype(np.int64)
            hist["v_height"] += np.histogram2d(
                vv, h, bins=(v_edges, height_edges)
            )[0].astype(np.int64)

    records: list[dict[str, object]] = []
    for acc in accumulators:
        chunks = acc["obstacle_heights"]
        assert isinstance(chunks, list)
        obstacle_heights = (
            np.concatenate(chunks).astype(np.float64, copy=False)
            if chunks
            else np.empty(0, dtype=np.float64)
        )
        if obstacle_heights.size:
            p10, p50, p90 = np.quantile(obstacle_heights, [0.10, 0.50, 0.90])
            minimum = float(np.min(obstacle_heights))
            maximum = float(np.max(obstacle_heights))
            p10_value: float | None = float(p10)
            p50_value: float | None = float(p50)
            p90_value: float | None = float(p90)
        else:
            minimum = None
            maximum = None
            p10_value = None
            p50_value = None
            p90_value = None

        bands = np.asarray(acc["bands"], dtype=np.int64)
        records.append(
            {
                "aisle_id": str(acc["aisle_id"]),
                "pair_index": int(acc["pair_index"]),
                "row": int(acc["row"]),
                "col": int(acc["col"]),
                "cause": str(acc["cause"]),
                "x_m": float(acc["x_m"]),
                "y_m": float(acc["y_m"]),
                "longitudinal_half_window_m": float(longitudinal_half_window_m),
                "classifiable_point_count": int(acc["classifiable"]),
                "obstacle_evidence_point_count": int(obstacle_heights.size),
                "height_band_counts": {
                    "below_0_12_m": int(bands[0]),
                    "0_12_to_0_25_m": int(bands[1]),
                    "0_25_to_0_50_m": int(bands[2]),
                    "0_50_to_1_00_m": int(bands[3]),
                    "above_1_00_m": int(bands[4]),
                },
                "obstacle_height_min_m": minimum,
                "obstacle_height_p10_m": p10_value,
                "obstacle_height_p50_m": p50_value,
                "obstacle_height_p90_m": p90_value,
                "obstacle_height_max_m": maximum,
                "plot_key": str(acc["plot_key"]),
            }
        )

    return CriticalBarrierHeightAuditResult(
        records=records,
        plot_histograms=plot_histograms,
        u_edges_m=u_edges,
        v_edges_normalized=v_edges,
        height_edges_m=height_edges,
    )


def write_critical_barrier_height_plots(
    result: CriticalBarrierHeightAuditResult,
    output_dir: str | Path,
) -> list[Path]:
    """Write three compact evidence plots per critical blocker."""

    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []

    for record in result.records:
        key = str(record["plot_key"])
        hist = result.plot_histograms[key]
        specs = (
            (
                "uv",
                result.u_edges_m,
                result.v_edges_normalized,
                "longitudinal offset u [m]",
                "normalized aisle transverse position",
                "top view point density",
            ),
            (
                "u_height",
                result.u_edges_m,
                result.height_edges_m,
                "longitudinal offset u [m]",
                "height above local ground [m]",
                "u-height point density",
            ),
            (
                "v_height",
                result.v_edges_normalized,
                result.height_edges_m,
                "normalized aisle transverse position",
                "height above local ground [m]",
                "v-height cross-section density",
            ),
        )
        for suffix, x_edges, y_edges, xlabel, ylabel, title in specs:
            fig, ax = plt.subplots(figsize=(6.4, 4.2))
            ax.imshow(
                np.asarray(hist[suffix], dtype=np.float64).T,
                origin="lower",
                aspect="auto",
                extent=[x_edges[0], x_edges[-1], y_edges[0], y_edges[-1]],
            )
            ax.set_xlabel(xlabel)
            ax.set_ylabel(ylabel)
            ax.set_title(f"{record['aisle_id']} {record['cause']} — {title}")
            path = output / f"{key}_{suffix}.png"
            fig.tight_layout()
            fig.savefig(path, dpi=150)
            plt.close(fig)
            written.append(path)
    return written
