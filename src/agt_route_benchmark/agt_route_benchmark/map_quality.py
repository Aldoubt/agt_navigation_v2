from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Mapping

import numpy as np
import yaml

from agt_offline_assets.navigation_map_derivation import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    apply_navigation_overrides,
)

from .map_io import Nav2Map, load_nav2_map, nav2_map_occupancy_data


_ALLOWED_MODES = {"force_free", "force_occupied"}
_ALLOWED_EVIDENCE = {
    "pcd_inspection",
    "site_photo",
    "measured_structure",
    "known_permanent_obstacle",
    "field_note",
    "other_documented",
}


def _occupancy_classes(nav_map: Nav2Map) -> np.ndarray:
    height, width = nav_map.image.shape[:2]
    ros_data = np.asarray(nav2_map_occupancy_data(nav_map), dtype=np.int16).reshape(
        height, width
    )
    output = np.full((height, width), UNKNOWN, dtype=np.uint8)
    output[ros_data == 0] = FREE
    output[ros_data >= 65] = OCCUPIED
    return output


def _counts(occupancy: np.ndarray) -> dict[str, int]:
    return {
        "free": int(np.count_nonzero(occupancy == FREE)),
        "occupied": int(np.count_nonzero(occupancy == OCCUPIED)),
        "unknown": int(np.count_nonzero(occupancy == UNKNOWN)),
    }


def _assert_compatible(generated: Nav2Map, accepted: Nav2Map) -> None:
    if generated.image.shape[:2] != accepted.image.shape[:2]:
        raise ValueError("generated/accepted map size mismatch")
    if not math.isclose(generated.resolution_m, accepted.resolution_m, abs_tol=1e-12):
        raise ValueError("generated/accepted map resolution mismatch")
    if any(
        not math.isclose(a, b, abs_tol=1e-12)
        for a, b in zip(generated.origin, accepted.origin)
    ):
        raise ValueError("generated/accepted map origin mismatch")
    if generated.negate != accepted.negate:
        raise ValueError("generated/accepted map negate mismatch")
    if not math.isclose(generated.occupied_thresh, accepted.occupied_thresh, abs_tol=1e-12):
        raise ValueError("generated/accepted occupied threshold mismatch")
    if not math.isclose(generated.free_thresh, accepted.free_thresh, abs_tol=1e-12):
        raise ValueError("generated/accepted free threshold mismatch")


def _load_overrides(path: Path | str) -> tuple[dict[str, object], ...]:
    derivation_path = Path(path).expanduser().resolve()
    document = yaml.safe_load(derivation_path.read_text(encoding="utf-8"))
    if not isinstance(document, Mapping):
        raise ValueError("navigation derivation must be a mapping")
    if str(document.get("schema", "")) != "agt_ground_relative_navigation_map/v1":
        raise ValueError("unsupported navigation derivation schema")
    if str(document.get("revision_kind", "")) != "generated_plus_accepted_override_revision":
        raise ValueError("navigation derivation is not a generated/accepted freeze revision")
    if str(document.get("frame_id", "")) != "map":
        raise ValueError("formal navigation derivation frame_id must be map")
    raw = document.get("overrides")
    if not isinstance(raw, list):
        raise ValueError("navigation derivation overrides must be a list")

    output: list[dict[str, object]] = []
    seen: set[str] = set()
    for index, record in enumerate(raw):
        if not isinstance(record, Mapping):
            raise ValueError(f"override {index} must be a mapping")
        override_id = str(record.get("id", "")).strip()
        if not override_id:
            raise ValueError("formal override id must be non-empty")
        if override_id in seen:
            raise ValueError(f"duplicate override id: {override_id}")
        seen.add(override_id)
        mode = str(record.get("mode", "")).strip().lower()
        if mode not in _ALLOWED_MODES:
            raise ValueError("formal override mode must be force_free or force_occupied")
        reason = str(record.get("reason", "")).strip()
        if not reason:
            raise ValueError("formal override reason must be non-empty")
        evidence = str(record.get("evidence_category", "")).strip()
        if evidence not in _ALLOWED_EVIDENCE:
            raise ValueError("formal override evidence category is invalid")
        polygon = record.get("polygon_xy")
        if not isinstance(polygon, list) or len(polygon) < 3:
            raise ValueError("formal override polygon_xy requires at least three vertices")
        normalized_polygon: list[list[float]] = []
        for point in polygon:
            if not isinstance(point, (list, tuple)) or len(point) != 2:
                raise ValueError("formal override polygon vertex must be [x, y]")
            x, y = float(point[0]), float(point[1])
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError("formal override polygon coordinates must be finite")
            normalized_polygon.append([x, y])
        output.append(
            {
                "id": override_id,
                "mode": mode,
                "polygon_xy": normalized_polygon,
                "reason": reason,
                "evidence_category": evidence,
            }
        )
    return tuple(output)


def _navigation_result(nav_map: Nav2Map, occupancy: np.ndarray) -> NavigationMapResult:
    height, width = occupancy.shape
    zeros_f = np.zeros((height, width), dtype=np.float64)
    zeros_i = np.zeros((height, width), dtype=np.int32)
    return NavigationMapResult(
        resolution_m=float(nav_map.resolution_m),
        origin_x_m=float(nav_map.origin[0]),
        origin_y_m=float(nav_map.origin[1]),
        width=int(width),
        height=int(height),
        ground_height_m=zeros_f.copy(),
        ground_valid=np.zeros((height, width), dtype=bool),
        point_count=zeros_i.copy(),
        ground_support_count=zeros_i.copy(),
        obstacle_count=zeros_i.copy(),
        slope_deg=zeros_f.copy(),
        step_m=zeros_f.copy(),
        occupancy=np.asarray(occupancy, dtype=np.uint8).copy(),
        config=GroundRelativeNavigationConfig(resolution_m=float(nav_map.resolution_m)),
    )


def _replay(
    generated_map: Nav2Map,
    generated: np.ndarray,
    overrides: tuple[dict[str, object], ...],
) -> tuple[np.ndarray, int, int]:
    current = generated.copy()
    force_free_changed = 0
    force_occupied_changed = 0
    for record in overrides:
        before = current
        current = apply_navigation_overrides(
            _navigation_result(generated_map, before), [record]
        )
        changed = int(np.count_nonzero(before != current))
        if record["mode"] == "force_free":
            force_free_changed += changed
        else:
            force_occupied_changed += changed
    return current, force_free_changed, force_occupied_changed


def audit_map_revision(
    generated_map_yaml: Path | str,
    accepted_map_yaml: Path | str,
    derivation_yaml: Path | str,
) -> dict[str, object]:
    """Replay a formal override sequence and compare it to the accepted map."""
    generated_map = load_nav2_map(generated_map_yaml)
    accepted_map = load_nav2_map(accepted_map_yaml)
    _assert_compatible(generated_map, accepted_map)

    generated = _occupancy_classes(generated_map)
    accepted = _occupancy_classes(accepted_map)
    overrides = _load_overrides(derivation_yaml)
    replay, force_free_changed, force_occupied_changed = _replay(
        generated_map, generated, overrides
    )

    changed_mask = generated != accepted
    replay_mismatch = accepted != replay
    unexplained_changed = changed_mask & replay_mismatch
    changed_count = int(np.count_nonzero(changed_mask))
    total = int(generated.size)
    cell_area = float(generated_map.resolution_m) ** 2

    return {
        "schema_version": "1.0",
        "generated_map_yaml": str(Path(generated_map_yaml).expanduser().resolve()),
        "accepted_map_yaml": str(Path(accepted_map_yaml).expanduser().resolve()),
        "derivation_yaml": str(Path(derivation_yaml).expanduser().resolve()),
        "resolution_m": float(generated_map.resolution_m),
        "width": int(generated.shape[1]),
        "height": int(generated.shape[0]),
        "before_counts": _counts(generated),
        "after_counts": _counts(accepted),
        "override_count": len(overrides),
        "override_ids": [str(record["id"]) for record in overrides],
        "changed_cell_count": changed_count,
        "changed_area_m2": changed_count * cell_area,
        "changed_fraction": (changed_count / total if total else 0.0),
        "force_free_changed_cell_count": int(force_free_changed),
        "force_occupied_changed_cell_count": int(force_occupied_changed),
        "replay_mismatch_cell_count": int(np.count_nonzero(replay_mismatch)),
        "unexplained_changed_cell_count": int(np.count_nonzero(unexplained_changed)),
        "accepted_matches_replay": bool(np.array_equal(accepted, replay)),
    }


def _display_values(occupancy: np.ndarray) -> np.ndarray:
    display = np.full(occupancy.shape, 0.5, dtype=np.float64)
    display[occupancy == FREE] = 1.0
    display[occupancy == OCCUPIED] = 0.0
    return display


def write_map_quality_evidence(
    generated_map_yaml: Path | str,
    accepted_map_yaml: Path | str,
    derivation_yaml: Path | str,
    *,
    output_dir: Path | str,
) -> dict[str, object]:
    """Write planner-independent real-map audit JSON and review figures."""
    output = Path(output_dir).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)

    report = audit_map_revision(
        generated_map_yaml,
        accepted_map_yaml,
        derivation_yaml,
    )
    (output / "map_qa_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    generated_map = load_nav2_map(generated_map_yaml)
    accepted_map = load_nav2_map(accepted_map_yaml)
    generated = _occupancy_classes(generated_map)
    accepted = _occupancy_classes(accepted_map)
    overrides = _load_overrides(derivation_yaml)
    replay, _, _ = _replay(generated_map, generated, overrides)
    changed = generated != accepted
    mismatch = accepted != replay

    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as PolygonPatch

    extent = generated_map.extent
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 5.0), constrained_layout=True)
    panels = (
        (axes[0], generated, "Generated (pre-override)"),
        (axes[1], accepted, "Accepted (post-override)"),
    )
    for axis, occupancy, title in panels:
        axis.imshow(
            _display_values(occupancy),
            origin="lower",
            extent=extent,
            cmap="gray",
            vmin=0.0,
            vmax=1.0,
            interpolation="nearest",
        )
        axis.set_title(title)
        axis.set_aspect("equal")
        axis.set_xlabel("map x [m]")
        axis.set_ylabel("map y [m]")

    axes[2].imshow(
        _display_values(accepted),
        origin="lower",
        extent=extent,
        cmap="gray",
        vmin=0.0,
        vmax=1.0,
        interpolation="nearest",
        alpha=0.65,
    )
    changed_overlay = np.ma.masked_where(~changed, changed.astype(float))
    mismatch_overlay = np.ma.masked_where(~mismatch, mismatch.astype(float))
    axes[2].imshow(
        changed_overlay,
        origin="lower",
        extent=extent,
        cmap="autumn",
        vmin=0.0,
        vmax=1.0,
        interpolation="nearest",
        alpha=0.75,
    )
    axes[2].imshow(
        mismatch_overlay,
        origin="lower",
        extent=extent,
        cmap="Reds",
        vmin=0.0,
        vmax=1.0,
        interpolation="nearest",
        alpha=0.95,
    )
    for record in overrides:
        polygon = PolygonPatch(
            np.asarray(record["polygon_xy"], dtype=float),
            closed=True,
            fill=False,
            linewidth=1.2,
            linestyle="--",
            edgecolor=("tab:blue" if record["mode"] == "force_free" else "tab:red"),
        )
        axes[2].add_patch(polygon)
    axes[2].set_title("Changed cells + recorded overrides")
    axes[2].set_aspect("equal")
    axes[2].set_xlabel("map x [m]")
    axes[2].set_ylabel("map y [m]")

    status = "PASS" if report["accepted_matches_replay"] else "FAIL"
    fig.suptitle(
        "Planner-independent map curation QA | "
        f"replay={status} | changed={report['changed_cell_count']} | "
        f"unexplained={report['unexplained_changed_cell_count']}"
    )
    for suffix in ("svg", "pdf", "png"):
        fig.savefig(output / f"map_curation_qa.{suffix}", dpi=180)
    plt.close(fig)
    return report
