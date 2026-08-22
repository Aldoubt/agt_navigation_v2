#!/usr/bin/env python3
"""Run the real, offline structure-aware Navigation Map validation chain.

This CLI is intentionally ROS-free: it consumes one PCD and publishes only
immutable offline evidence under the requested revision directory.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "agt_offline_assets"))
sys.path.insert(0, str(ROOT / "src" / "agt_ui_bridge"))
sys.path.insert(0, str(ROOT / "src" / "agt_coverage_planning"))

from agt_offline_assets import (  # noqa: E402
    CorridorRefinementConfig,
    GroundRelativeNavigationConfig,
    NavigationStructureConfig,
    SiteBoundary,
    derive_corridor_refinement,
    derive_ground_relative_navigation_map,
    derive_navigation_structure,
    evaluate_formal_navigation_qa,
    materialize_structure_aware_navigation_map,
    read_pcd,
    replay_formal_navigation_overrides,
    export_structure_aware_navigation_revision,
    write_formal_navigation_qa,
)
from agt_offline_assets.contracts import sha256_file  # noqa: E402


GRID_BOUNDS_FALLBACK = "GRID_BOUNDS_FALLBACK"
HUMAN_REVIEW_REQUIRED = "HUMAN_REVIEW_REQUIRED"


def _apply_release_gate(
    summary: dict[str, object], *, boundary_source: str, qa_status: str
) -> None:
    """Separate technical QA from human release authorization."""
    summary["boundary_source"] = str(boundary_source)
    summary["formal_ready"] = bool(
        qa_status == "PASS" and boundary_source == "HUMAN_CONFIRMED_WORKBENCH"
    )
    summary["review_status"] = (
        "READY" if summary["formal_ready"] else HUMAN_REVIEW_REQUIRED
    ) if qa_status == "PASS" else "QA_FAILED"


def _boundary_for_grid(result) -> SiteBoundary:
    x0, y0, x1, y1 = result.bounds_m()
    return SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((x0, y0), (x1, y0), (x1, y1), (x0, y1)),
        source={
            "boundary_source": GRID_BOUNDS_FALLBACK,
            "authoring_mode": "REAL_PCD_VALIDATION_GRID_BOUNDS",
        },
    )


def _write_review_png(revision: Path, *, interactive: bool) -> Path | None:
    if not interactive:
        return None
    try:
        from PIL import Image
    except ImportError as exc:
        raise RuntimeError("--interactive-review requires python3-pil") from exc
    from agt_offline_assets.navigation_grid import load_navigation_grid

    grid = load_navigation_grid(revision / "accepted" / "navigation_map.yaml")
    image = np.zeros((*grid.occupancy.shape, 4), dtype=np.uint8)
    image[grid.occupancy == 254] = (245, 245, 245, 255)
    image[grid.occupancy == 205] = (245, 190, 70, 255)
    image[grid.occupancy == 0] = (35, 35, 35, 255)
    output = revision / "validation" / "navigation_review_overlay.png"
    Image.fromarray(np.flipud(image), mode="RGBA").save(output)
    return output


def run(pcd_path: Path, output: Path, interactive: bool) -> dict[str, object]:
    cloud = read_pcd(pcd_path)
    points = int(cloud.points.shape[0])
    summary: dict[str, object] = {
        "input_pcd": str(pcd_path.resolve()),
        "points": points,
        "resolution": None,
        "free_cells": 0,
        "occupied_cells": 0,
        "unknown_cells": 0,
        "qa_status": "FAIL",
        "formal_ready": False,
        "review_status": "QA_FAILED",
        "boundary_source": GRID_BOUNDS_FALLBACK,
        "sha256": None,
    }
    revision: Path | None = None
    try:
        navigation = derive_ground_relative_navigation_map(
            cloud, GroundRelativeNavigationConfig()
        )
        summary["resolution"] = float(navigation.resolution_m)
        structure = derive_navigation_structure(
            navigation, NavigationStructureConfig()
        )
        corridor = derive_corridor_refinement(
            navigation, structure, CorridorRefinementConfig()
        )
        boundary = _boundary_for_grid(navigation)
        materialized = materialize_structure_aware_navigation_map(
            navigation, corridor, boundary, frame_id="map"
        )
        accepted = replay_formal_navigation_overrides(
            materialized.navigation, corridor, boundary, [], frame_id="map"
        )
        qa = evaluate_formal_navigation_qa(
            ground_evidence=navigation,
            materialized=materialized,
            accepted=accepted,
            overrides=[],
            structure=structure,
            corridor=corridor,
            site_boundary=boundary,
        )
        summary["qa_status"] = str(qa.get("status", "FAIL"))
        _apply_release_gate(
            summary,
            boundary_source=GRID_BOUNDS_FALLBACK,
            qa_status=str(summary["qa_status"]),
        )
        counts = accepted.navigation.counts()
        summary["free_cells"] = counts["free"]
        summary["occupied_cells"] = counts["occupied"]
        summary["unknown_cells"] = counts["unknown"]
        if qa.get("status") != "PASS":
            raise RuntimeError("formal navigation QA did not PASS")
        revision = export_structure_aware_navigation_revision(
            output,
            ground_evidence=navigation,
            materialized=materialized,
            accepted=accepted,
            overrides=[],
            source_asset=pcd_path.name,
            frame_id="map",
            boundary_source=GRID_BOUNDS_FALLBACK,
        )
        write_formal_navigation_qa(qa, revision / "validation" / "navigation_validation.json")
        _write_review_png(revision, interactive=interactive)
        summary["sha256"] = sha256_file(revision / "accepted" / "navigation_map.pgm")
    except Exception as exc:
        summary["error"] = str(exc)
        output.mkdir(parents=True, exist_ok=True)
    summary_path = (revision or output) / "validation_summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcd", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--interactive-review", action="store_true")
    args = parser.parse_args(argv)
    summary = run(args.pcd, args.output, args.interactive_review)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if summary["qa_status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

