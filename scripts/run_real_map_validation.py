#!/usr/bin/env python3
"""Run the real, offline structure-aware Navigation Map validation chain.

This CLI is intentionally ROS-free: it consumes one PCD and publishes only
immutable offline evidence under the requested revision directory. A human
Workbench Site Boundary may be supplied for a formal-ready check; otherwise the
PCD grid bounds remain an explicit non-authoritative fallback.
"""

from __future__ import annotations

import argparse
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
    export_structure_aware_navigation_revision,
    load_site_boundary,
    materialize_structure_aware_navigation_map,
    read_pcd,
    replay_formal_navigation_overrides,
    write_formal_navigation_qa,
)
from agt_offline_assets.contracts import sha256_file  # noqa: E402
from agt_offline_assets.formal_navigation_map import (  # noqa: E402
    StructureAwareNavigationConfig,
)


GRID_BOUNDS_FALLBACK = "GRID_BOUNDS_FALLBACK"
HUMAN_REVIEW_REQUIRED = "HUMAN_REVIEW_REQUIRED"
_HUMAN_BOUNDARY_SOURCES = {
    "HUMAN_CONFIRMED_WORKBENCH",
    "WORKBENCH_MANUAL_POLYGON",
}


def _boundary_source(boundary: SiteBoundary) -> str:
    source = dict(boundary.source or {})
    return str(source.get("boundary_source", source.get("authoring_mode", "UNKNOWN")))


def _apply_release_gate(
    summary: dict[str, object], *, boundary_source: str, qa: dict[str, object]
) -> None:
    """Separate technical QA, usability review and human release authorization."""

    structural = str(qa.get("structural_safety_status", qa.get("status", "FAIL")))
    usability = str(qa.get("navigation_usability_status", "NOT_EVALUATED"))
    boundary_warnings = list(qa.get("site_boundary_warnings") or [])
    human_boundary = boundary_source in _HUMAN_BOUNDARY_SOURCES
    summary["boundary_source"] = str(boundary_source)
    summary["formal_ready"] = bool(
        structural == "PASS"
        and usability == "PASS"
        and human_boundary
        and not boundary_warnings
    )
    if structural != "PASS":
        review = "QA_FAILED"
    elif not human_boundary:
        review = HUMAN_REVIEW_REQUIRED
    elif boundary_warnings:
        review = "BOUNDARY_REVIEW_REQUIRED"
    elif usability != "PASS":
        review = "NAVIGATION_REVIEW_REQUIRED"
    else:
        review = "READY"
    summary["review_status"] = review


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


def run(
    pcd_path: Path,
    output: Path,
    interactive: bool,
    *,
    site_boundary_path: Path | None = None,
    materialization_config: StructureAwareNavigationConfig | None = None,
) -> dict[str, object]:
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
        "structural_safety_status": "FAIL",
        "navigation_usability_status": "NOT_EVALUATED",
        "accepted_aisle_count": 0,
        "connected_aisle_count": 0,
        "largest_free_component_fraction": 0.0,
        "structure_recovered_soft_occupied_cell_count": 0,
        "formal_ready": False,
        "review_status": "QA_FAILED",
        "boundary_source": GRID_BOUNDS_FALLBACK,
        "site_boundary_warnings": [],
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
        boundary = (
            load_site_boundary(site_boundary_path, expected_frame_id="map")
            if site_boundary_path is not None
            else _boundary_for_grid(navigation)
        )
        source = _boundary_source(boundary)
        materialized = materialize_structure_aware_navigation_map(
            navigation,
            corridor,
            boundary,
            frame_id="map",
            row_direction_xy=structure.row_model.direction_xy,
            config=materialization_config or StructureAwareNavigationConfig(),
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
        summary["structural_safety_status"] = str(
            qa.get("structural_safety_status", qa.get("status", "FAIL"))
        )
        summary["navigation_usability_status"] = str(
            qa.get("navigation_usability_status", "NOT_EVALUATED")
        )
        summary["accepted_aisle_count"] = int(qa.get("accepted_aisle_count", 0))
        summary["connected_aisle_count"] = int(qa.get("connected_aisle_count", 0))
        summary["largest_free_component_fraction"] = float(
            qa.get("largest_free_component_fraction", 0.0)
        )
        summary["structure_recovered_soft_occupied_cell_count"] = int(
            qa.get("structure_recovered_soft_occupied_cell_count", 0)
        )
        summary["site_boundary_warnings"] = list(
            qa.get("site_boundary_warnings") or []
        )
        _apply_release_gate(summary, boundary_source=source, qa=qa)
        counts = accepted.navigation.counts()
        summary["free_cells"] = counts["free"]
        summary["occupied_cells"] = counts["occupied"]
        summary["unknown_cells"] = counts["unknown"]
        if qa.get("status") != "PASS":
            raise RuntimeError("formal navigation structural QA did not PASS")
        revision = export_structure_aware_navigation_revision(
            output,
            ground_evidence=navigation,
            materialized=materialized,
            accepted=accepted,
            overrides=[],
            source_asset=pcd_path.name,
            frame_id="map",
            boundary_source=source,
        )
        write_formal_navigation_qa(
            qa, revision / "validation" / "navigation_validation.json"
        )
        _write_review_png(revision, interactive=interactive)
        summary["sha256"] = sha256_file(
            revision / "accepted" / "navigation_map.pgm"
        )
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
    parser.add_argument("--site-boundary", type=Path)
    parser.add_argument("--interactive-review", action="store_true")
    parser.add_argument("--soft-obstacle-max-count", type=int, default=4)
    parser.add_argument("--soft-obstacle-max-ratio", type=float, default=0.05)
    parser.add_argument("--soft-recovery-max-gap-m", type=float, default=0.60)
    args = parser.parse_args(argv)
    policy = StructureAwareNavigationConfig(
        soft_obstacle_max_count=args.soft_obstacle_max_count,
        soft_obstacle_max_ratio=args.soft_obstacle_max_ratio,
        soft_recovery_max_gap_m=args.soft_recovery_max_gap_m,
    )
    summary = run(
        args.pcd,
        args.output,
        args.interactive_review,
        site_boundary_path=args.site_boundary,
        materialization_config=policy,
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if summary["qa_status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
