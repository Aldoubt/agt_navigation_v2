#!/usr/bin/env python3
"""Run controlled A0/A1/A2/A3 map ablations on one real PCD.

This runner is intentionally stricter than switching profiles in the GUI:
agricultural Row/Corridor geometry is derived once from A0 and then frozen for
all profiles. Therefore A1/A2/A3 change only Ground-navigation occupancy
policy while PCD, Site Boundary, Row geometry and Corridor geometry stay
identical.

Each profile also emits a planner-independent per-aisle blocker audit. For a
disconnected geometric aisle the audit finds a minimum-blocker end-to-end path
and attributes only the non-FREE cells on that path to sensor, terrain,
structure, boundary or UNKNOWN evidence. This is diagnostic evidence; it never
repairs the map.
"""

from __future__ import annotations

import argparse
from collections import Counter
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
from agt_offline_assets.aisle_blocker_audit import (  # noqa: E402
    aggregate_aisle_blocker_causes,
    build_aisle_blocker_audit,
)
from agt_offline_assets.aisle_centerlines import (  # noqa: E402
    derive_geometric_aisle_centerlines,
)
from agt_offline_assets.formal_navigation_map import (  # noqa: E402
    StructureAwareNavigationConfig,
    derive_hard_occupancy_provenance,
)
from agt_offline_assets.navigation_ablation import (  # noqa: E402
    ABLATION_PROFILE_KEYS,
    apply_navigation_ablation_profile,
    navigation_ablation_spec,
)


def _write_overlay(revision: Path) -> None:
    try:
        from PIL import Image
        from agt_offline_assets.navigation_grid import load_navigation_grid
    except ImportError:
        return
    grid = load_navigation_grid(revision / "accepted" / "navigation_map.yaml")
    image = np.zeros((*grid.occupancy.shape, 4), dtype=np.uint8)
    image[grid.occupancy == 254] = (245, 245, 245, 255)
    image[grid.occupancy == 205] = (245, 190, 70, 255)
    image[grid.occupancy == 0] = (35, 35, 35, 255)
    Image.fromarray(np.flipud(image), mode="RGBA").save(
        revision / "validation" / "navigation_review_overlay.png"
    )


def _failure_mode_counts(reports: list[dict[str, object]]) -> dict[str, int]:
    counts: Counter[str] = Counter()
    for report in reports:
        counts[str(report.get("failure_mode", "UNKNOWN"))] += 1
    return dict(sorted(counts.items()))


def _write_aisle_root_cause_report(
    revision: Path,
    *,
    profile: str,
    reports: list[dict[str, object]],
) -> tuple[Path, Path]:
    validation = revision / "validation"
    validation.mkdir(parents=True, exist_ok=True)
    json_path = validation / "aisle_root_cause.json"
    txt_path = validation / "aisle_root_cause.txt"
    document = {
        "schema": "agt_aisle_blocker_audit/v1",
        "profile": profile,
        "status": "EXPERIMENTAL_REVIEW_EVIDENCE",
        "connected_aisles": sum(bool(item.get("grid_connectivity")) for item in reports),
        "disconnected_aisles": sum(not bool(item.get("grid_connectivity")) for item in reports),
        "dominant_blocker_causes": aggregate_aisle_blocker_causes(reports),
        "failure_modes": _failure_mode_counts(reports),
        "aisles": reports,
    }
    json_path.write_text(
        json.dumps(document, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    lines = [
        f"profile: {profile}",
        f"connected: {document['connected_aisles']}/{len(reports)}",
        f"dominant causes: {json.dumps(document['dominant_blocker_causes'], ensure_ascii=False)}",
        "",
        "aisle | connected | failure mode | min blocker cells | dominant cause",
        "----- | --------- | ------------ | ----------------- | --------------",
    ]
    for report in reports:
        lines.append(
            f"{report['aisle_id']} | {bool(report['grid_connectivity'])} | "
            f"{report['failure_mode']} | {int(report['minimum_blocker_cell_count'])} | "
            f"{report['dominant_blocker_cause']}"
        )
    txt_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return json_path, txt_path


def _profile_summary(
    profile: str,
    *,
    navigation,
    geometric,
    materialized,
    accepted,
    qa,
    provenance,
    blocker_reports: list[dict[str, object]],
) -> dict[str, object]:
    interior = [
        dict(item)
        for item in (qa.get("aisles") or [])
        if str(item.get("pair_kind")) == "ROW_ROW"
    ]
    return {
        "profile": profile,
        "profile_spec": navigation_ablation_spec(profile).to_dict(),
        "ground_navigation": navigation.counts(),
        "hard_occupancy_provenance": provenance.counts(),
        "formal_generated": materialized.navigation.counts(),
        "formal_accepted": accepted.navigation.counts(),
        "materialization": materialized.counts(),
        "structural_safety_status": str(
            qa.get("structural_safety_status", qa.get("status", "NOT_EVALUATED"))
        ),
        "navigation_usability_status": str(
            qa.get("navigation_usability_status", "NOT_EVALUATED")
        ),
        "expected_interior_aisles": int(geometric.expected_interior_aisles),
        "geometric_interior_aisles": int(geometric.interior_geometric_aisle_count),
        "traversable_interior_aisles": len(interior),
        "connected_interior_aisles": sum(
            bool(item.get("grid_connectivity")) for item in interior
        ),
        "largest_free_component_fraction": float(
            qa.get("largest_free_component_fraction", 0.0)
        ),
        "map_unknown_fraction": float(qa.get("map_unknown_fraction", 0.0)),
        "aisle_root_cause": {
            "disconnected_aisle_count": sum(
                not bool(item.get("grid_connectivity")) for item in blocker_reports
            ),
            "dominant_blocker_causes": aggregate_aisle_blocker_causes(blocker_reports),
            "failure_modes": _failure_mode_counts(blocker_reports),
            "minimum_blocker_cells_total": sum(
                int(item.get("minimum_blocker_cell_count", 0))
                for item in blocker_reports
                if not bool(item.get("grid_connectivity"))
            ),
        },
    }


def run(args) -> dict[str, object]:
    output = args.output.expanduser().resolve()
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"ablation output must be empty/new: {output}")
    output.mkdir(parents=True, exist_ok=True)

    cloud = read_pcd(args.pcd)
    base_config = GroundRelativeNavigationConfig(
        resolution_m=args.resolution,
        ground_quantile=args.ground_quantile,
        maximum_ground_fill_distance_m=args.ground_fill_distance,
        ground_smoothing_radius_cells=args.ground_smoothing_radius,
        obstacle_min_height_m=args.obstacle_min_height,
        obstacle_max_height_m=args.obstacle_max_height,
        minimum_obstacle_points=args.minimum_obstacle_points,
        maximum_slope_deg=args.maximum_slope_deg,
        maximum_step_m=args.maximum_step_m,
        obstacle_padding_m=args.obstacle_padding_m,
    )
    base_config.validate()
    a0 = derive_ground_relative_navigation_map(cloud, base_config)

    # Freeze agricultural structure from A0. This is the key experimental
    # control: A1/A2/A3 must not silently change the Row/Corridor hypothesis.
    frozen_structure = derive_navigation_structure(a0, NavigationStructureConfig())
    frozen_corridor = derive_corridor_refinement(
        a0,
        frozen_structure,
        CorridorRefinementConfig(),
    )
    frozen_geometric = derive_geometric_aisle_centerlines(
        a0,
        frozen_structure,
        frozen_corridor,
    )
    boundary = load_site_boundary(args.site_boundary, expected_frame_id="map")
    formal_policy = StructureAwareNavigationConfig(
        soft_obstacle_max_count=args.soft_obstacle_max_count,
        soft_obstacle_max_ratio=args.soft_obstacle_max_ratio,
        soft_recovery_max_gap_m=args.soft_recovery_max_gap_m,
    )

    document: dict[str, object] = {
        "schema": "agt_navigation_ablation/v1",
        "input_pcd": str(args.pcd.expanduser().resolve()),
        "site_boundary": str(args.site_boundary.expanduser().resolve()),
        "points": int(cloud.points.shape[0]),
        "profiles": {},
        "control": {
            "structure_source_profile": "A0",
            "row_count": len(frozen_structure.row_model.centers_v_m),
            "accepted_row_count": len(frozen_corridor.accepted_row_centers_v_m),
            "expected_interior_aisles": int(frozen_geometric.expected_interior_aisles),
            "geometric_interior_aisles": int(
                frozen_geometric.interior_geometric_aisle_count
            ),
            "navigation_config_A0": vars(base_config),
        },
    }

    for profile in args.profiles:
        navigation = apply_navigation_ablation_profile(a0, profile)
        materialized = materialize_structure_aware_navigation_map(
            navigation,
            frozen_corridor,
            boundary,
            frame_id="map",
            row_direction_xy=frozen_structure.row_model.direction_xy,
            config=formal_policy,
        )
        accepted = replay_formal_navigation_overrides(
            materialized.navigation,
            frozen_corridor,
            boundary,
            [],
            frame_id="map",
        )
        qa = evaluate_formal_navigation_qa(
            ground_evidence=navigation,
            materialized=materialized,
            accepted=accepted,
            overrides=[],
            structure=frozen_structure,
            corridor=frozen_corridor,
            site_boundary=boundary,
        )
        provenance = derive_hard_occupancy_provenance(navigation, formal_policy)
        blocker_reports = build_aisle_blocker_audit(
            navigation,
            frozen_structure,
            frozen_corridor,
            accepted.navigation.occupancy,
            provenance,
            materialized=materialized,
        )

        destination = output / profile
        revision = export_structure_aware_navigation_revision(
            destination,
            ground_evidence=navigation,
            materialized=materialized,
            accepted=accepted,
            overrides=[],
            source_asset=args.pcd.name,
            frame_id="map",
            boundary_source="WORKBENCH_MANUAL_POLYGON",
        )
        write_formal_navigation_qa(
            qa,
            revision / "validation" / "navigation_validation.json",
        )
        _write_aisle_root_cause_report(
            revision,
            profile=profile,
            reports=blocker_reports,
        )
        (revision / "validation" / "ablation_profile.json").write_text(
            json.dumps(
                {
                    **navigation_ablation_spec(profile).to_dict(),
                    "status": "EXPERIMENTAL_REVIEW_EVIDENCE",
                    "frozen_structure_source": "A0",
                },
                indent=2,
                sort_keys=True,
                ensure_ascii=False,
            )
            + "\n",
            encoding="utf-8",
        )
        if args.write_overlays:
            _write_overlay(revision)

        document["profiles"][profile] = _profile_summary(
            profile,
            navigation=navigation,
            geometric=frozen_geometric,
            materialized=materialized,
            accepted=accepted,
            qa=qa,
            provenance=provenance,
            blocker_reports=blocker_reports,
        )

    (output / "ablation_summary.json").write_text(
        json.dumps(document, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return document


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcd", type=Path, required=True)
    parser.add_argument("--site-boundary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--profiles",
        nargs="+",
        choices=ABLATION_PROFILE_KEYS,
        default=list(ABLATION_PROFILE_KEYS),
    )
    parser.add_argument("--write-overlays", action="store_true")
    parser.add_argument("--resolution", type=float, default=0.10)
    parser.add_argument("--ground-quantile", type=float, default=0.10)
    parser.add_argument("--ground-fill-distance", type=float, default=0.35)
    parser.add_argument("--ground-smoothing-radius", type=int, default=2)
    parser.add_argument("--obstacle-min-height", type=float, default=0.12)
    parser.add_argument("--obstacle-max-height", type=float, default=1.00)
    parser.add_argument("--minimum-obstacle-points", type=int, default=2)
    parser.add_argument("--maximum-slope-deg", type=float, default=18.0)
    parser.add_argument("--maximum-step-m", type=float, default=0.12)
    parser.add_argument("--obstacle-padding-m", type=float, default=0.05)
    parser.add_argument("--soft-obstacle-max-count", type=int, default=4)
    parser.add_argument("--soft-obstacle-max-ratio", type=float, default=0.05)
    parser.add_argument("--soft-recovery-max-gap-m", type=float, default=0.60)
    args = parser.parse_args(argv)
    document = run(args)
    print(json.dumps(document, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
