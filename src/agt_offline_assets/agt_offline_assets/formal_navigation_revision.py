"""Serialization for formal V25 structure-aware Navigation Map revisions."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
from typing import Iterable, Mapping
import uuid

import numpy as np
import yaml

from .contracts import sha256_file
from .formal_navigation_map import StructureAwareNavigationResult
from .formal_navigation_override import FormalOverrideReplayResult
from .navigation_map_derivation import (
    NAVIGATION_DERIVATION_SCHEMA,
    NavigationMapResult,
    write_navigation_map_files,
)


FORMAL_NAVIGATION_REVISION_SCHEMA = "agt_structure_aware_navigation_revision/v1"
FORMAL_NAVIGATION_REVISION_KIND = (
    "structure_aware_generated_plus_accepted_override_revision"
)


def _write_ground_evidence(
    evidence_dir: Path,
    ground_evidence: NavigationMapResult,
) -> dict[str, str]:
    written = write_navigation_map_files(ground_evidence, evidence_dir)
    old_pgm = Path(written["pgm_path"])
    old_yaml = Path(written["yaml_path"])
    pgm = evidence_dir / "ground_only_navigation_map.pgm"
    yaml_path = evidence_dir / "ground_only_navigation_map.yaml"
    old_pgm.replace(pgm)

    document = yaml.safe_load(old_yaml.read_text(encoding="utf-8"))
    document["image"] = pgm.name
    yaml_path.write_text(
        yaml.safe_dump(document, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    old_yaml.unlink()
    return {
        "pgm": "evidence/ground_only_navigation_map.pgm",
        "yaml": "evidence/ground_only_navigation_map.yaml",
        "pgm_sha256": sha256_file(pgm),
        "yaml_sha256": sha256_file(yaml_path),
    }


def _write_evidence_arrays(
    evidence_dir: Path,
    ground_evidence: NavigationMapResult,
    materialized: StructureAwareNavigationResult,
) -> Path:
    np.save(evidence_dir / "ground_height.npy", ground_evidence.ground_height_m)
    np.save(evidence_dir / "slope_deg.npy", ground_evidence.slope_deg)
    np.save(evidence_dir / "step_m.npy", ground_evidence.step_m)
    np.save(evidence_dir / "obstacle_count.npy", ground_evidence.obstacle_count)
    np.save(
        evidence_dir / "ground_support_count.npy",
        ground_evidence.ground_support_count,
    )
    masks = evidence_dir / "formal_materialization_masks.npz"
    np.savez_compressed(
        masks,
        observed_free_mask=materialized.observed_free_mask.astype(np.uint8),
        structure_inferred_free_mask=materialized.structure_inferred_free_mask.astype(
            np.uint8
        ),
        base_hard_occupied_mask=materialized.base_hard_occupied_mask.astype(np.uint8),
        row_structural_blocked_mask=materialized.row_structural_blocked_mask.astype(
            np.uint8
        ),
        site_boundary_blocked_mask=materialized.site_boundary_blocked_mask.astype(
            np.uint8
        ),
        unresolved_unknown_mask=materialized.unresolved_unknown_mask.astype(np.uint8),
    )
    return masks


def _grid_record(navigation: NavigationMapResult) -> dict[str, object]:
    return {
        "resolution_m": float(navigation.resolution_m),
        "origin_xy_m": [
            float(navigation.origin_x_m),
            float(navigation.origin_y_m),
        ],
        "width": int(navigation.width),
        "height": int(navigation.height),
        "bounds_m": [float(value) for value in navigation.bounds_m()],
    }


def write_structure_aware_navigation_payload(
    root: str | Path,
    *,
    ground_evidence: NavigationMapResult,
    materialized: StructureAwareNavigationResult,
    accepted: FormalOverrideReplayResult,
    overrides: Iterable[Mapping[str, object]],
    source_asset: str | None = None,
    frame_id: str = "map",
    boundary_source: str | None = None,
) -> Path:
    """Write Evidence, Generated and Accepted products into an existing root."""

    root = Path(root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    evidence_dir = root / "evidence"
    generated_dir = root / "generated"
    accepted_dir = root / "accepted"
    evidence_dir.mkdir(parents=True, exist_ok=True)

    expected_shape = ground_evidence.occupancy.shape
    if materialized.navigation.occupancy.shape != expected_shape:
        raise ValueError("materialized grid does not match Ground Evidence")
    if accepted.navigation.occupancy.shape != expected_shape:
        raise ValueError("Accepted grid does not match Ground Evidence")
    for navigation in (materialized.navigation, accepted.navigation):
        if (
            navigation.resolution_m != ground_evidence.resolution_m
            or navigation.origin_x_m != ground_evidence.origin_x_m
            or navigation.origin_y_m != ground_evidence.origin_y_m
            or navigation.width != ground_evidence.width
            or navigation.height != ground_evidence.height
        ):
            raise ValueError("formal navigation grid geometry mismatch")

    evidence_output = _write_ground_evidence(evidence_dir, ground_evidence)
    masks_path = _write_evidence_arrays(evidence_dir, ground_evidence, materialized)
    generated_output = write_navigation_map_files(
        materialized.navigation, generated_dir
    )
    accepted_output = write_navigation_map_files(accepted.navigation, accepted_dir)
    ordered_overrides = [dict(record) for record in overrides]

    record = {
        "schema": NAVIGATION_DERIVATION_SCHEMA,
        "revision_kind": FORMAL_NAVIGATION_REVISION_KIND,
        "formal_revision_schema": FORMAL_NAVIGATION_REVISION_SCHEMA,
        "materialization_schema": materialized.schema,
        "frame_id": str(frame_id),
        "boundary_source": boundary_source,
        "source_asset": source_asset,
        "grid": _grid_record(ground_evidence),
        "counts": {
            "evidence": ground_evidence.counts(),
            "generated": materialized.navigation.counts(),
            "accepted": accepted.navigation.counts(),
        },
        "materialization_counts": materialized.counts(),
        "override_metrics": {
            "force_free_changed_cell_count": int(
                accepted.force_free_changed_cell_count
            ),
            "force_occupied_changed_cell_count": int(
                accepted.force_occupied_changed_cell_count
            ),
            "force_free_area_m2": float(accepted.force_free_area_m2),
            "force_occupied_area_m2": float(accepted.force_occupied_area_m2),
        },
        "overrides": ordered_overrides,
        "outputs": {
            "evidence": {
                **evidence_output,
                "materialization_masks": "evidence/formal_materialization_masks.npz",
                "materialization_masks_sha256": sha256_file(masks_path),
            },
            "generated": {
                "pgm": "generated/navigation_map.pgm",
                "yaml": "generated/navigation_map.yaml",
                "pgm_sha256": generated_output["pgm_sha256"],
                "yaml_sha256": generated_output["yaml_sha256"],
            },
            "accepted": {
                "pgm": "accepted/navigation_map.pgm",
                "yaml": "accepted/navigation_map.yaml",
                "pgm_sha256": accepted_output["pgm_sha256"],
                "yaml_sha256": accepted_output["yaml_sha256"],
            },
        },
    }
    (root / "derivation.yaml").write_text(
        yaml.safe_dump(record, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return root


def export_structure_aware_navigation_revision(
    destination: str | Path,
    *,
    ground_evidence: NavigationMapResult,
    materialized: StructureAwareNavigationResult,
    accepted: FormalOverrideReplayResult,
    overrides: Iterable[Mapping[str, object]],
    source_asset: str | None = None,
    frame_id: str = "map",
    boundary_source: str | None = None,
) -> Path:
    """Atomically publish one immutable navigation-only formal revision."""

    destination = Path(destination).expanduser().resolve()
    if destination.exists():
        raise FileExistsError(f"formal navigation revision already exists: {destination}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    staging = destination.parent / f".{destination.name}.tmp-{uuid.uuid4().hex}"
    staging.mkdir(parents=False, exist_ok=False)
    try:
        write_structure_aware_navigation_payload(
            staging,
            ground_evidence=ground_evidence,
            materialized=materialized,
            accepted=accepted,
            overrides=overrides,
            source_asset=source_asset,
            frame_id=frame_id,
            boundary_source=boundary_source,
        )
        os.replace(staging, destination)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return destination
