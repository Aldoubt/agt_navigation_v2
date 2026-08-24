from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from agt_offline_assets import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    SiteBoundary,
)
from agt_offline_assets.contracts import sha256_file
from agt_offline_assets.formal_navigation_map import (
    materialize_structure_aware_navigation_map,
)
from agt_offline_assets.formal_navigation_override import (
    replay_formal_navigation_overrides,
)
from agt_offline_assets.formal_navigation_revision import (
    FORMAL_NAVIGATION_REVISION_SCHEMA,
    export_structure_aware_navigation_revision,
    write_structure_aware_navigation_payload,
)


def _navigation():
    occupancy = np.asarray(
        [[FREE, UNKNOWN, UNKNOWN, OCCUPIED, FREE]], dtype=np.uint8
    )
    shape = occupancy.shape
    return NavigationMapResult(
        resolution_m=0.10,
        origin_x_m=1.0,
        origin_y_m=2.0,
        width=shape[1],
        height=shape[0],
        ground_height_m=np.zeros(shape, dtype=np.float64),
        ground_valid=np.ones(shape, dtype=bool),
        point_count=np.full(shape, 4, dtype=np.int32),
        ground_support_count=np.full(shape, 3, dtype=np.int32),
        obstacle_count=np.zeros(shape, dtype=np.int32),
        slope_deg=np.zeros(shape, dtype=np.float64),
        step_m=np.zeros(shape, dtype=np.float64),
        occupancy=occupancy,
        config=GroundRelativeNavigationConfig(resolution_m=0.10),
    )


def _corridor():
    return SimpleNamespace(
        aisle_geometric_envelope=np.asarray(
            [[False, True, True, True, False]], dtype=bool
        ),
        row_structural_band=np.asarray(
            [[False, False, True, False, False]], dtype=bool
        ),
    )


def _boundary():
    return SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((1.0, 2.0), (1.5, 2.0), (1.5, 2.1), (1.0, 2.1)),
    )


def _override():
    return {
        "id": "ovr_0001",
        "mode": "force_occupied",
        "polygon_xy": [[1.4, 2.0], [1.5, 2.0], [1.5, 2.1], [1.4, 2.1]],
        "reason": "fixture obstacle",
        "evidence_category": "field_note",
    }


def _states():
    ground = _navigation()
    corridor = _corridor()
    boundary = _boundary()
    materialized = materialize_structure_aware_navigation_map(
        ground, corridor, boundary
    )
    accepted = replay_formal_navigation_overrides(
        materialized.navigation,
        corridor,
        boundary,
        [_override()],
    )
    return ground, materialized, accepted


def test_payload_writes_evidence_generated_accepted_and_derivation(tmp_path: Path):
    ground, materialized, accepted = _states()
    root = tmp_path / "payload"
    root.mkdir()

    write_structure_aware_navigation_payload(
        root,
        ground_evidence=ground,
        materialized=materialized,
        accepted=accepted,
        overrides=[_override()],
        source_asset="processed.pcd",
        frame_id="map",
    )

    required = {
        "evidence/ground_only_navigation_map.pgm",
        "evidence/ground_only_navigation_map.yaml",
        "evidence/ground_height.npy",
        "evidence/slope_deg.npy",
        "evidence/step_m.npy",
        "evidence/obstacle_count.npy",
        "evidence/ground_support_count.npy",
        "evidence/formal_materialization_masks.npz",
        "generated/navigation_map.pgm",
        "generated/navigation_map.yaml",
        "accepted/navigation_map.pgm",
        "accepted/navigation_map.yaml",
        "derivation.yaml",
    }
    assert required.issubset(
        {path.relative_to(root).as_posix() for path in root.rglob("*") if path.is_file()}
    )

    derivation = yaml.safe_load((root / "derivation.yaml").read_text())
    assert derivation["schema"] == "agt_ground_relative_navigation_map/v1"
    assert derivation["revision_kind"] == (
        "structure_aware_generated_plus_accepted_override_revision"
    )
    assert derivation["formal_revision_schema"] == FORMAL_NAVIGATION_REVISION_SCHEMA
    assert derivation["source_asset"] == "processed.pcd"
    assert derivation["frame_id"] == "map"
    assert derivation["outputs"]["generated"]["pgm_sha256"] == sha256_file(
        root / "generated/navigation_map.pgm"
    )
    assert derivation["outputs"]["accepted"]["yaml_sha256"] == sha256_file(
        root / "accepted/navigation_map.yaml"
    )


def test_payload_is_byte_deterministic_for_authority_files(tmp_path: Path):
    ground, materialized, accepted = _states()
    roots = [tmp_path / "first", tmp_path / "second"]
    for root in roots:
        root.mkdir()
        write_structure_aware_navigation_payload(
            root,
            ground_evidence=ground,
            materialized=materialized,
            accepted=accepted,
            overrides=[_override()],
            source_asset="processed.pcd",
            frame_id="map",
        )

    for relative in (
        Path("generated/navigation_map.pgm"),
        Path("generated/navigation_map.yaml"),
        Path("accepted/navigation_map.pgm"),
        Path("accepted/navigation_map.yaml"),
        Path("derivation.yaml"),
    ):
        assert (roots[0] / relative).read_bytes() == (roots[1] / relative).read_bytes()


def test_atomic_export_refuses_existing_destination(tmp_path: Path):
    ground, materialized, accepted = _states()
    destination = tmp_path / "revision"
    destination.mkdir()

    with pytest.raises(FileExistsError):
        export_structure_aware_navigation_revision(
            destination,
            ground_evidence=ground,
            materialized=materialized,
            accepted=accepted,
            overrides=[_override()],
            source_asset="processed.pcd",
            frame_id="map",
        )


def test_atomic_export_cleans_staging_when_payload_writer_fails(
    tmp_path: Path, monkeypatch
):
    ground, materialized, accepted = _states()
    destination = tmp_path / "revision"

    import agt_offline_assets.formal_navigation_revision as module

    def fail(*args, **kwargs):
        raise RuntimeError("injected")

    monkeypatch.setattr(module, "write_structure_aware_navigation_payload", fail)
    with pytest.raises(RuntimeError, match="injected"):
        export_structure_aware_navigation_revision(
            destination,
            ground_evidence=ground,
            materialized=materialized,
            accepted=accepted,
            overrides=[_override()],
            source_asset="processed.pcd",
            frame_id="map",
        )

    assert not destination.exists()
    assert not list(tmp_path.glob(".revision.tmp-*"))
