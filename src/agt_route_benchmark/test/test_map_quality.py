from pathlib import Path

import numpy as np
import pytest
import yaml

from agt_route_benchmark.map_quality import audit_map_revision


FREE = 254
OCCUPIED = 0


def _write_map(root: Path, name: str, occupancy_bottom_up: np.ndarray, *, resolution=1.0):
    root.mkdir(parents=True, exist_ok=True)
    image = np.flipud(np.asarray(occupancy_bottom_up, dtype=np.uint8))
    pgm = root / f"{name}.pgm"
    rows = [" ".join(str(int(v)) for v in row) for row in image]
    pgm.write_text(
        "P2\n{} {}\n255\n{}\n".format(
            image.shape[1], image.shape[0], "\n".join(rows)
        ),
        encoding="ascii",
    )
    map_yaml = root / f"{name}.yaml"
    map_yaml.write_text(
        yaml.safe_dump(
            {
                "image": pgm.name,
                "mode": "trinary",
                "resolution": resolution,
                "origin": [0.0, 0.0, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return map_yaml


def _derivation(path: Path):
    document = {
        "schema": "agt_ground_relative_navigation_map/v1",
        "revision_kind": "generated_plus_accepted_override_revision",
        "frame_id": "map",
        "overrides": [
            {
                "id": "ovr_0001",
                "mode": "force_occupied",
                "polygon_xy": [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
                "reason": "Measured permanent support post.",
                "evidence_category": "measured_structure",
            }
        ],
    }
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return path


def test_audit_accepts_exact_canonical_override_replay(tmp_path: Path):
    generated = np.full((3, 3), FREE, dtype=np.uint8)
    accepted = generated.copy()
    accepted[0, 0] = OCCUPIED
    generated_yaml = _write_map(tmp_path / "generated", "map", generated)
    accepted_yaml = _write_map(tmp_path / "accepted", "map", accepted)
    derivation = _derivation(tmp_path / "derivation.yaml")

    report = audit_map_revision(generated_yaml, accepted_yaml, derivation)

    assert report["accepted_matches_replay"] is True
    assert report["unexplained_changed_cell_count"] == 0
    assert report["changed_cell_count"] == 1
    assert report["changed_area_m2"] == pytest.approx(1.0)
    assert report["changed_fraction"] == pytest.approx(1.0 / 9.0)
    assert report["force_occupied_changed_cell_count"] == 1
    assert report["force_free_changed_cell_count"] == 0
    assert report["before_counts"] == {"free": 9, "occupied": 0, "unknown": 0}
    assert report["after_counts"] == {"free": 8, "occupied": 1, "unknown": 0}


def test_audit_fails_closed_on_unexplained_accepted_map_edit(tmp_path: Path):
    generated = np.full((3, 3), FREE, dtype=np.uint8)
    accepted = generated.copy()
    accepted[0, 0] = OCCUPIED
    accepted[2, 2] = OCCUPIED  # not covered by the recorded override
    generated_yaml = _write_map(tmp_path / "generated", "map", generated)
    accepted_yaml = _write_map(tmp_path / "accepted", "map", accepted)
    derivation = _derivation(tmp_path / "derivation.yaml")

    report = audit_map_revision(generated_yaml, accepted_yaml, derivation)

    assert report["accepted_matches_replay"] is False
    assert report["unexplained_changed_cell_count"] == 1
    assert report["changed_cell_count"] == 2


def test_audit_rejects_map_geometry_or_threshold_mismatch(tmp_path: Path):
    occupancy = np.full((3, 3), FREE, dtype=np.uint8)
    generated_yaml = _write_map(tmp_path / "generated", "map", occupancy, resolution=1.0)
    accepted_yaml = _write_map(tmp_path / "accepted", "map", occupancy, resolution=0.5)
    derivation = _derivation(tmp_path / "derivation.yaml")

    with pytest.raises(ValueError, match="resolution"):
        audit_map_revision(generated_yaml, accepted_yaml, derivation)
