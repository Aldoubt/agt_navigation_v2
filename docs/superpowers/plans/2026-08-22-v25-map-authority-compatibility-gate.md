# V25 Map Authority Compatibility Gate Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prove that the new structure-aware map revision remains a valid `V25_MAP_WORKBENCH / BOUND_VERIFIED` authority and does not reopen the competing Paper-owned raster path fixed by M1.5-4B.

**Architecture:** This is a compatibility gate, not a new subsystem. Generate one real structure-aware revision with the Plan 1 revision writer, bind it through the existing `agt_map_pipeline.map_authority` contract, and require exact accepted-map grid/hash identity. No production code in `map_authority.py` should change unless the failing test exposes a real incompatibility.

**Tech Stack:** Python 3, pytest, PyYAML, existing `agt_map_pipeline` + `agt_offline_assets` contracts.

**Spec:** `docs/superpowers/specs/2026-08-22-v25-unified-map-authoring-structure-aware-pgm-design.md`

**Dependency:** Execute after Task 3 of `docs/superpowers/plans/2026-08-22-v25-structure-aware-pgm-and-revision.md` and before its Workbench/real-map gate.

## Global Constraints

- `V25 Map Workbench accepted revision` remains the only formal Paper I Navigation Map authority.
- `agt_map_pipeline` remains consumer/verifier/evidence producer only.
- The new revision must retain `generated/navigation_map.yaml`, `accepted/navigation_map.yaml`, and `derivation.yaml`.
- Generated and Accepted must share exactly one grid.
- The bound accepted YAML/PGM SHA256 values must be the files produced by the structure-aware revision writer.
- Do not weaken `MAP_AUTHORITY_SCHEMA`, `MAP_AUTHORITY_NAME`, `MAP_AUTHORITY_STATUS`, frame checks, grid checks, or drift detection.

---

### Task 1: Bind a Structure-Aware Revision Through the Existing V25 Authority Contract

**Files:**
- Modify: `src/agt_map_pipeline/test/test_map_authority.py`
- Modify only if required by a demonstrated incompatibility: `src/agt_map_pipeline/agt_map_pipeline/map_authority.py`

**Interfaces:**
- Consumes: `export_structure_aware_navigation_revision` from Plan 1 Task 3 and `bind_v25_map_revision` / `verify_bound_map_authority` from `agt_map_pipeline.map_authority`.
- Produces: regression coverage proving the new revision binds as `V25_MAP_WORKBENCH` with `BOUND_VERIFIED` status.

- [ ] **Step 1: Add one self-contained structure-aware revision fixture to `test_map_authority.py`**

Use the real Plan 1 public classes/functions. The fixture must create a 2x3 Ground Evidence map, a full Site Boundary, a corridor with one UNKNOWN aisle cell, materialize Generated, replay no manual overrides, and export the revision:

```python
from types import SimpleNamespace
import numpy as np

from agt_offline_assets import (
    FREE,
    UNKNOWN,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    SiteBoundary,
    materialize_structure_aware_navigation_map,
    replay_formal_navigation_overrides,
    export_structure_aware_navigation_revision,
)


def _structure_aware_revision(tmp_path):
    occupancy = np.array(
        [
            [FREE, UNKNOWN, FREE],
            [FREE, FREE, FREE],
        ],
        dtype=np.uint8,
    )
    shape = occupancy.shape
    zeros_f = np.zeros(shape, dtype=np.float64)
    zeros_i = np.zeros(shape, dtype=np.int32)
    ground = NavigationMapResult(
        resolution_m=0.10,
        origin_x_m=1.0,
        origin_y_m=2.0,
        width=3,
        height=2,
        ground_height_m=zeros_f.copy(),
        ground_valid=np.ones(shape, dtype=bool),
        point_count=np.ones(shape, dtype=np.int32),
        ground_support_count=np.ones(shape, dtype=np.int32),
        obstacle_count=zeros_i.copy(),
        slope_deg=zeros_f.copy(),
        step_m=zeros_f.copy(),
        occupancy=occupancy,
        config=GroundRelativeNavigationConfig(resolution_m=0.10),
    )
    corridor = SimpleNamespace(
        aisle_geometric_envelope=np.array(
            [[False, True, False], [False, False, False]], dtype=bool
        ),
        row_structural_band=np.zeros(shape, dtype=bool),
    )
    boundary = SiteBoundary(
        frame_id="map",
        outer_boundary_xy=(
            (1.0, 2.0),
            (1.3, 2.0),
            (1.3, 2.2),
            (1.0, 2.2),
        ),
    )
    generated = materialize_structure_aware_navigation_map(
        ground,
        corridor,
        boundary,
    )
    accepted = replay_formal_navigation_overrides(
        generated.navigation,
        corridor,
        boundary,
        (),
    )
    revision = tmp_path / "structure_aware_revision"
    export_structure_aware_navigation_revision(
        revision,
        ground_evidence=ground,
        materialized=generated,
        accepted=accepted,
        overrides=(),
        source_asset="processed.pcd",
        frame_id="map",
    )
    return revision
```

If the Plan 1 exporter uses positional parameters for any of the authority-data arguments, change the test to the exact Plan 1 public signature rather than adding a compatibility wrapper.

- [ ] **Step 2: Add the failing authority-binding assertion**

```python
def test_structure_aware_revision_binds_as_v25_workbench_authority(tmp_path):
    revision = _structure_aware_revision(tmp_path)
    binding = bind_v25_map_revision(revision)

    assert binding["schema"] == MAP_AUTHORITY_SCHEMA
    assert binding["authority"] == "V25_MAP_WORKBENCH"
    assert binding["status"] == "BOUND_VERIFIED"
    assert binding["frame_id"] == "map"
    assert binding["grid"] == {
        "resolution_m": 0.10,
        "origin_xy_m": [1.0, 2.0],
        "width": 3,
        "height": 2,
    }
    assert Path(binding["accepted_map_yaml_path"]) == (
        revision / "accepted/navigation_map.yaml"
    ).resolve()
    assert Path(binding["accepted_map_pgm_path"]) == (
        revision / "accepted/navigation_map.pgm"
    ).resolve()
```

- [ ] **Step 3: Run the focused test**

```bash
PYTHONPATH=src/agt_map_pipeline:src/agt_offline_assets \
python3 -m pytest -q \
  src/agt_map_pipeline/test/test_map_authority.py
```

Expected after Plan 1 Task 3: PASS without changing `map_authority.py`. If it fails, stop and classify the exact mismatch before editing production authority code.

- [ ] **Step 4: Preserve drift rejection on the new revision**

Add:

```python
def test_structure_aware_bound_authority_rejects_accepted_map_drift(tmp_path):
    revision = _structure_aware_revision(tmp_path)
    binding = bind_v25_map_revision(revision)
    accepted_pgm = revision / "accepted/navigation_map.pgm"
    accepted_pgm.write_bytes(accepted_pgm.read_bytes() + b"drift")

    with pytest.raises(MapAuthorityError, match="accepted map PGM hash mismatch"):
        verify_bound_map_authority(binding)
```

- [ ] **Step 5: Run the M1.5-4B map-authority regressions**

```bash
PYTHONPATH=src/agt_map_pipeline:src/agt_offline_assets \
python3 -m pytest -q \
  src/agt_map_pipeline/test/test_map_authority.py \
  src/agt_map_pipeline/test/test_prepare.py \
  src/agt_map_pipeline/test/test_project_contract.py \
  src/agt_map_pipeline/test/test_frame_verification.py
```

Required: PASS. Formal Paper prepare must still not expose Paper-owned `navigation.nav2_yaml`, `navigation.nav2_pgm`, or `navigation.raw_occupancy` layers.

- [ ] **Step 6: Commit only the compatibility test unless production change was necessary**

Normal expected commit:

```bash
git add src/agt_map_pipeline/test/test_map_authority.py
git commit -m "test(map): bind structure-aware revision to V25 authority"
```

If a production authority fix was genuinely required, include only the minimal verified `map_authority.py` change in the same commit and document why the old invariant could not bind the otherwise valid revision.

---

## Completion Gate

```text
structure-aware revision -> bind_v25_map_revision PASS
authority == V25_MAP_WORKBENCH
status == BOUND_VERIFIED
accepted map/grid/hash identity exact
drift rejection PASS
M1.5-4B prepare/project/frame regressions PASS
no Paper-owned formal raster producer reintroduced
```
