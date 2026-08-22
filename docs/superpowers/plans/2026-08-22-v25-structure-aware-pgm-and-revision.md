# V25 Structure-Aware PGM and Revision Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn Ground-only evidence plus validated greenhouse row/aisle structure into one deterministic formal Nav2 Generated PGM, replay validated human overrides into Accepted, and freeze a planner-independent auditable map revision with QA.

**Architecture:** Keep `NavigationMapResult` as the common grid/evidence carrier. Add a pure offline structure-aware materializer that may promote only Ground-only `UNKNOWN` cells inside valid aisle geometry, while preserving Site Boundary, base `OCCUPIED`, and row structural bands as hard blockers. Separate generated materialization, validated accepted-map replay, serialization, and QA into focused modules so the Workbench can compose them without owning map logic.

**Tech Stack:** Python 3, NumPy, SciPy, PyYAML, ROS2 Humble/ament_cmake_python, PyQt5 Workbench integration, pytest.

**Spec:** `docs/superpowers/specs/2026-08-22-v25-unified-map-authoring-structure-aware-pgm-design.md`

## Global Constraints

- One PCD lineage -> one AGT Map Workbench authoring workflow -> one immutable Map Revision -> one accepted Navigation Map authority -> one semantic task bound by SHA256 to that accepted map.
- `agt_map_pipeline` remains a consumer/verifier/evidence producer; it must not create a competing formal Navigation Map.
- Only Ground-only `UNKNOWN` cells may be promoted automatically by agricultural structure.
- Ground-only `OCCUPIED` cells must never be automatically promoted to FREE.
- Cells outside Site Boundary are formal OCCUPIED.
- Cells inside `row_structural_band` are formal OCCUPIED.
- No global `UNKNOWN -> FREE`, map-wide flood fill, cross-row morphology, or route-dependent PGM whitening.
- No new automatic headland detector in this increment.
- Paper I formal manual raster modes remain `FORCE_FREE` and `FORCE_OCCUPIED`.
- `FORCE_FREE` must be rejected outside Site Boundary and inside `row_structural_band`.
- Ground-only output is Evidence; automatic structure-aware output is Generated; Generated plus validated formal overrides is Accepted.
- Existing M1.5-4B minimum revision paths `generated/navigation_map.*`, `accepted/navigation_map.*`, and `derivation.yaml` remain present.
- Formal freeze is fail-closed and deterministic.

---

## File Structure

### New focused core files

- `src/agt_offline_assets/agt_offline_assets/formal_navigation_map.py` — pure Generated-map materialization and provenance masks.
- `src/agt_offline_assets/agt_offline_assets/formal_navigation_override.py` — formal override rasterization plus Site Boundary / row-band FORCE_FREE safety checks.
- `src/agt_offline_assets/agt_offline_assets/formal_navigation_revision.py` — Evidence/Generated/Accepted serialization into a revision payload and atomic navigation-only export helper.
- `src/agt_offline_assets/agt_offline_assets/formal_navigation_qa.py` — planner-independent invariant, aisle occupancy, connectivity, and replay QA.

### Existing files modified

- `src/agt_offline_assets/agt_offline_assets/__init__.py` — export the new public interfaces.
- `src/agt_offline_assets/CMakeLists.txt` — register focused tests.
- `src/agt_route_benchmark/agt_route_benchmark/map_quality.py` — accept/replay both historical and new structure-aware revision kinds without changing historical behavior.
- `src/agt_map_workbench/agt_map_workbench/navigation_preview.py` — add Generated/materialization-mask preview support without replacing Ground evidence state.
- `src/agt_map_workbench/agt_map_workbench/paper1_workbench.py` — build and preview the structure-aware Generated map, validate overrides, run QA, and use the new revision payload writer.
- `src/agt_map_workbench/CMakeLists.txt` — register Workbench integration tests.

### New tests

- `src/agt_offline_assets/test/test_formal_navigation_map.py`
- `src/agt_offline_assets/test/test_formal_navigation_override.py`
- `src/agt_offline_assets/test/test_formal_navigation_revision.py`
- `src/agt_offline_assets/test/test_formal_navigation_qa.py`
- `src/agt_route_benchmark/test/test_map_quality_structure_aware.py`
- `src/agt_map_workbench/test/test_structure_aware_navigation_workbench.py`

---

### Task 1: Pure Structure-Aware Generated Map Materializer

**Files:**
- Create: `src/agt_offline_assets/agt_offline_assets/formal_navigation_map.py`
- Create: `src/agt_offline_assets/test/test_formal_navigation_map.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/__init__.py`
- Modify: `src/agt_offline_assets/CMakeLists.txt`

**Interfaces:**
- Consumes: `NavigationMapResult`, `CorridorRefinementResult`, `SiteBoundary`.
- Produces: `FORMAL_NAVIGATION_MATERIALIZATION_SCHEMA`, `StructureAwareNavigationResult`, `materialize_structure_aware_navigation_map(...)`.

- [ ] **Step 1: Write failing precedence and grid-preservation tests**

Create a compact 3x6 fixture where Ground-only occupancy includes all three states and where row/aisle/Site Boundary masks overlap deliberately:

```python
import numpy as np
from types import SimpleNamespace

from agt_offline_assets import FREE, OCCUPIED, UNKNOWN
from agt_offline_assets.formal_navigation_map import (
    materialize_structure_aware_navigation_map,
)


def test_materializer_only_promotes_unknown_inside_aisle():
    navigation = make_navigation(
        np.array([
            [FREE, UNKNOWN, UNKNOWN, OCCUPIED, UNKNOWN, FREE],
            [FREE, UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN, FREE],
            [FREE, FREE, FREE, FREE, FREE, FREE],
        ], dtype=np.uint8)
    )
    corridor = make_corridor(
        aisle=np.array([
            [False, True, True, True, False, False],
            [False, True, True, True, True, False],
            [False, False, False, False, False, False],
        ]),
        row_band=np.array([
            [False, False, True, False, False, False],
            [False, False, False, False, True, False],
            [False, False, False, False, False, False],
        ]),
    )
    result = materialize_structure_aware_navigation_map(
        navigation,
        corridor,
        full_site_boundary(navigation),
    )

    assert result.occupancy[0, 1] == FREE
    assert result.structure_inferred_free_mask[0, 1]
    assert result.occupancy[0, 2] == OCCUPIED  # row band wins
    assert result.occupancy[0, 3] == OCCUPIED  # base occupied is never freed
    assert result.occupancy[0, 4] == UNKNOWN   # outside aisle stays unknown
```

Also assert:

```python
assert result.navigation.resolution_m == navigation.resolution_m
assert result.navigation.origin_x_m == navigation.origin_x_m
assert result.navigation.origin_y_m == navigation.origin_y_m
assert result.navigation.occupancy.shape == navigation.occupancy.shape
```

- [ ] **Step 2: Run the focused test and verify it fails before implementation**

Run:

```bash
PYTHONPATH=src/agt_offline_assets \
python3 -m pytest -q src/agt_offline_assets/test/test_formal_navigation_map.py
```

Expected: import failure for `agt_offline_assets.formal_navigation_map`.

- [ ] **Step 3: Implement the result contract and deterministic precedence**

Use this public shape:

```python
FORMAL_NAVIGATION_MATERIALIZATION_SCHEMA = "agt_structure_aware_navigation_map/v1"

@dataclass(frozen=True)
class StructureAwareNavigationResult:
    navigation: NavigationMapResult
    observed_free_mask: np.ndarray
    structure_inferred_free_mask: np.ndarray
    base_hard_occupied_mask: np.ndarray
    row_structural_blocked_mask: np.ndarray
    site_boundary_blocked_mask: np.ndarray
    unresolved_unknown_mask: np.ndarray
    schema: str = FORMAL_NAVIGATION_MATERIALIZATION_SCHEMA

    def counts(self) -> dict[str, int]:
        return {
            "observed_free": int(np.count_nonzero(self.observed_free_mask)),
            "structure_inferred_free": int(np.count_nonzero(self.structure_inferred_free_mask)),
            "base_hard_occupied": int(np.count_nonzero(self.base_hard_occupied_mask)),
            "row_structural_blocked": int(np.count_nonzero(self.row_structural_blocked_mask)),
            "site_boundary_blocked": int(np.count_nonzero(self.site_boundary_blocked_mask)),
            "unresolved_unknown": int(np.count_nonzero(self.unresolved_unknown_mask)),
        }
```

Implement:

```python
def materialize_structure_aware_navigation_map(
    navigation: NavigationMapResult,
    corridor: CorridorRefinementResult,
    site_boundary: SiteBoundary,
    *,
    frame_id: str = "map",
) -> StructureAwareNavigationResult:
    site_boundary.validate(expected_frame_id=frame_id)
    shape = navigation.occupancy.shape
    aisle = np.asarray(corridor.aisle_geometric_envelope, dtype=bool)
    row_band = np.asarray(corridor.row_structural_band, dtype=bool)
    if aisle.shape != shape or row_band.shape != shape:
        raise ValueError("formal navigation evidence grid shape mismatch")

    inside = rasterize_site_boundary(site_boundary, navigation, expected_frame_id=frame_id)
    base_free = navigation.occupancy == FREE
    base_occupied = navigation.occupancy == OCCUPIED
    base_unknown = navigation.occupancy == UNKNOWN
    outside = ~inside
    inferred = base_unknown & aisle & ~row_band & inside

    occupancy = np.full(shape, UNKNOWN, dtype=np.uint8)
    occupancy[base_free] = FREE
    occupancy[inferred] = FREE
    occupancy[base_occupied | row_band | outside] = OCCUPIED

    generated = NavigationMapResult(**{**navigation.__dict__, "occupancy": occupancy})
    return StructureAwareNavigationResult(
        navigation=generated,
        observed_free_mask=base_free & ~row_band & inside,
        structure_inferred_free_mask=inferred,
        base_hard_occupied_mask=base_occupied,
        row_structural_blocked_mask=row_band,
        site_boundary_blocked_mask=outside,
        unresolved_unknown_mask=occupancy == UNKNOWN,
    )
```

Do not add a Ground-confidence requirement to `inferred`; the corridor geometry already supplies the structural gate, and the purpose of this layer is to repair missing Ground evidence without freeing base OCCUPIED.

- [ ] **Step 4: Add edge-case tests**

Add tests that assert:

```python
# outside Site Boundary is OCCUPIED even if Ground says FREE
assert result.occupancy[outside_cell] == OCCUPIED

# row band is OCCUPIED even if Ground says FREE
assert result.occupancy[row_cell] == OCCUPIED

# base OCCUPIED inside aisle stays OCCUPIED
assert result.occupancy[obstacle_cell] == OCCUPIED

# UNKNOWN inside non-aisle stays UNKNOWN
assert result.occupancy[unknown_non_aisle] == UNKNOWN
```

Also add failures for mismatched array shape and wrong Site Boundary frame.

- [ ] **Step 5: Export the API and register the pytest target**

Add the imports and `__all__` entries in `agt_offline_assets/__init__.py`, then add:

```cmake
ament_add_pytest_test(test_formal_navigation_map test/test_formal_navigation_map.py)
```

- [ ] **Step 6: Run focused and nearby regression tests**

Run:

```bash
PYTHONPATH=src/agt_offline_assets \
python3 -m pytest -q \
  src/agt_offline_assets/test/test_formal_navigation_map.py \
  src/agt_offline_assets/test/test_navigation_corridor.py \
  src/agt_offline_assets/test/test_traversability.py
```

Expected: PASS.

- [ ] **Step 7: Commit Task 1**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/formal_navigation_map.py \
  src/agt_offline_assets/agt_offline_assets/__init__.py \
  src/agt_offline_assets/test/test_formal_navigation_map.py \
  src/agt_offline_assets/CMakeLists.txt
git commit -m "feat(map): materialize structure-aware navigation grid"
```

---

### Task 2: Safe Formal Override Replay

**Files:**
- Create: `src/agt_offline_assets/agt_offline_assets/formal_navigation_override.py`
- Create: `src/agt_offline_assets/test/test_formal_navigation_override.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/__init__.py`
- Modify: `src/agt_offline_assets/CMakeLists.txt`

**Interfaces:**
- Consumes: Generated `NavigationMapResult`, `CorridorRefinementResult.row_structural_band`, `SiteBoundary`, normalized formal override mappings.
- Produces: `FormalOverrideReplayResult`, `replay_formal_navigation_overrides(...)`.

- [ ] **Step 1: Write failing FORCE_FREE safety tests**

Use three polygons: one valid aisle correction, one crossing Site Boundary, and one crossing a row structural band.

```python
def test_force_free_cannot_punch_through_site_boundary():
    with pytest.raises(ValueError, match="FORCE_FREE_SITE_BOUNDARY_CONFLICT"):
        replay_formal_navigation_overrides(
            generated,
            corridor,
            boundary,
            [force_free_polygon_crossing_boundary],
        )


def test_force_free_cannot_punch_through_row_band():
    with pytest.raises(ValueError, match="FORCE_FREE_ROW_STRUCTURAL_CONFLICT"):
        replay_formal_navigation_overrides(
            generated,
            corridor,
            boundary,
            [force_free_polygon_crossing_row],
        )
```

Also assert `force_occupied` remains allowed within Site Boundary and wins over FREE.

- [ ] **Step 2: Run the new tests and verify failure**

```bash
PYTHONPATH=src/agt_offline_assets \
python3 -m pytest -q src/agt_offline_assets/test/test_formal_navigation_override.py
```

Expected: import failure for the new module.

- [ ] **Step 3: Implement ordered replay using existing world-coordinate rasterization**

Define:

```python
@dataclass(frozen=True)
class FormalOverrideReplayResult:
    navigation: NavigationMapResult
    force_free_changed_cell_count: int
    force_occupied_changed_cell_count: int
    force_free_area_m2: float
    force_occupied_area_m2: float
```

Implement one-record-at-a-time ordered replay so change counts reflect actual mutations:

```python
def replay_formal_navigation_overrides(
    generated: NavigationMapResult,
    corridor: CorridorRefinementResult,
    site_boundary: SiteBoundary,
    overrides: Iterable[Mapping[str, object]],
    *,
    frame_id: str = "map",
) -> FormalOverrideReplayResult:
    site_boundary.validate(expected_frame_id=frame_id)
    inside = rasterize_site_boundary(site_boundary, generated, expected_frame_id=frame_id)
    row_band = np.asarray(corridor.row_structural_band, dtype=bool)
    current = generated
    free_changed = 0
    occupied_changed = 0

    for record in overrides:
        mode = str(record.get("mode", "")).strip().lower()
        if mode not in {"force_free", "force_occupied"}:
            raise ValueError("formal override mode must be force_free or force_occupied")
        candidate = apply_navigation_overrides(current, [record])
        changed = candidate != current.occupancy
        if mode == "force_free":
            if np.any(changed & ~inside):
                raise ValueError("FORCE_FREE_SITE_BOUNDARY_CONFLICT")
            if np.any(changed & row_band):
                raise ValueError("FORCE_FREE_ROW_STRUCTURAL_CONFLICT")
            free_changed += int(np.count_nonzero(changed))
        else:
            occupied_changed += int(np.count_nonzero(changed))
        current = NavigationMapResult(**{**current.__dict__, "occupancy": candidate})
```

Compute areas from `resolution_m ** 2` and return the final Accepted result.

- [ ] **Step 4: Add deterministic-order and no-op tests**

Verify:

```python
assert first.navigation.occupancy.tobytes() == second.navigation.occupancy.tobytes()
assert no_op.force_free_changed_cell_count == 0
```

Also test that an unsupported mode fails before mutation.

- [ ] **Step 5: Export/register and run regression**

Run:

```bash
PYTHONPATH=src/agt_offline_assets \
python3 -m pytest -q \
  src/agt_offline_assets/test/test_formal_navigation_override.py \
  src/agt_offline_assets/test/test_navigation_map_freeze_export.py
```

Expected: PASS; historical freeze tests remain unchanged.

- [ ] **Step 6: Commit Task 2**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/formal_navigation_override.py \
  src/agt_offline_assets/agt_offline_assets/__init__.py \
  src/agt_offline_assets/test/test_formal_navigation_override.py \
  src/agt_offline_assets/CMakeLists.txt
git commit -m "feat(map): enforce safe formal override replay"
```

---

### Task 3: Evidence / Generated / Accepted Revision Payload Writer

**Files:**
- Create: `src/agt_offline_assets/agt_offline_assets/formal_navigation_revision.py`
- Create: `src/agt_offline_assets/test/test_formal_navigation_revision.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/__init__.py`
- Modify: `src/agt_offline_assets/CMakeLists.txt`

**Interfaces:**
- Consumes: Ground Evidence `NavigationMapResult`, `StructureAwareNavigationResult`, `FormalOverrideReplayResult`, overrides, source identity.
- Produces: `FORMAL_NAVIGATION_REVISION_SCHEMA`, `write_structure_aware_navigation_payload(...)`, `export_structure_aware_navigation_revision(...)`.
- Later Plan 2 will call `write_structure_aware_navigation_payload()` inside a larger unified staging transaction before semantic files are written.

- [ ] **Step 1: Write the failing directory-contract test**

```python
def test_payload_writes_evidence_generated_and_accepted(tmp_path):
    root = tmp_path / "revision_stage"
    root.mkdir()
    write_structure_aware_navigation_payload(
        root,
        ground_evidence=ground,
        materialized=generated,
        accepted=accepted,
        overrides=[override],
        source_asset="processed.pcd",
        frame_id="map",
    )

    expected = {
        "evidence/ground_only_navigation_map.pgm",
        "evidence/ground_only_navigation_map.yaml",
        "evidence/ground_height.npy",
        "evidence/slope_deg.npy",
        "evidence/step_m.npy",
        "evidence/obstacle_count.npy",
        "evidence/ground_support_count.npy",
        "generated/navigation_map.pgm",
        "generated/navigation_map.yaml",
        "accepted/navigation_map.pgm",
        "accepted/navigation_map.yaml",
        "derivation.yaml",
    }
    assert expected.issubset(
        {path.relative_to(root).as_posix() for path in root.rglob("*") if path.is_file()}
    )
```

- [ ] **Step 2: Run and verify failure**

```bash
PYTHONPATH=src/agt_offline_assets \
python3 -m pytest -q src/agt_offline_assets/test/test_formal_navigation_revision.py
```

Expected: import failure.

- [ ] **Step 3: Implement the payload writer by reusing the existing Nav2 serializer**

Do not duplicate PGM/YAML encoding. Import and call `write_navigation_map_files` for Evidence, Generated, and Accepted.

Use a derivation record compatible with the restored authority binder while distinguishing the new pipeline:

```python
FORMAL_NAVIGATION_REVISION_SCHEMA = "agt_structure_aware_navigation_revision/v1"

record = {
    "schema": "agt_ground_relative_navigation_map/v1",
    "revision_kind": "structure_aware_generated_plus_accepted_override_revision",
    "materialization_schema": FORMAL_NAVIGATION_MATERIALIZATION_SCHEMA,
    "formal_revision_schema": FORMAL_NAVIGATION_REVISION_SCHEMA,
    "frame_id": frame_id,
    "source_asset": source_asset,
    "grid": {...},
    "counts": {
        "evidence": ground_evidence.counts(),
        "generated": materialized.navigation.counts(),
        "accepted": accepted.navigation.counts(),
        "materialization": materialized.counts(),
    },
    "manual_override_metrics": {...},
    "overrides": list(overrides),
    "outputs": {...sha256 records...},
}
```

Preserve top-level `schema = agt_ground_relative_navigation_map/v1` because historical map-quality tooling already recognizes it; use `formal_revision_schema` to identify the extended contract.

- [ ] **Step 4: Implement an atomic navigation-only export helper**

`export_structure_aware_navigation_revision(destination, ...)` must create a hidden sibling staging directory, call `write_structure_aware_navigation_payload()`, and publish with `os.replace()` only after all writers and hashes succeed. Existing destination must raise `FileExistsError`.

- [ ] **Step 5: Add byte-determinism and rollback tests**

Verify two independent payload directories have byte-identical deterministic files except no timestamp is written in `derivation.yaml`. Inject a writer failure through a private test hook or monkeypatch `write_navigation_map_files` and assert the final destination does not exist.

- [ ] **Step 6: Export/register and run tests**

```bash
PYTHONPATH=src/agt_offline_assets \
python3 -m pytest -q \
  src/agt_offline_assets/test/test_formal_navigation_revision.py \
  src/agt_offline_assets/test/test_navigation_map_freeze_export.py
```

Expected: PASS.

- [ ] **Step 7: Commit Task 3**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/formal_navigation_revision.py \
  src/agt_offline_assets/agt_offline_assets/__init__.py \
  src/agt_offline_assets/test/test_formal_navigation_revision.py \
  src/agt_offline_assets/CMakeLists.txt
git commit -m "feat(map): freeze structure-aware navigation revisions"
```

---

### Task 4: Planner-Independent Formal Navigation QA

**Files:**
- Create: `src/agt_offline_assets/agt_offline_assets/formal_navigation_qa.py`
- Create: `src/agt_offline_assets/test/test_formal_navigation_qa.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/__init__.py`
- Modify: `src/agt_offline_assets/CMakeLists.txt`

**Interfaces:**
- Consumes: Generated/Accepted navigation results, materialization masks, corridor, structure direction, Site Boundary, replay result.
- Produces: `FORMAL_NAVIGATION_QA_SCHEMA`, `evaluate_formal_navigation_qa(...) -> dict[str, object]`, `write_formal_navigation_qa(...) -> Path`.

- [ ] **Step 1: Write failing hard-invariant tests**

Create fixtures that deliberately leak one FREE cell outside Site Boundary and one FREE cell into the row band:

```python
def test_qa_fails_free_outside_site_boundary():
    report = evaluate_formal_navigation_qa(...)
    assert report["outside_site_boundary_free_count"] == 1
    assert report["status"] == "FAIL"
    assert "OUTSIDE_SITE_BOUNDARY_FREE" in report["hard_failures"]


def test_qa_fails_row_band_free_leak():
    report = evaluate_formal_navigation_qa(...)
    assert report["row_structural_band_free_leak_count"] == 1
    assert "ROW_STRUCTURAL_FREE_LEAK" in report["hard_failures"]
```

- [ ] **Step 2: Write aisle statistics/connectivity tests**

For a synthetic straight aisle, assert:

```python
assert aisle["free_fraction"] == pytest.approx(0.8)
assert aisle["unknown_fraction"] == pytest.approx(0.2)
assert aisle["occupied_conflict_fraction"] == pytest.approx(0.0)
assert aisle["grid_connected"] is True
```

Break the center with an OCCUPIED cell and assert `grid_connected is False`.

Connectivity must be 8-connected and restricted to FREE cells inside the owned aisle geometry.

- [ ] **Step 3: Run and verify failure**

```bash
PYTHONPATH=src/agt_offline_assets \
python3 -m pytest -q src/agt_offline_assets/test/test_formal_navigation_qa.py
```

Expected: import failure.

- [ ] **Step 4: Implement QA metrics and hard-failure status**

The report must include exactly these top-level metrics:

```python
{
    "schema": "agt_formal_navigation_qa/v1",
    "status": "PASS" | "FAIL",
    "hard_failures": [...],
    "outside_site_boundary_free_count": int,
    "row_structural_band_free_leak_count": int,
    "accepted_aisle_count": int,
    "aisles": [...],
    "map_unknown_fraction": float,
    "structure_inferred_free_fraction": float,
    "manual_force_free_area_m2": float,
    "manual_force_occupied_area_m2": float,
    "accepted_matches_replay": bool,
}
```

Build per-aisle ownership masks from each accepted `AislePairDiagnostic` using the same row-direction coordinate system used by `navigation_corridor.py`. Do not use planner paths as QA truth.

- [ ] **Step 5: Implement deterministic JSON serialization**

`write_formal_navigation_qa(report, path)` writes `json.dumps(..., indent=2, sort_keys=True) + "\n"` and creates the parent directory.

- [ ] **Step 6: Export/register and run focused tests**

```bash
PYTHONPATH=src/agt_offline_assets \
python3 -m pytest -q \
  src/agt_offline_assets/test/test_formal_navigation_qa.py \
  src/agt_offline_assets/test/test_formal_navigation_map.py \
  src/agt_offline_assets/test/test_formal_navigation_override.py
```

Expected: PASS.

- [ ] **Step 7: Commit Task 4**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/formal_navigation_qa.py \
  src/agt_offline_assets/agt_offline_assets/__init__.py \
  src/agt_offline_assets/test/test_formal_navigation_qa.py \
  src/agt_offline_assets/CMakeLists.txt
git commit -m "feat(map): add planner-independent formal map QA"
```

---

### Task 5: Preserve Historical Map-Quality Replay While Accepting the New Revision Kind

**Files:**
- Modify: `src/agt_route_benchmark/agt_route_benchmark/map_quality.py`
- Create: `src/agt_route_benchmark/test/test_map_quality_structure_aware.py`

**Interfaces:**
- Consumes: historical `generated_plus_accepted_override_revision` and new `structure_aware_generated_plus_accepted_override_revision` derivations.
- Produces: unchanged public `audit_map_revision(...)` / `write_map_quality_evidence(...)` behavior for both revision kinds.

- [ ] **Step 1: Write a failing new-revision replay test**

Serialize a tiny new-format revision through Task 3, then:

```python
report = audit_map_revision(
    revision / "generated/navigation_map.yaml",
    revision / "accepted/navigation_map.yaml",
    revision / "derivation.yaml",
)
assert report["accepted_matches_replay"] is True
assert report["unexplained_changed_cell_count"] == 0
```

- [ ] **Step 2: Run and verify the current strict revision-kind check fails**

```bash
MPLBACKEND=Agg \
PYTHONPATH=src/agt_route_benchmark:src/agt_offline_assets \
python3 -m pytest -q src/agt_route_benchmark/test/test_map_quality_structure_aware.py
```

Expected: failure saying the derivation is not a generated/accepted freeze revision.

- [ ] **Step 3: Broaden only the allowed revision-kind set**

Replace the single-value comparison with:

```python
_ALLOWED_REVISION_KINDS = {
    "generated_plus_accepted_override_revision",
    "structure_aware_generated_plus_accepted_override_revision",
}
```

Do not weaken schema, frame, metadata, or replay checks.

- [ ] **Step 4: Run historical and new map-quality tests**

```bash
MPLBACKEND=Agg \
PYTHONPATH=src/agt_route_benchmark:src/agt_offline_assets \
python3 -m pytest -q \
  src/agt_route_benchmark/test/test_map_quality.py \
  src/agt_route_benchmark/test/test_map_quality_cli.py \
  src/agt_route_benchmark/test/test_map_quality_structure_aware.py
```

Expected: PASS.

- [ ] **Step 5: Commit Task 5**

```bash
git add \
  src/agt_route_benchmark/agt_route_benchmark/map_quality.py \
  src/agt_route_benchmark/test/test_map_quality_structure_aware.py
git commit -m "fix(benchmark): audit structure-aware map revisions"
```

---

### Task 6: Wire Generated PGM Preview and Formal Export into Paper I Workbench

**Files:**
- Modify: `src/agt_map_workbench/agt_map_workbench/navigation_preview.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/paper1_workbench.py`
- Create: `src/agt_map_workbench/test/test_structure_aware_navigation_workbench.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 1 materializer, Task 2 replay, Task 3 payload writer, Task 4 QA.
- Produces Workbench state: `_formal_materialization`, `_formal_accepted_result`, `_formal_navigation_qa`; explicit `生成 Structure-Aware 正式地图预览` action; formal export no longer starts from `_navigation_base_result` alone.

- [ ] **Step 1: Write a failing Workbench-state test**

Instantiate `Paper1MapWorkbenchWindow` offscreen and inject minimal synthetic navigation/structure/corridor/site-boundary state. Call the new pure-ish method `_build_formal_navigation_state()` and assert:

```python
assert window._formal_materialization is not None
assert window._formal_accepted_result is not None
assert window._formal_navigation_qa["status"] == "PASS"
assert window._formal_materialization.navigation.occupancy[aisle_unknown_cell] == FREE
```

Also assert missing Site Boundary or missing corridor produces a user-facing reason and no formal state.

- [ ] **Step 2: Run and verify failure**

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_offline_assets:src/agt_ui_bridge:src/agt_coverage_planning \
python3 -m pytest -q src/agt_map_workbench/test/test_structure_aware_navigation_workbench.py
```

Expected: missing `_build_formal_navigation_state`.

- [ ] **Step 3: Add explicit formal-state members and invalidation rules**

In `Paper1MapWorkbenchWindow.__init__` initialize:

```python
self._formal_materialization = None
self._formal_accepted_result = None
self._formal_navigation_qa = None
self._formal_last_error = ""
```

Clear them whenever Ground map, structure/corridor, Site Boundary, or formal overrides change. Do not mutate `_navigation_base_result`, `_navigation_result`, or the V25-12F candidate state.

- [ ] **Step 4: Add the formal map build method**

The method must perform this exact order:

```python
materialized = materialize_structure_aware_navigation_map(
    self._navigation_base_result,
    self._corridor_refinement_result,
    self._site_boundary,
)
overrides = self._validated_formal_overrides()
replay = replay_formal_navigation_overrides(
    materialized.navigation,
    self._corridor_refinement_result,
    self._site_boundary,
    overrides,
)
qa = evaluate_formal_navigation_qa(
    ground_evidence=self._navigation_base_result,
    materialized=materialized,
    accepted=replay,
    structure=self._navigation_structure_result,
    corridor=self._corridor_refinement_result,
    site_boundary=self._site_boundary,
)
```

Store all three only after every call succeeds.

- [ ] **Step 5: Add Generated preview layers without replacing Ground evidence**

Extend `navigation_preview.py` with mask rendering labels that accept Task 1 masks. Add Workbench combo entries for:

```text
正式 Generated PGM
Structure-Inferred FREE
Row Structural Block
Site Boundary Block
Unresolved UNKNOWN
```

Use `NavigationPreviewItem.set_result(materialized.navigation, "final")` for the Generated raster and `set_mask()` for diagnostic masks.

- [ ] **Step 6: Change `_export_navigation_map()` to export the structure-aware payload**

Before opening the destination dialog, build formal state and reject export if QA status is FAIL. Replace the call to historical `write_navigation_map_freeze_bundle(self._navigation_base_result, ...)` with `export_structure_aware_navigation_revision(...)` using Ground Evidence, materialization, accepted replay and the validated overrides.

Keep the existing immutable revision name behavior and the user message that planner-independent QA is required.

- [ ] **Step 7: Register and run Workbench regression tests**

Add:

```cmake
ament_add_pytest_test(
  test_structure_aware_navigation_workbench
  test/test_structure_aware_navigation_workbench.py
)
```

Run:

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_offline_assets:src/agt_ui_bridge:src/agt_coverage_planning \
python3 -m pytest -q \
  src/agt_map_workbench/test/test_navigation_override_metadata.py \
  src/agt_map_workbench/test/test_site_boundary_workbench.py \
  src/agt_map_workbench/test/test_structure_aware_navigation_workbench.py
```

Expected: PASS.

- [ ] **Step 8: Commit Task 6**

```bash
git add \
  src/agt_map_workbench/agt_map_workbench/navigation_preview.py \
  src/agt_map_workbench/agt_map_workbench/paper1_workbench.py \
  src/agt_map_workbench/test/test_structure_aware_navigation_workbench.py \
  src/agt_map_workbench/CMakeLists.txt
git commit -m "feat(workbench): preview and freeze structure-aware PGM"
```

---

### Task 7: Full Software Gate and Real-Map Acceptance Checkpoint

**Files:**
- Modify only if a failure identifies a concrete defect in Tasks 1-6.
- Evidence output on target machine: `runtime/maps/greenhouse_01/derivation/<new_revision>/...`

**Interfaces:**
- Consumes all Task 1-6 interfaces.
- Produces a verified checkpoint that Plan 2 may build on; this task does not start semantic-GUI work until the map core is green.

- [ ] **Step 1: Run offline-assets tests**

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=src/agt_offline_assets \
python3 -m pytest -q src/agt_offline_assets/test
```

Expected: PASS.

- [ ] **Step 2: Run map-quality tests**

```bash
MPLBACKEND=Agg \
PYTHONPATH=src/agt_route_benchmark:src/agt_offline_assets:src/agt_coverage_planning:src/agt_ui_bridge \
python3 -m pytest -q \
  src/agt_route_benchmark/test/test_map_quality.py \
  src/agt_route_benchmark/test/test_map_quality_cli.py \
  src/agt_route_benchmark/test/test_map_quality_structure_aware.py
```

Expected: PASS.

- [ ] **Step 3: Run Workbench tests**

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_offline_assets:src/agt_ui_bridge:src/agt_coverage_planning \
python3 -m pytest -q src/agt_map_workbench/test
```

Expected: PASS.

- [ ] **Step 4: Run syntax gate**

```bash
python3 -m compileall -q \
  src/agt_offline_assets/agt_offline_assets \
  src/agt_map_workbench/agt_map_workbench \
  src/agt_route_benchmark/agt_route_benchmark
```

Expected: exit code 0.

- [ ] **Step 5: Launch the Paper I Workbench with the authoritative processed/canonical greenhouse PCD**

```bash
source install/setup.bash
ROS_LOG_DIR="$PWD/runtime/log/unified_map_authoring" \
ros2 run agt_map_workbench agt_map_workbench_paper1
```

Manual acceptance criteria:

```text
[ ] Ground-only Evidence preview remains available.
[ ] Row structural band visually aligns with crop rows.
[ ] Aisle geometric envelope visually aligns with actual inter-row corridors.
[ ] Structure-Inferred FREE fills Ground-only UNKNOWN inside valid aisles.
[ ] No base OCCUPIED cell is automatically whitened.
[ ] No row structural cell is FREE.
[ ] No Site Boundary exterior cell is FREE.
[ ] Headland UNKNOWN remains UNKNOWN unless directly Ground-FREE or formally overridden.
[ ] Formal QA reports per-aisle FREE/UNKNOWN/conflict/connectivity values.
[ ] Export creates evidence/, generated/, accepted/, derivation.yaml.
```

- [ ] **Step 6: Run planner-independent replay audit against the exported revision**

```bash
ros2 run agt_route_benchmark route_benchmark_map_quality.py \
  --generated-map-yaml "$REV/generated/navigation_map.yaml" \
  --accepted-map-yaml "$REV/accepted/navigation_map.yaml" \
  --derivation-yaml "$REV/derivation.yaml" \
  --output-dir "$REV/validation/replay_qa"
```

Required:

```text
accepted_matches_replay == true
unexplained_changed_cell_count == 0
```

- [ ] **Step 7: Record the checkpoint commit**

If no fixes were needed, do not create an empty commit. Record the verified head SHA in the implementation handoff. If fixes were required, commit only the concrete fixes with a focused message before proceeding to Plan 2.

---

## Plan 1 Completion Gate

Do not start semantic-GUI integration until all of the following are true:

```text
Structure-aware unit tests PASS
Formal override safety tests PASS
Revision serialization tests PASS
Formal QA tests PASS
Historical map-quality regression PASS
Workbench formal-map integration PASS
compileall PASS
real greenhouse Generated PGM visually reviewed
accepted replay QA PASS
```

At that point the repository has one working automatic PGM production chain even before the legacy semantic editor is retired.