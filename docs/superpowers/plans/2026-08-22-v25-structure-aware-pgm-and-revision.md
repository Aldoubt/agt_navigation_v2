# V25 Structure-Aware PGM and Revision Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn Ground-only evidence plus validated greenhouse row/aisle structure into one deterministic formal Nav2 Generated PGM, replay validated human overrides into Accepted, and freeze a planner-independent auditable map revision with QA.

**Architecture:** Keep `NavigationMapResult` as the common grid/evidence carrier. Add a pure offline structure-aware materializer that may promote only Ground-only `UNKNOWN` cells inside valid aisle geometry, while preserving Site Boundary, base `OCCUPIED`, and row structural bands as hard blockers. Keep Generated materialization, Accepted replay, revision serialization, and QA in separate modules so the Workbench only orchestrates them.

**Tech Stack:** Python 3, NumPy, SciPy, PyYAML, ROS2 Humble/ament_cmake_python, PyQt5, pytest.

**Spec:** `docs/superpowers/specs/2026-08-22-v25-unified-map-authoring-structure-aware-pgm-design.md`

## Global Constraints

- One PCD lineage -> one AGT Map Workbench authoring workflow -> one immutable Map Revision -> one accepted Navigation Map authority -> one semantic task bound by SHA256 to that accepted map.
- `agt_map_pipeline` remains a consumer/verifier/evidence producer and must not create a competing formal Navigation Map.
- Only Ground-only `UNKNOWN` cells may be promoted automatically by agricultural structure.
- Ground-only `OCCUPIED` cells must never be automatically promoted to FREE.
- Cells outside Site Boundary are formal OCCUPIED.
- Cells inside `row_structural_band` are formal OCCUPIED.
- No global `UNKNOWN -> FREE`, map-wide flood fill, cross-row morphology, route-dependent PGM whitening, or new automatic headland detector.
- Paper I formal manual raster modes remain `FORCE_FREE` and `FORCE_OCCUPIED`.
- `FORCE_FREE` is rejected outside Site Boundary and inside `row_structural_band`.
- Ground-only output is Evidence; automatic structure-aware output is Generated; Generated plus validated formal overrides is Accepted.
- Existing M1.5-4B paths `generated/navigation_map.*`, `accepted/navigation_map.*`, and `derivation.yaml` remain present.
- Formal freeze is fail-closed and deterministic.

---

## File Structure

**Create**
- `src/agt_offline_assets/agt_offline_assets/formal_navigation_map.py` — Generated-map materialization.
- `src/agt_offline_assets/agt_offline_assets/formal_navigation_override.py` — safe formal override replay.
- `src/agt_offline_assets/agt_offline_assets/formal_navigation_revision.py` — Evidence/Generated/Accepted serialization and atomic navigation-only export helper.
- `src/agt_offline_assets/agt_offline_assets/formal_navigation_qa.py` — planner-independent QA.
- `src/agt_offline_assets/test/test_formal_navigation_map.py`
- `src/agt_offline_assets/test/test_formal_navigation_override.py`
- `src/agt_offline_assets/test/test_formal_navigation_revision.py`
- `src/agt_offline_assets/test/test_formal_navigation_qa.py`
- `src/agt_route_benchmark/test/test_map_quality_structure_aware.py`
- `src/agt_map_workbench/test/test_structure_aware_navigation_workbench.py`

**Modify**
- `src/agt_offline_assets/agt_offline_assets/__init__.py`
- `src/agt_offline_assets/CMakeLists.txt`
- `src/agt_route_benchmark/agt_route_benchmark/map_quality.py`
- `src/agt_map_workbench/agt_map_workbench/navigation_preview.py`
- `src/agt_map_workbench/agt_map_workbench/paper1_workbench.py`
- `src/agt_map_workbench/CMakeLists.txt`

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

- [ ] **Step 1: Write the failing precedence fixture and tests**

Put these local helpers in `test_formal_navigation_map.py` so the test is self-contained:

```python
from types import SimpleNamespace
import numpy as np
from agt_offline_assets import (
    FREE, OCCUPIED, UNKNOWN,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    SiteBoundary,
)


def make_navigation(occupancy):
    occupancy = np.asarray(occupancy, dtype=np.uint8)
    h, w = occupancy.shape
    zeros_f = np.zeros((h, w), dtype=np.float64)
    zeros_i = np.zeros((h, w), dtype=np.int32)
    return NavigationMapResult(
        resolution_m=1.0,
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=w,
        height=h,
        ground_height_m=zeros_f.copy(),
        ground_valid=np.ones((h, w), dtype=bool),
        point_count=np.ones((h, w), dtype=np.int32),
        ground_support_count=np.ones((h, w), dtype=np.int32),
        obstacle_count=zeros_i.copy(),
        slope_deg=zeros_f.copy(),
        step_m=zeros_f.copy(),
        occupancy=occupancy,
        config=GroundRelativeNavigationConfig(resolution_m=1.0),
    )


def make_corridor(aisle, row_band):
    return SimpleNamespace(
        aisle_geometric_envelope=np.asarray(aisle, dtype=bool),
        row_structural_band=np.asarray(row_band, dtype=bool),
    )


def full_boundary(navigation):
    return SiteBoundary(
        frame_id="map",
        outer_boundary_xy=(
            (0.0, 0.0),
            (float(navigation.width), 0.0),
            (float(navigation.width), float(navigation.height)),
            (0.0, float(navigation.height)),
        ),
    )
```

Then test exact precedence:

```python
def test_materializer_only_promotes_unknown_inside_aisle():
    navigation = make_navigation([
        [FREE, UNKNOWN, UNKNOWN, OCCUPIED, UNKNOWN, FREE],
        [FREE, UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN, FREE],
        [FREE, FREE, FREE, FREE, FREE, FREE],
    ])
    corridor = make_corridor(
        aisle=[
            [False, True, True, True, False, False],
            [False, True, True, True, True, False],
            [False, False, False, False, False, False],
        ],
        row_band=[
            [False, False, True, False, False, False],
            [False, False, False, False, True, False],
            [False, False, False, False, False, False],
        ],
    )
    result = materialize_structure_aware_navigation_map(
        navigation, corridor, full_boundary(navigation)
    )
    occupancy = result.navigation.occupancy
    assert occupancy[0, 1] == FREE
    assert result.structure_inferred_free_mask[0, 1]
    assert occupancy[0, 2] == OCCUPIED
    assert occupancy[0, 3] == OCCUPIED
    assert occupancy[0, 4] == UNKNOWN
    assert result.navigation.resolution_m == navigation.resolution_m
    assert result.navigation.origin_x_m == navigation.origin_x_m
    assert result.navigation.origin_y_m == navigation.origin_y_m
```

- [ ] **Step 2: Run the test and verify import failure**

```bash
PYTHONPATH=src/agt_offline_assets \
python3 -m pytest -q src/agt_offline_assets/test/test_formal_navigation_map.py
```

Expected: module import failure.

- [ ] **Step 3: Implement the materialization contract**

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

Implement precedence exactly as:

```python
def materialize_structure_aware_navigation_map(
    navigation, corridor, site_boundary, *, frame_id="map"
):
    site_boundary.validate(expected_frame_id=frame_id)
    shape = navigation.occupancy.shape
    aisle = np.asarray(corridor.aisle_geometric_envelope, dtype=bool)
    row_band = np.asarray(corridor.row_structural_band, dtype=bool)
    if aisle.shape != shape or row_band.shape != shape:
        raise ValueError("formal navigation evidence grid shape mismatch")

    inside = rasterize_site_boundary(
        site_boundary, navigation, expected_frame_id=frame_id
    )
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
```

Return all masks in `StructureAwareNavigationResult`. Do not add Ground-confidence gating to `inferred`.

- [ ] **Step 4: Add hard-boundary edge tests**

Add explicit tests for outside-boundary FREE becoming OCCUPIED, row-band Ground FREE becoming OCCUPIED, base OCCUPIED staying OCCUPIED, non-aisle UNKNOWN staying UNKNOWN, mismatched array shapes, and wrong Site Boundary frame.

- [ ] **Step 5: Export API and register test**

Add imports/`__all__` entries and:

```cmake
ament_add_pytest_test(test_formal_navigation_map test/test_formal_navigation_map.py)
```

- [ ] **Step 6: Run focused regression**

```bash
PYTHONPATH=src/agt_offline_assets \
python3 -m pytest -q \
  src/agt_offline_assets/test/test_formal_navigation_map.py \
  src/agt_offline_assets/test/test_navigation_corridor.py \
  src/agt_offline_assets/test/test_traversability.py
```

Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add src/agt_offline_assets/agt_offline_assets/formal_navigation_map.py \
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
- Consumes: Generated `NavigationMapResult`, `CorridorRefinementResult`, `SiteBoundary`, normalized override mappings.
- Produces: `FormalOverrideReplayResult`, `replay_formal_navigation_overrides(...)`.

- [ ] **Step 1: Write failing FORCE_FREE safety tests**

Reuse Task 1 fixture helpers by copying them into this test file. Create one valid in-boundary polygon, one polygon crossing the boundary, and one crossing `row_structural_band`. Assert boundary/row conflicts raise these exact codes:

```python
with pytest.raises(ValueError, match="FORCE_FREE_SITE_BOUNDARY_CONFLICT"):
    replay_formal_navigation_overrides(...)

with pytest.raises(ValueError, match="FORCE_FREE_ROW_STRUCTURAL_CONFLICT"):
    replay_formal_navigation_overrides(...)
```

Also assert `force_occupied` may change FREE to OCCUPIED.

- [ ] **Step 2: Run and verify import failure**

```bash
PYTHONPATH=src/agt_offline_assets \
python3 -m pytest -q src/agt_offline_assets/test/test_formal_navigation_override.py
```

- [ ] **Step 3: Implement ordered replay**

```python
@dataclass(frozen=True)
class FormalOverrideReplayResult:
    navigation: NavigationMapResult
    force_free_changed_cell_count: int
    force_occupied_changed_cell_count: int
    force_free_area_m2: float
    force_occupied_area_m2: float
```

For each record call existing `apply_navigation_overrides(current, [record])`; compare the candidate array to `current.occupancy`; reject `force_free` changes intersecting `~inside_boundary` or `row_structural_band`; then create the next immutable `NavigationMapResult`. Reject any mode outside `{force_free, force_occupied}` before mutation.

- [ ] **Step 4: Add deterministic-order/no-op tests**

```python
assert first.navigation.occupancy.tobytes() == second.navigation.occupancy.tobytes()
assert no_op.force_free_changed_cell_count == 0
```

- [ ] **Step 5: Register/run regression**

```bash
PYTHONPATH=src/agt_offline_assets \
python3 -m pytest -q \
  src/agt_offline_assets/test/test_formal_navigation_override.py \
  src/agt_offline_assets/test/test_navigation_map_freeze_export.py
```

- [ ] **Step 6: Commit**

```bash
git add src/agt_offline_assets/agt_offline_assets/formal_navigation_override.py \
  src/agt_offline_assets/agt_offline_assets/__init__.py \
  src/agt_offline_assets/test/test_formal_navigation_override.py \
  src/agt_offline_assets/CMakeLists.txt
git commit -m "feat(map): enforce safe formal override replay"
```

---

### Task 3: Evidence / Generated / Accepted Revision Payload

**Files:**
- Create: `src/agt_offline_assets/agt_offline_assets/formal_navigation_revision.py`
- Create: `src/agt_offline_assets/test/test_formal_navigation_revision.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/__init__.py`
- Modify: `src/agt_offline_assets/CMakeLists.txt`

**Interfaces:**
- Consumes: Ground Evidence `NavigationMapResult`, `StructureAwareNavigationResult`, `FormalOverrideReplayResult`, ordered overrides, source identity.
- Produces: `FORMAL_NAVIGATION_REVISION_SCHEMA`, `write_structure_aware_navigation_payload(...)`, `export_structure_aware_navigation_revision(...)`.
- Plan 2 will call the payload writer inside a larger unified staging transaction.

- [ ] **Step 1: Write the failing file-contract test**

Create a Task 1 generated fixture and Task 2 replay fixture, create `root`, call the new payload writer, and assert these files exist:

```text
evidence/ground_only_navigation_map.pgm
evidence/ground_only_navigation_map.yaml
evidence/ground_height.npy
evidence/slope_deg.npy
evidence/step_m.npy
evidence/obstacle_count.npy
evidence/ground_support_count.npy
evidence/formal_materialization_masks.npz
generated/navigation_map.pgm
generated/navigation_map.yaml
accepted/navigation_map.pgm
accepted/navigation_map.yaml
derivation.yaml
```

- [ ] **Step 2: Run and verify import failure**

```bash
PYTHONPATH=src/agt_offline_assets \
python3 -m pytest -q src/agt_offline_assets/test/test_formal_navigation_revision.py
```

- [ ] **Step 3: Implement payload serialization with existing Nav2 writer**

Reuse `write_navigation_map_files` for all three raster products. Save materialization masks with exact keys:

```python
np.savez_compressed(
    root / "evidence/formal_materialization_masks.npz",
    observed_free_mask=materialized.observed_free_mask.astype(np.uint8),
    structure_inferred_free_mask=materialized.structure_inferred_free_mask.astype(np.uint8),
    base_hard_occupied_mask=materialized.base_hard_occupied_mask.astype(np.uint8),
    row_structural_blocked_mask=materialized.row_structural_blocked_mask.astype(np.uint8),
    site_boundary_blocked_mask=materialized.site_boundary_blocked_mask.astype(np.uint8),
    unresolved_unknown_mask=materialized.unresolved_unknown_mask.astype(np.uint8),
)
```

Use:

```python
FORMAL_NAVIGATION_REVISION_SCHEMA = "agt_structure_aware_navigation_revision/v1"
```

Keep `derivation.yaml` top-level schema compatible with existing audit tooling:

```yaml
schema: agt_ground_relative_navigation_map/v1
revision_kind: structure_aware_generated_plus_accepted_override_revision
formal_revision_schema: agt_structure_aware_navigation_revision/v1
materialization_schema: agt_structure_aware_navigation_map/v1
```

Record grid, Evidence/Generated/Accepted counts, materialization counts, override metrics, ordered override records, and SHA256 identities for every formal raster plus the mask NPZ.

- [ ] **Step 4: Implement atomic navigation-only export helper**

`export_structure_aware_navigation_revision(destination, ...)` creates a hidden sibling staging directory, calls the payload writer, and publishes with `os.replace()` only after successful writes/hashes. Existing destination raises `FileExistsError`.

- [ ] **Step 5: Add determinism/rollback tests**

Two payload roots with identical inputs must have byte-identical authority files. Monkeypatch a serializer to raise and assert the final destination and staging directory are absent.

- [ ] **Step 6: Register/run tests**

```bash
PYTHONPATH=src/agt_offline_assets \
python3 -m pytest -q \
  src/agt_offline_assets/test/test_formal_navigation_revision.py \
  src/agt_offline_assets/test/test_navigation_map_freeze_export.py
```

- [ ] **Step 7: Commit**

```bash
git add src/agt_offline_assets/agt_offline_assets/formal_navigation_revision.py \
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
- Consumes: Ground Evidence, `StructureAwareNavigationResult`, `FormalOverrideReplayResult`, ordered overrides, `NavigationStructureResult`, `CorridorRefinementResult`, `SiteBoundary`.
- Produces: `FORMAL_NAVIGATION_QA_SCHEMA`, `evaluate_formal_navigation_qa(...)`, `write_formal_navigation_qa(...)`.

- [ ] **Step 1: Write hard-invariant tests**

Create a valid synthetic fixture, then copy Accepted occupancy and inject one FREE cell outside Site Boundary and one inside row band in separate tests. Assert:

```python
assert report["status"] == "FAIL"
assert "OUTSIDE_SITE_BOUNDARY_FREE" in report["hard_failures"]
assert "ROW_STRUCTURAL_FREE_LEAK" in report["hard_failures"]
```

- [ ] **Step 2: Write replay verification test**

Pass the exact ordered overrides to QA. QA must call `replay_formal_navigation_overrides(materialized.navigation, corridor, site_boundary, overrides)` again and compare the recomputed occupancy with `accepted.navigation.occupancy`:

```python
assert report["accepted_matches_replay"] is True
```

Tamper one Accepted cell and assert `ACCEPTED_REPLAY_MISMATCH` is a hard failure.

- [ ] **Step 3: Write per-aisle fraction/connectivity test**

Use one accepted synthetic aisle geometry with 8 FREE and 2 UNKNOWN cells. Assert `free_fraction=0.8`, `unknown_fraction=0.2`, `occupied_conflict_fraction=0.0`, and 8-connected FREE connectivity is true. Break the corridor with OCCUPIED cells and assert connectivity false.

- [ ] **Step 4: Run and verify import failure**

```bash
PYTHONPATH=src/agt_offline_assets \
python3 -m pytest -q src/agt_offline_assets/test/test_formal_navigation_qa.py
```

- [ ] **Step 5: Implement deterministic QA report**

The public signature is:

```python
def evaluate_formal_navigation_qa(
    *,
    ground_evidence: NavigationMapResult,
    materialized: StructureAwareNavigationResult,
    accepted: FormalOverrideReplayResult,
    overrides: Iterable[Mapping[str, object]],
    structure: NavigationStructureResult,
    corridor: CorridorRefinementResult,
    site_boundary: SiteBoundary,
) -> dict[str, object]:
    ...
```

Top-level keys:

```text
schema
status
hard_failures
outside_site_boundary_free_count
row_structural_band_free_leak_count
accepted_aisle_count
aisles
map_unknown_fraction
structure_inferred_free_fraction
manual_force_free_area_m2
manual_force_occupied_area_m2
accepted_matches_replay
```

Per-aisle masks are built from accepted `AislePairDiagnostic` row/boundary intervals in the same row-direction coordinate system as `navigation_corridor.py`. Connectivity is 8-connected and restricted to FREE cells inside that owned geometry.

- [ ] **Step 6: Implement deterministic JSON writer and register tests**

```python
path.write_text(
    json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
    encoding="utf-8",
)
```

Run:

```bash
PYTHONPATH=src/agt_offline_assets \
python3 -m pytest -q \
  src/agt_offline_assets/test/test_formal_navigation_qa.py \
  src/agt_offline_assets/test/test_formal_navigation_map.py \
  src/agt_offline_assets/test/test_formal_navigation_override.py
```

- [ ] **Step 7: Commit**

```bash
git add src/agt_offline_assets/agt_offline_assets/formal_navigation_qa.py \
  src/agt_offline_assets/agt_offline_assets/__init__.py \
  src/agt_offline_assets/test/test_formal_navigation_qa.py \
  src/agt_offline_assets/CMakeLists.txt
git commit -m "feat(map): add planner-independent formal map QA"
```

---

### Task 5: Keep Historical Map-Quality Replay Compatible

**Files:**
- Modify: `src/agt_route_benchmark/agt_route_benchmark/map_quality.py`
- Create: `src/agt_route_benchmark/test/test_map_quality_structure_aware.py`

**Interfaces:**
- Consumes both revision kinds.
- Produces unchanged `audit_map_revision(...)` and `write_map_quality_evidence(...)` behavior.

- [ ] **Step 1: Write a new-revision replay test using Task 3 exporter**

```python
report = audit_map_revision(
    revision / "generated/navigation_map.yaml",
    revision / "accepted/navigation_map.yaml",
    revision / "derivation.yaml",
)
assert report["accepted_matches_replay"] is True
assert report["unexplained_changed_cell_count"] == 0
```

- [ ] **Step 2: Verify current strict kind check fails**

```bash
MPLBACKEND=Agg \
PYTHONPATH=src/agt_route_benchmark:src/agt_offline_assets \
python3 -m pytest -q src/agt_route_benchmark/test/test_map_quality_structure_aware.py
```

- [ ] **Step 3: Broaden only revision-kind acceptance**

```python
_ALLOWED_REVISION_KINDS = {
    "generated_plus_accepted_override_revision",
    "structure_aware_generated_plus_accepted_override_revision",
}
```

Do not weaken schema/frame/override metadata/replay checks.

- [ ] **Step 4: Run historical + new tests**

```bash
MPLBACKEND=Agg \
PYTHONPATH=src/agt_route_benchmark:src/agt_offline_assets \
python3 -m pytest -q \
  src/agt_route_benchmark/test/test_map_quality.py \
  src/agt_route_benchmark/test/test_map_quality_cli.py \
  src/agt_route_benchmark/test/test_map_quality_structure_aware.py
```

- [ ] **Step 5: Commit**

```bash
git add src/agt_route_benchmark/agt_route_benchmark/map_quality.py \
  src/agt_route_benchmark/test/test_map_quality_structure_aware.py
git commit -m "fix(benchmark): audit structure-aware map revisions"
```

---

### Task 6: Wire Generated Preview and Formal Export into Paper I Workbench

**Files:**
- Modify: `src/agt_map_workbench/agt_map_workbench/navigation_preview.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/paper1_workbench.py`
- Create: `src/agt_map_workbench/test/test_structure_aware_navigation_workbench.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**
- Consumes Tasks 1-4.
- Produces `_formal_materialization`, `_formal_accepted_result`, `_formal_navigation_qa`, explicit Generated preview, and navigation-only structure-aware export pending Plan 2 unified freeze.

- [ ] **Step 1: Write failing Workbench-state test**

Offscreen instantiate `Paper1MapWorkbenchWindow`, inject synthetic `_navigation_base_result`, `_navigation_structure_result`, `_corridor_refinement_result`, and `_site_boundary`, then call `_build_formal_navigation_state()` and assert:

```python
assert window._formal_materialization is not None
assert window._formal_accepted_result is not None
assert window._formal_navigation_qa["status"] == "PASS"
```

Missing Site Boundary or corridor must leave formal state unset with a non-empty `_formal_last_error`.

- [ ] **Step 2: Run and verify missing method**

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_offline_assets:src/agt_ui_bridge:src/agt_coverage_planning \
python3 -m pytest -q src/agt_map_workbench/test/test_structure_aware_navigation_workbench.py
```

- [ ] **Step 3: Add formal-state members/invalidation**

Initialize:

```python
self._formal_materialization = None
self._formal_accepted_result = None
self._formal_navigation_qa = None
self._formal_last_error = ""
```

Clear them whenever Ground result, corridor, Site Boundary, or formal override list changes. Do not replace `_navigation_result` or V25-12F state.

- [ ] **Step 4: Implement exact build sequence**

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
    overrides=overrides,
    structure=self._navigation_structure_result,
    corridor=self._corridor_refinement_result,
    site_boundary=self._site_boundary,
)
```

- [ ] **Step 5: Add preview modes**

Add these combo entries and render with existing `NavigationPreviewItem.set_result` / `set_mask`:

```text
正式 Generated PGM
Structure-Inferred FREE
Row Structural Block
Site Boundary Block
Unresolved UNKNOWN
```

Ground-only and existing structure layers remain available.

- [ ] **Step 6: Change `_export_navigation_map()` to Task 3 exporter**

Build formal state first and reject export when QA status is FAIL. Export Ground Evidence + Generated + Accepted + derivation through `export_structure_aware_navigation_revision(...)`.

- [ ] **Step 7: Register/run Workbench tests**

```cmake
ament_add_pytest_test(
  test_structure_aware_navigation_workbench
  test/test_structure_aware_navigation_workbench.py
)
```

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_offline_assets:src/agt_ui_bridge:src/agt_coverage_planning \
python3 -m pytest -q \
  src/agt_map_workbench/test/test_navigation_override_metadata.py \
  src/agt_map_workbench/test/test_site_boundary_workbench.py \
  src/agt_map_workbench/test/test_structure_aware_navigation_workbench.py
```

- [ ] **Step 8: Commit**

```bash
git add src/agt_map_workbench/agt_map_workbench/navigation_preview.py \
  src/agt_map_workbench/agt_map_workbench/paper1_workbench.py \
  src/agt_map_workbench/test/test_structure_aware_navigation_workbench.py \
  src/agt_map_workbench/CMakeLists.txt
git commit -m "feat(workbench): preview and freeze structure-aware PGM"
```

---

### Task 7: Software Gate and Real Greenhouse Checkpoint

**Files:**
- Modify only if a failing gate identifies a concrete defect.

**Interfaces:**
- Consumes Tasks 1-6.
- Produces the verified map-core checkpoint required before Plan 2.

- [ ] **Step 1: Run all offline-assets tests**

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=src/agt_offline_assets \
python3 -m pytest -q src/agt_offline_assets/test
```

- [ ] **Step 2: Run map-quality tests**

```bash
MPLBACKEND=Agg \
PYTHONPATH=src/agt_route_benchmark:src/agt_offline_assets:src/agt_coverage_planning:src/agt_ui_bridge \
python3 -m pytest -q \
  src/agt_route_benchmark/test/test_map_quality.py \
  src/agt_route_benchmark/test/test_map_quality_cli.py \
  src/agt_route_benchmark/test/test_map_quality_structure_aware.py
```

- [ ] **Step 3: Run Workbench tests**

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_offline_assets:src/agt_ui_bridge:src/agt_coverage_planning \
python3 -m pytest -q src/agt_map_workbench/test
```

- [ ] **Step 4: Run syntax gate**

```bash
python3 -m compileall -q \
  src/agt_offline_assets/agt_offline_assets \
  src/agt_map_workbench/agt_map_workbench \
  src/agt_route_benchmark/agt_route_benchmark
```

- [ ] **Step 5: Launch Paper I Workbench and export a fixed-name real revision**

Use revision directory name `greenhouse_01_map_revision_unified_001` in the GUI so later commands are exact:

```bash
source install/setup.bash
ROS_LOG_DIR="$PWD/runtime/log/unified_map_authoring" \
ros2 run agt_map_workbench agt_map_workbench_paper1
```

Manual acceptance:

```text
[ ] Ground-only Evidence remains visible.
[ ] Row structural band aligns with crop rows.
[ ] Aisle geometric envelope aligns with actual inter-row corridors.
[ ] Structure-Inferred FREE fills Ground-only UNKNOWN only inside valid aisles.
[ ] No base OCCUPIED is automatically whitened.
[ ] No row structural cell is FREE.
[ ] No Site Boundary exterior cell is FREE.
[ ] Headland UNKNOWN remains UNKNOWN unless Ground-FREE or formally overridden.
[ ] Formal QA exposes per-aisle FREE/UNKNOWN/conflict/connectivity.
[ ] Export contains evidence/, generated/, accepted/, derivation.yaml.
```

- [ ] **Step 6: Run replay audit on the exact revision**

```bash
REV="$PWD/runtime/maps/greenhouse_01/derivation/greenhouse_01_map_revision_unified_001"
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

- [ ] **Step 7: Record verification**

Do not create an empty commit. If fixes were needed, commit only those fixes. Record the verified head SHA and real revision path in the implementation handoff.

---

## Plan 1 Completion Gate

Do not start semantic-GUI integration until all are true:

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
