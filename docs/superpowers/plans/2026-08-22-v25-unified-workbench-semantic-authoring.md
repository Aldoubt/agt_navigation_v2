# V25 Unified Workbench Semantic Authoring Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move semantic/task authoring into the authoritative AGT Map Workbench, bind the semantic task to the exact Accepted structure-aware Nav2 map, freeze all assets in one atomic revision, and retire direct PGM mutation from the standalone semantic editor.

**Architecture:** Reuse `agt_ui_bridge` semantic models, validation, IO, and rasterization rather than creating a second semantic contract. Add a small Workbench semantic controller plus Qt panel that uses the existing map-frame canvas, explicit automatic-structure promotion, and one unified revision transaction that composes Plan 1 navigation payloads with semantics, authoring evidence, and QA before atomic publication.

**Tech Stack:** Python 3, PyQt5, NumPy, Shapely, Pillow, PyYAML, ROS2 Humble/ament_cmake_python, pytest.

**Spec:** `docs/superpowers/specs/2026-08-22-v25-unified-map-authoring-structure-aware-pgm-design.md`

**Dependency:** Complete `docs/superpowers/plans/2026-08-22-v25-structure-aware-pgm-and-revision.md` through its real-map acceptance gate first.

## Global Constraints

- Formal map production has one GUI authority: AGT Map Workbench.
- `Site Boundary` and semantic `field_boundary` remain distinct objects.
- The semantic task binds to `accepted/navigation_map.yaml`, never Evidence and never an independently edited raster.
- `coverage.yaml.base_map_sha256` must exactly equal the SHA256 of the accepted Nav2 YAML.
- Existing semantic schema stays at the supported 1.0/1.1 contracts; no schema 2.0 redesign.
- Automatic rows/lanes are candidates and require explicit operator accept/edit/reject.
- Promoted automatic semantics record `source` and `authoring_state` provenance.
- Purely manual semantics record `source: manual` and `authoring_state: accepted`.
- Semantic `keepout_zone` remains a separate mask and must not silently rewrite Generated or Accepted base PGM.
- The standalone semantic editor may no longer mutate formal navigation rasters.
- The final revision is atomic; any map, semantic, hash-binding, or validation error leaves no published final revision directory.
- Resource bundles package frozen assets; they do not independently regenerate formal raster authority.

---

## File Structure

### New Workbench files

- `src/agt_map_workbench/agt_map_workbench/semantic_authoring.py` — UI-independent semantic Workbench state/controller around existing `SemanticScene` and `CoverageParameters`.
- `src/agt_map_workbench/agt_map_workbench/semantic_candidates.py` — deterministic conversion of accepted automatic row/aisle structure into semantic Feature candidates.
- `src/agt_map_workbench/agt_map_workbench/semantic_authoring_panel.py` — Qt controls/object tree/properties/promotion controls; no file authority by itself.
- `src/agt_map_workbench/agt_map_workbench/semantic_revision.py` — write validated `semantic/` payload bound to Accepted map.
- `src/agt_map_workbench/agt_map_workbench/unified_map_revision.py` — atomic composition of Plan 1 navigation payload + semantic payload + authoring files + validation files.

### Existing files modified

- `src/agt_ui_bridge/agt_ui_bridge/semantic_rasterizer.py` — add deterministic Nav2-compatible keepout mask serialization helper.
- `src/agt_map_workbench/agt_map_workbench/paper1_workbench.py` — add `语义与任务` tab, integrate click/edit modes, auto-candidate promotion, unified freeze, frozen-revision tracking, and semantic resource bundle group.
- `src/agt_map_workbench/agt_map_workbench/resource_bundle.py` — add semantic group to recommended frozen-resource selection.
- `src/agt_map_workbench/agt_map_workbench/resource_bundle_dialog.py` — display semantic group availability/selection through existing generic group UI if no code change is needed; modify only if current label rendering assumes a fixed group set.
- `src/agt_ui_bridge/scripts/semantic_editor_qt5.py` — retire direct raster editing and in-place base-map save authority.
- `src/agt_map_workbench/CMakeLists.txt` — register tests.

### New tests

- `src/agt_map_workbench/test/test_semantic_authoring.py`
- `src/agt_map_workbench/test/test_semantic_candidates.py`
- `src/agt_map_workbench/test/test_semantic_authoring_panel.py`
- `src/agt_map_workbench/test/test_semantic_revision.py`
- `src/agt_map_workbench/test/test_unified_map_revision.py`
- `src/agt_map_workbench/test/test_unified_semantic_workbench.py`
- `src/agt_map_workbench/test/test_semantic_resource_bundle.py`
- `src/agt_ui_bridge/test/test_semantic_editor_no_raster_authority.py`

---

### Task 1: UI-Independent Semantic Authoring State for Workbench

**Files:**
- Create: `src/agt_map_workbench/agt_map_workbench/semantic_authoring.py`
- Create: `src/agt_map_workbench/test/test_semantic_authoring.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**
- Consumes: `SemanticMap`, `SemanticFeature`, `SemanticScene`, `CoverageParameters`, `ValidationContext`, `validate_task` from `agt_ui_bridge`.
- Produces: `SemanticAuthoringState`, `new_semantic_authoring_state(...)`, `semantic_validation_report(...)`.

- [ ] **Step 1: Write failing initial-state and manual-feature tests**

```python
def test_new_state_uses_map_frame_and_existing_coverage_contract():
    state = new_semantic_authoring_state(
        map_id="greenhouse_01",
        robot_profile="mk_mini",
        robot_width=0.60,
        operation_width=0.60,
        min_turning_radius=1.20,
        headland_width=1.50,
        allow_reverse=True,
    )
    assert state.scene.semantic_map.map_id == "greenhouse_01"
    assert state.scene.semantic_map.frame_id == "map"
    assert state.coverage.map_id == "greenhouse_01"
    assert state.coverage.frame_id == "map"


def test_manual_feature_gets_authoring_provenance():
    feature = state.add_manual_feature(
        feature_type="field_boundary",
        feature_id="field_01",
        name="Main field",
        geometry_type="Polygon",
        coordinates=[[[0, 0], [5, 0], [5, 8], [0, 8], [0, 0]]],
    )
    assert feature.properties["source"] == "manual"
    assert feature.properties["authoring_state"] == "accepted"
```

- [ ] **Step 2: Run and verify import failure**

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_ui_bridge \
python3 -m pytest -q src/agt_map_workbench/test/test_semantic_authoring.py
```

Expected: missing `semantic_authoring` module.

- [ ] **Step 3: Implement the controller around `SemanticScene`**

Define:

```python
@dataclass
class SemanticAuthoringState:
    scene: SemanticScene
    coverage: CoverageParameters
    selected_feature_id: str | None = None

    def add_manual_feature(...):
        properties = dict(properties or {})
        properties.update({"source": "manual", "authoring_state": "accepted"})
        feature = SemanticFeature(..., properties=properties)
        self.scene.add(feature)
        self.selected_feature_id = feature.id
        return feature
```

`new_semantic_authoring_state()` uses existing coverage schema `1.0`, `planning_mode="polygon"`, `row_interpretation="direct_swaths"`, and leaves `base_map` / `base_map_sha256` as explicit temporary placeholders only inside memory:

```python
base_map="<accepted-map-bound-at-freeze>"
base_map_sha256="<accepted-map-bound-at-freeze>"
```

These values are never serialized; Task 4 replaces both with the exact Accepted map identity before calling `save_semantic_task`.

- [ ] **Step 4: Add replace/delete/undo/redo tests using existing `SemanticScene` semantics**

Verify state operations delegate to `scene.replace_by_id`, `scene.remove`, `scene.undo`, and `scene.redo` without creating a second history implementation.

- [ ] **Step 5: Add `semantic_validation_report()`**

```python
def semantic_validation_report(
    state: SemanticAuthoringState,
    *,
    validation_context: ValidationContext,
):
    return validate_task(
        state.scene.semantic_map,
        state.coverage,
        context=validation_context,
    )
```

This pre-freeze report is allowed to include the base-map placeholder mismatch; Task 4 performs the authoritative validation after exact hash binding. The panel should present missing semantic-feature errors independently from the eventual map hash binding.

- [ ] **Step 6: Register/run tests and commit**

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_ui_bridge \
python3 -m pytest -q src/agt_map_workbench/test/test_semantic_authoring.py
```

Then:

```bash
git add \
  src/agt_map_workbench/agt_map_workbench/semantic_authoring.py \
  src/agt_map_workbench/test/test_semantic_authoring.py \
  src/agt_map_workbench/CMakeLists.txt
git commit -m "feat(workbench): add semantic authoring state"
```

---

### Task 2: Deterministic Automatic Row/Aisle Semantic Candidates

**Files:**
- Create: `src/agt_map_workbench/agt_map_workbench/semantic_candidates.py`
- Create: `src/agt_map_workbench/test/test_semantic_candidates.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**
- Consumes: `NavigationMapResult`, `NavigationStructureResult`, `CorridorRefinementResult`, `AgriculturalAisleGraph`.
- Produces: `SemanticCandidate`, `derive_row_semantic_candidates(...)`, `derive_access_lane_semantic_candidates(...)`, `promote_semantic_candidate(...)`.

- [ ] **Step 1: Write failing deterministic-candidate tests**

For a synthetic two-row/one-aisle fixture, assert stable IDs and map-frame coordinates:

```python
rows = derive_row_semantic_candidates(navigation, structure, corridor)
lanes = derive_access_lane_semantic_candidates(aisle_graph)

assert [item.feature.id for item in rows] == ["auto_row_01", "auto_row_02"]
assert [item.feature.id for item in lanes] == ["auto_lane_001"]
assert rows[0].feature.feature_type == "row_centerline"
assert lanes[0].feature.feature_type == "access_lane"
assert lanes[0].feature.coordinates == [list(p[:2]) for p in aisle_graph.aisles[0].centerline_xyz]
```

- [ ] **Step 2: Run and verify failure**

```bash
PYTHONPATH=src/agt_map_workbench:src/agt_offline_assets:src/agt_ui_bridge \
python3 -m pytest -q src/agt_map_workbench/test/test_semantic_candidates.py
```

- [ ] **Step 3: Implement candidate and promotion contracts**

```python
@dataclass(frozen=True)
class SemanticCandidate:
    candidate_id: str
    feature: SemanticFeature
    source_kind: str


def promote_semantic_candidate(candidate, *, edited=False):
    feature = SemanticFeature.from_geojson(candidate.feature.to_geojson())
    feature.properties["source"] = candidate.source_kind
    feature.properties["authoring_state"] = (
        "manually_edited" if edited else "accepted"
    )
    return feature
```

For access lanes, use `AgriculturalAisleGraph.centerline_xyz` directly.

For row lines, derive one centerline per accepted row center from `corridor.row_centerline`: project grid centers onto the row direction/perpendicular axes, select the cells nearest each `accepted_row_centers_v_m` value, sort by longitudinal `u`, collapse duplicate longitudinal samples, and emit at least two XY points. Do not infer semantic rows from rejected row hypotheses.

- [ ] **Step 4: Add provenance/edit tests**

```python
accepted = promote_semantic_candidate(candidate, edited=False)
edited = promote_semantic_candidate(candidate, edited=True)
assert accepted.properties == {"source": "auto_row_detection", "authoring_state": "accepted"}
assert edited.properties["authoring_state"] == "manually_edited"
```

Preserve any non-authoring properties already present on the candidate.

- [ ] **Step 5: Register/run tests and commit**

```bash
PYTHONPATH=src/agt_map_workbench:src/agt_offline_assets:src/agt_ui_bridge \
python3 -m pytest -q src/agt_map_workbench/test/test_semantic_candidates.py
```

Then:

```bash
git add \
  src/agt_map_workbench/agt_map_workbench/semantic_candidates.py \
  src/agt_map_workbench/test/test_semantic_candidates.py \
  src/agt_map_workbench/CMakeLists.txt
git commit -m "feat(workbench): derive semantic row and lane candidates"
```

---

### Task 3: Add the `语义与任务` Workbench Panel and Map-Frame Drawing

**Files:**
- Create: `src/agt_map_workbench/agt_map_workbench/semantic_authoring_panel.py`
- Create: `src/agt_map_workbench/test/test_semantic_authoring_panel.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/paper1_workbench.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 1 `SemanticAuthoringState`, Task 2 candidates, existing `PointCloudView.mapClicked` through `Paper1MapWorkbenchWindow._on_map_clicked`.
- Produces: fourth right-side tab, object tree, semantic tool selection, candidate accept/reject actions, semantic validation summary.

- [ ] **Step 1: Write a failing tab/controls test**

Offscreen instantiate the Paper Workbench and assert:

```python
labels = [window._control_tabs.tabText(i) for i in range(window._control_tabs.count())]
assert labels == ["点云编辑", "坐标系标定", "导航地图", "语义与任务"]
assert window._semantic_panel is not None
assert window._semantic_panel.tool_keys() == {
    "field_boundary",
    "exclusion_zone",
    "row_centerline",
    "access_lane",
    "entry_pose",
    "work_direction",
    "headland_zone",
    "keepout_zone",
}
```

- [ ] **Step 2: Run and verify failure**

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_offline_assets:src/agt_ui_bridge:src/agt_coverage_planning \
python3 -m pytest -q src/agt_map_workbench/test/test_semantic_authoring_panel.py
```

- [ ] **Step 3: Implement a panel that owns controls but not map authority**

`SemanticAuthoringPanel(QWidget)` receives the controller/state from the parent and exposes Qt signals:

```python
toolRequested = pyqtSignal(str)
finishRequested = pyqtSignal()
cancelRequested = pyqtSignal()
undoRequested = pyqtSignal()
redoRequested = pyqtSignal()
deleteRequested = pyqtSignal()
acceptCandidateRequested = pyqtSignal(str)
rejectCandidateRequested = pyqtSignal(str)
```

Use a `QTreeWidget` with columns `[ID, 类型, 名称, 来源]`. Do not embed a second `QGraphicsScene` or a second map image.

- [ ] **Step 4: Integrate drawing into the existing Workbench click path**

Extend `_on_map_clicked(x, y)` in `Paper1MapWorkbenchWindow` before delegating to `super()`:

```python
if self._interaction_mode.startswith("semantic:"):
    self._append_semantic_vertex(x, y)
    return
```

Use the existing map-frame click coordinates directly; do not round-trip through PGM pixel coordinates.

Polygon tools finish with >=3 vertices, line tools with >=2, and `entry_pose` / `work_direction` use the same two-click position+direction convention as the legacy editor. Convert entry pose to `Point` plus `properties["yaw"]`; convert work direction to a two-point `LineString`.

- [ ] **Step 5: Implement object properties and vertex editing without direct raster painting**

Use `SemanticScene.replace_by_id()` for ID/name/geometry changes and existing Workbench graphics primitives for selected vertices. All geometry remains world/map coordinates. Semantic editing must never assign to a PGM/occupancy array.

- [ ] **Step 6: Add automatic candidate accept/reject controls**

After row/corridor analysis exists, populate candidates from Task 2. Accept calls `state.scene.add(promote_semantic_candidate(...))`; reject records the candidate ID in an in-memory rejected set so it is not re-presented until structure is recomputed.

- [ ] **Step 7: Register and run offscreen tests**

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_offline_assets:src/agt_ui_bridge:src/agt_coverage_planning \
python3 -m pytest -q \
  src/agt_map_workbench/test/test_semantic_authoring.py \
  src/agt_map_workbench/test/test_semantic_candidates.py \
  src/agt_map_workbench/test/test_semantic_authoring_panel.py
```

Expected: PASS.

- [ ] **Step 8: Commit Task 3**

```bash
git add \
  src/agt_map_workbench/agt_map_workbench/semantic_authoring_panel.py \
  src/agt_map_workbench/agt_map_workbench/paper1_workbench.py \
  src/agt_map_workbench/test/test_semantic_authoring_panel.py \
  src/agt_map_workbench/CMakeLists.txt
git commit -m "feat(workbench): integrate semantic task authoring tab"
```

---

### Task 4: Bind and Serialize Semantic Task to the Exact Accepted Map

**Files:**
- Modify: `src/agt_ui_bridge/agt_ui_bridge/semantic_rasterizer.py`
- Create: `src/agt_map_workbench/agt_map_workbench/semantic_revision.py`
- Create: `src/agt_map_workbench/test/test_semantic_revision.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**
- Consumes: `SemanticAuthoringState`, accepted `navigation_map.yaml`, map geometry, vehicle validation context.
- Produces: `write_semantic_revision_payload(root, ...) -> dict[str, object]`; files `semantic/semantic_map.geojson`, `semantic/coverage.yaml`, `semantic/keepout_mask.pgm`, `semantic/keepout_mask.yaml`, `validation/semantic_validation.json`.

- [ ] **Step 1: Write the failing exact-hash binding test**

```python
def test_semantic_payload_binds_exact_accepted_yaml(tmp_path):
    write_semantic_revision_payload(
        root,
        state=valid_state,
        accepted_map_yaml=root / "accepted/navigation_map.yaml",
        validation_context=context,
    )
    coverage = yaml.safe_load((root / "semantic/coverage.yaml").read_text())
    expected = sha256_file(root / "accepted/navigation_map.yaml")
    assert coverage["base_map_sha256"] == expected
    assert (root / "semantic" / coverage["base_map"]).resolve() == (
        root / "accepted/navigation_map.yaml"
    ).resolve()
```

- [ ] **Step 2: Add a failing semantic-validation hard-fail test**

Remove `entry_pose` and assert `write_semantic_revision_payload` raises `SemanticFileError` containing `missing_feature_type` and leaves no semantic files behind.

- [ ] **Step 3: Run and verify failure**

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_ui_bridge:src/agt_offline_assets \
python3 -m pytest -q src/agt_map_workbench/test/test_semantic_revision.py
```

- [ ] **Step 4: Add deterministic keepout-mask serialization**

In `semantic_rasterizer.py` add:

```python
def write_keepout_nav2_mask(mask, map_geometry, pgm_path, yaml_path):
    image = np.asarray(mask.data, dtype=np.int16).reshape(mask.height, mask.width)
    pgm = np.where(image >= 100, 0, 254).astype(np.uint8)
    pgm_top_down = np.flipud(pgm)
    # P5 header + bytes, same trinary orientation as the base map.
```

The YAML uses the accepted map geometry's exact resolution and origin, `mode: trinary`, `negate: 0`, `occupied_thresh: 0.65`, `free_thresh: 0.196`.

- [ ] **Step 5: Implement authoritative coverage rebinding before `save_semantic_task`**

Deep-copy the in-memory coverage and set:

```python
coverage.base_map = os.path.relpath(
    accepted_map_yaml,
    (root / "semantic"),
)
coverage.base_map_sha256 = sha256_file(accepted_map_yaml)
```

Build `ValidationContext(map_geometry=MapGeometry.from_nav2_yaml(accepted_map_yaml), navigation_footprint=..., minimum_boundary_clearance=..., base_map_path=accepted_map_yaml)` and call `save_semantic_task(...)` only after `validate_task(...).valid` is true.

- [ ] **Step 6: Rasterize keepout separately from the base PGM**

Call `rasterize_keepout_mask(state.scene.semantic_map, map_geometry, outside_field_is_keepout=True)` and write `keepout_mask.*`. Assert in tests that the accepted base PGM bytes are unchanged before/after semantic payload creation.

- [ ] **Step 7: Write `semantic_validation.json`**

Serialize all issues, including severity/code/object ID/message, and top-level `status: PASS`. A payload with any ERROR must raise instead of writing a misleading PASS file.

- [ ] **Step 8: Run tests and commit**

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_ui_bridge:src/agt_offline_assets \
python3 -m pytest -q src/agt_map_workbench/test/test_semantic_revision.py
```

Then:

```bash
git add \
  src/agt_ui_bridge/agt_ui_bridge/semantic_rasterizer.py \
  src/agt_map_workbench/agt_map_workbench/semantic_revision.py \
  src/agt_map_workbench/test/test_semantic_revision.py \
  src/agt_map_workbench/CMakeLists.txt
git commit -m "feat(map): bind semantic task to accepted navigation map"
```

---

### Task 5: One Atomic Unified Map Revision Freeze

**Files:**
- Create: `src/agt_map_workbench/agt_map_workbench/unified_map_revision.py`
- Create: `src/agt_map_workbench/test/test_unified_map_revision.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/paper1_workbench.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**
- Consumes: Plan 1 `write_structure_aware_navigation_payload`, `write_formal_navigation_qa`; Task 4 `write_semantic_revision_payload`; optional authoring writers for Recipe/map_frame/site_boundary.
- Produces: `export_unified_map_revision(parent_dir, revision_name, inputs) -> Path`; final immutable revision containing navigation + semantic + authoring + validation.

- [ ] **Step 1: Write a failing full-contract test**

```python
revision = export_unified_map_revision(...)
assert (revision / "evidence/ground_only_navigation_map.pgm").is_file()
assert (revision / "generated/navigation_map.pgm").is_file()
assert (revision / "accepted/navigation_map.pgm").is_file()
assert (revision / "semantic/semantic_map.geojson").is_file()
assert (revision / "semantic/coverage.yaml").is_file()
assert (revision / "authoring/site_boundary.yaml").is_file()
assert (revision / "validation/navigation_validation.json").is_file()
assert (revision / "validation/semantic_validation.json").is_file()
```

- [ ] **Step 2: Write rollback tests for navigation, semantic, and authoring failures**

Monkeypatch each writer in turn to raise `RuntimeError("injected")`. For every case assert:

```python
assert not final_revision.exists()
assert not any(parent.glob(f".{revision_name}.tmp-*"))
```

- [ ] **Step 3: Run and verify failure**

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_ui_bridge:src/agt_offline_assets \
python3 -m pytest -q src/agt_map_workbench/test/test_unified_map_revision.py
```

- [ ] **Step 4: Implement the staging transaction**

Use this order inside a hidden sibling staging directory:

```text
1. write_structure_aware_navigation_payload(stage, ...)
2. write authoring/processing_recipe.yaml when available
3. write authoring/map_frame.yaml when available
4. write authoring/site_boundary.yaml (required)
5. write_semantic_revision_payload(stage, accepted_map_yaml=stage/accepted/navigation_map.yaml, ...)
6. write validation/navigation_validation.json
7. re-hash accepted/navigation_map.yaml and verify semantic coverage binding
8. os.replace(stage, final_revision)
```

No timestamp belongs in deterministic derivation files. If a human-readable `created_at` is desired later, it belongs in a separate non-authority manifest; do not add it in this increment.

- [ ] **Step 5: Change Paper Workbench formal export to call the unified exporter**

Replace the Plan 1 navigation-only export call. The Workbench must refuse freeze when:

```text
formal navigation state missing
navigation QA FAIL
semantic validation ERROR
Site Boundary missing
```

On success store:

```python
self._last_frozen_revision = output.resolve()
```

for Task 6 bundle export.

- [ ] **Step 6: Add an integration test for exact accepted-map binding after final rename**

Load `semantic/coverage.yaml` from the final revision and assert its relative `base_map` resolves to final `accepted/navigation_map.yaml`, not the temporary staging absolute path. This is why `coverage.base_map` must be written as a relative path.

- [ ] **Step 7: Run and commit**

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_ui_bridge:src/agt_offline_assets:src/agt_coverage_planning \
python3 -m pytest -q \
  src/agt_map_workbench/test/test_semantic_revision.py \
  src/agt_map_workbench/test/test_unified_map_revision.py
```

Then:

```bash
git add \
  src/agt_map_workbench/agt_map_workbench/unified_map_revision.py \
  src/agt_map_workbench/agt_map_workbench/paper1_workbench.py \
  src/agt_map_workbench/test/test_unified_map_revision.py \
  src/agt_map_workbench/CMakeLists.txt
git commit -m "feat(workbench): freeze one unified map revision"
```

---

### Task 6: Package Frozen Semantic Assets in Map Resource Bundles

**Files:**
- Modify: `src/agt_map_workbench/agt_map_workbench/resource_bundle.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/paper1_workbench.py`
- Create: `src/agt_map_workbench/test/test_semantic_resource_bundle.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**
- Consumes: `_last_frozen_revision` from Task 5.
- Produces ResourceBundle group key `semantic_task`; writer copies only frozen semantic/validation files and never regenerates them.

- [ ] **Step 1: Write a failing recommended-selection test**

```python
selection = ResourceBundleSelection.recommended(
    available_groups={"navigation_revision", "semantic_task", "site_boundary"}
)
assert "semantic_task" in selection.selected_groups
```

- [ ] **Step 2: Write a failing frozen-copy test**

Create a fake frozen revision and assert bundle output includes byte-identical:

```text
semantic/semantic_map.geojson
semantic/coverage.yaml
semantic/keepout_mask.pgm
semantic/keepout_mask.yaml
validation/semantic_validation.json
```

No semantic algorithms may be invoked during this test.

- [ ] **Step 3: Run and verify failure**

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_ui_bridge:src/agt_offline_assets \
python3 -m pytest -q src/agt_map_workbench/test/test_semantic_resource_bundle.py
```

- [ ] **Step 4: Add `semantic_task` to recommended groups and Workbench availability**

Add `"semantic_task"` to `RECOMMENDED_GROUPS`. In `_bundle_groups()`, the group is available only when `_last_frozen_revision` exists and contains the five expected frozen files. The writer uses `shutil.copy2`; it must not call `save_semantic_task`, `rasterize_keepout_mask`, or map materialization.

- [ ] **Step 5: Run bundle regressions and commit**

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_ui_bridge:src/agt_offline_assets \
python3 -m pytest -q \
  src/agt_map_workbench/test/test_resource_bundle.py \
  src/agt_map_workbench/test/test_paper1_resource_bundle_integration.py \
  src/agt_map_workbench/test/test_semantic_resource_bundle.py
```

Then:

```bash
git add \
  src/agt_map_workbench/agt_map_workbench/resource_bundle.py \
  src/agt_map_workbench/agt_map_workbench/paper1_workbench.py \
  src/agt_map_workbench/test/test_semantic_resource_bundle.py \
  src/agt_map_workbench/CMakeLists.txt
git commit -m "feat(workbench): bundle frozen semantic task assets"
```

---

### Task 7: Retire Direct Raster Authority from the Standalone Semantic Editor

**Files:**
- Modify: `src/agt_ui_bridge/scripts/semantic_editor_qt5.py`
- Create: `src/agt_ui_bridge/test/test_semantic_editor_no_raster_authority.py`
- Modify: the owning package test registration file if this test directory is explicitly enumerated.

**Interfaces:**
- Consumes: existing standalone semantic editor.
- Produces: semantic-only legacy editor; no map FREE/OCCUPIED/UNKNOWN painting and no in-place base-map write.

- [ ] **Step 1: Write a failing authority-retirement test**

Offscreen instantiate `SemanticEditorWindow` and assert:

```python
assert "map_occupied" not in window._tool_actions
assert "map_free" not in window._tool_actions
assert "map_unknown" not in window._tool_actions
```

Also directly call the legacy save-map method and assert it cannot write:

```python
before = base_pgm.read_bytes()
assert window._save_map_in_place() is False
assert base_pgm.read_bytes() == before
```

- [ ] **Step 2: Run and verify current behavior fails the test**

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_ui_bridge \
python3 -m pytest -q src/agt_ui_bridge/test/test_semantic_editor_no_raster_authority.py
```

- [ ] **Step 3: Remove raster edit actions from `_build_ui()`**

Do not add the `底图编辑` toolbar or the `map_occupied/map_free/map_unknown` actions. Keep semantic drawing and loading unchanged.

- [ ] **Step 4: Fail closed if stale callers invoke raster mutation methods**

At the start of `apply_map_brush`, `apply_map_line`, `begin_map_edit`, and `_save_map_in_place`, return without mutation and show/raise a clear legacy message as appropriate:

```text
正式 Navigation Map 栅格编辑已迁移到 AGT Map Workbench；此编辑器仅保留语义任务编辑。
```

`save()` must no longer attempt `_save_map_in_place()` even if a stale object somehow sets `map_dirty=True`; clear `map_dirty` only by reload, not by writing the file.

- [ ] **Step 5: Keep semantic save behavior and base-map hash validation**

Run existing semantic editor/model/IO tests to ensure semantic-only task editing remains compatible with existing files.

- [ ] **Step 6: Commit Task 7**

```bash
git add \
  src/agt_ui_bridge/scripts/semantic_editor_qt5.py \
  src/agt_ui_bridge/test/test_semantic_editor_no_raster_authority.py
git commit -m "refactor(ui): retire standalone raster map authority"
```

---

### Task 8: End-to-End Unified Workbench Acceptance

**Files:**
- Create: `src/agt_map_workbench/test/test_unified_semantic_workbench.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`
- Documentation: update `src/agt_map_workbench/README.md` only after behavior is verified.

**Interfaces:**
- Consumes all Plan 1 and Plan 2 interfaces.
- Produces the final verified workflow and migration documentation.

- [ ] **Step 1: Write an end-to-end synthetic integration test**

The test should construct/inject:

```text
Ground Evidence with one aisle UNKNOWN gap
valid row/corridor structure
Site Boundary
one evidence-backed FORCE_FREE or FORCE_OCCUPIED override
required semantic objects
```

Then freeze and assert:

```python
revision = window._freeze_unified_revision_to(test_parent, "revision_001")
assert load_navigation_grid(revision / "generated/navigation_map.yaml").occupancy[gap] == FREE
assert sha256_file(revision / "accepted/navigation_map.yaml") == yaml.safe_load(
    (revision / "semantic/coverage.yaml").read_text()
)["base_map_sha256"]
assert json.loads(
    (revision / "validation/navigation_validation.json").read_text()
)["status"] == "PASS"
assert json.loads(
    (revision / "validation/semantic_validation.json").read_text()
)["status"] == "PASS"
```

- [ ] **Step 2: Run the complete Workbench test suite**

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_offline_assets:src/agt_ui_bridge:src/agt_coverage_planning \
python3 -m pytest -q src/agt_map_workbench/test
```

Expected: PASS.

- [ ] **Step 3: Run offline and benchmark regressions**

```bash
PYTHONPATH=src/agt_offline_assets \
python3 -m pytest -q src/agt_offline_assets/test

MPLBACKEND=Agg \
PYTHONPATH=src/agt_route_benchmark:src/agt_offline_assets:src/agt_coverage_planning:src/agt_ui_bridge \
python3 -m pytest -q src/agt_route_benchmark/test
```

Expected: PASS.

- [ ] **Step 4: Run syntax gate**

```bash
python3 -m compileall -q \
  src/agt_offline_assets/agt_offline_assets \
  src/agt_map_workbench/agt_map_workbench \
  src/agt_ui_bridge/agt_ui_bridge \
  src/agt_ui_bridge/scripts \
  src/agt_route_benchmark/agt_route_benchmark
```

Expected: exit code 0.

- [ ] **Step 5: Perform one real greenhouse unified authoring session**

Launch:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR="$PWD/runtime/log/unified_map_authoring" \
ros2 run agt_map_workbench agt_map_workbench_paper1
```

Manual acceptance checklist:

```text
[ ] Open the authoritative processed/canonical greenhouse PCD.
[ ] Review Ground-only Evidence.
[ ] Review row structural bands and aisle geometry.
[ ] Review Structure-Aware Generated PGM.
[ ] Draw/freeze Site Boundary.
[ ] Add only evidence-backed formal raster overrides when necessary.
[ ] Open `语义与任务` without launching another editor.
[ ] Accept/edit/reject automatic row/lane candidates.
[ ] Draw field boundary, exclusion, entry pose, work direction, headland/keepout as needed.
[ ] Freeze one immutable unified revision.
[ ] `semantic/coverage.yaml` binds exactly to `accepted/navigation_map.yaml` SHA256.
[ ] Resource bundle copies frozen semantic assets without regeneration.
[ ] Standalone semantic editor cannot paint or save PGM changes.
```

- [ ] **Step 6: Run map replay QA on the final real revision**

```bash
REV="$PWD/runtime/maps/greenhouse_01/derivation/<the-new-unified-revision>"
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

- [ ] **Step 7: Update Workbench documentation after verification**

Update `src/agt_map_workbench/README.md` so its canonical flow is:

```text
PCD -> Ground Evidence -> Agricultural Structure -> Structure-Aware Generated PGM
-> evidence-backed Accepted PGM -> Semantic Task -> Unified Immutable Revision
```

Mark the standalone semantic editor as legacy semantic-only and explicitly state it is not a formal raster authority.

- [ ] **Step 8: Commit the final integration/docs task**

```bash
git add \
  src/agt_map_workbench/test/test_unified_semantic_workbench.py \
  src/agt_map_workbench/CMakeLists.txt \
  src/agt_map_workbench/README.md
git commit -m "docs(map): finalize unified authoring workflow"
```

---

## Final Completion Gate

The unified authoring work is complete only when all of these are evidenced:

```text
Plan 1 real-map gate PASS
all offline-assets tests PASS
all Workbench tests PASS
all route-benchmark tests PASS
compileall PASS
Structure-Aware Generated PGM is usable on the reviewed greenhouse map
Accepted map replay is exact
semantic validation PASS
coverage.yaml accepted-map SHA256 binding PASS
one unified immutable revision exported
resource bundle copies frozen semantics without regeneration
standalone semantic editor cannot mutate PGM
```

Only after this gate should Paper I downstream planner tuning / experiment matrix work resume.