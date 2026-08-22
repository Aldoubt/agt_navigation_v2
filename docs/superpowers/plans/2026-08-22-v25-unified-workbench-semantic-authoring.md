# V25 Unified Workbench Semantic Authoring Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move semantic/task authoring into the authoritative AGT Map Workbench, bind the semantic task to the exact Accepted structure-aware Nav2 map, freeze all assets in one atomic revision, and retire direct PGM mutation from the standalone semantic editor.

**Architecture:** Reuse `agt_ui_bridge` semantic models, validation, IO, and rasterization rather than creating a second contract. Add a Workbench semantic controller plus Qt panel using the existing map-frame canvas, explicit automatic-structure promotion, and one unified staging transaction that composes Plan 1 navigation payloads with semantics, authoring evidence, and QA before atomic publication.

**Tech Stack:** Python 3, PyQt5, NumPy, Shapely, Pillow, PyYAML, ROS2 Humble/ament_cmake_python, pytest.

**Spec:** `docs/superpowers/specs/2026-08-22-v25-unified-map-authoring-structure-aware-pgm-design.md`

**Dependency:** `docs/superpowers/plans/2026-08-22-v25-structure-aware-pgm-and-revision.md` must pass its real-map gate first.

## Global Constraints

- Formal map production has one GUI authority: AGT Map Workbench.
- `Site Boundary` and semantic `field_boundary` remain distinct objects.
- Semantic task binds to `accepted/navigation_map.yaml`, never Evidence and never an independently edited raster.
- `coverage.yaml.base_map_sha256` equals the exact SHA256 of accepted Nav2 YAML.
- Existing semantic schema remains 1.0/1.1; no schema 2.0 redesign.
- Automatic rows/lanes are candidates requiring explicit accept/edit/reject.
- Promoted automatic semantics record `source` and `authoring_state`; manual semantics record `source: manual`, `authoring_state: accepted`.
- Semantic `keepout_zone` stays a separate mask and never silently rewrites Generated or Accepted base PGM.
- Standalone semantic editor may no longer mutate formal navigation rasters.
- Unified revision is atomic and fail-closed.
- Resource bundle copies frozen assets and never regenerates formal raster authority.

---

## File Structure

**Create**
- `src/agt_map_workbench/agt_map_workbench/semantic_authoring.py` — UI-independent semantic state/controller.
- `src/agt_map_workbench/agt_map_workbench/semantic_candidates.py` — deterministic automatic row/lane candidates.
- `src/agt_map_workbench/agt_map_workbench/semantic_authoring_panel.py` — Qt controls/object tree/properties/candidate actions.
- `src/agt_map_workbench/agt_map_workbench/semantic_revision.py` — exact Accepted-map semantic binding and semantic payload writing.
- `src/agt_map_workbench/agt_map_workbench/unified_map_revision.py` — one atomic revision transaction.
- `src/agt_map_workbench/test/test_semantic_authoring.py`
- `src/agt_map_workbench/test/test_semantic_candidates.py`
- `src/agt_map_workbench/test/test_semantic_authoring_panel.py`
- `src/agt_map_workbench/test/test_semantic_revision.py`
- `src/agt_map_workbench/test/test_unified_map_revision.py`
- `src/agt_map_workbench/test/test_semantic_resource_bundle.py`
- `src/agt_map_workbench/test/test_unified_semantic_workbench.py`
- `src/agt_ui_bridge/test/test_semantic_editor_no_raster_authority.py`

**Modify**
- `src/agt_ui_bridge/agt_ui_bridge/semantic_rasterizer.py`
- `src/agt_ui_bridge/scripts/semantic_editor_qt5.py`
- `src/agt_ui_bridge/CMakeLists.txt`
- `src/agt_map_workbench/agt_map_workbench/paper1_workbench.py`
- `src/agt_map_workbench/agt_map_workbench/resource_bundle.py`
- `src/agt_map_workbench/CMakeLists.txt`
- `src/agt_map_workbench/README.md`

---

### Task 1: UI-Independent Semantic Authoring State

**Files:**
- Create: `src/agt_map_workbench/agt_map_workbench/semantic_authoring.py`
- Create: `src/agt_map_workbench/test/test_semantic_authoring.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**
- Consumes: `SemanticMap`, `SemanticFeature`, `SemanticScene`, `CoverageParameters`, `ValidationContext`, `validate_semantic_map`.
- Produces: `UNBOUND_ACCEPTED_MAP`, `SemanticAuthoringState`, `new_semantic_authoring_state(...)`, `semantic_document_report(...)`.

- [ ] **Step 1: Write failing initial-state/manual-feature tests**

```python
def test_new_state_is_map_frame_and_unbound_until_freeze():
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
    assert state.coverage.base_map == UNBOUND_ACCEPTED_MAP
    assert state.coverage.base_map_sha256 == UNBOUND_ACCEPTED_MAP


def test_manual_feature_gets_provenance():
    state = new_semantic_authoring_state(...same explicit arguments...)
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

In the actual test file, replace the repeated constructor call with a local `_state()` helper containing those exact arguments; do not use ellipsis in executable test code.

- [ ] **Step 2: Run and verify import failure**

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_ui_bridge \
python3 -m pytest -q src/agt_map_workbench/test/test_semantic_authoring.py
```

- [ ] **Step 3: Implement controller and sentinel**

```python
UNBOUND_ACCEPTED_MAP = "__UNBOUND_ACCEPTED_MAP__"

@dataclass
class SemanticAuthoringState:
    scene: SemanticScene
    coverage: CoverageParameters
    selected_feature_id: str | None = None

    def add_manual_feature(self, *, feature_type, feature_id, name,
                           geometry_type, coordinates, properties=None):
        values = dict(properties or {})
        values.update({"source": "manual", "authoring_state": "accepted"})
        feature = SemanticFeature(
            id=feature_id,
            feature_type=feature_type,
            name=name,
            geometry_type=geometry_type,
            coordinates=coordinates,
            properties=values,
        )
        self.scene.add(feature)
        self.selected_feature_id = feature.id
        return feature
```

`new_semantic_authoring_state()` creates coverage schema 1.0 with `base_map` and `base_map_sha256` both set to `UNBOUND_ACCEPTED_MAP`. Those sentinel values are never serialized by Task 4.

- [ ] **Step 4: Use semantic-only live validation**

```python
def semantic_document_report(state, *, validation_context):
    return validate_semantic_map(
        state.scene.semantic_map,
        context=validation_context,
    )
```

This checks required coverage features and geometry while deliberately avoiding base-map hash validation before freeze.

- [ ] **Step 5: Test replace/delete/undo/redo delegation**

Use `SemanticScene.replace_by_id`, `remove`, `undo`, and `redo`; do not implement a second history stack.

- [ ] **Step 6: Register/run/commit**

```cmake
ament_add_pytest_test(test_semantic_authoring test/test_semantic_authoring.py)
```

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_ui_bridge \
python3 -m pytest -q src/agt_map_workbench/test/test_semantic_authoring.py

git add src/agt_map_workbench/agt_map_workbench/semantic_authoring.py \
  src/agt_map_workbench/test/test_semantic_authoring.py \
  src/agt_map_workbench/CMakeLists.txt
git commit -m "feat(workbench): add semantic authoring state"
```

---

### Task 2: Deterministic Automatic Row/Aisle Candidates

**Files:**
- Create: `src/agt_map_workbench/agt_map_workbench/semantic_candidates.py`
- Create: `src/agt_map_workbench/test/test_semantic_candidates.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**
- Consumes: `NavigationMapResult`, `NavigationStructureResult`, `CorridorRefinementResult`, `AgriculturalAisleGraph`.
- Produces: `SemanticCandidate`, `derive_row_semantic_candidates(...)`, `derive_access_lane_semantic_candidates(...)`, `promote_semantic_candidate(...)`.

- [ ] **Step 1: Write failing stable-ID tests**

Build a synthetic two-row/one-aisle fixture using real `NavigationMapResult` plus a `SimpleNamespace` structure/corridor with row direction, accepted centers, and row-centerline mask. Assert:

```python
rows = derive_row_semantic_candidates(navigation, structure, corridor)
lanes = derive_access_lane_semantic_candidates(aisle_graph)
assert [c.feature.id for c in rows] == ["auto_row_01", "auto_row_02"]
assert [c.feature.id for c in lanes] == ["auto_lane_001"]
assert rows[0].feature.feature_type == "row_centerline"
assert lanes[0].feature.feature_type == "access_lane"
```

- [ ] **Step 2: Run and verify import failure**

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

Use `AgriculturalAisleGraph.centerline_xyz` for access lanes. For rows, project grid centers onto the existing row direction/perpendicular axes, select `corridor.row_centerline` cells nearest each accepted `v` center, sort by longitudinal `u`, collapse duplicate `u` samples, and emit at least two XY points. Rejected row hypotheses never become candidates.

- [ ] **Step 4: Test provenance**

Assert row candidates use `source_kind="auto_row_detection"`, lane candidates use `source_kind="auto_aisle_detection"`, and edited promotion changes only `authoring_state` while preserving other properties.

- [ ] **Step 5: Register/run/commit**

```cmake
ament_add_pytest_test(test_semantic_candidates test/test_semantic_candidates.py)
```

```bash
PYTHONPATH=src/agt_map_workbench:src/agt_offline_assets:src/agt_ui_bridge \
python3 -m pytest -q src/agt_map_workbench/test/test_semantic_candidates.py

git add src/agt_map_workbench/agt_map_workbench/semantic_candidates.py \
  src/agt_map_workbench/test/test_semantic_candidates.py \
  src/agt_map_workbench/CMakeLists.txt
git commit -m "feat(workbench): derive semantic row and lane candidates"
```

---

### Task 3: Add `语义与任务` Tab on the Existing Workbench Canvas

**Files:**
- Create: `src/agt_map_workbench/agt_map_workbench/semantic_authoring_panel.py`
- Create: `src/agt_map_workbench/test/test_semantic_authoring_panel.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/paper1_workbench.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 1 state, Task 2 candidates, existing `PointCloudView.mapClicked` path.
- Produces: fourth control tab, object tree, map-frame semantic drawing/editing, candidate accept/reject UI.

- [ ] **Step 1: Write failing tab/tool test**

```python
window = Paper1MapWorkbenchWindow()
labels = [window._control_tabs.tabText(i) for i in range(window._control_tabs.count())]
assert labels == ["点云编辑", "坐标系标定", "导航地图", "语义与任务"]
assert window._semantic_panel.tool_keys() == {
    "field_boundary", "exclusion_zone", "row_centerline", "access_lane",
    "entry_pose", "work_direction", "headland_zone", "keepout_zone",
}
```

- [ ] **Step 2: Run and verify failure**

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_offline_assets:src/agt_ui_bridge:src/agt_coverage_planning \
python3 -m pytest -q src/agt_map_workbench/test/test_semantic_authoring_panel.py
```

- [ ] **Step 3: Implement panel signals and object tree**

`SemanticAuthoringPanel(QWidget)` exposes:

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

Tree columns are `[ID, 类型, 名称, 来源]`. The panel owns no `QGraphicsScene`, PGM, or file writer.

- [ ] **Step 4: Route map clicks through the existing canvas**

In `Paper1MapWorkbenchWindow._on_map_clicked`:

```python
if self._interaction_mode and self._interaction_mode.startswith("semantic:"):
    self._append_semantic_vertex(x, y)
    return
super()._on_map_clicked(x, y)
```

Polygon tools require >=3 points; row/access/work direction require >=2; `entry_pose` uses first click as position and second as yaw direction. Geometry is stored directly in map-frame meters.

- [ ] **Step 5: Use `SemanticScene` for edits**

ID/name/geometry changes call `replace_by_id`; delete/undo/redo call existing scene methods. Semantic editing never writes or mutates `NavigationMapResult.occupancy` or PGM bytes.

- [ ] **Step 6: Add candidate accept/reject**

Accept adds `promote_semantic_candidate(...)`; reject stores candidate ID in an in-memory rejected set cleared when structure is recomputed.

- [ ] **Step 7: Register/run/commit**

```cmake
ament_add_pytest_test(test_semantic_authoring_panel test/test_semantic_authoring_panel.py)
```

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_offline_assets:src/agt_ui_bridge:src/agt_coverage_planning \
python3 -m pytest -q \
  src/agt_map_workbench/test/test_semantic_authoring.py \
  src/agt_map_workbench/test/test_semantic_candidates.py \
  src/agt_map_workbench/test/test_semantic_authoring_panel.py

git add src/agt_map_workbench/agt_map_workbench/semantic_authoring_panel.py \
  src/agt_map_workbench/agt_map_workbench/paper1_workbench.py \
  src/agt_map_workbench/test/test_semantic_authoring_panel.py \
  src/agt_map_workbench/CMakeLists.txt
git commit -m "feat(workbench): integrate semantic task authoring tab"
```

---

### Task 4: Bind Semantic Payload to Exact Accepted Map

**Files:**
- Modify: `src/agt_ui_bridge/agt_ui_bridge/semantic_rasterizer.py`
- Create: `src/agt_map_workbench/agt_map_workbench/semantic_revision.py`
- Create: `src/agt_map_workbench/test/test_semantic_revision.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**
- Consumes: `SemanticAuthoringState`, accepted Nav2 YAML, navigation footprint/clearance context.
- Produces: `write_semantic_revision_payload(...)`; `semantic/semantic_map.geojson`, `semantic/coverage.yaml`, `semantic/keepout_mask.pgm`, `semantic/keepout_mask.yaml`, `validation/semantic_validation.json`.

- [ ] **Step 1: Write exact-hash binding test**

Create a valid semantic state containing required field/exclusion/entry/direction features and a real tiny accepted Nav2 YAML/PGM fixture. After writing:

```python
coverage = yaml.safe_load((root / "semantic/coverage.yaml").read_text())
assert coverage["base_map_sha256"] == sha256_file(
    root / "accepted/navigation_map.yaml"
)
assert (root / "semantic" / coverage["base_map"]).resolve() == (
    root / "accepted/navigation_map.yaml"
).resolve()
```

- [ ] **Step 2: Write semantic-error hard-fail test**

Remove `entry_pose`; assert `SemanticFileError` mentions `missing_feature_type` and no `semantic/semantic_map.geojson` is left behind.

- [ ] **Step 3: Run and verify failure**

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_ui_bridge:src/agt_offline_assets \
python3 -m pytest -q src/agt_map_workbench/test/test_semantic_revision.py
```

- [ ] **Step 4: Add deterministic keepout Nav2 mask writer**

Add `import numpy as np` and:

```python
def write_keepout_nav2_mask(mask, map_geometry, pgm_path, yaml_path):
    grid = np.asarray(mask.data, dtype=np.int16).reshape(mask.height, mask.width)
    image = np.where(grid >= 100, 0, 254).astype(np.uint8)
    image = np.flipud(image)
    header = f"P5\n# AGT semantic keepout mask\n{mask.width} {mask.height}\n255\n"
    Path(pgm_path).write_bytes(header.encode("ascii") + image.tobytes(order="C"))
```

Write YAML with the accepted map geometry's exact resolution/origin and standard trinary thresholds.

- [ ] **Step 5: Rebind copied coverage before authoritative validation**

```python
coverage = deepcopy(state.coverage)
coverage.base_map = os.path.relpath(
    accepted_map_yaml, root / "semantic"
)
coverage.base_map_sha256 = sha256_file(accepted_map_yaml)
```

Build `MapGeometry.from_nav2_yaml(accepted_map_yaml)` and a full `ValidationContext`, call `validate_task`, reject any ERROR, then call existing `save_semantic_task`.

- [ ] **Step 6: Write keepout separately and prove base PGM unchanged**

Rasterize with existing `rasterize_keepout_mask`; write `keepout_mask.*`; compare accepted PGM bytes before/after and assert equality.

- [ ] **Step 7: Write validation JSON**

Serialize `status: PASS` plus all issue fields. Never create PASS when any ERROR exists.

- [ ] **Step 8: Register/run/commit**

```cmake
ament_add_pytest_test(test_semantic_revision test/test_semantic_revision.py)
```

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_ui_bridge:src/agt_offline_assets \
python3 -m pytest -q src/agt_map_workbench/test/test_semantic_revision.py

git add src/agt_ui_bridge/agt_ui_bridge/semantic_rasterizer.py \
  src/agt_map_workbench/agt_map_workbench/semantic_revision.py \
  src/agt_map_workbench/test/test_semantic_revision.py \
  src/agt_map_workbench/CMakeLists.txt
git commit -m "feat(map): bind semantic task to accepted navigation map"
```

---

### Task 5: One Atomic Unified Map Revision

**Files:**
- Create: `src/agt_map_workbench/agt_map_workbench/unified_map_revision.py`
- Create: `src/agt_map_workbench/test/test_unified_map_revision.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/paper1_workbench.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**
- Consumes: Plan 1 `write_structure_aware_navigation_payload`, Plan 1 navigation QA, Task 4 semantic writer, Workbench Recipe/map_frame/site_boundary writers.
- Produces: `export_unified_map_revision(parent_dir, revision_name, inputs) -> Path`, `_last_frozen_revision`.

- [ ] **Step 1: Write complete-contract test**

Use synthetic Plan 1 data plus valid semantic state and assert final revision contains Evidence, Generated, Accepted, semantic payload, `authoring/site_boundary.yaml`, `validation/navigation_validation.json`, and `validation/semantic_validation.json`.

- [ ] **Step 2: Write three rollback tests**

Monkeypatch navigation payload, semantic payload, and authoring writer separately to raise `RuntimeError("injected")`; each case must leave neither final revision nor hidden staging directories.

- [ ] **Step 3: Run and verify failure**

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_ui_bridge:src/agt_offline_assets \
python3 -m pytest -q src/agt_map_workbench/test/test_unified_map_revision.py
```

- [ ] **Step 4: Implement staging order**

```text
1. create hidden sibling staging directory
2. write_structure_aware_navigation_payload(stage, ...)
3. write authoring/processing_recipe.yaml when current Recipe exists
4. write authoring/map_frame.yaml when current calibration exists
5. write authoring/site_boundary.yaml (required)
6. write_semantic_revision_payload(stage, accepted_map_yaml=stage/accepted/navigation_map.yaml, ...)
7. write validation/navigation_validation.json
8. reload semantic/coverage.yaml and re-check accepted YAML SHA256 binding
9. os.replace(stage, final_revision)
```

No timestamp is added to authority/derivation files.

- [ ] **Step 5: Replace Paper Workbench navigation-only export**

Formal freeze refuses missing formal navigation state, navigation QA FAIL, semantic ERROR, or missing Site Boundary. On success:

```python
self._last_frozen_revision = output.resolve()
```

- [ ] **Step 6: Test relative path survives staging rename**

After final rename, resolve `semantic/coverage.yaml.base_map` relative to the final semantic directory and assert it points to final `accepted/navigation_map.yaml`.

- [ ] **Step 7: Register/run/commit**

```cmake
ament_add_pytest_test(test_unified_map_revision test/test_unified_map_revision.py)
```

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_ui_bridge:src/agt_offline_assets:src/agt_coverage_planning \
python3 -m pytest -q \
  src/agt_map_workbench/test/test_semantic_revision.py \
  src/agt_map_workbench/test/test_unified_map_revision.py

git add src/agt_map_workbench/agt_map_workbench/unified_map_revision.py \
  src/agt_map_workbench/agt_map_workbench/paper1_workbench.py \
  src/agt_map_workbench/test/test_unified_map_revision.py \
  src/agt_map_workbench/CMakeLists.txt
git commit -m "feat(workbench): freeze one unified map revision"
```

---

### Task 6: Bundle Frozen Semantic Assets Without Regeneration

**Files:**
- Modify: `src/agt_map_workbench/agt_map_workbench/resource_bundle.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/paper1_workbench.py`
- Create: `src/agt_map_workbench/test/test_semantic_resource_bundle.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**
- Consumes: `_last_frozen_revision`.
- Produces ResourceBundle key `semantic_task`.

- [ ] **Step 1: Write recommended-selection test**

```python
selection = ResourceBundleSelection.recommended(
    available_groups={"navigation_revision", "semantic_task", "site_boundary"}
)
assert "semantic_task" in selection.selected_groups
```

- [ ] **Step 2: Write byte-identical frozen-copy test**

Create a frozen revision containing the five semantic/validation files; bundle it; assert destination bytes equal source bytes. Monkeypatch `save_semantic_task` or semantic writer to fail if called, proving bundle export does not regenerate.

- [ ] **Step 3: Run and verify failure**

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_ui_bridge:src/agt_offline_assets \
python3 -m pytest -q src/agt_map_workbench/test/test_semantic_resource_bundle.py
```

- [ ] **Step 4: Add semantic group**

Add `semantic_task` to `RECOMMENDED_GROUPS`. It is available only when `_last_frozen_revision` contains:

```text
semantic/semantic_map.geojson
semantic/coverage.yaml
semantic/keepout_mask.pgm
semantic/keepout_mask.yaml
validation/semantic_validation.json
```

Writer uses `shutil.copy2` only.

- [ ] **Step 5: Register/run/commit**

```cmake
ament_add_pytest_test(test_semantic_resource_bundle test/test_semantic_resource_bundle.py)
```

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_ui_bridge:src/agt_offline_assets \
python3 -m pytest -q \
  src/agt_map_workbench/test/test_resource_bundle.py \
  src/agt_map_workbench/test/test_paper1_resource_bundle_integration.py \
  src/agt_map_workbench/test/test_semantic_resource_bundle.py

git add src/agt_map_workbench/agt_map_workbench/resource_bundle.py \
  src/agt_map_workbench/agt_map_workbench/paper1_workbench.py \
  src/agt_map_workbench/test/test_semantic_resource_bundle.py \
  src/agt_map_workbench/CMakeLists.txt
git commit -m "feat(workbench): bundle frozen semantic task assets"
```

---

### Task 7: Retire Standalone Raster Authority

**Files:**
- Modify: `src/agt_ui_bridge/scripts/semantic_editor_qt5.py`
- Create: `src/agt_ui_bridge/test/test_semantic_editor_no_raster_authority.py`
- Modify: `src/agt_ui_bridge/CMakeLists.txt`

**Interfaces:**
- Consumes existing standalone editor.
- Produces semantic-only legacy editor with no PGM painting or in-place base-map save.

- [ ] **Step 1: Write failing toolbar/save-authority test**

Offscreen create a temporary Nav2 map, instantiate `SemanticEditorWindow(map_path=...)`, and assert:

```python
assert "map_occupied" not in window._tool_actions
assert "map_free" not in window._tool_actions
assert "map_unknown" not in window._tool_actions
before = pgm_path.read_bytes()
assert window._save_map_in_place() is False
assert pgm_path.read_bytes() == before
```

- [ ] **Step 2: Run and verify current behavior fails**

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_ui_bridge \
python3 -m pytest -q src/agt_ui_bridge/test/test_semantic_editor_no_raster_authority.py
```

- [ ] **Step 3: Remove raster toolbar/actions**

Do not create `map_occupied`, `map_free`, `map_unknown`, brush/line raster actions, or map brush-size actions in `_build_ui()`.

- [ ] **Step 4: Fail closed for stale raster calls**

`apply_map_brush`, `apply_map_line`, `begin_map_edit`, `commit_map_edit`, and `_save_map_in_place` must return without mutation. `_save_map_in_place()` returns `False` and shows the message:

```text
正式 Navigation Map 栅格编辑已迁移到 AGT Map Workbench；此编辑器仅保留语义任务编辑。
```

`save()` no longer calls `_save_map_in_place()`.

- [ ] **Step 5: Register and run all UI-bridge semantic regressions**

```cmake
ament_add_pytest_test(
  test_semantic_editor_no_raster_authority
  test/test_semantic_editor_no_raster_authority.py
)
```

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_ui_bridge \
python3 -m pytest -q \
  src/agt_ui_bridge/test/test_semantic_io.py \
  src/agt_ui_bridge/test/test_semantic_validation.py \
  src/agt_ui_bridge/test/test_semantic_scene.py \
  src/agt_ui_bridge/test/test_semantic_editor.py \
  src/agt_ui_bridge/test/test_semantic_rasterizer.py \
  src/agt_ui_bridge/test/test_semantic_editor_no_raster_authority.py
```

- [ ] **Step 6: Commit**

```bash
git add src/agt_ui_bridge/scripts/semantic_editor_qt5.py \
  src/agt_ui_bridge/test/test_semantic_editor_no_raster_authority.py \
  src/agt_ui_bridge/CMakeLists.txt
git commit -m "refactor(ui): retire standalone raster map authority"
```

---

### Task 8: End-to-End Unified Acceptance and Documentation

**Files:**
- Create: `src/agt_map_workbench/test/test_unified_semantic_workbench.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`
- Modify after verification: `src/agt_map_workbench/README.md`

**Interfaces:**
- Consumes all Plan 1 + Plan 2 interfaces.
- Produces final verified authoring workflow.

- [ ] **Step 1: Write end-to-end synthetic test**

Inject Ground Evidence containing one aisle UNKNOWN gap, valid row/corridor structure, Site Boundary, one formal override, and required semantic features. Freeze to a temporary parent and assert:

```python
revision = window._freeze_unified_revision_to(test_parent, "revision_001")
generated = load_navigation_grid(revision / "generated/navigation_map.yaml")
coverage = yaml.safe_load((revision / "semantic/coverage.yaml").read_text())
assert generated.occupancy[gap_row, gap_col] == FREE
assert coverage["base_map_sha256"] == sha256_file(
    revision / "accepted/navigation_map.yaml"
)
assert json.loads(
    (revision / "validation/navigation_validation.json").read_text()
)["status"] == "PASS"
assert json.loads(
    (revision / "validation/semantic_validation.json").read_text()
)["status"] == "PASS"
```

Define `gap_row` and `gap_col` as constants in the test fixture; do not leave them implicit.

- [ ] **Step 2: Register/run complete Workbench suite**

```cmake
ament_add_pytest_test(
  test_unified_semantic_workbench
  test/test_unified_semantic_workbench.py
)
```

```bash
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_offline_assets:src/agt_ui_bridge:src/agt_coverage_planning \
python3 -m pytest -q src/agt_map_workbench/test
```

- [ ] **Step 3: Run offline + route benchmark regressions**

```bash
PYTHONPATH=src/agt_offline_assets \
python3 -m pytest -q src/agt_offline_assets/test

MPLBACKEND=Agg \
PYTHONPATH=src/agt_route_benchmark:src/agt_offline_assets:src/agt_coverage_planning:src/agt_ui_bridge \
python3 -m pytest -q src/agt_route_benchmark/test
```

- [ ] **Step 4: Run compile gate**

```bash
python3 -m compileall -q \
  src/agt_offline_assets/agt_offline_assets \
  src/agt_map_workbench/agt_map_workbench \
  src/agt_ui_bridge/agt_ui_bridge \
  src/agt_ui_bridge/scripts \
  src/agt_route_benchmark/agt_route_benchmark
```

- [ ] **Step 5: Perform real greenhouse unified session**

Use final revision name `greenhouse_01_map_revision_unified_001` if Plan 1 did not already consume it; otherwise use `greenhouse_01_map_revision_unified_002`. Record the chosen exact name in the handoff before running replay QA.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR="$PWD/runtime/log/unified_map_authoring" \
ros2 run agt_map_workbench agt_map_workbench_paper1
```

Manual checklist:

```text
[ ] Review Ground Evidence, row band, aisle geometry, Generated PGM.
[ ] Site Boundary is frozen separately from field_boundary.
[ ] Add only evidence-backed formal raster overrides.
[ ] `语义与任务` is available without another top-level editor.
[ ] Accept/edit/reject automatic row/lane candidates.
[ ] Draw required field/exclusion/entry/direction semantics.
[ ] Freeze one immutable unified revision.
[ ] coverage.yaml hash equals Accepted map YAML hash.
[ ] Resource bundle copies frozen semantic assets.
[ ] Standalone semantic editor cannot paint/save PGM changes.
```

- [ ] **Step 6: Run replay QA using the exact chosen revision name**

If the real revision is `_002`, run:

```bash
REV="$PWD/runtime/maps/greenhouse_01/derivation/greenhouse_01_map_revision_unified_002"
ros2 run agt_route_benchmark route_benchmark_map_quality.py \
  --generated-map-yaml "$REV/generated/navigation_map.yaml" \
  --accepted-map-yaml "$REV/accepted/navigation_map.yaml" \
  --derivation-yaml "$REV/derivation.yaml" \
  --output-dir "$REV/validation/replay_qa"
```

If Plan 1 did not publish `_001` and this task used `_001`, substitute only the literal final suffix from `_002` to `_001` in the four paths. Required values remain:

```text
accepted_matches_replay == true
unexplained_changed_cell_count == 0
```

- [ ] **Step 7: Update README after verification**

Document the canonical flow exactly as:

```text
PCD -> Ground Evidence -> Agricultural Structure -> Structure-Aware Generated PGM
-> evidence-backed Accepted PGM -> Semantic Task -> Unified Immutable Revision
```

Mark standalone semantic editor as legacy semantic-only, not formal raster authority.

- [ ] **Step 8: Commit final integration/docs**

```bash
git add src/agt_map_workbench/test/test_unified_semantic_workbench.py \
  src/agt_map_workbench/CMakeLists.txt \
  src/agt_map_workbench/README.md
git commit -m "docs(map): finalize unified authoring workflow"
```

---

## Final Completion Gate

```text
Plan 1 real-map gate PASS
all offline-assets tests PASS
all Workbench tests PASS
all route-benchmark tests PASS
all UI-bridge semantic tests PASS
compileall PASS
Structure-Aware Generated PGM reviewed on real greenhouse data
Accepted replay exact
semantic validation PASS
coverage.yaml Accepted-map SHA256 binding PASS
one unified immutable revision exported
resource bundle copies frozen semantics without regeneration
standalone semantic editor cannot mutate PGM
```

Only after this gate should Paper I downstream planner tuning or experiment-matrix work resume.
