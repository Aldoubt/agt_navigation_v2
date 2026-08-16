# V25-12G-A1 Workbench Visualization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a review-only Workbench layer that renders the explicit geometry of active `VehicleFeasibleSegment`s while preserving rejected short fragments only as non-spatial diagnostics.

**Architecture:** A focused `VehicleFeasibleSegmentPreview` owns all A1 graphics and consumes only a loaded `VehicleFeasibleSegmentPlan`; it never derives segments or reconstructs rejected-fragment geometry. `ReviewMapWorkbenchWindow` owns sibling-YAML loading and layer visibility dispatch, while all existing navigation, Site Boundary, 12F, 3D Review, and Route Debug behavior remains delegated to the existing code paths.

**Tech Stack:** Python 3.10, ROS 2 Humble/ament, PyQt5 `QGraphicsScene` / `QPainterPath` / `QGraphicsPathItem`, pytest with `QT_QPA_PLATFORM=offscreen`, `agt_offline_assets.vehicle_feasible_segment` loader and dataclasses.

## Global Constraints

- Governing design amendment: `docs/superpowers/specs/2026-08-16-v25-12g-a1-workbench-visualization-amendment-design.md`.
- Keep `agt_vehicle_feasible_segment_plan/v1` unchanged.
- Render only geometry explicitly present in active `VehicleFeasibleSegment.centerline_xyz`.
- Do not synthesize or reconstruct rejected-fragment XY/XYZ geometry.
- Do not load `aisle_graph.yaml` for Task 5 visualization.
- No Workbench method may call `derive_vehicle_feasible_segment_plan`.
- All A1 graphics are review-only: `ItemIsMovable` and `ItemIsSelectable` must remain disabled.
- Scene coordinates are `(x, -y)`.
- A1 graphics use fixed z-order `12.0`: above raster navigation overlays (`5.0`) and below Site Boundary/authoring graphics (`18.0` or higher).
- Missing sibling `vehicle_feasible_segments.yaml` is not an error.
- Invalid sibling YAML disables only the A1 layer and must not break legacy Workbench dispatch.
- Source changes clear stale A1 graphics before accepting another sibling asset.
- Do not touch `tools/rosbag_sensor_trimmer` or unrelated local files.

---

### Task 1: Implement the pure review-only segment renderer

**Files:**
- Create: `src/agt_map_workbench/agt_map_workbench/vehicle_feasible_segment_preview.py`
- Create: `src/agt_map_workbench/test/test_vehicle_feasible_segment_preview.py`

**Interfaces:**
- Consumes: `VehicleFeasibleSegmentPlan` from `agt_offline_assets.vehicle_feasible_segment`.
- Produces:
  - `VehicleFeasibleSegmentPreview(scene)`
  - `set_plan(plan: VehicleFeasibleSegmentPlan | None) -> None`
  - `set_visible(visible: bool) -> None`
  - `clear() -> None`
  - read-only properties `active_item_count`, `rejected_fragment_count`, `rejected_fragment_total_length_m` for deterministic diagnostics/tests.

- [ ] **Step 1: Write the renderer RED test**

Create a plan fixture with two active segments and one rejected short fragment. The active segment centerlines must contain non-zero Y so the `(x, -y)` transform is observable.

```python
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtWidgets import QApplication, QGraphicsItem, QGraphicsScene

from agt_offline_assets.vehicle_feasible_segment import (
    AisleFeasibleSegmentResult,
    RejectedFeasibleFragment,
    VehicleFeasibleSegment,
    VehicleFeasibleSegmentPlan,
)
from agt_map_workbench.vehicle_feasible_segment_preview import (
    VehicleFeasibleSegmentPreview,
)


def _plan():
    first = VehicleFeasibleSegment(
        segment_id="aisle_001.segment_001",
        aisle_id="aisle_001",
        ordinal_in_aisle=1,
        start_distance_m=0.0,
        end_distance_m=1.5,
        length_m=1.5,
        coverage_fraction_of_aisle=0.30,
        low_endpoint_type="LOW_U_HEADLAND",
        high_endpoint_type="INTERIOR_BLOCKED_END",
        centerline_xyz=((0.0, 1.0, 0.0), (1.5, 1.5, 0.0)),
        lateral_offsets_m=(0.0, 0.0),
        maximum_used_lateral_shift_m=0.0,
        low_endpoint_pose=(0.0, 1.0, 0.0, 0.0),
        high_endpoint_pose=(1.5, 1.5, 0.0, 0.0),
    )
    second = VehicleFeasibleSegment(
        segment_id="aisle_001.segment_002",
        aisle_id="aisle_001",
        ordinal_in_aisle=2,
        start_distance_m=3.0,
        end_distance_m=5.0,
        length_m=2.0,
        coverage_fraction_of_aisle=0.40,
        low_endpoint_type="INTERIOR_BLOCKED_END",
        high_endpoint_type="HIGH_U_HEADLAND",
        centerline_xyz=((3.0, -0.5, 0.0), (5.0, -1.0, 0.0)),
        lateral_offsets_m=(0.0, 0.0),
        maximum_used_lateral_shift_m=0.0,
        low_endpoint_pose=(3.0, -0.5, 0.0, 0.0),
        high_endpoint_pose=(5.0, -1.0, 0.0, 0.0),
    )
    fragment = RejectedFeasibleFragment(
        fragment_id="aisle_001.fragment_001",
        aisle_id="aisle_001",
        start_distance_m=2.0,
        end_distance_m=2.5,
        length_m=0.5,
    )
    aisle = AisleFeasibleSegmentResult(
        aisle_id="aisle_001",
        structural_length_m=5.0,
        active_segments=(first, second),
        rejected_fragments=(fragment,),
        raw_feasible_fragment_count=3,
        allowed_lateral_shift_m=0.5,
        site_boundary_rejected_pose_count=0,
        site_boundary_limited_sample_count=0,
        grid_rejected_pose_count=1,
        reason="fixture",
    )
    return VehicleFeasibleSegmentPlan(
        frame_id="map",
        platform_id="mk_mini",
        platform_profile_sha256="abc",
        row_direction_xy=(1.0, 0.0),
        aisles=(aisle,),
    )


def test_preview_renders_only_explicit_active_geometry_and_keeps_fragment_diagnostics():
    QApplication.instance() or QApplication([])
    scene = QGraphicsScene()
    preview = VehicleFeasibleSegmentPreview(scene)

    preview.set_plan(_plan())

    assert preview.active_item_count == 2
    assert preview.rejected_fragment_count == 1
    assert preview.rejected_fragment_total_length_m == 0.5
    assert len(scene.items()) == 2

    for item in scene.items():
        assert not bool(item.flags() & QGraphicsItem.ItemIsMovable)
        assert not bool(item.flags() & QGraphicsItem.ItemIsSelectable)
        assert item.zValue() == 12.0

    path = next(item.path() for item in scene.items() if item.path().elementAt(0).x == 0.0)
    assert path.elementAt(0).y == -1.0
    assert path.elementAt(1).y == -1.5

    preview.set_visible(False)
    assert not any(item.isVisible() for item in scene.items())

    preview.clear()
    assert preview.active_item_count == 0
    assert preview.rejected_fragment_count == 0
    assert preview.rejected_fragment_total_length_m == 0.0
    assert scene.items() == []
```

- [ ] **Step 2: Run the single renderer test and verify RED**

Run:

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/agt_map_workbench/test/test_vehicle_feasible_segment_preview.py::test_preview_renders_only_explicit_active_geometry_and_keeps_fragment_diagnostics
```

Expected: import failure because `agt_map_workbench.vehicle_feasible_segment_preview` does not exist yet.

- [ ] **Step 3: Implement the minimal renderer**

Implementation rules:

```python
class VehicleFeasibleSegmentPreview:
    REVIEW_Z = 12.0

    def __init__(self, scene):
        self._scene = scene
        self._active_items = []
        self._visible = True
        self._rejected_fragment_count = 0
        self._rejected_fragment_total_length_m = 0.0

    @property
    def active_item_count(self):
        return len(self._active_items)

    @property
    def rejected_fragment_count(self):
        return self._rejected_fragment_count

    @property
    def rejected_fragment_total_length_m(self):
        return self._rejected_fragment_total_length_m
```

For each active segment, create exactly one `QPainterPath` by `moveTo(first_x, -first_y)` and `lineTo(x, -y)` for remaining points. Add one `QGraphicsPathItem` to the supplied scene, use a cosmetic `QPen`, set z-value `12.0`, explicitly clear `ItemIsMovable` and `ItemIsSelectable`, and apply current visibility. Rejected fragments only update the two diagnostic counters.

`set_plan(None)` must behave as `clear()`.

`clear()` removes every owned path item from the scene and resets all counters.

- [ ] **Step 4: Run the renderer test and verify GREEN**

Run the command from Step 2. Expected: `1 passed`.

- [ ] **Step 5: Commit Task 1**

```bash
git add \
  src/agt_map_workbench/agt_map_workbench/vehicle_feasible_segment_preview.py \
  src/agt_map_workbench/test/test_vehicle_feasible_segment_preview.py
git commit -m "feat(v25-12g): render A1 vehicle-feasible segments"
```

---

### Task 2: Integrate sibling YAML lifecycle and navigation-layer dispatch

**Files:**
- Modify: `src/agt_map_workbench/agt_map_workbench/review_workbench.py`
- Create: `src/agt_map_workbench/test/test_vehicle_feasible_segment_workbench.py`

**Interfaces:**
- Consumes: `load_vehicle_feasible_segment_plan(path)` and `VehicleFeasibleSegmentPreview` from Task 1.
- Produces Workbench state:
  - `_vehicle_feasible_segment_plan`
  - `_vehicle_feasible_segment_last_error`
  - `_vehicle_feasible_segment_preview`
  - `_load_vehicle_feasible_segment_sibling()`
  - nav-layer key `vehicle_feasible_segments` with label `12G-A1 Vehicle-Feasible Segments`.

- [ ] **Step 1: Write sibling-lifecycle RED tests**

Use a temporary source file such as `tmp_path / "run" / "map.pcd"` and write a valid `vehicle_feasible_segments.yaml` through the existing deterministic writer. Cover these behaviors with real loader data, not mocks:

```text
missing sibling YAML -> no exception, plan None, zero active graphics
valid sibling YAML -> plan loads, active_item_count matches file, rejected diagnostics retained
invalid sibling YAML -> plan None, local error non-empty, existing non-A1 layer dispatch remains callable
source change -> previous graphics cleared before missing/next sibling result
```

Do not assert any rejected-fragment graphics.

- [ ] **Step 2: Run the lifecycle tests and verify RED**

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/agt_map_workbench/test/test_vehicle_feasible_segment_workbench.py
```

Expected: missing Workbench A1 state/loader/layer integration.

- [ ] **Step 3: Add Workbench A1 state and sibling loader**

In `ReviewMapWorkbenchWindow.__init__`, initialize the plan/error/preview state before or immediately after `super().__init__()` as required by scene availability. Construct `VehicleFeasibleSegmentPreview(self._scene)` only after the base scene exists.

Add `12G-A1 Vehicle-Feasible Segments` exactly once to `_nav_layer`.

Implement `_load_vehicle_feasible_segment_sibling()` with this order:

```text
preview.clear()
plan = None
last_error = ""
if source_path is None: return
candidate = source_path.parent / "vehicle_feasible_segments.yaml"
if candidate missing: return
try load_vehicle_feasible_segment_plan(candidate)
except Exception as exc: last_error = str(exc); return
plan = loaded
preview.set_plan(loaded)
preview.set_visible(False)
```

No modal error dialog is required for invalid A1 YAML; keep failure local so legacy Workbench remains usable.

- [ ] **Step 4: Add source-change lifecycle**

In `_open_pcd`, when `_source_path` changes, call `_load_vehicle_feasible_segment_sibling()` in addition to the existing Site Boundary sibling load. Do not alter existing 3D or corridor reset semantics.

- [ ] **Step 5: Add navigation visibility dispatch**

At the start of `_update_navigation_overlay()`:

```text
if selected layer == vehicle_feasible_segments:
    clear/hide raster NavigationPreviewItem for this dispatch
    preview visible iff nav overlay checkbox checked AND valid A1 plan loaded
    return
else:
    A1 preview hidden
    continue existing 12F / base dispatch unchanged
```

When route-debug mode becomes active, hide the A1 preview together with other review overlays. When leaving route-debug mode, call normal navigation dispatch rather than blindly forcing A1 visible.

- [ ] **Step 6: Add layer-switch RED/GREEN coverage**

Extend the integration tests to assert:

```text
select vehicle_feasible_segments + overlay enabled + valid plan -> active graphics visible
select another nav layer -> A1 graphics hidden
select A1 layer with overlay disabled -> A1 graphics hidden
all A1 graphics remain non-movable/non-selectable after Workbench integration
```

- [ ] **Step 7: Run integration tests and verify GREEN**

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/agt_map_workbench/test/test_vehicle_feasible_segment_preview.py \
  src/agt_map_workbench/test/test_vehicle_feasible_segment_workbench.py \
  src/agt_map_workbench/test/test_site_boundary_workbench.py
```

Expected: all pass.

- [ ] **Step 8: Commit Task 2**

```bash
git add \
  src/agt_map_workbench/agt_map_workbench/review_workbench.py \
  src/agt_map_workbench/test/test_vehicle_feasible_segment_workbench.py
git commit -m "feat(v25-12g): load A1 segments in Workbench"
```

---

### Task 3: Register tests and freeze focused Workbench verification

**Files:**
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**
- Produces registered ament tests:
  - `test_vehicle_feasible_segment_preview`
  - `test_vehicle_feasible_segment_workbench`.

- [ ] **Step 1: Add CMake test registration**

Inside `if(BUILD_TESTING)` add:

```cmake
ament_add_pytest_test(
  test_vehicle_feasible_segment_preview
  test/test_vehicle_feasible_segment_preview.py
)
ament_add_pytest_test(
  test_vehicle_feasible_segment_workbench
  test/test_vehicle_feasible_segment_workbench.py
)
```

- [ ] **Step 2: Rebuild the Workbench package**

```bash
bash --noprofile --norc -c '
  source /opt/ros/humble/setup.bash
  cd ~/agt_navigation_v2
  colcon build --packages-select agt_map_workbench --symlink-install
'
```

- [ ] **Step 3: Run the full focused offscreen Workbench regression set**

```bash
source /opt/ros/humble/setup.bash
source ~/agt_navigation_v2/install/setup.bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/agt_map_workbench/test/test_vehicle_feasible_segment_preview.py \
  src/agt_map_workbench/test/test_vehicle_feasible_segment_workbench.py \
  src/agt_map_workbench/test/test_site_boundary_workbench.py \
  src/agt_map_workbench/test/test_route_debug_view.py \
  src/agt_map_workbench/test/test_route_debug_panel.py
```

- [ ] **Step 4: Run package-level registered tests**

```bash
source /opt/ros/humble/setup.bash
source ~/agt_navigation_v2/install/setup.bash
colcon test --packages-select agt_map_workbench --event-handlers console_direct+
colcon test-result --verbose
```

Expected: no failed registered test.

- [ ] **Step 5: Commit Task 3**

```bash
git add src/agt_map_workbench/CMakeLists.txt
git commit -m "test(v25-12g): register A1 Workbench visualization tests"
```

---

## Plan self-review

- Spec coverage: active geometry, rejected non-spatial diagnostics, sibling lifecycle, invalid/missing behavior, visibility dispatch, source clearing, non-editability, and focused tests are all mapped to explicit tasks.
- Placeholder scan: no `TBD`, `TODO`, or unspecified implementation step remains.
- Type consistency: renderer method names match the approved amendment; Workbench consumes the existing strict loader and never derives A1 plans.
- Scope check: no A2/A3/A4 behavior, route editing, fragment reconstruction, or A1 schema change is included.
