# V25-12G-A1 Vehicle-Feasible Segment Extraction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extract every deterministic contiguous preview-footprint-free aisle segment of useful length, serialize it as `vehicle_feasible_segments.yaml`, expose a review-only Workbench layer, and run the first real-greenhouse segment-recovery experiment without changing V25-12F safety semantics or the existing `VehicleSafeLanePlan` public behavior.

**Architecture:** Extract the current sample-level aisle footprint feasibility logic into one internal trace primitive shared by the stable Vehicle-Safe Lane reducer and the new A1 segment extractor. Build an independent `VehicleFeasibleSegmentPlan` that preserves all maximal feasible runs, separates active segments from short diagnostic fragments, classifies headland endpoints, and writes deterministic YAML. A standalone acceptance harness evaluates frozen greenhouse assets, while Workbench only loads and renders the resulting YAML.

**Tech Stack:** Python 3.10, NumPy, PyYAML, ROS 2 Humble, ament_cmake_pytest, PyQt5, existing AGT Navigation Grid / Site Boundary / vehicle-profile / aisle-graph contracts.

## Global Constraints

- Work only on `feat/v25-12g-maximum-feasible-coverage`, based on frozen V25-12F checkpoint `bfcf6bac77955f015c6088088f1b6de95116f608`.
- Scope is V25-12G-A1 only. Do not implement A2 service states, A3 connector integration, A4 optimization, interactive route editing, Hybrid A*, MILP, or RL.
- `VehicleSafeLanePlan`, `derive_vehicle_safe_lane_plan`, and `vehicle_safe_lane_plan_to_dict` must remain backward compatible.
- Structural Aisle Graph, Navigation Grid, Site Boundary, vehicle profile, and V25-12F assets remain immutable truth inputs.
- Site Boundary stays a hard footprint-level invariant: touch/cross is conflict.
- Navigation Grid occupancy, semantic NO_GO, structural width, Ackermann preview footprint, and minimum turning truth are not weakened.
- Active-segment threshold is exactly `VehicleSafeLaneConfig.minimum_contiguous_span_m`; the default remains `1.0 m`.
- Every maximal feasible run is preserved; only runs below the configured threshold become rejected diagnostics.
- A1 does not infer cross-aisle connectivity. Endpoint classification is metadata only.
- A1 may write only `vehicle_feasible_segments.yaml`; it must never overwrite canonical or V25-12F map/traversability/route assets.
- Workbench visualization is review-only and must not regenerate, edit, or drag segment truth.
- Do not use `git reset --hard`, `git clean`, or disturb unrelated local changes, especially `tools/rosbag_sensor_trimmer`.
- Do not claim tests, package verification, or real-data results passed until operator-machine output proves them.

---

## File Structure

```text
src/agt_offline_assets/agt_offline_assets/
  vehicle_lane_feasibility.py
  vehicle_safe_lane.py
  vehicle_feasible_segment.py
  __init__.py

src/agt_offline_assets/test/
  test_vehicle_lane_feasibility.py
  test_vehicle_feasible_segment.py
  test_vehicle_safe_lane.py

src/agt_offline_assets/CMakeLists.txt

tools/v25_12g_a1_acceptance.py
tests/test_v25_12g_a1_contract.py

src/agt_map_workbench/agt_map_workbench/
  vehicle_feasible_segment_preview.py
  review_workbench.py

src/agt_map_workbench/test/
  test_vehicle_feasible_segment_preview.py
  test_vehicle_feasible_segment_workbench.py

src/agt_map_workbench/CMakeLists.txt

docs/v2.5/V25_12G_A1_CURRENT_STATE.md
docs/v2.5/V25_12G_A1_REAL_DATA_2026-08-15.md
```

Do not add graph-search, connector-search, or learned-planner modules in A1.

---

### Task 1: Extract the shared vehicle-lane feasibility trace

**Files:**
- Create: `src/agt_offline_assets/agt_offline_assets/vehicle_lane_feasibility.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/vehicle_safe_lane.py`
- Create: `src/agt_offline_assets/test/test_vehicle_lane_feasibility.py`
- Modify: `src/agt_offline_assets/test/test_vehicle_safe_lane.py`

**Interfaces:**

The new internal trace type has exactly these fields:

```text
VehicleLaneFeasibilityTrace
  aisle_id: str
  structural_length_m: float
  distances_m: tuple[float]
  selected_points: tuple[point-or-None]
  selected_offsets_m: tuple[float-or-None]
  allowed_lateral_shift_m: float
  site_boundary_rejected_pose_count: int
  site_boundary_limited_sample_count: int
  grid_rejected_pose_count: int
  structural_width_blocked_reason: str-or-None
```

The exact callable signature is:

```text
derive_vehicle_lane_feasibility_trace(
  aisle: AislePrimitive,
  navigation: NavigationGridEvidence,
  vehicle: CanonicalVehicleProfile,
  config: VehicleSafeLaneConfig,
  row_direction_xy: keyword-only tuple[float, float],
  site_boundary: keyword-only SiteBoundary-or-None = None,
) -> VehicleLaneFeasibilityTrace
```

`VehicleSafeLaneConfig` moves verbatim into `vehicle_lane_feasibility.py`. `vehicle_safe_lane.py` imports it so existing callers importing the class from `vehicle_safe_lane` continue to work.

- [ ] **Step 1: Write the failing all-free trace test**

Create the same `_vehicle`, `_aisle`, `_navigation`, and `_config` fixtures already used by `test_vehicle_safe_lane.py`, then add:

```python
def test_trace_preserves_every_sample_on_all_free_aisle():
    trace = derive_vehicle_lane_feasibility_trace(
        _aisle(),
        _navigation(),
        _vehicle(),
        _config(),
        row_direction_xy=(1.0, 0.0),
    )
    assert trace.aisle_id == "aisle_001"
    assert trace.structural_width_blocked_reason is None
    assert math.isclose(trace.structural_length_m, 4.0)
    assert len(trace.distances_m) == len(trace.selected_points)
    assert len(trace.selected_points) == len(trace.selected_offsets_m)
    assert all(point is not None for point in trace.selected_points)
    assert all(offset == 0.0 for offset in trace.selected_offsets_m)
```

- [ ] **Step 2: Run the new test and verify RED**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_vehicle_lane_feasibility.py
```

Expected: import failure because the new trace module/API does not exist.

- [ ] **Step 3: Move the config and existing sample-selection logic into `vehicle_lane_feasibility.py`**

Move without semantic edits:

```text
VehicleSafeLaneConfig validation/defaults
_resample_polyline
_offset_candidates
row-direction normalization
preview footprint generation
Navigation Grid footprint check
Site Boundary footprint check
sample-by-sample lateral continuity
```

Preserve the existing candidate ranking exactly:

```python
feasible.sort(key=lambda item: (item[0], abs(item[1]), item[1]))
_, chosen_offset, point = feasible[0]
```

When width is structurally impossible, return a trace whose `selected_points` are all `None` and whose reason is exactly:

```python
structural_width_blocked_reason = (
    "STRUCTURAL_WIDTH_BELOW_PREVIEW_VEHICLE_WIDTH: "
    f"aisle={aisle.geometric_width_m:.3f} m "
    f"required={required_lateral_width:.3f} m"
)
```

- [ ] **Step 4: Refactor `_derive_one_lane` into a reducer over the trace**

`vehicle_safe_lane.py` must call the shared trace and then retain the existing longest-run behavior:

```text
trace -> maximal contiguous feasible runs -> choose longest with existing tie-break -> existing READY/PARTIAL/NO status logic
```

Do not change public statuses, reason strings, diagnostic counters, dataclass fields, or YAML payload structure.

- [ ] **Step 5: Add a stable serialization regression**

Add:

```python
def test_vehicle_safe_lane_serialization_contract_stays_stable():
    plan = derive_vehicle_safe_lane_plan(
        _graph(width=1.60),
        _navigation(center_obstacle=True),
        _vehicle(),
        _config(),
    )
    payload = vehicle_safe_lane_plan_to_dict(plan)
    assert payload["schema"] == "agt_vehicle_safe_aisle_lane/v1"
    assert payload["summary"] == {"ready": 1, "partial": 0, "unavailable": 0}
    assert payload["aisles"][0]["status"] == "VEHICLE_SAFE_LANE_READY"
    assert "segments" not in payload["aisles"][0]
```

- [ ] **Step 6: Run focused stable-lane tests and verify GREEN**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_lane_feasibility.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane_width_gate.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane_diagnostics.py
```

- [ ] **Step 7: Commit Task 1**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/vehicle_lane_feasibility.py \
  src/agt_offline_assets/agt_offline_assets/vehicle_safe_lane.py \
  src/agt_offline_assets/test/test_vehicle_lane_feasibility.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane.py
git commit -m "refactor(v25-12g): expose shared vehicle lane feasibility trace"
```

---

### Task 2: Implement `VehicleFeasibleSegmentPlan`

**Files:**
- Create: `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_segment.py`
- Create: `src/agt_offline_assets/test/test_vehicle_feasible_segment.py`

**Interfaces:**

Freeze these names:

```text
VEHICLE_FEASIBLE_SEGMENT_SCHEMA = agt_vehicle_feasible_segment_plan/v1
LOW_U_HEADLAND
HIGH_U_HEADLAND
INTERIOR_BLOCKED_END
ACTIVE_COVERAGE_SEGMENT
BELOW_MINIMUM_USEFUL_SEGMENT_LENGTH
```

Dataclasses:

```text
VehicleFeasibleSegment
  segment_id
  aisle_id
  ordinal_in_aisle
  start_distance_m
  end_distance_m
  length_m
  coverage_fraction_of_aisle
  low_endpoint_type
  high_endpoint_type
  centerline_xyz
  lateral_offsets_m
  maximum_used_lateral_shift_m
  low_endpoint_pose
  high_endpoint_pose
  status
  reason

RejectedFeasibleFragment
  fragment_id
  aisle_id
  start_distance_m
  end_distance_m
  length_m
  reason

AisleFeasibleSegmentResult
  aisle_id
  structural_length_m
  active_segments
  rejected_fragments
  raw_feasible_fragment_count
  allowed_lateral_shift_m
  site_boundary_rejected_pose_count
  site_boundary_limited_sample_count
  grid_rejected_pose_count
  reason

VehicleFeasibleSegmentPlan
  frame_id
  platform_id
  platform_profile_sha256
  row_direction_xy
  aisles
  source
  schema
  status
```

Exact derivation signature:

```text
derive_vehicle_feasible_segment_plan(
  graph: AgriculturalAisleGraph,
  navigation: NavigationGridEvidence,
  vehicle: CanonicalVehicleProfile,
  config: VehicleSafeLaneConfig-or-None = None,
  site_boundary: keyword-only SiteBoundary-or-None = None,
  source: keyword-only Mapping-or-None = None,
) -> VehicleFeasibleSegmentPlan
```

- [ ] **Step 1: Create deterministic synthetic helpers and write the two-active-segment RED test**

In `test_vehicle_feasible_segment.py`, use a 5.0 m straight aisle and a 0.05 m Navigation Grid. Provide helper `_navigation_with_blocked_x_ranges(ranges)` that marks the full vehicle-accessible lateral band OCCUPIED for each `(x0, x1)` interval. Then add:

```python
def test_two_disjoint_useful_runs_are_both_emitted():
    plan = derive_vehicle_feasible_segment_plan(
        _graph(length_m=5.0),
        _navigation_with_blocked_x_ranges(((2.20, 2.80),)),
        _vehicle(),
        _config(minimum_contiguous_span_m=1.0),
    )
    aisle = plan.aisles[0]
    assert len(aisle.active_segments) == 2
    assert [segment.segment_id for segment in aisle.active_segments] == [
        "aisle_001.segment_001",
        "aisle_001.segment_002",
    ]
    assert aisle.active_segments[0].end_distance_m < aisle.active_segments[1].start_distance_m
    assert all(segment.length_m >= 1.0 for segment in aisle.active_segments)
```

- [ ] **Step 2: Run that single test and verify RED**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_segment.py::test_two_disjoint_useful_runs_are_both_emitted
```

Expected: import/API failure.

- [ ] **Step 3: Implement maximal contiguous-run extraction**

Use increasing trace index order. A run starts on the first non-`None` point and ends immediately before the next `None`. Run length is:

```python
length_m = float(trace.distances_m[end_index] - trace.distances_m[start_index])
```

Active iff:

```python
length_m + 1.0e-9 >= cfg.minimum_contiguous_span_m
```

Never merge across an infeasible sample.

- [ ] **Step 4: Implement deterministic identifiers and endpoint poses**

Assign IDs in longitudinal order:

```text
aisle_001.segment_001
aisle_001.segment_002
aisle_001.fragment_001
```

Use canonical row yaw:

```python
yaw = math.atan2(float(direction[1]), float(direction[0]))
```

Store endpoint poses as `(x, y, z, yaw)` using the first/last selected point of the run.

- [ ] **Step 5: Implement frozen endpoint classification**

Only the first active segment can be `LOW_U_HEADLAND`, and only when:

```python
first.start_distance_m <= cfg.maximum_endpoint_retreat_m + 1.0e-9
```

Only the last active segment can be `HIGH_U_HEADLAND`, and only when:

```python
structural_length_m - last.end_distance_m <= cfg.maximum_endpoint_retreat_m + 1.0e-9
```

All other endpoints are `INTERIOR_BLOCKED_END`.

- [ ] **Step 6: Add the short-fragment diagnostic test**

Use blocked ranges that create two long runs plus one short run below 1.0 m. Assert:

```python
assert len(aisle.active_segments) == 2
assert len(aisle.rejected_fragments) == 1
assert aisle.rejected_fragments[0].fragment_id == "aisle_001.fragment_001"
assert aisle.rejected_fragments[0].length_m < 1.0
assert aisle.rejected_fragments[0].reason == "BELOW_MINIMUM_USEFUL_SEGMENT_LENGTH"
```

- [ ] **Step 7: Add fail-closed and safety cases**

Implement separate tests for:

```text
frame mismatch -> ValueError
vehicle not planning-preview-ready -> ValueError
invalid Site Boundary -> validation error
invalid aisle polyline -> ValueError
structural width below preview width -> zero active segments with width reason
Site Boundary clipping -> emitted poses remain strictly inside
Site Boundary fully blocks aisle -> zero active segments
all poses blocked -> zero active segments
lateral-shift feasible run -> emitted offsets preserve shared trace selection
same input twice -> equal plan objects and equal IDs
```

- [ ] **Step 8: Run A1 core tests and verify GREEN**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_lane_feasibility.py \
  src/agt_offline_assets/test/test_vehicle_feasible_segment.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane.py
```

- [ ] **Step 9: Commit Task 2**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_segment.py \
  src/agt_offline_assets/test/test_vehicle_feasible_segment.py
git commit -m "feat(v25-12g): extract all vehicle-feasible aisle segments"
```

---

### Task 3: Add deterministic YAML I/O and package exports

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_segment.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/__init__.py`
- Modify: `src/agt_offline_assets/test/test_vehicle_feasible_segment.py`
- Modify: `src/agt_offline_assets/CMakeLists.txt`

**Interfaces:**

Freeze these callables:

```text
vehicle_feasible_segment_plan_to_dict(plan) -> dict
write_vehicle_feasible_segment_plan(plan, path, overwrite=False) -> Path
load_vehicle_feasible_segment_plan(path) -> VehicleFeasibleSegmentPlan
```

- [ ] **Step 1: Write the deterministic YAML round-trip RED test**

```python
def test_segment_yaml_round_trip_is_deterministic(tmp_path):
    plan = derive_vehicle_feasible_segment_plan(
        _graph(length_m=5.0),
        _navigation_with_blocked_x_ranges(((2.20, 2.80),)),
        _vehicle(),
        _config(minimum_contiguous_span_m=1.0),
    )
    first = write_vehicle_feasible_segment_plan(plan, tmp_path / "first.yaml")
    second = write_vehicle_feasible_segment_plan(plan, tmp_path / "second.yaml")
    assert first.read_bytes() == second.read_bytes()
    assert load_vehicle_feasible_segment_plan(first) == plan
```

Expected before implementation: import failure for writer/loader.

- [ ] **Step 2: Implement deterministic dictionary serialization**

Top-level order:

```text
schema
status
frame_id
platform_id
platform_profile_sha256
row_direction_xy
source
configuration
summary
aisles
```

`configuration` records all eight `VehicleSafeLaneConfig` fields. `summary` records:

```text
aisle_count
active_segment_count
rejected_fragment_count
active_segment_length_m
structural_length_m
segment_recovery_fraction
```

Serialize with:

```python
yaml.safe_dump(payload, sort_keys=False, allow_unicode=True)
```

- [ ] **Step 3: Implement fail-safe write semantics**

```python
if output.exists() and not overwrite:
    raise FileExistsError(f"vehicle-feasible segment asset already exists: {output}")
```

The module has no API that writes any other filename.

- [ ] **Step 4: Implement strict loader validation**

Reject wrong schema, missing required keys, non-monotonic segment ordering, invalid endpoint type strings, segment length below the serialized configured threshold, duplicate IDs, and malformed point/pose shapes with `ValueError`.

- [ ] **Step 5: Export the public A1 contract from `__init__.py`**

Export exactly:

```text
VEHICLE_FEASIBLE_SEGMENT_SCHEMA
LOW_U_HEADLAND
HIGH_U_HEADLAND
INTERIOR_BLOCKED_END
VehicleFeasibleSegment
RejectedFeasibleFragment
AisleFeasibleSegmentResult
VehicleFeasibleSegmentPlan
derive_vehicle_feasible_segment_plan
vehicle_feasible_segment_plan_to_dict
write_vehicle_feasible_segment_plan
load_vehicle_feasible_segment_plan
```

Keep the trace type internal; do not add it to top-level `__all__`.

- [ ] **Step 6: Register offline tests in CMake**

```cmake
ament_add_pytest_test(test_vehicle_lane_feasibility test/test_vehicle_lane_feasibility.py)
ament_add_pytest_test(test_vehicle_feasible_segment test/test_vehicle_feasible_segment.py)
```

- [ ] **Step 7: Run YAML + stable-lane tests**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_segment.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane_diagnostics.py
```

- [ ] **Step 8: Commit Task 3**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_segment.py \
  src/agt_offline_assets/agt_offline_assets/__init__.py \
  src/agt_offline_assets/test/test_vehicle_feasible_segment.py \
  src/agt_offline_assets/CMakeLists.txt
git commit -m "feat(v25-12g): serialize vehicle-feasible segment plan"
```

---

### Task 4: Add the repeatable A1 real-data acceptance harness

**Files:**
- Create: `tools/v25_12g_a1_acceptance.py`
- Create: `tests/test_v25_12g_a1_contract.py`

**Interfaces:**

Freeze:

```python
REPORT_SCHEMA = "agt_v25_12g_a1_acceptance_report/v1"
VALIDATION_SCOPE = "A1_SEGMENT_EXTRACTION_DIAGNOSTIC_NOT_ROUTE_READY"
DIAGNOSTIC_AISLE_IDS = (
    "aisle_016",
    "aisle_017",
    "aisle_018",
    "aisle_019",
    "aisle_020",
)
```

CLI arguments:

```text
--run-dir
--vehicle-profile
--navigation-map, default navigation_map.yaml
--write-segments
--overwrite-segments
--pretty
```

- [ ] **Step 1: Write contract RED tests**

Assert the constants above and assert parser default `navigation_map.yaml`. Add a temp-run test proving the harness refuses to overwrite an existing `vehicle_feasible_segments.yaml` unless `--overwrite-segments` is supplied.

- [ ] **Step 2: Implement strict frozen-input loading**

Require:

```text
run-dir/aisle_graph.yaml
run-dir/site_boundary.yaml
run-dir/<selected navigation filename>
vehicle-profile path
```

Use existing loaders. Reject frame mismatch explicitly.

- [ ] **Step 3: Freeze the A1 config used by the experiment**

```python
cfg = VehicleSafeLaneConfig(
    sample_spacing_m=0.10,
    lateral_search_step_m=0.05,
    maximum_lateral_shift_m=0.50,
    maximum_lateral_step_m=0.15,
    preview_footprint_padding_m=0.05,
    minimum_lane_coverage_fraction=0.70,
    maximum_endpoint_retreat_m=2.00,
    minimum_contiguous_span_m=1.00,
)
```

- [ ] **Step 4: Derive stable longest-only lanes and A1 segments from identical inputs**

```python
lane_plan = derive_vehicle_safe_lane_plan(
    graph,
    navigation,
    vehicle,
    cfg,
    site_boundary=boundary,
    source={"acceptance_stage": "v25_12g_a1"},
)
segment_plan = derive_vehicle_feasible_segment_plan(
    graph,
    navigation,
    vehicle,
    cfg,
    site_boundary=boundary,
    source={"acceptance_stage": "v25_12g_a1"},
)
```

Reject duplicate/missing aisle IDs before comparison.

- [ ] **Step 5: Emit exact frozen metrics**

Per aisle:

```text
aisle_id
structural_length_m
raw_feasible_fragment_count
active_segment_count
rejected_fragment_count
active_segment_length_m
current_longest_only_selected_span_m
recoverable_additional_length_m
active_segment_ids
endpoint_classifications
```

Compute:

```python
recoverable_additional_length_m = max(
    0.0,
    active_segment_length_m - current_longest_only_selected_span_m,
)
segment_recovery_fraction = (
    0.0
    if structural_length_total <= 1.0e-12
    else active_segment_length_total / structural_length_total
)
```

- [ ] **Step 6: Make JSON output deterministic and segment writing narrow**

Compact output uses `sort_keys=True` with compact separators; pretty output uses `indent=2, sort_keys=True`. `--write-segments` may call only `write_vehicle_feasible_segment_plan` on `run_dir / "vehicle_feasible_segments.yaml"`.

- [ ] **Step 7: Add canonical-file sentinel regression**

In a temp run, pre-create byte sentinels for:

```text
navigation_map.yaml
navigation_map.pgm
derivation.yaml
navigation_map_12f.yaml
navigation_map_12f.pgm
navigation_map_12f_derivation.yaml
traversability_evidence.yaml
```

Run the harness in write mode and assert every sentinel remains byte-identical.

- [ ] **Step 8: Run harness contract tests**

```bash
python3 -m pytest -q \
  tests/test_v25_12g_a1_contract.py \
  src/agt_offline_assets/test/test_vehicle_feasible_segment.py
```

- [ ] **Step 9: Commit Task 4**

```bash
git add tools/v25_12g_a1_acceptance.py tests/test_v25_12g_a1_contract.py
git commit -m "test(v25-12g): add repeatable A1 segment recovery harness"
```

---

### Task 5: Add review-only Workbench visualization

**Files:**
- Create: `src/agt_map_workbench/agt_map_workbench/vehicle_feasible_segment_preview.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/review_workbench.py`
- Create: `src/agt_map_workbench/test/test_vehicle_feasible_segment_preview.py`
- Create: `src/agt_map_workbench/test/test_vehicle_feasible_segment_workbench.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**

Freeze a small renderer API:

```text
VehicleFeasibleSegmentPreview(scene)
set_plan(plan-or-None)
set_visible(bool)
clear()
```

Layer key:

```text
vehicle_feasible_segments
```

- [ ] **Step 1: Write the pure preview RED test**

Create a plan containing two active segments and one rejected short fragment. Assert after `set_plan` that the preview owns two active path items and one diagnostic fragment item, and that no graphics item has `ItemIsMovable` or `ItemIsSelectable` enabled.

- [ ] **Step 2: Implement the renderer with map coordinates `(x, -y)`**

Use `QPainterPath` + `QGraphicsPathItem` for active segment polylines and a visually distinct dashed path or endpoint marks for rejected fragments. Keep every item in the preview manager so `clear()` removes all items deterministically. Set a fixed review z-order above raster overlays and below authoring handles.

- [ ] **Step 3: Add sibling YAML lifecycle to `ReviewMapWorkbenchWindow`**

On source/PCD change, clear the previous plan and look only for:

```text
vehicle_feasible_segments.yaml
```

Missing file means unavailable layer with no exception. Invalid file records a local error and keeps other Workbench/Route Debug functions usable.

- [ ] **Step 4: Add the nav-layer item and visibility dispatch**

Add once:

```python
self._nav_layer.addItem(
    "12G-A1 Vehicle-Feasible Segments",
    "vehicle_feasible_segments",
)
```

Selecting this layer hides the raster preview and shows the segment preview only when `_nav_overlay_visible` is enabled and a valid plan is loaded. Selecting any other layer hides A1 segment graphics before delegating to existing dispatch.

No A1 Workbench method may call `derive_vehicle_feasible_segment_plan`.

- [ ] **Step 5: Add Workbench integration tests**

Cover:

```text
missing sibling YAML -> no exception
valid sibling YAML -> loads and renders
invalid sibling YAML -> A1 unavailable, legacy layers still dispatch
source change -> prior graphics cleared
switching layer -> A1 graphics hidden
all A1 graphics -> non-editable/non-draggable
```

- [ ] **Step 6: Register Workbench tests**

```cmake
ament_add_pytest_test(test_vehicle_feasible_segment_preview test/test_vehicle_feasible_segment_preview.py)
ament_add_pytest_test(test_vehicle_feasible_segment_workbench test/test_vehicle_feasible_segment_workbench.py)
```

- [ ] **Step 7: Run GUI tests offscreen**

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/agt_map_workbench/test/test_vehicle_feasible_segment_preview.py \
  src/agt_map_workbench/test/test_vehicle_feasible_segment_workbench.py \
  src/agt_map_workbench/test/test_site_boundary_workbench.py \
  src/agt_map_workbench/test/test_route_debug_view.py \
  src/agt_map_workbench/test/test_route_debug_panel.py
```

- [ ] **Step 8: Commit Task 5**

```bash
git add \
  src/agt_map_workbench/agt_map_workbench/vehicle_feasible_segment_preview.py \
  src/agt_map_workbench/agt_map_workbench/review_workbench.py \
  src/agt_map_workbench/test/test_vehicle_feasible_segment_preview.py \
  src/agt_map_workbench/test/test_vehicle_feasible_segment_workbench.py \
  src/agt_map_workbench/CMakeLists.txt
git commit -m "feat(v25-12g): visualize A1 vehicle-feasible segments"
```

---

### Task 6: Freeze A1 documentation and focused automated verification

**Files:**
- Create: `docs/v2.5/V25_12G_A1_CURRENT_STATE.md`
- Modify only if a contract assertion needs correction: `tests/test_v25_12g_a1_contract.py`

- [ ] **Step 1: Create the pre-verification checkpoint**

Initial status must be exactly:

```text
CODE IMPLEMENTED / FOCUSED AUTOMATED VERIFICATION PENDING / REAL-DATA A1 EXPERIMENT PENDING
```

Document A1 schema, 1.0 m default threshold, endpoint semantics, output filename, no-canonical-overwrite invariant, and the real-data command from Task 7.

- [ ] **Step 2: Run focused offline verification**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_lane_feasibility.py \
  src/agt_offline_assets/test/test_vehicle_feasible_segment.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane_width_gate.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane_diagnostics.py \
  src/agt_offline_assets/test/test_site_boundary.py \
  tests/test_v25_12g_a1_contract.py
```

- [ ] **Step 3: Run focused Workbench verification**

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/agt_map_workbench/test/test_vehicle_feasible_segment_preview.py \
  src/agt_map_workbench/test/test_vehicle_feasible_segment_workbench.py \
  src/agt_map_workbench/test/test_site_boundary_workbench.py \
  src/agt_map_workbench/test/test_route_debug_view.py \
  src/agt_map_workbench/test/test_route_debug_panel.py
```

- [ ] **Step 4: Copy the exact final pytest summary lines into the checkpoint**

Do not predict counts. Paste the literal final summary lines produced by Step 2 and Step 3 into fenced text blocks and mark focused verification PASS only when both commands exit zero.

- [ ] **Step 5: Commit Task 6**

```bash
git add docs/v2.5/V25_12G_A1_CURRENT_STATE.md tests/test_v25_12g_a1_contract.py
git commit -m "docs(v25-12g): freeze A1 extraction contract"
```

---

### Task 7: Run the first real-greenhouse experiment and scoped package gate

**Files:**
- Runtime output: `/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run/vehicle_feasible_segments.yaml`
- Create after observation: `docs/v2.5/V25_12G_A1_REAL_DATA_2026-08-15.md`
- Modify after observation: `docs/v2.5/V25_12G_A1_CURRENT_STATE.md`

- [ ] **Step 1: Switch/pull without cleaning unrelated local state**

```bash
cd ~/agt_navigation_v2
git switch feat/v25-12g-maximum-feasible-coverage
git pull --ff-only
git status --short
```

Do not reset or clean `tools/rosbag_sensor_trimmer` or any other unrelated local change.

- [ ] **Step 2: Build only touched packages**

```bash
source /opt/ros/humble/setup.bash
colcon build \
  --packages-select agt_offline_assets agt_map_workbench \
  --symlink-install \
  --event-handlers console_direct+
source install/setup.bash
```

- [ ] **Step 3: Run the canonical-map A1 experiment read-only**

```bash
python3 tools/v25_12g_a1_acceptance.py \
  --run-dir /home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run \
  --vehicle-profile /home/yangxuan/agt_navigation_v2/profiles/platforms/mk_mini.yaml \
  --navigation-map navigation_map.yaml \
  --pretty | tee /tmp/v25_12g_a1_current.json
```

Inspect all aisles, with explicit attention to `aisle_016` through `aisle_020`.

- [ ] **Step 4: If the report is structurally valid, write the A1 review asset**

```bash
python3 tools/v25_12g_a1_acceptance.py \
  --run-dir /home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run \
  --vehicle-profile /home/yangxuan/agt_navigation_v2/profiles/platforms/mk_mini.yaml \
  --navigation-map navigation_map.yaml \
  --write-segments \
  --pretty | tee /tmp/v25_12g_a1_current_written.json
```

If `vehicle_feasible_segments.yaml` already exists, inspect it first. Replace it only by intentionally rerunning with `--overwrite-segments`.

- [ ] **Step 5: Review `12G-A1 Vehicle-Feasible Segments` in Workbench**

Verify visually:

```text
active segments follow vehicle-safe aisle geometry
blocked gaps remain gaps
short rejected fragments are visually diagnostic, not executable
A1 graphics cannot be dragged or edited
switching away restores normal 12F/Route Debug behavior
```

- [ ] **Step 6: Run scoped package tests**

```bash
colcon test \
  --packages-select agt_offline_assets agt_map_workbench \
  --event-handlers console_direct+

colcon test-result --test-result-base build/agt_offline_assets --all
colcon test-result --test-result-base build/agt_map_workbench --all
```

Acceptance requires zero errors/failures in both selected package result sets. Ignore historical failures from unrelated packages.

- [ ] **Step 7: Write the real-data observation document**

Record:

```text
navigation source
vehicle profile hash
A1 config
aggregate structural length
aggregate active segment length
segment_recovery_fraction
aggregate current longest-only selected span
aggregate recoverable_additional_length_m
per-aisle aisle_016..aisle_020 metrics
all aisles with more than one active segment
rejected short-fragment counts
Workbench visual review notes
scoped package verification summaries
```

Use exactly one evidence-matching conclusion:

```text
REAL-DATA ADDITIONAL SEGMENTS OBSERVED / NOT ROUTE READY
```

or:

```text
REAL-DATA NO ADDITIONAL >=1.0M SEGMENTS OBSERVED / SOFTWARE CONTRACT VERIFIED / NOT ROUTE READY
```

- [ ] **Step 8: Freeze A1 status and commit docs**

When focused and scoped package tests are both green, set:

```text
CODE IMPLEMENTED / FOCUSED AUTOMATED VERIFICATION PASS / PACKAGE-LEVEL TEST PASS / REAL-DATA A1 OBSERVED / NOT ROUTE READY
```

Then:

```bash
git add \
  docs/v2.5/V25_12G_A1_CURRENT_STATE.md \
  docs/v2.5/V25_12G_A1_REAL_DATA_2026-08-15.md
git commit -m "docs(v25-12g): record first A1 real-data segment experiment"
```

Do not commit the runtime YAML by default.

---

## Final A1 Acceptance Checklist

```text
[ ] VehicleSafeLanePlan public behavior unchanged
[ ] emitted active segments are maximal contiguous feasible runs
[ ] all emitted poses satisfy the same footprint/Grid/Site-Boundary gates
[ ] structural-width-blocked aisles emit no active segment
[ ] active threshold equals configured minimum_contiguous_span_m; default 1.0 m
[ ] short runs remain rejected diagnostics
[ ] segment and fragment IDs deterministic
[ ] active segments non-overlapping and longitudinally ordered
[ ] endpoint classification follows maximum_endpoint_retreat_m
[ ] repeated derivation and YAML serialization deterministic
[ ] vehicle_feasible_segments.yaml is the only A1 runtime asset
[ ] canonical and V25-12F assets unchanged
[ ] Workbench A1 layer is review-only and non-editable
[ ] focused offline tests pass
[ ] focused Workbench tests pass
[ ] agt_offline_assets scoped package tests pass
[ ] agt_map_workbench scoped package tests pass
[ ] real-data report truthfully states whether additional >=1.0 m segments exist
[ ] A1 remains NOT ROUTE READY
```

Only after every applicable item is evidence-backed should V25-12G move to a separate A2 design/plan for Segment Service States and Reachability Graph.
