# V25-12G-A1 Vehicle-Feasible Segment Extraction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extract every deterministic contiguous preview-footprint-free aisle segment of useful length, serialize it as `vehicle_feasible_segments.yaml`, expose a review-only Workbench layer, and run the first real-greenhouse segment-recovery experiment without changing V25-12F safety semantics or the existing `VehicleSafeLanePlan` public behavior.

**Architecture:** First extract the existing per-sample vehicle-feasibility logic into one internal trace primitive shared by the stable Vehicle-Safe Lane layer and the new segment extractor. Then build an independent `VehicleFeasibleSegmentPlan` contract that preserves all maximal feasible runs, separates active `>= minimum_contiguous_span_m` segments from rejected short fragments, classifies headland endpoints, and writes deterministic YAML. A standalone A1 acceptance harness evaluates the frozen greenhouse assets, while Workbench only loads and renders the resulting YAML; it does not edit or regenerate segment truth.

**Tech Stack:** Python 3.10, NumPy, PyYAML, ROS 2 Humble/ament_cmake_pytest, PyQt5, existing AGT offline asset contracts and Navigation Grid / Site Boundary / canonical vehicle-profile primitives.

## Global Constraints

- Work on `feat/v25-12g-maximum-feasible-coverage`, based on frozen V25-12F checkpoint `bfcf6bac77955f015c6088088f1b6de95116f608`.
- Scope is **V25-12G-A1 only**. Do not implement A2 service states, A3 connector integration, A4 global optimization, interactive route editing, Hybrid A*, MILP, or RL.
- `VehicleSafeLanePlan` and its serialized public behavior must remain backward compatible.
- Structural Aisle Graph, Navigation Grid, Site Boundary, canonical vehicle profile, and V25-12F candidate assets are immutable truth inputs during A1 extraction.
- Site Boundary remains a hard footprint-level invariant: touch/cross is conflict; no A1 path may bypass it.
- Semantic NO_GO and Navigation Grid occupancy semantics are not weakened in A1.
- Structural width rejection remains unchanged.
- Active-segment threshold is exactly `VehicleSafeLaneConfig.minimum_contiguous_span_m`; default remains `1.0 m`.
- Every maximal feasible run is preserved; A1 must not keep only the longest run.
- Cross-aisle reachability is not inferred in A1. Endpoint classification is metadata only.
- The writer may create/overwrite only `vehicle_feasible_segments.yaml` when explicitly requested; it must not modify `navigation_map.yaml`, `navigation_map.pgm`, `derivation.yaml`, `navigation_map_12f.*`, `traversability_evidence.*`, lane assets, or route assets.
- Workbench A1 visualization is review-only. It loads the segment YAML and renders it; it does not alter extractor output.
- Do not use `git reset --hard`, `git clean`, or otherwise disturb unrelated local modifications, especially `tools/rosbag_sensor_trimmer`.
- Do not claim local tests or the real-data experiment passed until operator-machine output confirms them.

---

## File Structure

Create or modify only the following A1-focused files unless a failing test proves an additional narrow change is required:

```text
src/agt_offline_assets/agt_offline_assets/
  vehicle_lane_feasibility.py        # shared deterministic sample-level footprint feasibility trace
  vehicle_safe_lane.py               # stable public lane reducer; consumes shared trace
  vehicle_feasible_segment.py        # new A1 segment domain model, derivation, YAML load/write
  __init__.py                         # package exports for the new public A1 contract

src/agt_offline_assets/test/
  test_vehicle_lane_feasibility.py   # trace-level invariants
  test_vehicle_feasible_segment.py   # multi-run extraction, fragments, endpoints, safety
  test_vehicle_safe_lane.py          # unchanged-behavior regression additions only

src/agt_offline_assets/CMakeLists.txt

tools/
  v25_12g_a1_acceptance.py           # frozen real-data experiment / deterministic JSON report

tests/
  test_v25_12g_a1_contract.py        # repo-level artifact/contract guard

src/agt_map_workbench/agt_map_workbench/
  vehicle_feasible_segment_preview.py # review-only QGraphics renderer
  review_workbench.py                 # sibling YAML load + layer lifecycle only

src/agt_map_workbench/test/
  test_vehicle_feasible_segment_preview.py
  test_vehicle_feasible_segment_workbench.py

src/agt_map_workbench/CMakeLists.txt

docs/v2.5/
  V25_12G_A1_CURRENT_STATE.md         # implementation/experiment checkpoint
```

Do not put A1 graph search or connector code into any of these files.

---

### Task 1: Extract a shared deterministic lane-feasibility trace without changing `VehicleSafeLanePlan`

**Files:**
- Create: `src/agt_offline_assets/agt_offline_assets/vehicle_lane_feasibility.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/vehicle_safe_lane.py`
- Create: `src/agt_offline_assets/test/test_vehicle_lane_feasibility.py`
- Modify: `src/agt_offline_assets/test/test_vehicle_safe_lane.py`

**Interfaces:**
- Consumes: `AislePrimitive`, `NavigationGridEvidence`, `CanonicalVehicleProfile`, `SiteBoundary | None`, and the exact existing `VehicleSafeLaneConfig` fields.
- Produces:

```python
@dataclass(frozen=True)
class VehicleLaneFeasibilityTrace:
    aisle_id: str
    structural_length_m: float
    distances_m: tuple[float, ...]
    selected_points: tuple[tuple[float, float, float] | None, ...]
    selected_offsets_m: tuple[float | None, ...]
    allowed_lateral_shift_m: float
    site_boundary_rejected_pose_count: int
    site_boundary_limited_sample_count: int
    grid_rejected_pose_count: int
    structural_width_blocked_reason: str | None


def derive_vehicle_lane_feasibility_trace(
    aisle: AislePrimitive,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    config: VehicleSafeLaneConfig,
    *,
    row_direction_xy: tuple[float, float],
    site_boundary: SiteBoundary | None = None,
) -> VehicleLaneFeasibilityTrace:
    ...
```

- `VehicleSafeLaneConfig` moves unchanged into `vehicle_lane_feasibility.py`, then `vehicle_safe_lane.py` imports and re-exports it naturally so existing imports such as `from agt_offline_assets.vehicle_safe_lane import VehicleSafeLaneConfig` continue working.
- `derive_vehicle_safe_lane_plan(...)` signature and `vehicle_safe_lane_plan_to_dict(...)` output remain unchanged.

- [ ] **Step 1: Add a failing trace test for an all-free aisle**

Create `test_vehicle_lane_feasibility.py` using the same MK-mini/aisle/navigation fixture semantics as `test_vehicle_safe_lane.py` and assert the trace itself, not a lane reduction:

```python
def test_trace_preserves_every_sample_on_all_free_aisle():
    trace = derive_vehicle_lane_feasibility_trace(
        _aisle(),
        _navigation(),
        _vehicle(),
        _config(),
        row_direction_xy=(1.0, 0.0),
    )
    assert trace.structural_width_blocked_reason is None
    assert trace.structural_length_m == 4.0
    assert len(trace.distances_m) == len(trace.selected_points)
    assert len(trace.selected_points) == len(trace.selected_offsets_m)
    assert all(point is not None for point in trace.selected_points)
    assert all(offset == 0.0 for offset in trace.selected_offsets_m)
```

- [ ] **Step 2: Run the new test and confirm the shared trace API is absent**

Run:

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_lane_feasibility.py
```

Expected: FAIL during import because `vehicle_lane_feasibility.py` / `derive_vehicle_lane_feasibility_trace` does not exist yet.

- [ ] **Step 3: Move `VehicleSafeLaneConfig` verbatim and implement the minimal trace primitive**

In `vehicle_lane_feasibility.py`, move the existing config class without changing defaults or validation messages. Move/reuse the existing deterministic helpers needed for:

```text
polyline resampling
offset candidate ordering
row-direction normalization
preview footprint construction
Navigation Grid footprint check
Site Boundary footprint check
sample-by-sample lateral continuity
```

The trace loop must preserve the current selection order exactly:

```python
feasible.sort(key=lambda item: (item[0], abs(item[1]), item[1]))
_, chosen_offset, point = feasible[0]
```

When structural width is below preview width, return a trace with all samples unavailable and set:

```python
structural_width_blocked_reason = (
    "STRUCTURAL_WIDTH_BELOW_PREVIEW_VEHICLE_WIDTH: "
    f"aisle={aisle.geometric_width_m:.3f} m "
    f"required={required_lateral_width:.3f} m"
)
```

Do not silently emit an empty trace for invalid aisle geometry; retain the existing explicit validation behavior from `_resample_polyline`.

- [ ] **Step 4: Refactor `_derive_one_lane` into a pure reducer over the shared trace**

`vehicle_safe_lane.py` should call:

```python
trace = derive_vehicle_lane_feasibility_trace(
    aisle,
    navigation,
    vehicle,
    cfg,
    row_direction_xy=(float(direction[0]), float(direction[1])),
    site_boundary=site_boundary,
)
```

Then reproduce the existing stable behavior from `trace.selected_points`:

```text
find all maximal contiguous feasible runs
choose only the longest run using existing tie-breaks
compute endpoint retreat / coverage / span gates
produce identical status/reason/diagnostic counters
```

Do not change field names, statuses, reasons, or serialization text while extracting the primitive.

- [ ] **Step 5: Add an explicit stable-public-output regression**

Add a test that compares a representative `vehicle_safe_lane_plan_to_dict(...)` payload against exact expected stable fields before any A1 segment logic is used:

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

- [ ] **Step 6: Run focused trace + stable lane tests**

Run:

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_lane_feasibility.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane_width_gate.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane_diagnostics.py
```

Expected: PASS with no changes to existing lane semantics.

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

### Task 2: Implement the independent `VehicleFeasibleSegmentPlan` domain model and segmentation semantics

**Files:**
- Create: `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_segment.py`
- Create: `src/agt_offline_assets/test/test_vehicle_feasible_segment.py`

**Interfaces:**
- Consumes: `VehicleLaneFeasibilityTrace`, `AgriculturalAisleGraph`, `NavigationGridEvidence`, `CanonicalVehicleProfile`, `VehicleSafeLaneConfig`, `SiteBoundary | None`.
- Produces exact A1 public types:

```python
VEHICLE_FEASIBLE_SEGMENT_SCHEMA = "agt_vehicle_feasible_segment_plan/v1"

LOW_U_HEADLAND = "LOW_U_HEADLAND"
HIGH_U_HEADLAND = "HIGH_U_HEADLAND"
INTERIOR_BLOCKED_END = "INTERIOR_BLOCKED_END"

@dataclass(frozen=True)
class VehicleFeasibleSegment:
    segment_id: str
    aisle_id: str
    ordinal_in_aisle: int
    start_distance_m: float
    end_distance_m: float
    length_m: float
    coverage_fraction_of_aisle: float
    low_endpoint_type: str
    high_endpoint_type: str
    centerline_xyz: tuple[tuple[float, float, float], ...]
    lateral_offsets_m: tuple[float, ...]
    maximum_used_lateral_shift_m: float
    low_endpoint_pose: tuple[float, float, float, float]
    high_endpoint_pose: tuple[float, float, float, float]
    status: str = "ACTIVE_COVERAGE_SEGMENT"
    reason: str = "maximal contiguous preview-footprint-free run meets minimum useful length"

@dataclass(frozen=True)
class RejectedFeasibleFragment:
    fragment_id: str
    aisle_id: str
    start_distance_m: float
    end_distance_m: float
    length_m: float
    reason: str = "BELOW_MINIMUM_USEFUL_SEGMENT_LENGTH"

@dataclass(frozen=True)
class AisleFeasibleSegmentResult:
    aisle_id: str
    structural_length_m: float
    active_segments: tuple[VehicleFeasibleSegment, ...]
    rejected_fragments: tuple[RejectedFeasibleFragment, ...]
    raw_feasible_fragment_count: int
    allowed_lateral_shift_m: float
    site_boundary_rejected_pose_count: int
    site_boundary_limited_sample_count: int
    grid_rejected_pose_count: int
    reason: str

@dataclass(frozen=True)
class VehicleFeasibleSegmentPlan:
    frame_id: str
    platform_id: str
    platform_profile_sha256: str
    row_direction_xy: tuple[float, float]
    aisles: tuple[AisleFeasibleSegmentResult, ...]
    source: Mapping[str, Any] = field(default_factory=dict)
    schema: str = VEHICLE_FEASIBLE_SEGMENT_SCHEMA
    status: str = "DRAFT"
```

Main derivation signature:

```python
def derive_vehicle_feasible_segment_plan(
    graph: AgriculturalAisleGraph,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    config: VehicleSafeLaneConfig | None = None,
    *,
    site_boundary: SiteBoundary | None = None,
    source: Mapping[str, Any] | None = None,
) -> VehicleFeasibleSegmentPlan:
    ...
```

- [ ] **Step 1: Write the failing two-active-segment fixture**

Construct a synthetic aisle where a full-width blocked strip breaks the centerline into two `>=1.0 m` feasible runs:

```python
def test_two_disjoint_useful_runs_are_both_emitted():
    plan = derive_vehicle_feasible_segment_plan(
        _graph(length_m=5.0),
        _navigation_with_full_width_block(x0=2.2, x1=2.8),
        _vehicle(),
        _config(minimum_contiguous_span_m=1.0),
    )
    aisle = plan.aisles[0]
    assert len(aisle.active_segments) == 2
    assert aisle.active_segments[0].segment_id == "aisle_001.segment_001"
    assert aisle.active_segments[1].segment_id == "aisle_001.segment_002"
    assert aisle.active_segments[0].end_distance_m < aisle.active_segments[1].start_distance_m
    assert all(segment.length_m >= 1.0 for segment in aisle.active_segments)
```

- [ ] **Step 2: Run and verify it fails because the A1 segment module is absent**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_segment.py::test_two_disjoint_useful_runs_are_both_emitted
```

Expected: FAIL on import/missing API.

- [ ] **Step 3: Implement maximal-run extraction without longest-run reduction**

Use the trace arrays directly:

```python
def _contiguous_runs(points):
    runs = []
    start = None
    for index, point in enumerate(points):
        if point is not None and start is None:
            start = index
        elif point is None and start is not None:
            runs.append((start, index - 1))
            start = None
    if start is not None:
        runs.append((start, len(points) - 1))
    return tuple(runs)
```

For each run `(a, b)`:

```python
length_m = float(trace.distances_m[b] - trace.distances_m[a])
```

Classify exactly by the configured threshold:

```python
if length_m + 1.0e-9 >= cfg.minimum_contiguous_span_m:
    # active segment
else:
    # rejected fragment diagnostic
```

Do not merge runs across one or more infeasible samples.

- [ ] **Step 4: Implement deterministic active/fragment identifiers and longitudinal ordering**

Assign ordinals after scanning in increasing longitudinal distance:

```text
aisle_001.segment_001
aisle_001.segment_002
...
aisle_001.fragment_001
```

Active-segment ordinals and rejected-fragment ordinals are independent deterministic counters.

- [ ] **Step 5: Implement endpoint pose and classification semantics**

Canonical yaw is the graph row direction:

```python
yaw = math.atan2(plan.row_direction_xy[1], plan.row_direction_xy[0])
```

For each active segment, construct:

```python
low_endpoint_pose = (x_low, y_low, z_low, yaw)
high_endpoint_pose = (x_high, y_high, z_high, yaw)
```

Only the first active segment may classify low side as `LOW_U_HEADLAND`, and only if:

```python
start_distance_m <= cfg.maximum_endpoint_retreat_m + 1.0e-9
```

Only the last active segment may classify high side as `HIGH_U_HEADLAND`, and only if:

```python
structural_length_m - end_distance_m <= cfg.maximum_endpoint_retreat_m + 1.0e-9
```

All other endpoints are `INTERIOR_BLOCKED_END`.

- [ ] **Step 6: Add the short-fragment diagnostic regression**

Create a fixture with three feasible runs where the middle run is `<1.0 m`:

```python
def test_short_fragment_is_diagnostic_not_active():
    plan = derive_vehicle_feasible_segment_plan(...)
    aisle = plan.aisles[0]
    assert len(aisle.active_segments) == 2
    assert len(aisle.rejected_fragments) == 1
    fragment = aisle.rejected_fragments[0]
    assert fragment.fragment_id == "aisle_001.fragment_001"
    assert fragment.length_m < 1.0
    assert fragment.reason == "BELOW_MINIMUM_USEFUL_SEGMENT_LENGTH"
```

- [ ] **Step 7: Add fail-closed and safety regressions**

Cover these cases explicitly in `test_vehicle_feasible_segment.py`:

```text
frame mismatch -> ValueError
invalid Site Boundary -> validation error
vehicle not planning_preview_ready -> ValueError
invalid aisle geometry -> ValueError
structural-width-blocked aisle -> zero active segments + width reason
Site-Boundary-clipped endpoint -> clipped segment, no illegal emitted pose
Site-Boundary-fully-blocked aisle -> zero active segments
no feasible poses -> zero active segments, clear reason
lateral-shifted run -> emitted offsets preserve selected safe trace
```

For every emitted active segment in boundary tests, independently validate all centerline poses through the same footprint predicate used by the trace test; do not merely assert a status string.

- [ ] **Step 8: Run the complete A1 synthetic segment suite**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_lane_feasibility.py \
  src/agt_offline_assets/test/test_vehicle_feasible_segment.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane.py
```

Expected: PASS.

- [ ] **Step 9: Commit Task 2**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_segment.py \
  src/agt_offline_assets/test/test_vehicle_feasible_segment.py

git commit -m "feat(v25-12g): extract all vehicle-feasible aisle segments"
```

---

### Task 3: Add deterministic YAML serialization, loading, package exports, and package test registration

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_segment.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/__init__.py`
- Modify: `src/agt_offline_assets/test/test_vehicle_feasible_segment.py`
- Modify: `src/agt_offline_assets/CMakeLists.txt`

**Interfaces:**
- Produces:

```python
def vehicle_feasible_segment_plan_to_dict(
    plan: VehicleFeasibleSegmentPlan,
) -> dict[str, Any]: ...


def write_vehicle_feasible_segment_plan(
    plan: VehicleFeasibleSegmentPlan,
    path: str | Path,
    *,
    overwrite: bool = False,
) -> Path: ...


def load_vehicle_feasible_segment_plan(
    path: str | Path,
) -> VehicleFeasibleSegmentPlan: ...
```

- [ ] **Step 1: Write a failing deterministic round-trip test**

```python
def test_segment_yaml_round_trip_is_deterministic(tmp_path):
    plan = derive_vehicle_feasible_segment_plan(...)
    first = write_vehicle_feasible_segment_plan(plan, tmp_path / "first.yaml")
    second = write_vehicle_feasible_segment_plan(plan, tmp_path / "second.yaml")
    assert first.read_bytes() == second.read_bytes()
    loaded = load_vehicle_feasible_segment_plan(first)
    assert loaded == plan
```

Also assert top-level keys:

```python
payload = yaml.safe_load(first.read_text(encoding="utf-8"))
assert payload["schema"] == "agt_vehicle_feasible_segment_plan/v1"
assert payload["summary"]["active_segment_count"] == len(plan.active_segments)
assert "configuration" in payload
assert "aisles" in payload
```

- [ ] **Step 2: Implement exact deterministic serialization**

Top-level YAML shape must be:

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

`configuration` records every field that affects extraction:

```text
sample_spacing_m
lateral_search_step_m
maximum_lateral_shift_m
maximum_lateral_step_m
preview_footprint_padding_m
minimum_lane_coverage_fraction
maximum_endpoint_retreat_m
minimum_contiguous_span_m
```

`summary` must contain at least:

```text
aisle_count
active_segment_count
rejected_fragment_count
active_segment_length_m
structural_length_m
segment_recovery_fraction
```

Use `yaml.safe_dump(..., sort_keys=False, allow_unicode=True)` and deterministic list ordering already guaranteed by aisle/longitudinal iteration.

- [ ] **Step 3: Make writes fail safe by default**

Implement:

```python
if output.exists() and not overwrite:
    raise FileExistsError(f"vehicle-feasible segment asset already exists: {output}")
```

No function in this module may infer or overwrite canonical map filenames.

- [ ] **Step 4: Add public package exports**

Export from `agt_offline_assets.__init__`:

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

Do not export `VehicleLaneFeasibilityTrace` as a stable top-level API; it is an internal shared implementation contract for A1.

- [ ] **Step 5: Register the two new offline pytest targets**

Add to `src/agt_offline_assets/CMakeLists.txt`:

```cmake
ament_add_pytest_test(test_vehicle_lane_feasibility test/test_vehicle_lane_feasibility.py)
ament_add_pytest_test(test_vehicle_feasible_segment test/test_vehicle_feasible_segment.py)
```

- [ ] **Step 6: Run focused serialization + legacy tests**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_segment.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane_diagnostics.py
```

Expected: PASS.

- [ ] **Step 7: Commit Task 3**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_segment.py \
  src/agt_offline_assets/agt_offline_assets/__init__.py \
  src/agt_offline_assets/test/test_vehicle_feasible_segment.py \
  src/agt_offline_assets/CMakeLists.txt

git commit -m "feat(v25-12g): serialize vehicle-feasible segment plan"
```

---

### Task 4: Add the repeatable V25-12G-A1 real-data experiment harness

**Files:**
- Create: `tools/v25_12g_a1_acceptance.py`
- Create: `tests/test_v25_12g_a1_contract.py`

**Interfaces:**
- CLI:

```bash
python3 tools/v25_12g_a1_acceptance.py \
  --run-dir <run_dir> \
  --vehicle-profile <profile.yaml> \
  [--navigation-map navigation_map.yaml] \
  [--write-segments] \
  [--pretty]
```

- Default `--navigation-map` is `navigation_map.yaml`. Passing `navigation_map_12f.yaml` is explicit and does not imply canonical promotion.
- `--write-segments` writes only `<run-dir>/vehicle_feasible_segments.yaml`, with overwrite refused unless a separate explicit `--overwrite-segments` flag is also present.
- JSON report schema:

```text
agt_v25_12g_a1_acceptance_report/v1
```

- [ ] **Step 1: Write a failing contract test for required names and non-promotion behavior**

`tests/test_v25_12g_a1_contract.py` should import the tool module and assert:

```python
def test_a1_harness_declares_diagnostic_not_route_ready_scope():
    assert REPORT_SCHEMA == "agt_v25_12g_a1_acceptance_report/v1"
    assert VALIDATION_SCOPE == "A1_SEGMENT_EXTRACTION_DIAGNOSTIC_NOT_ROUTE_READY"
```

Also inspect the source text and assert it never writes these canonical filenames:

```python
for forbidden in (
    '"navigation_map.yaml"',
    '"navigation_map.pgm"',
    '"derivation.yaml"',
):
    assert f"write_{forbidden}" not in source
```

The stronger runtime protection is the writer API from Task 3; this source-text check only guards accidental new output code.

- [ ] **Step 2: Implement strict input loading**

Require:

```text
<run-dir>/aisle_graph.yaml
<run-dir>/<navigation-map argument>
<run-dir>/site_boundary.yaml
<vehicle-profile path>
```

Load with existing APIs:

```python
graph = load_agricultural_aisle_graph(...)
navigation = load_navigation_grid(...)
boundary = load_site_boundary(..., expected_frame_id=graph.frame_id)
vehicle = load_canonical_vehicle_profile(...)
```

Use the same frozen A1 config as the V25-12F greenhouse lane diagnostic:

```python
VehicleSafeLaneConfig(
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

- [ ] **Step 3: Derive both stable longest-only lanes and the A1 all-segment plan from identical truth inputs**

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

Pair results by `aisle_id` and fail if either plan has missing/duplicate aisle IDs.

- [ ] **Step 4: Emit the frozen per-aisle experiment metrics**

For every aisle report:

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

Compute exactly:

```python
recoverable_additional_length_m = max(
    0.0,
    active_segment_length_m - current_longest_only_selected_span_m,
)
```

Aggregate:

```python
segment_recovery_fraction = (
    0.0
    if structural_length_total <= 1.0e-12
    else active_segment_length_total / structural_length_total
)
```

Report diagnostic aisles in a dedicated dictionary for:

```text
aisle_016
aisle_017
aisle_018
aisle_019
aisle_020
```

Missing diagnostic aisle IDs must be reported as missing, not fabricated.

- [ ] **Step 5: Make report output deterministic**

Compact mode:

```python
json.dumps(report, sort_keys=True, separators=(",", ":"))
```

Pretty mode:

```python
json.dumps(report, indent=2, sort_keys=True)
```

Run the same synthetic temporary run fixture twice and assert exact JSON equality.

- [ ] **Step 6: Test optional segment YAML writing**

In a temp directory, call the harness with `--write-segments` and assert only:

```text
vehicle_feasible_segments.yaml
```

is created by A1. Pre-create sentinel bytes for canonical map files and assert their bytes are unchanged after the harness run.

- [ ] **Step 7: Run the harness contract suite**

```bash
python3 -m pytest -q \
  tests/test_v25_12g_a1_contract.py \
  src/agt_offline_assets/test/test_vehicle_feasible_segment.py
```

Expected: PASS.

- [ ] **Step 8: Commit Task 4**

```bash
git add \
  tools/v25_12g_a1_acceptance.py \
  tests/test_v25_12g_a1_contract.py

git commit -m "test(v25-12g): add repeatable A1 segment recovery harness"
```

---

### Task 5: Add review-only Workbench visualization for `vehicle_feasible_segments.yaml`

**Files:**
- Create: `src/agt_map_workbench/agt_map_workbench/vehicle_feasible_segment_preview.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/review_workbench.py`
- Create: `src/agt_map_workbench/test/test_vehicle_feasible_segment_preview.py`
- Create: `src/agt_map_workbench/test/test_vehicle_feasible_segment_workbench.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**
- Consumes: loaded `VehicleFeasibleSegmentPlan`; does not derive it.
- Produces a `VehicleFeasibleSegmentPreview` manager with:

```python
class VehicleFeasibleSegmentPreview:
    def __init__(self, scene): ...
    def set_plan(self, plan: VehicleFeasibleSegmentPlan | None) -> None: ...
    def set_visible(self, visible: bool) -> None: ...
    def clear(self) -> None: ...
```

- Workbench adds one layer key:

```text
vehicle_feasible_segments
```

- [ ] **Step 1: Write the failing pure preview-item test**

Build a two-segment plan and assert the preview creates one path item per active segment and separate marker items for rejected fragments only when diagnostic-fragment visibility is enabled internally by the preview test fixture.

The production A1 layer should show active segments prominently and rejected fragments as visually distinct diagnostic marks, not as executable routes.

- [ ] **Step 2: Implement a focused QGraphics renderer**

Use `QPainterPath` / `QGraphicsPathItem` for active segment centerlines and `QGraphicsEllipseItem` or short dashed paths for rejected fragments. Keep all graphics in the manager so `clear()` can remove every item deterministically.

Do not assign editable/movable flags.

Set `zValue` above Navigation Map analysis overlays but below authoring handles; use the existing Workbench scene coordinate convention `(x, -y)`.

- [ ] **Step 3: Add sibling YAML lifecycle to `ReviewMapWorkbenchWindow`**

Add fields:

```python
self._vehicle_feasible_segment_plan = None
self._vehicle_feasible_segment_preview = VehicleFeasibleSegmentPreview(self._scene)
self._vehicle_feasible_segment_last_error = ""
```

Add nav-layer item once:

```python
self._nav_layer.addItem("12G-A1 Vehicle-Feasible Segments", "vehicle_feasible_segments")
```

When a PCD/source run changes, look only for sibling:

```text
vehicle_feasible_segments.yaml
```

Load it with `load_vehicle_feasible_segment_plan`. Missing file means unavailable layer, not an error. Invalid file records an error and keeps the layer unavailable; it must not break 12F Route Debug or other Workbench tabs.

- [ ] **Step 4: Wire layer visibility without regenerating truth**

In `_update_navigation_overlay` or the narrowest existing layer-dispatch hook:

```python
if layer == "vehicle_feasible_segments":
    self._navigation_preview_item.clear_result()
    self._vehicle_feasible_segment_preview.set_visible(
        self._nav_overlay_visible.isChecked()
        and self._vehicle_feasible_segment_plan is not None
    )
    return
```

When any other navigation layer is selected, hide the segment preview before delegating.

No Workbench button may call `derive_vehicle_feasible_segment_plan` in A1.

- [ ] **Step 5: Add Workbench integration regressions**

Cover:

```text
missing sibling YAML -> no exception, layer unavailable
valid sibling YAML -> plan loads and layer can be selected
invalid sibling YAML -> error recorded, other navigation layers still work
switching away from segment layer hides all A1 graphics
opening a second source clears the prior plan/graphics before loading the new sibling
no item is movable/editable
```

- [ ] **Step 6: Register Workbench tests**

Add:

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

Expected: PASS.

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

### Task 6: Freeze A1 contract documentation and run focused automated verification

**Files:**
- Create: `docs/v2.5/V25_12G_A1_CURRENT_STATE.md`
- Modify only if needed by tests: `tests/test_v25_12g_a1_contract.py`

**Interfaces:**
- Produces a checkpoint document that explicitly distinguishes code verification from real-data observation.

- [ ] **Step 1: Write the current-state document before real-data claims**

Use this initial status:

```text
CODE IMPLEMENTED / FOCUSED AUTOMATED VERIFICATION PENDING / REAL-DATA A1 EXPERIMENT PENDING
```

Document the frozen A1 scope, schema, threshold, endpoint semantics, writer filename, no-canonical-overwrite invariant, and the exact operator command from Task 7 below.

- [ ] **Step 2: Run the full focused offline A1 regression group**

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

Expected: all selected tests PASS.

- [ ] **Step 3: Run the focused Workbench group**

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/agt_map_workbench/test/test_vehicle_feasible_segment_preview.py \
  src/agt_map_workbench/test/test_vehicle_feasible_segment_workbench.py \
  src/agt_map_workbench/test/test_site_boundary_workbench.py \
  src/agt_map_workbench/test/test_route_debug_view.py \
  src/agt_map_workbench/test/test_route_debug_panel.py
```

Expected: all selected tests PASS.

- [ ] **Step 4: Update current-state doc with exact focused counts from operator output**

Do not guess counts in advance. Record the actual final summaries verbatim, for example:

```text
Offline focused: <actual> passed
Workbench focused: <actual> passed
```

If any test fails, leave status PENDING and document the failure instead of marking PASS.

- [ ] **Step 5: Commit Task 6**

```bash
git add docs/v2.5/V25_12G_A1_CURRENT_STATE.md tests/test_v25_12g_a1_contract.py
git commit -m "docs(v25-12g): freeze A1 extraction contract"
```

---

### Task 7: Run the first real-greenhouse A1 experiment and package-level gate

**Files:**
- Generated runtime artifact only when explicitly requested: `/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run/vehicle_feasible_segments.yaml`
- Modify after observation: `docs/v2.5/V25_12G_A1_CURRENT_STATE.md`
- Create after observation: `docs/v2.5/V25_12G_A1_REAL_DATA_2026-08-15.md`

**Interfaces:**
- Real data:

```text
run dir: /home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run
vehicle profile: /home/yangxuan/agt_navigation_v2/profiles/platforms/mk_mini.yaml
navigation source: navigation_map.yaml unless explicitly testing navigation_map_12f.yaml as a separate labeled run
```

- [ ] **Step 1: Pull the branch without cleaning unrelated local state**

```bash
cd ~/agt_navigation_v2
git switch feat/v25-12g-maximum-feasible-coverage
git pull --ff-only

git status --short
```

Expected: branch is correct; unrelated local changes such as `tools/rosbag_sensor_trimmer` may remain and must not be reset/cleaned.

- [ ] **Step 2: Build the two touched packages**

```bash
source /opt/ros/humble/setup.bash
colcon build \
  --packages-select agt_offline_assets agt_map_workbench \
  --symlink-install \
  --event-handlers console_direct+
source install/setup.bash
```

Expected: both selected packages build successfully.

- [ ] **Step 3: Run the real A1 experiment against the frozen canonical Navigation Map**

First run read-only:

```bash
python3 tools/v25_12g_a1_acceptance.py \
  --run-dir /home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run \
  --vehicle-profile /home/yangxuan/agt_navigation_v2/profiles/platforms/mk_mini.yaml \
  --navigation-map navigation_map.yaml \
  --pretty | tee /tmp/v25_12g_a1_current.json
```

Inspect especially:

```text
aisle_016
aisle_017
aisle_018
aisle_019
aisle_020
```

Record exactly which aisles contain multiple active `>=1.0 m` segments and each `recoverable_additional_length_m`.

- [ ] **Step 4: If the report is structurally valid, write the dedicated A1 asset**

```bash
python3 tools/v25_12g_a1_acceptance.py \
  --run-dir /home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run \
  --vehicle-profile /home/yangxuan/agt_navigation_v2/profiles/platforms/mk_mini.yaml \
  --navigation-map navigation_map.yaml \
  --write-segments \
  --pretty | tee /tmp/v25_12g_a1_current_written.json
```

Expected: creates only:

```text
runtime/maps/agt_workbench_run/vehicle_feasible_segments.yaml
```

If that file already exists from a prior run, do not overwrite automatically. Inspect it first, then rerun only with the explicit `--overwrite-segments` flag if replacement is intended.

- [ ] **Step 5: Review the A1 Workbench layer**

Launch the existing Review Workbench entry point and open the same run/PCD. Select:

```text
12G-A1 Vehicle-Feasible Segments
```

Visually verify:

```text
active segments align with aisle-safe geometry
gaps remain visible as gaps
short rejected fragments are not rendered like executable coverage
no graphics are draggable/editable
switching layers leaves 12F/Route Debug unaffected
```

Record observations only; do not hand-edit A1 segment truth.

- [ ] **Step 6: Run scoped package-level tests**

```bash
colcon test \
  --packages-select agt_offline_assets agt_map_workbench \
  --event-handlers console_direct+

colcon test-result \
  --test-result-base build/agt_offline_assets \
  --all

colcon test-result \
  --test-result-base build/agt_map_workbench \
  --all
```

Expected for acceptance: both selected package result summaries show zero failures/errors. Do not use workspace-wide historical failures from unrelated packages as the A1 result.

- [ ] **Step 7: Freeze the real-data report without overstating route readiness**

Create `docs/v2.5/V25_12G_A1_REAL_DATA_2026-08-15.md` with:

```text
navigation source used
vehicle profile hash
A1 config
aggregate structural length
aggregate active segment length
segment_recovery_fraction
aggregate current longest-only selected span
aggregate recoverable_additional_length_m
per-aisle rows for aisle_016..020
all aisles that have >1 active segment
all rejected short-fragment counts
Workbench visual review notes
package-level verification summary
```

Use one of these conclusions exactly according to evidence:

```text
REAL-DATA ADDITIONAL SEGMENTS OBSERVED / NOT ROUTE READY
```

or:

```text
REAL-DATA NO ADDITIONAL >=1.0M SEGMENTS OBSERVED / SOFTWARE CONTRACT VERIFIED / NOT ROUTE READY
```

A1 never claims an executable global route; A2-A4 are still required.

- [ ] **Step 8: Update `V25_12G_A1_CURRENT_STATE.md` and commit the observation**

If focused + package tests are green, use:

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

Do **not** commit runtime map artifacts unless the repository's existing runtime-data policy explicitly requires it; the default is to keep the generated run asset local.

---

## Final A1 Acceptance Checklist

Before declaring V25-12G-A1 complete, verify all of the following from test/output evidence:

```text
[ ] VehicleSafeLanePlan public behavior is unchanged
[ ] every emitted segment is a maximal contiguous run of selected footprint-free poses
[ ] every emitted pose respects Site Boundary when supplied
[ ] structural-width-blocked aisles do not emit active segments
[ ] minimum active length is exactly configured minimum_contiguous_span_m (default 1.0 m)
[ ] short fragments remain diagnostics only
[ ] segment/fragment IDs are deterministic
[ ] active segments are non-overlapping and longitudinally ordered
[ ] endpoint headland classifications follow maximum_endpoint_retreat_m
[ ] repeated derivation/serialization is byte-deterministic
[ ] vehicle_feasible_segments.yaml is the only new A1 runtime asset
[ ] canonical Navigation Map and 12F candidate assets are unchanged
[ ] Workbench visualization is review-only and non-editable
[ ] focused offline tests pass
[ ] focused Workbench tests pass
[ ] agt_offline_assets package-level tests pass
[ ] agt_map_workbench package-level tests pass
[ ] real-data report truthfully records whether additional >=1.0 m segments exist
[ ] A1 remains explicitly NOT ROUTE READY
```

Only after this checklist is satisfied should V25-12G proceed to a separate A2 design/plan for Segment Service States and Reachability Graph.
