# AGT Navigation V25-12F Traversability + Hard Boundary Repair Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a deterministic V25-12F candidate planning-space pipeline that adds a manually frozen vehicle-permitted greenhouse boundary, preserves structural aisle geometry before ground filtering, recovers only short longitudinal vegetation-occlusion gaps, enforces the boundary at continuous vehicle-footprint level, and exposes the candidate in Route Debug for real-data A/B review.

**Architecture:** `agt_offline_assets` owns all safety and traversability semantics. `site_boundary.py` owns the frozen polygon contract and strict footprint containment; `navigation_corridor.py` exposes structural aisle geometry before `safe_base`; `traversability.py` owns rich evidence states, directional gap recovery, candidate serialization, and reload. The Workbench only authors/requests/visualizes these assets; lane/connector preview modules receive an optional `SiteBoundary`; Route Debug only reads frozen outputs.

**Tech Stack:** Python 3.10, ROS 2 Humble, NumPy, SciPy, PyYAML, Shapely, PyQt5, `ament_cmake_python`, pytest.

## Global Constraints

- Implementation branch: `feat/v25-12f-traversability-hard-boundary`
- Approved design: `docs/superpowers/specs/2026-08-15-v25-12f-traversability-hard-boundary-design.md`
- `site_boundary` is the vehicle-permitted **inner** perimeter, not the wall centerline
- Continuous vehicle-footprint touching or crossing the boundary is `SITE_BOUNDARY_CONFLICT`
- Missing/invalid `site_boundary.yaml` means **no 12F candidate map**
- Never infer a boundary from map bounds
- Never perform global `UNKNOWN -> FREE`
- Recovery is longitudinal in the agricultural row/aisle frame, never generic 2D closing
- Initial `maximum_inferred_gap_m = 0.60`
- Current OCCUPIED cells are not recovered in the A-first main candidate
- Main candidate preserves current map padding; padding removal remains diagnostic only
- `aisle_003` remains direct-obstacle conservative
- `aisle_005` remains structurally width-blocked
- `aisle_013` remains the padding-dominant probe
- R6B search parameters/budget/costs remain unchanged
- Current `navigation_map.yaml`, `navigation_map.pgm`, `derivation.yaml` remain untouched during A/B acceptance
- Fixed outputs: `site_boundary.yaml`, `traversability_evidence.yaml`, `traversability_evidence.npz`, `navigation_map_12f.yaml`, `navigation_map_12f.pgm`, `navigation_map_12f_derivation.yaml`
- Compatibility mapping: `OBSERVED_FREE -> FREE`, `INFERRED_TRAVERSABLE -> FREE`, `HARD_BLOCKED -> OCCUPIED`, `SENSOR_OBSTACLE -> OCCUPIED`, `UNKNOWN -> UNKNOWN`, `NO_GO -> OCCUPIED`
- Route Debug remains render-only
- Real-data acceptance run: `/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run`
- No real-data PASS claim until operator A/B review is completed

---

## Task 1: Site Boundary core contract

**Files:**
- Create: `src/agt_offline_assets/agt_offline_assets/site_boundary.py`
- Create: `src/agt_offline_assets/test/test_site_boundary.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/__init__.py`
- Modify: `src/agt_offline_assets/CMakeLists.txt`

**Interfaces:**

```python
SITE_BOUNDARY_SCHEMA = "agt_site_boundary/v1"
SITE_BOUNDARY_SEMANTICS = "VEHICLE_PERMITTED_INNER_BOUNDARY"

@dataclass(frozen=True)
class SiteBoundary:
    frame_id: str
    outer_boundary_xy: tuple[tuple[float, float], ...]
    source: Mapping[str, Any] = field(default_factory=dict)
    schema: str = SITE_BOUNDARY_SCHEMA
    status: str = "READY"
    boundary_semantics: str = SITE_BOUNDARY_SEMANTICS

    def validate(self, *, expected_frame_id: str | None = None) -> None: ...

def load_site_boundary(path: str | Path, *, expected_frame_id: str | None = None) -> SiteBoundary: ...
def write_site_boundary(boundary: SiteBoundary, path: str | Path, *, overwrite: bool = False) -> Path: ...
def rasterize_site_boundary(boundary: SiteBoundary, grid, *, expected_frame_id: str | None = None) -> np.ndarray: ...
def polygon_strictly_inside_site_boundary(boundary: SiteBoundary, polygon_xy) -> bool: ...
```

`rasterize_site_boundary()` must support both `NavigationGridEvidence` and `NavigationMapResult`. `NavigationMapResult` has no `frame_id`; therefore use:

```python
grid_frame = getattr(grid, "frame_id", None)
if expected_frame_id is not None:
    grid_frame = expected_frame_id
if grid_frame is not None:
    validate_site_boundary(boundary, expected_frame_id=str(grid_frame))
else:
    validate_site_boundary(boundary)
```

- [ ] **Step 1: Write failing contract tests**

```python
# src/agt_offline_assets/test/test_site_boundary.py
from pathlib import Path
import numpy as np
import pytest

from agt_offline_assets import (
    NavigationGridEvidence,
    SiteBoundary,
    load_site_boundary,
    polygon_strictly_inside_site_boundary,
    rasterize_site_boundary,
    write_site_boundary,
)


def _boundary():
    return SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0)),
        source={"authoring_mode": "WORKBENCH_MANUAL_POLYGON"},
    )


def test_round_trip_and_frame_mismatch(tmp_path: Path):
    path = write_site_boundary(_boundary(), tmp_path / "site_boundary.yaml")
    assert load_site_boundary(path, expected_frame_id="map").outer_boundary_xy == _boundary().outer_boundary_xy
    with pytest.raises(ValueError, match="frame_id mismatch"):
        load_site_boundary(path, expected_frame_id="odom")


def test_self_intersection_fails_closed():
    boundary = SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((0.0, 0.0), (2.0, 2.0), (0.0, 2.0), (2.0, 0.0)),
    )
    with pytest.raises(ValueError, match="simple"):
        boundary.validate()


def test_touching_or_crossing_footprint_is_rejected():
    boundary = _boundary()
    inside = ((1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0))
    touching = ((0.0, 1.0), (1.0, 1.0), (1.0, 2.0), (0.0, 2.0))
    crossing = ((-0.1, 1.0), (1.0, 1.0), (1.0, 2.0), (-0.1, 2.0))
    assert polygon_strictly_inside_site_boundary(boundary, inside)
    assert not polygon_strictly_inside_site_boundary(boundary, touching)
    assert not polygon_strictly_inside_site_boundary(boundary, crossing)
```

- [ ] **Step 2: Run the test and confirm import failure**

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest -q src/agt_offline_assets/test/test_site_boundary.py
```

Expected: FAIL because the API does not exist.

- [ ] **Step 3: Implement validation with Shapely and strict boundary-contact rejection**

```python
from shapely.geometry import Polygon


def _validated_polygon(boundary: SiteBoundary) -> Polygon:
    points = np.asarray(boundary.outer_boundary_xy, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] < 3 or points.shape[1] != 2:
        raise ValueError("site_boundary requires at least three [x, y] vertices")
    if not np.all(np.isfinite(points)) or np.unique(points, axis=0).shape[0] < 3:
        raise ValueError("site_boundary requires three unique finite vertices")
    polygon = Polygon(points)
    if not polygon.is_valid or not polygon.exterior.is_simple:
        raise ValueError("site_boundary polygon must be simple and non-self-intersecting")
    if polygon.area <= 0.0:
        raise ValueError("site_boundary polygon must have non-zero area")
    return polygon


def polygon_strictly_inside_site_boundary(boundary, polygon_xy) -> bool:
    site = _validated_polygon(boundary)
    footprint = Polygon(np.asarray(polygon_xy, dtype=np.float64))
    if not footprint.is_valid or footprint.area <= 0.0:
        return False
    return bool(site.contains(footprint) and site.boundary.disjoint(footprint))
```

Rasterize cell centers using existing `_points_inside_polygon`; continuous footprint safety is still enforced by Shapely, not by the raster alone.

- [ ] **Step 4: Implement exact YAML I/O and register exports/tests**

Serialized fields:

```yaml
schema: agt_site_boundary/v1
frame_id: map
status: READY
boundary_semantics: VEHICLE_PERMITTED_INNER_BOUNDARY
outer_boundary_xy: []
source:
  authoring_mode: WORKBENCH_MANUAL_POLYGON
```

Add `ament_add_pytest_test(test_site_boundary test/test_site_boundary.py)`.

- [ ] **Step 5: Run focused test**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_site_boundary.py
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add src/agt_offline_assets/agt_offline_assets/site_boundary.py \
        src/agt_offline_assets/agt_offline_assets/__init__.py \
        src/agt_offline_assets/test/test_site_boundary.py \
        src/agt_offline_assets/CMakeLists.txt
git commit -m "feat(v25-12f): add site boundary contract"
```

---

## Task 2: Workbench Site Boundary authoring

**Files:**
- Modify: `src/agt_map_workbench/agt_map_workbench/agricultural_workbench.py`
- Create: `src/agt_map_workbench/test/test_site_boundary_workbench.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**

```python
self._site_boundary: SiteBoundary | None
self._site_boundary_vertices: list[tuple[float, float]]

def _start_site_boundary_authoring(self) -> None: ...
def _append_site_boundary_vertex(self, x: float, y: float) -> None: ...
def _undo_site_boundary_vertex(self) -> None: ...
def _finish_site_boundary_authoring(self, *, show_errors: bool = True) -> bool: ...
def _clear_site_boundary(self) -> None: ...
def _load_sibling_site_boundary(self) -> bool: ...
def _export_site_boundary(self) -> Path | None: ...
```

- [ ] **Step 1: Write failing headless tests**

```python
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtWidgets import QApplication
from agt_map_workbench.review_workbench import ReviewMapWorkbenchWindow


def _qapp():
    return QApplication.instance() or QApplication([])


def test_finish_builds_ready_boundary():
    app = _qapp()
    window = ReviewMapWorkbenchWindow()
    try:
        window._site_boundary_vertices = [(0, 0), (5, 0), (5, 4), (0, 4)]
        assert window._finish_site_boundary_authoring(show_errors=False)
        assert window._site_boundary.status == "READY"
        app.processEvents()
    finally:
        window.close()


def test_boundary_clear_does_not_clear_no_go_overrides():
    app = _qapp()
    window = ReviewMapWorkbenchWindow()
    try:
        window._navigation_overrides = [{"mode": "no_go", "polygon_xy": [[1, 1], [2, 1], [2, 2]]}]
        window._site_boundary_vertices = [(0, 0), (5, 0), (5, 4)]
        window._clear_site_boundary()
        assert len(window._navigation_overrides) == 1
    finally:
        window.close()
```

- [ ] **Step 2: Run and confirm missing authoring state**

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q src/agt_map_workbench/test/test_site_boundary_workbench.py
```

- [ ] **Step 3: Initialize dedicated boundary state before `super().__init__()`**

Do not reuse `_nav_vertices`/`_nav_polygon_item`; Site Boundary and Navigation Override are different truth types.

- [ ] **Step 4: Add navigation-page controls**

```text
Site Boundary（车辆允许区域内边界）
[开始绘制] [完成] [撤销顶点] [清除]
[导出 site_boundary.yaml]
状态：未定义 / 草稿 N 点 / READY / INVALID
```

- [ ] **Step 5: Route map clicks through a dedicated interaction mode**

```python
def _on_map_clicked(self, x: float, y: float) -> None:
    if self._interaction_mode == "site_boundary":
        self._append_site_boundary_vertex(x, y)
        return
    super()._on_map_clicked(x, y)
```

- [ ] **Step 6: Render boundary distinctly from NO_GO**

Use a solid high-contrast edge and low-opacity permitted-area fill; numbered vertices are visible only while editing. Do not reuse the magenta NO_GO encoding.

- [ ] **Step 7: Auto-load sibling `site_boundary.yaml` after opening `processed.pcd`**

```python
candidate = self._source_path.parent / "site_boundary.yaml"
if candidate.is_file():
    self._site_boundary = load_site_boundary(candidate, expected_frame_id="map")
```

Invalid sibling boundary -> warning + no boundary, never fallback to map bounds.

- [ ] **Step 8: Export READY boundary to the selected path, defaulting to the PCD run directory**

Default:

```python
self._source_path.parent / "site_boundary.yaml"
```

- [ ] **Step 9: Register/run tests and commit**

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/agt_map_workbench/test/test_site_boundary_workbench.py \
  src/agt_map_workbench/test/test_route_debug_workbench_integration.py

git add src/agt_map_workbench/agt_map_workbench/agricultural_workbench.py \
        src/agt_map_workbench/test/test_site_boundary_workbench.py \
        src/agt_map_workbench/CMakeLists.txt
git commit -m "feat(v25-12f): author site boundary in workbench"
```

---

## Task 3: Preserve structural aisle geometry before Ground filtering

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/navigation_corridor.py`
- Modify: `src/agt_offline_assets/test/test_navigation_corridor.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/agricultural_workbench.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/navigation_preview.py`

**Produces:** `CorridorRefinementResult.aisle_geometric_envelope: np.ndarray`

- [ ] **Step 1: Add failing tests**

```python
def test_geometric_envelope_survives_ground_hole():
    navigation, structure = _fixture()
    navigation.ground_valid[14:17, 20:24] = False
    result = derive_corridor_refinement(navigation, structure, _feasible_config())
    assert np.any(result.aisle_geometric_envelope[14:17, 20:24])
    assert not np.any(result.aisle_candidate[14:17, 20:24])
    assert np.all(~result.aisle_candidate | result.aisle_geometric_envelope)


def test_width_rejected_pair_never_enters_geometric_envelope():
    navigation, structure = _fixture()
    result = derive_corridor_refinement(navigation, structure)
    assert np.count_nonzero(result.aisle_geometric_envelope) == 0
```

- [ ] **Step 2: Run test; expect missing field**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_navigation_corridor.py
```

- [ ] **Step 3: Make `evaluate_corridor()` return geometry separately**

After structural width/support/overlap gates pass:

```python
geometry = longitudinal & (vv >= corridor_min) & (vv <= corridor_max)
safe = geometry & safe_base
```

Return `(geometry, safe, centerline, diagnostic)` and accumulate:

```python
aisle_geometric_envelope |= geometry
```

Rejected pairs return an all-false geometry mask.

- [ ] **Step 4: Add Workbench preview layer**

Label:

```text
结构行道几何包络（未经过 Ground FREE 过滤）
```

Use a distinct low-opacity color; never label it FREE/safe.

- [ ] **Step 5: Run regressions and commit**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_navigation_corridor.py \
  src/agt_offline_assets/test/test_vehicle_corridor.py

git add src/agt_offline_assets/agt_offline_assets/navigation_corridor.py \
        src/agt_offline_assets/test/test_navigation_corridor.py \
        src/agt_map_workbench/agt_map_workbench/agricultural_workbench.py \
        src/agt_map_workbench/agt_map_workbench/navigation_preview.py
git commit -m "feat(v25-12f): preserve structural aisle envelope"
```

---

## Task 4: Rich traversability and longitudinal occlusion recovery

**Files:**
- Create: `src/agt_offline_assets/agt_offline_assets/traversability.py`
- Create: `src/agt_offline_assets/test/test_traversability.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/__init__.py`
- Modify: `src/agt_offline_assets/CMakeLists.txt`

**Interfaces:**

```python
TRAVERSABILITY_EVIDENCE_SCHEMA = "agt_traversability_evidence/v1"
TRAVERSABILITY_CANDIDATE_DERIVATION_SCHEMA = "agt_traversability_candidate_navigation_map/v1"

@dataclass(frozen=True)
class TraversabilityConfig:
    maximum_inferred_gap_m: float = 0.60

@dataclass(frozen=True)
class TraversabilityEvidence:
    frame_id: str
    observed_free_mask: np.ndarray
    inferred_traversable_mask: np.ndarray
    hard_blocked_mask: np.ndarray
    sensor_obstacle_mask: np.ndarray
    unknown_mask: np.ndarray
    semantic_no_go_mask: np.ndarray
    aisle_geometric_envelope_mask: np.ndarray
    config: TraversabilityConfig
    source: Mapping[str, Any] = field(default_factory=dict)

    def candidate_occupancy(self) -> np.ndarray: ...

def derive_traversability_evidence(
    navigation: NavigationMapResult,
    structure: NavigationStructureResult,
    corridor: CorridorRefinementResult,
    site_boundary: SiteBoundary,
    *,
    overrides: Iterable[Mapping] = (),
    config: TraversabilityConfig | None = None,
    frame_id: str = "map",
) -> TraversabilityEvidence: ...
```

- [ ] **Step 1: Add synthetic recovery tests**

Test all six cases:

```text
0.4 m longitudinal UNKNOWN hole -> inferred
0.9 m hole -> remains UNKNOWN
row_structural_band intersection -> not inferred
current OCCUPIED -> not inferred
NO_GO -> not inferred
outside Site Boundary -> HARD_BLOCKED, not inferred
```

Core assertion pattern:

```python
evidence = derive_traversability_evidence(
    navigation, structure, corridor, boundary,
    config=TraversabilityConfig(maximum_inferred_gap_m=0.60),
)
assert not np.any(evidence.inferred_traversable_mask & evidence.hard_blocked_mask)
assert not np.any(evidence.inferred_traversable_mask & evidence.semantic_no_go_mask)
assert not np.any(evidence.inferred_traversable_mask & evidence.sensor_obstacle_mask)
```

- [ ] **Step 2: Run test; expect missing module/API**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_traversability.py
```

- [ ] **Step 3: Build initial masks from the current candidate input map**

```python
occupancy = np.asarray(navigation.occupancy, dtype=np.uint8)
base_free = occupancy == FREE
base_occupied = occupancy == OCCUPIED
base_unknown = occupancy == UNKNOWN
inside_boundary = rasterize_site_boundary(
    site_boundary, navigation, expected_frame_id=frame_id
)
hard_blocked = ~inside_boundary
semantic_no_go = _rasterize_no_go(navigation, overrides)
```

A-first semantics deliberately treat current OCCUPIED as non-recoverable. Existing RAW/GEOMETRY/PADDING source audit remains the provenance layer for why those cells are occupied.

- [ ] **Step 4: Implement row-coordinate directional recovery**

Compute cell-center world coordinates and row projections:

```python
direction = np.asarray(structure.row_model.direction_xy, dtype=np.float64)
direction /= np.linalg.norm(direction)
perpendicular = np.array([-direction[1], direction[0]], dtype=np.float64)
u = xx * direction[0] + yy * direction[1]
v = xx * perpendicular[0] + yy * perpendicular[1]
v_bin = np.rint((v - float(np.min(v))) / navigation.resolution_m).astype(np.int64)
```

For each `v_bin`, process only cells inside `corridor.aisle_geometric_envelope`, sorted by `u`. A candidate run is inferable only when:

```text
all run cells were UNKNOWN
run is bracketed by OBSERVED/FREE support on both longitudinal sides
gap length <= maximum_inferred_gap_m
no run cell intersects row_structural_band
no run cell intersects current OCCUPIED
no run cell intersects NO_GO
no run cell intersects HARD_BLOCKED
endpoint ground heights are finite
endpoint height difference / separation <= maximum_slope_deg
endpoint absolute height difference <= maximum_step_m
```

Do not use `binary_closing`, isotropic dilation, or Euclidean nearest-free filling.

- [ ] **Step 5: Apply fixed precedence**

```text
HARD_BLOCKED
> NO_GO
> SENSOR_OBSTACLE
> INFERRED_TRAVERSABLE
> OBSERVED_FREE
> UNKNOWN
```

Build `candidate_occupancy()` exactly from that precedence.

- [ ] **Step 6: Register/run tests and commit**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_traversability.py \
  src/agt_offline_assets/test/test_navigation_corridor.py \
  src/agt_offline_assets/test/test_navigation_map_derivation.py

git add src/agt_offline_assets/agt_offline_assets/traversability.py \
        src/agt_offline_assets/agt_offline_assets/__init__.py \
        src/agt_offline_assets/test/test_traversability.py \
        src/agt_offline_assets/CMakeLists.txt
git commit -m "feat(v25-12f): derive occlusion-aware traversability"
```

---

## Task 5: Candidate serialization + Workbench candidate workflow

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/traversability.py`
- Modify: `src/agt_offline_assets/test/test_traversability.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/agricultural_workbench.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/navigation_preview.py`
- Modify: `src/agt_map_workbench/test/test_site_boundary_workbench.py`

**Interfaces:**

```python
def write_traversability_candidate(
    evidence: TraversabilityEvidence,
    navigation: NavigationMapResult,
    run_dir: str | Path,
    *,
    site_boundary_path: str | Path,
    overwrite: bool = False,
) -> Path: ...

def load_traversability_evidence(
    path: str | Path,
    *,
    expected_frame_id: str | None = None,
) -> TraversabilityEvidence: ...
```

- [ ] **Step 1: Add failing writer/loader tests**

Writer must create exactly these five files in addition to the already-authored `site_boundary.yaml`:

```text
traversability_evidence.yaml
traversability_evidence.npz
navigation_map_12f.yaml
navigation_map_12f.pgm
navigation_map_12f_derivation.yaml
```

Test that pre-existing canonical `navigation_map.yaml`, `navigation_map.pgm`, `derivation.yaml` bytes are unchanged.

- [ ] **Step 2: Run; expect writer/loader failure**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_traversability.py
```

- [ ] **Step 3: Serialize deterministic masks and lineage**

NPZ keys:

```python
np.savez_compressed(
    npz_path,
    observed_free_mask=evidence.observed_free_mask.astype(np.uint8),
    inferred_traversable_mask=evidence.inferred_traversable_mask.astype(np.uint8),
    hard_blocked_mask=evidence.hard_blocked_mask.astype(np.uint8),
    sensor_obstacle_mask=evidence.sensor_obstacle_mask.astype(np.uint8),
    unknown_mask=evidence.unknown_mask.astype(np.uint8),
    semantic_no_go_mask=evidence.semantic_no_go_mask.astype(np.uint8),
    aisle_geometric_envelope_mask=evidence.aisle_geometric_envelope_mask.astype(np.uint8),
)
```

`navigation_map_12f.yaml` references `navigation_map_12f.pgm`. `navigation_map_12f_derivation.yaml` records grid geometry, source current map, source Site Boundary hash, state counts, `maximum_inferred_gap_m`, requested map padding, effective padding cells `ceil(obstacle_padding_m / resolution_m)`, and output hashes.

- [ ] **Step 4: Add Workbench candidate state and controls**

```python
self._traversability_evidence: TraversabilityEvidence | None = None
self._navigation_12f_result: NavigationMapResult | None = None
```

UI:

```text
V25-12F Candidate
最大遮挡补全缺口：0.60 m
[生成 12F Candidate]
[导出到当前 run 目录]
```

Generate only when current navigation, structure, corridor, and READY Site Boundary all exist.

- [ ] **Step 5: Preserve current map object and create candidate by replacement**

```python
self._traversability_evidence = derive_traversability_evidence(...)
self._navigation_12f_result = replace(
    self._navigation_result,
    occupancy=self._traversability_evidence.candidate_occupancy(),
)
```

Never assign the candidate back to `self._navigation_result`.

- [ ] **Step 6: Add preview layers**

```text
12F Candidate 三态
12F OBSERVED_FREE
12F INFERRED_TRAVERSABLE
12F HARD_BLOCKED
12F SENSOR_OBSTACLE
12F UNKNOWN
12F Aisle Geometric Envelope
```

Qt receives precomputed masks; it does not derive traversability.

- [ ] **Step 7: Export to `self._source_path.parent` and protect fixed candidate files with explicit overwrite confirmation**

- [ ] **Step 8: Run tests and commit**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_traversability.py
QT_QPA_PLATFORM=offscreen python3 -m pytest -q src/agt_map_workbench/test/test_site_boundary_workbench.py

git add src/agt_offline_assets/agt_offline_assets/traversability.py \
        src/agt_offline_assets/test/test_traversability.py \
        src/agt_map_workbench/agt_map_workbench/agricultural_workbench.py \
        src/agt_map_workbench/agt_map_workbench/navigation_preview.py \
        src/agt_map_workbench/test/test_site_boundary_workbench.py
git commit -m "feat(v25-12f): export candidate navigation map"
```

---

## Task 6: Continuous Site Boundary enforcement in vehicle-lane and connector preview gates

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/reverse_primitive_connector.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/forward_connector_navigation_gate.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/vehicle_safe_lane.py`
- Modify: `src/agt_offline_assets/test/test_reverse_primitive_connector.py`
- Modify: `src/agt_offline_assets/test/test_forward_connector_navigation_gate.py`
- Modify: `src/agt_offline_assets/test/test_vehicle_safe_lane.py`

**Signatures:**

```python
def derive_vehicle_safe_lane_plan(..., *, site_boundary: SiteBoundary | None = None, source=None) -> VehicleSafeLanePlan: ...
def derive_forward_connector_navigation_gate(..., *, site_boundary: SiteBoundary | None = None, source=None) -> ForwardConnectorNavigationPlan: ...
def derive_reverse_primitive_connector_plan(..., *, site_boundary: SiteBoundary | None = None, source=None) -> ReversePrimitiveConnectorPlan: ...
```

- [ ] **Step 1: Write tests where pose center is inside but footprint touches boundary**

Expected boundary-specific rejection even when all sampled Navigation Grid cells are FREE.

- [ ] **Step 2: Run current gate tests and confirm missing optional API**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_safe_lane.py \
  src/agt_offline_assets/test/test_forward_connector_navigation_gate.py \
  src/agt_offline_assets/test/test_reverse_primitive_connector.py
```

- [ ] **Step 3: Replace the private boolean pose gate with a status helper while retaining compatibility wrapper**

```python
def _preview_pose_status(x, y, yaw, navigation, local_footprint, site_boundary=None) -> str:
    sample = ForwardConnectorSample(x=float(x), y=float(y), z=0.0, yaw=float(yaw))
    polygon = _transform_polygon(local_footprint, sample)
    if site_boundary is not None and not polygon_strictly_inside_site_boundary(site_boundary, polygon):
        return "SITE_BOUNDARY_CONFLICT"
    # existing bounds / occupancy checks
    ...


def _preview_pose_free(..., site_boundary=None) -> bool:
    return _preview_pose_status(...) == "PREVIEW_POSE_FREE"
```

- [ ] **Step 4: Vehicle-Safe Lane passes boundary into every lateral pose check**

Structural width gate is unchanged. If no feasible pose exists because boundary checks reject candidates, include `SITE_BOUNDARY_CONFLICT` in the reason.

- [ ] **Step 5: Forward connector checks every transformed footprint polygon**

Boundary conflict must force candidate rejection before `PREVIEW_FOOTPRINT_FREE` can be emitted.

- [ ] **Step 6: R6B checks start footprint, every primitive edge sample, and every goal-shot sample**

Start-boundary failure result:

```text
status = SITE_BOUNDARY_CONFLICT
search_expansions = 0
```

Do not modify primitive length, cost multipliers, cusp penalty, max expansions, goal tolerances, or search envelope settings.

- [ ] **Step 7: Run regressions and commit**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_safe_lane.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane_width_gate.py \
  src/agt_offline_assets/test/test_forward_connector_navigation_gate.py \
  src/agt_offline_assets/test/test_reverse_primitive_connector.py

git add src/agt_offline_assets/agt_offline_assets/vehicle_safe_lane.py \
        src/agt_offline_assets/agt_offline_assets/forward_connector_navigation_gate.py \
        src/agt_offline_assets/agt_offline_assets/reverse_primitive_connector.py \
        src/agt_offline_assets/test/test_vehicle_safe_lane.py \
        src/agt_offline_assets/test/test_forward_connector_navigation_gate.py \
        src/agt_offline_assets/test/test_reverse_primitive_connector.py
git commit -m "feat(v25-12f): enforce site boundary on preview footprints"
```

---

## Task 7: Route Debug 12F A/B layers

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/route_debug_dataset.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/route_debug_overlay.py`
- Modify: `src/agt_offline_assets/test/test_route_debug_dataset.py`
- Modify: `src/agt_offline_assets/test/test_route_debug_overlay.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/route_debug_view.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/route_debug_panel.py`
- Modify: `src/agt_map_workbench/test/test_route_debug_view.py`
- Modify: `src/agt_map_workbench/test/test_route_debug_panel.py`

**Dataset additions:**

```python
candidate_navigation: NavigationGridEvidence | None
site_boundary: SiteBoundary | None
traversability: TraversabilityEvidence | None
```

**Asset-state keys:** `site_boundary`, `navigation_12f`, `traversability_evidence`

**Layer keys:**

```text
base.navigation_12f
traversability.observed
traversability.inferred
traversability.hard_blocked
traversability.sensor_obstacle
traversability.unknown
traversability.aisle_geometric_envelope
semantics.site_boundary
```

- [ ] **Step 1: Add failing dataset tests for optional 12F discovery and frame mismatch fail-closed behavior**

Existing 12E layers must continue loading if optional 12F files are MISSING/INVALID.

- [ ] **Step 2: Add Site Boundary GeoJSON overlay test**

Required properties:

```text
layer_key = semantics.site_boundary
feature_kind = SITE_BOUNDARY
status = READY
source_asset = site_boundary.yaml
boundary_semantics = VEHICLE_PERMITTED_INNER_BOUNDARY
```

Do not serialize one GeoJSON feature per raster cell.

- [ ] **Step 3: Load candidate navigation with `load_navigation_grid(run_dir / "navigation_map_12f.yaml")` and traversability with `load_traversability_evidence()`**

- [ ] **Step 4: Render candidate/rich masks as rasters and Site Boundary as vector outline**

Recommended semantics:

```text
Candidate trinary            normal map palette
OBSERVED_FREE                low-opacity green
INFERRED_TRAVERSABLE         cyan
HARD_BLOCKED                 strong red
SENSOR_OBSTACLE              orange/red
UNKNOWN                      gray
Aisle Geometric Envelope     light cyan
Site Boundary                solid high-contrast edge
NO_GO                        existing separate semantic color
```

- [ ] **Step 5: Add `12F A/B` preset**

Preset enables current map, candidate map, inferred cells, hard blocked cells, envelope, Site Boundary, NO_GO, and structural aisles. Operator can toggle current/candidate base rasters manually.

- [ ] **Step 6: Run tests and commit**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_route_debug_dataset.py \
  src/agt_offline_assets/test/test_route_debug_overlay.py

QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/agt_map_workbench/test/test_route_debug_view.py \
  src/agt_map_workbench/test/test_route_debug_panel.py \
  src/agt_map_workbench/test/test_route_debug_workbench_integration.py

git add src/agt_offline_assets/agt_offline_assets/route_debug_dataset.py \
        src/agt_offline_assets/agt_offline_assets/route_debug_overlay.py \
        src/agt_offline_assets/test/test_route_debug_dataset.py \
        src/agt_offline_assets/test/test_route_debug_overlay.py \
        src/agt_map_workbench/agt_map_workbench/route_debug_view.py \
        src/agt_map_workbench/agt_map_workbench/route_debug_panel.py \
        src/agt_map_workbench/test/test_route_debug_view.py \
        src/agt_map_workbench/test/test_route_debug_panel.py
git commit -m "feat(v25-12f): visualize traversability a-b evidence"
```

---

## Task 8: Repeatable real-data acceptance harness, docs, and final verification

**Files:**
- Create: `tools/v25_12f_acceptance.py`
- Create: `tests/test_v25_12f_traversability_contract.py`
- Modify: `src/agt_map_workbench/README.md`
- Modify: `docs/v2.5/V25_12E_CURRENT_STATE.md`

**Purpose:** The acceptance harness does not change production assets. It compares current vs candidate Navigation Grid using the same MK-mini vehicle profile and Vehicle-Safe Lane config, checks continuous Site Boundary containment, and replays only the two frozen R6B regression connectors (`015`, `017`) with unchanged R6B config.

- [ ] **Step 1: Add cross-package contract test**

```python
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_v25_12f_contract_is_frozen():
    spec = (ROOT / "docs/superpowers/specs/2026-08-15-v25-12f-traversability-hard-boundary-design.md").read_text()
    for token in (
        "VEHICLE_PERMITTED_INNER_BOUNDARY",
        "SITE_BOUNDARY_CONFLICT",
        "maximum_inferred_gap_m",
        "navigation_map_12f.pgm",
        "traversability_evidence.npz",
    ):
        assert token in spec


def test_qt_does_not_own_traversability_algorithm():
    qt = (ROOT / "src/agt_map_workbench/agt_map_workbench/agricultural_workbench.py").read_text()
    assert "def derive_traversability_evidence" not in qt
    assert "binary_closing" not in qt
    assert "max_expansions" not in qt
```

- [ ] **Step 2: Implement `tools/v25_12f_acceptance.py` with concrete A/B metrics**

Use these imports/signatures after Tasks 1-7:

```python
from pathlib import Path
import json

from agt_offline_assets import (
    load_agricultural_aisle_graph,
    load_canonical_vehicle_profile,
    load_coverage_connector_requests,
    load_navigation_grid,
    load_site_boundary,
    load_turn_zones,
)
from agt_offline_assets.vehicle_safe_lane import (
    VehicleSafeLaneConfig,
    derive_vehicle_safe_lane_plan,
)
from agt_offline_assets.reverse_fallback_admission import (
    ReverseFallbackAdmissionItem,
    ReverseFallbackAdmissionPlan,
)
from agt_offline_assets.reverse_primitive_connector import (
    ReversePrimitiveConnectorConfig,
    derive_reverse_primitive_connector_plan,
)
```

The script accepts:

```text
--run-dir
--vehicle-profile
```

and loads:

```python
run_dir = Path(args.run_dir).resolve()
graph = load_agricultural_aisle_graph(run_dir / "aisle_graph.yaml")
current_nav = load_navigation_grid(run_dir / "navigation_map.yaml")
candidate_nav = load_navigation_grid(run_dir / "navigation_map_12f.yaml")
boundary = load_site_boundary(run_dir / "site_boundary.yaml", expected_frame_id="map")
vehicle = load_canonical_vehicle_profile(args.vehicle_profile)
```

Use one frozen lane config for both maps:

```python
lane_cfg = VehicleSafeLaneConfig(
    sample_spacing_m=0.10,
    lateral_search_step_m=0.05,
    maximum_lateral_shift_m=0.50,
    maximum_lateral_step_m=0.15,
    preview_footprint_padding_m=0.05,
    minimum_lane_coverage_fraction=0.70,
    maximum_endpoint_retreat_m=2.00,
    minimum_contiguous_span_m=1.00,
)

current_lane = derive_vehicle_safe_lane_plan(
    graph, current_nav, vehicle, lane_cfg, site_boundary=boundary,
)
candidate_lane = derive_vehicle_safe_lane_plan(
    graph, candidate_nav, vehicle, lane_cfg, site_boundary=boundary,
)
```

Print JSON containing for each map:

```text
ready
partial
unavailable
mean_coverage_fraction
site_boundary_reason_count
per_aisle {status, coverage_fraction, reason}
```

- [ ] **Step 3: Replay only `connector_015` and `connector_017` under unchanged R6B configuration**

Load requests/zones:

```python
requests = load_coverage_connector_requests(run_dir / "coverage_order.yaml")
zones = load_turn_zones(run_dir / "turn_zones.yaml")
selected = tuple(r for r in requests if r.connector_id in {"connector_015", "connector_017"})
```

Construct a diagnostic-only admission object from those existing frozen regression IDs without changing R6A policy:

```python
items = tuple(
    ReverseFallbackAdmissionItem(
        connector_id=r.connector_id,
        from_aisle_id=r.from_aisle_id,
        to_aisle_id=r.to_aisle_id,
        turn_zone_id=r.turn_zone_id,
        forward_audit_status="FROZEN_REGRESSION_REPLAY",
        decision="ELIGIBLE_REVERSE_FALLBACK",
        reason="V25-12F candidate-map regression replay of previously admitted connector",
    )
    for r in selected
)
admission = ReverseFallbackAdmissionPlan(
    frame_id=zones.frame_id,
    platform_id=vehicle.profile_id,
    platform_profile_sha256=vehicle.profile_sha256,
    items=items,
    source={"validation_scope": "V25_12F_REGRESSION_REPLAY_ONLY"},
)
```

Run identical default `ReversePrimitiveConnectorConfig()` on current and candidate maps with `site_boundary=boundary`. Print for each connector/map:

```text
status
path_length_m
reverse_distance_m
cusp_count
search_expansions
goal_position_error_m
goal_yaw_error_rad
reason
```

This script is diagnostic only and writes nothing.

- [ ] **Step 4: Run focused automated tests**

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_site_boundary.py \
  src/agt_offline_assets/test/test_navigation_corridor.py \
  src/agt_offline_assets/test/test_traversability.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane_width_gate.py \
  src/agt_offline_assets/test/test_forward_connector_navigation_gate.py \
  src/agt_offline_assets/test/test_reverse_primitive_connector.py \
  src/agt_offline_assets/test/test_route_debug_dataset.py \
  src/agt_offline_assets/test/test_route_debug_overlay.py \
  tests/test_v25_12f_traversability_contract.py

QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/agt_map_workbench/test/test_site_boundary_workbench.py \
  src/agt_map_workbench/test/test_route_debug_view.py \
  src/agt_map_workbench/test/test_route_debug_panel.py \
  src/agt_map_workbench/test/test_route_debug_workbench_integration.py
```

- [ ] **Step 5: Clean-build affected packages**

```bash
rm -rf build/agt_offline_assets build/agt_map_workbench \
       install/agt_offline_assets install/agt_map_workbench
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  agt_offline_assets agt_map_workbench \
  --event-handlers console_direct+
source install/setup.bash
```

- [ ] **Step 6: Real Site Boundary authoring**

Launch:

```bash
cd ~/agt_navigation_v2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run agt_map_workbench agt_map_workbench
```

Open:

```text
/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run/processed.pcd
```

Draw the permitted inner perimeter and export:

```text
/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run/site_boundary.yaml
```

- [ ] **Step 7: Generate/export 12F candidate and verify canonical files did not change**

New files must exist:

```text
traversability_evidence.yaml
traversability_evidence.npz
navigation_map_12f.yaml
navigation_map_12f.pgm
navigation_map_12f_derivation.yaml
```

Before/after SHA-256 of these canonical files must match:

```bash
sha256sum runtime/maps/agt_workbench_run/navigation_map.yaml \
          runtime/maps/agt_workbench_run/navigation_map.pgm \
          runtime/maps/agt_workbench_run/derivation.yaml
```

- [ ] **Step 8: Run the repeatable acceptance harness**

```bash
python3 tools/v25_12f_acceptance.py \
  --run-dir /home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run \
  --vehicle-profile /home/yangxuan/agt_navigation_v2/profiles/platforms/mk_mini.yaml
```

Record the JSON output in the experiment log/current-state update.

- [ ] **Step 9: Route Debug visual A/B acceptance**

In `路径调试 -> 12F A/B`, verify:

```text
Site Boundary visually distinct from NO_GO
no INFERRED_TRAVERSABLE outside boundary
short aisle gaps recovered only inside Aisle Geometric Envelope
no lateral bridge through crop rows
aisle_003 remains blocked by direct obstacle evidence
aisle_005 remains width rejected
aisle_013 remains explainable as padding-dominant/current-OCCUPIED in main candidate
connector_015/017 statuses match the acceptance harness and never violate Site Boundary
```

- [ ] **Step 10: Update docs only with measured results, then commit**

Before operator review:

```text
V25-12F CORE IMPLEMENTED / REAL-DATA A/B PENDING
```

Only after Steps 6-9 succeed may the state be promoted. On failure, record the exact failing aisle/connector/boundary condition and keep PENDING.

```bash
git add tools/v25_12f_acceptance.py \
        tests/test_v25_12f_traversability_contract.py \
        src/agt_map_workbench/README.md \
        docs/v2.5/V25_12E_CURRENT_STATE.md
git commit -m "test(v25-12f): lock traversability acceptance contract"
```

---

## Final Self-Verification Checklist

```text
[ ] Site Boundary round-trip/schema/frame validation passes
[ ] Self-intersection fails closed
[ ] Touching boundary counts as conflict
[ ] `NavigationMapResult` rasterization does not assume a `frame_id` attribute
[ ] Missing Site Boundary cannot generate candidate
[ ] `aisle_geometric_envelope` survives Ground holes but not structural width rejection
[ ] 0.4 m longitudinal UNKNOWN test recovers
[ ] 0.9 m longitudinal UNKNOWN test remains UNKNOWN
[ ] No generic 2D closing is used
[ ] Current OCCUPIED is not recovered in A-first candidate
[ ] NO_GO and HARD_BLOCKED precedence tests pass
[ ] Current canonical map files remain unchanged
[ ] Candidate fixed filenames and hashes are deterministic
[ ] Vehicle-Safe Lane enforces Site Boundary at footprint level
[ ] Forward connector enforces Site Boundary at every sampled footprint
[ ] R6B enforces Site Boundary without parameter changes
[ ] Route Debug 12F assets are optional/fail-closed per layer
[ ] Route Debug current/candidate A/B view works
[ ] Acceptance harness uses same MK-mini profile/config on both maps
[ ] connector_015 and connector_017 regression replay uses unchanged R6B config
[ ] zero Site Boundary footprint violations in real-data acceptance
[ ] no real-data PASS claim before operator visual review
```
