# AGT Navigation V25-12F Traversability + Hard Boundary Repair Execution Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Supersedes:** the earlier V25-12F plan drafts created during self-review. Use this file as the authoritative execution plan.

**Goal:** Add a manually frozen vehicle-permitted greenhouse boundary, retain structural aisle geometry before ground filtering, recover only bounded longitudinal vegetation-occlusion gaps, export a non-destructive 12F candidate map, enforce the boundary at continuous vehicle-footprint level, and validate current-vs-candidate behavior in Route Debug and a repeatable acceptance harness.

**Architecture:** Production semantics stay in `agt_offline_assets`; PyQt5 only authors assets and renders evidence. `site_boundary.py` owns the hard-boundary contract and continuous polygon containment. `navigation_corridor.py` exposes `aisle_geometric_envelope`. `traversability.py` owns rich states, recovery, serialization, and reload. Existing lane/connector preview stages receive an optional `SiteBoundary`. Route Debug reads only frozen outputs.

**Tech Stack:** Python 3.10, ROS 2 Humble, NumPy, SciPy, PyYAML, Shapely, PyQt5, pytest, `ament_cmake_python`.

## Global Constraints

- Branch: `feat/v25-12f-traversability-hard-boundary`
- Design spec: `docs/superpowers/specs/2026-08-15-v25-12f-traversability-hard-boundary-design.md`
- `site_boundary` means the vehicle-permitted inner perimeter
- Footprint touching or crossing the boundary is `SITE_BOUNDARY_CONFLICT`
- No READY Site Boundary means no 12F candidate
- No automatic wall extraction and no map-bounds fallback
- Never globally promote UNKNOWN to FREE
- Recovery is longitudinal in the row/aisle frame
- Initial maximum inferred gap is `0.60 m`
- Current OCCUPIED remains blocked in the A-first main candidate
- Current map padding remains in the main candidate
- `aisle_003` stays direct-obstacle conservative
- `aisle_005` stays structurally width-blocked
- `aisle_013` stays the padding-dominant probe
- R6B search parameters and budget remain unchanged
- Existing `navigation_map.yaml`, `navigation_map.pgm`, `derivation.yaml` remain untouched
- Fixed new outputs: `site_boundary.yaml`, `traversability_evidence.yaml`, `traversability_evidence.npz`, `navigation_map_12f.yaml`, `navigation_map_12f.pgm`, `navigation_map_12f_derivation.yaml`
- Compatibility mapping is fixed: observed/inferred -> FREE; hard/sensor/no-go -> OCCUPIED; unknown -> UNKNOWN
- Route Debug remains render-only
- Real acceptance directory: `/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run`

---

## Task 1: Site Boundary contract and strict geometry

**Files:**
- Create `src/agt_offline_assets/agt_offline_assets/site_boundary.py`
- Create `src/agt_offline_assets/test/test_site_boundary.py`
- Modify `src/agt_offline_assets/agt_offline_assets/__init__.py`
- Modify `src/agt_offline_assets/CMakeLists.txt`

**Public interface to implement exactly:**

- `SITE_BOUNDARY_SCHEMA: str = "agt_site_boundary/v1"`
- `SITE_BOUNDARY_SEMANTICS: str = "VEHICLE_PERMITTED_INNER_BOUNDARY"`
- `SiteBoundary(frame_id, outer_boundary_xy, source, schema, status, boundary_semantics)`
- `validate_site_boundary(boundary, expected_frame_id=None) -> None`
- `load_site_boundary(path, expected_frame_id=None) -> SiteBoundary`
- `write_site_boundary(boundary, path, overwrite=False) -> Path`
- `rasterize_site_boundary(boundary, grid, expected_frame_id=None) -> np.ndarray`
- `polygon_strictly_inside_site_boundary(boundary, polygon_xy) -> bool`

- [ ] **Step 1: Write the failing tests**

```python
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


def boundary_fixture():
    return SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0)),
        source={"authoring_mode": "WORKBENCH_MANUAL_POLYGON"},
    )


def grid_fixture():
    return NavigationGridEvidence(
        resolution_m=1.0,
        origin_x_m=-1.0,
        origin_y_m=-1.0,
        width=6,
        height=5,
        occupancy=np.full((5, 6), 205, dtype=np.uint8),
        frame_id="map",
    )


def test_round_trip_and_frame_check(tmp_path: Path):
    path = write_site_boundary(boundary_fixture(), tmp_path / "site_boundary.yaml")
    loaded = load_site_boundary(path, expected_frame_id="map")
    assert loaded.outer_boundary_xy == boundary_fixture().outer_boundary_xy
    assert loaded.boundary_semantics == "VEHICLE_PERMITTED_INNER_BOUNDARY"
    with pytest.raises(ValueError, match="frame_id mismatch"):
        load_site_boundary(path, expected_frame_id="odom")


def test_self_intersection_is_invalid():
    boundary = SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((0.0, 0.0), (2.0, 2.0), (0.0, 2.0), (2.0, 0.0)),
    )
    with pytest.raises(ValueError, match="simple"):
        validate_site_boundary(boundary)


def test_boundary_touch_is_conflict():
    boundary = boundary_fixture()
    inside = ((1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0))
    touching = ((0.0, 1.0), (1.0, 1.0), (1.0, 2.0), (0.0, 2.0))
    crossing = ((-0.1, 1.0), (1.0, 1.0), (1.0, 2.0), (-0.1, 2.0))
    assert polygon_strictly_inside_site_boundary(boundary, inside)
    assert not polygon_strictly_inside_site_boundary(boundary, touching)
    assert not polygon_strictly_inside_site_boundary(boundary, crossing)


def test_rasterize_returns_permitted_cell_centers():
    mask = rasterize_site_boundary(boundary_fixture(), grid_fixture())
    assert mask.shape == (5, 6)
    assert mask[1, 1]
    assert not mask[0, 0]
```

- [ ] **Step 2: Prove the tests fail before implementation**

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest -q src/agt_offline_assets/test/test_site_boundary.py
```

Expected: import/collection failure for the new API.

- [ ] **Step 3: Implement the dataclass and validation**

```python
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml
from shapely.geometry import Polygon

from .turn_zones import _points_inside_polygon

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

    def validate(self, *, expected_frame_id: str | None = None) -> None:
        validate_site_boundary(self, expected_frame_id=expected_frame_id)


def _validated_polygon(boundary: SiteBoundary) -> Polygon:
    points = np.asarray(boundary.outer_boundary_xy, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] < 3 or points.shape[1] != 2:
        raise ValueError("site_boundary requires at least three [x, y] vertices")
    if not np.all(np.isfinite(points)):
        raise ValueError("site_boundary vertices must be finite")
    if np.unique(points, axis=0).shape[0] < 3:
        raise ValueError("site_boundary requires at least three unique vertices")
    polygon = Polygon(points)
    if not polygon.is_valid or not polygon.exterior.is_simple:
        raise ValueError("site_boundary polygon must be simple and non-self-intersecting")
    if polygon.area <= 0.0:
        raise ValueError("site_boundary polygon must have non-zero area")
    return polygon


def validate_site_boundary(boundary: SiteBoundary, *, expected_frame_id: str | None = None) -> None:
    if boundary.schema != SITE_BOUNDARY_SCHEMA:
        raise ValueError(f"expected {SITE_BOUNDARY_SCHEMA}, got {boundary.schema}")
    if boundary.status != "READY":
        raise ValueError("site_boundary status must be READY")
    if boundary.boundary_semantics != SITE_BOUNDARY_SEMANTICS:
        raise ValueError("site_boundary boundary_semantics mismatch")
    if expected_frame_id is not None and boundary.frame_id != expected_frame_id:
        raise ValueError(
            f"site_boundary frame_id mismatch: expected {expected_frame_id}, got {boundary.frame_id}"
        )
    _validated_polygon(boundary)
```

- [ ] **Step 4: Implement strict footprint containment and rasterization**

```python
def polygon_strictly_inside_site_boundary(boundary: SiteBoundary, polygon_xy) -> bool:
    site = _validated_polygon(boundary)
    footprint = Polygon(np.asarray(polygon_xy, dtype=np.float64))
    if not footprint.is_valid or footprint.area <= 0.0:
        return False
    return bool(site.contains(footprint) and site.boundary.disjoint(footprint))


def rasterize_site_boundary(boundary: SiteBoundary, grid, *, expected_frame_id: str | None = None) -> np.ndarray:
    grid_frame = expected_frame_id
    if grid_frame is None:
        grid_frame = getattr(grid, "frame_id", None)
    validate_site_boundary(
        boundary,
        expected_frame_id=None if grid_frame is None else str(grid_frame),
    )
    rows, cols = np.indices((int(grid.height), int(grid.width)), dtype=np.float64)
    x = float(grid.origin_x_m) + (cols + 0.5) * float(grid.resolution_m)
    y = float(grid.origin_y_m) + (rows + 0.5) * float(grid.resolution_m)
    return _points_inside_polygon(x, y, boundary.outer_boundary_xy).astype(bool)
```

- [ ] **Step 5: Implement YAML round-trip and overwrite protection**

Write exactly:

```yaml
schema: agt_site_boundary/v1
frame_id: map
status: READY
boundary_semantics: VEHICLE_PERMITTED_INNER_BOUNDARY
outer_boundary_xy: []
source:
  authoring_mode: WORKBENCH_MANUAL_POLYGON
```

`write_site_boundary()` raises `FileExistsError` unless `overwrite=True`.

- [ ] **Step 6: Export API, register test, verify**

Add to CMake:

```cmake
ament_add_pytest_test(test_site_boundary test/test_site_boundary.py)
```

Run:

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_site_boundary.py
```

Expected: PASS.

- [ ] **Step 7: Commit**

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
- Modify `src/agt_map_workbench/agt_map_workbench/agricultural_workbench.py`
- Create `src/agt_map_workbench/test/test_site_boundary_workbench.py`
- Modify `src/agt_map_workbench/CMakeLists.txt`

- [ ] **Step 1: Write failing headless tests**

```python
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtWidgets import QApplication
from agt_map_workbench.review_workbench import ReviewMapWorkbenchWindow


def qapp():
    return QApplication.instance() or QApplication([])


def test_finish_site_boundary_creates_ready_asset():
    app = qapp()
    window = ReviewMapWorkbenchWindow()
    try:
        window._site_boundary_vertices = [(0, 0), (5, 0), (5, 4), (0, 4)]
        assert window._finish_site_boundary_authoring(show_errors=False)
        assert window._site_boundary is not None
        assert window._site_boundary.status == "READY"
        app.processEvents()
    finally:
        window.close()


def test_clear_boundary_does_not_clear_navigation_overrides():
    app = qapp()
    window = ReviewMapWorkbenchWindow()
    try:
        window._navigation_overrides = [
            {"mode": "no_go", "polygon_xy": [[1, 1], [2, 1], [2, 2]]}
        ]
        window._site_boundary_vertices = [(0, 0), (5, 0), (5, 4)]
        window._clear_site_boundary()
        assert len(window._navigation_overrides) == 1
    finally:
        window.close()
```

- [ ] **Step 2: Run and confirm missing methods**

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q src/agt_map_workbench/test/test_site_boundary_workbench.py
```

- [ ] **Step 3: Add dedicated boundary state before the base constructor**

Initialize:

```python
self._site_boundary = None
self._site_boundary_vertices = []
self._site_boundary_item = None
self._site_boundary_vertex_items = []
```

Do not reuse Navigation Override state.

- [ ] **Step 4: Add navigation-page controls**

```text
Site Boundary（车辆允许区域内边界）
[开始绘制] [完成] [撤销顶点] [清除]
[导出 site_boundary.yaml]
状态：未定义 / 草稿 N 点 / READY / INVALID
```

- [ ] **Step 5: Handle clicks with a dedicated interaction mode**

```python
def _on_map_clicked(self, x: float, y: float) -> None:
    if self._interaction_mode == "site_boundary":
        self._append_site_boundary_vertex(x, y)
        return
    super()._on_map_clicked(x, y)
```

- [ ] **Step 6: Implement finish/undo/clear without modifying NO_GO or other overrides**

`_finish_site_boundary_authoring(show_errors=False)` returns `False` on invalid polygon and preserves the last READY boundary.

- [ ] **Step 7: Render Site Boundary distinctly**

Use a solid edge and low-opacity permitted-area fill. Authoring vertices are numbered. Do not use the NO_GO magenta/purple encoding.

- [ ] **Step 8: Auto-load sibling boundary after PCD load**

```python
candidate = self._source_path.parent / "site_boundary.yaml"
if candidate.is_file():
    self._site_boundary = load_site_boundary(candidate, expected_frame_id="map")
```

Invalid sibling file causes explicit warning and leaves `_site_boundary = None`.

- [ ] **Step 9: Export READY boundary to `self._source_path.parent / "site_boundary.yaml"` by default**

- [ ] **Step 10: Register/run tests and commit**

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

## Task 3: Preserve `aisle_geometric_envelope`

**Files:**
- Modify `src/agt_offline_assets/agt_offline_assets/navigation_corridor.py`
- Modify `src/agt_offline_assets/test/test_navigation_corridor.py`
- Modify `src/agt_map_workbench/agt_map_workbench/agricultural_workbench.py`
- Modify `src/agt_map_workbench/agt_map_workbench/navigation_preview.py`

- [ ] **Step 1: Add failing geometry-vs-ground tests**

```python
def test_geometric_envelope_survives_ground_hole():
    navigation, structure = _fixture()
    navigation.ground_valid[14:17, 20:24] = False
    result = derive_corridor_refinement(navigation, structure, _feasible_config())
    assert np.any(result.aisle_geometric_envelope[14:17, 20:24])
    assert not np.any(result.aisle_candidate[14:17, 20:24])
    assert np.all(~result.aisle_candidate | result.aisle_geometric_envelope)


def test_width_rejected_pair_has_empty_geometric_envelope():
    navigation, structure = _fixture()
    result = derive_corridor_refinement(navigation, structure)
    assert np.count_nonzero(result.aisle_geometric_envelope) == 0
```

- [ ] **Step 2: Run; expect missing field**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_navigation_corridor.py
```

- [ ] **Step 3: Extend `CorridorRefinementResult` with `aisle_geometric_envelope`**

For a structurally accepted pair only:

```python
geometry = longitudinal & (vv >= corridor_min) & (vv <= corridor_max)
safe = geometry & safe_base
```

Change `evaluate_corridor()` to return `geometry, safe, centerline, diagnostic`. Width/support/overlap rejection returns an all-false geometry mask. Accumulate accepted geometry into one union mask.

- [ ] **Step 4: Add Workbench preview key**

```text
结构行道几何包络（未经过 Ground FREE 过滤）
```

The preview color is distinct and low-opacity.

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

## Task 4: Rich traversability and directional recovery

**Files:**
- Create `src/agt_offline_assets/agt_offline_assets/traversability.py`
- Create `src/agt_offline_assets/test/test_traversability.py`
- Modify `src/agt_offline_assets/agt_offline_assets/__init__.py`
- Modify `src/agt_offline_assets/CMakeLists.txt`

**Implement these exact data members:**

```text
TraversabilityConfig.maximum_inferred_gap_m = 0.60
TraversabilityEvidence.frame_id
TraversabilityEvidence.observed_free_mask
TraversabilityEvidence.inferred_traversable_mask
TraversabilityEvidence.hard_blocked_mask
TraversabilityEvidence.sensor_obstacle_mask
TraversabilityEvidence.unknown_mask
TraversabilityEvidence.semantic_no_go_mask
TraversabilityEvidence.aisle_geometric_envelope_mask
TraversabilityEvidence.config
TraversabilityEvidence.source
```

- [ ] **Step 1: Write synthetic tests for six required cases**

```text
0.4 m longitudinal UNKNOWN hole -> inferred
0.9 m longitudinal UNKNOWN hole -> remains UNKNOWN
row structural band -> not inferred
current OCCUPIED -> not inferred
NO_GO -> not inferred
outside Site Boundary -> hard blocked
```

Required intersection assertions:

```python
assert not np.any(evidence.inferred_traversable_mask & evidence.hard_blocked_mask)
assert not np.any(evidence.inferred_traversable_mask & evidence.semantic_no_go_mask)
assert not np.any(evidence.inferred_traversable_mask & evidence.sensor_obstacle_mask)
```

- [ ] **Step 2: Run; expect import failure**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_traversability.py
```

- [ ] **Step 3: Implement initial masks from current Navigation Map**

```python
occupancy = np.asarray(navigation.occupancy, dtype=np.uint8)
base_free = occupancy == FREE
base_occupied = occupancy == OCCUPIED
base_unknown = occupancy == UNKNOWN
inside_boundary = rasterize_site_boundary(
    site_boundary,
    navigation,
    expected_frame_id=frame_id,
)
hard_blocked = ~inside_boundary
semantic_no_go = _rasterize_no_go(navigation, overrides)
```

The main A-first candidate treats current OCCUPIED as non-recoverable.

- [ ] **Step 4: Implement `_rasterize_no_go()` using the same world-cell-center convention as Navigation Override**

Only overrides with `mode == "no_go"` enter `semantic_no_go_mask`.

- [ ] **Step 5: Implement longitudinal recovery in row coordinates**

```python
direction = np.asarray(structure.row_model.direction_xy, dtype=np.float64)
direction /= np.linalg.norm(direction)
perpendicular = np.array([-direction[1], direction[0]], dtype=np.float64)
rows, cols = np.indices(navigation.occupancy.shape, dtype=np.float64)
xx = navigation.origin_x_m + (cols + 0.5) * navigation.resolution_m
yy = navigation.origin_y_m + (rows + 0.5) * navigation.resolution_m
u = xx * direction[0] + yy * direction[1]
v = xx * perpendicular[0] + yy * perpendicular[1]
v_bin = np.rint((v - float(np.min(v))) / navigation.resolution_m).astype(np.int64)
```

For every lateral bin, sort cells by `u`. Infer one run only if it is entirely UNKNOWN, lies inside `aisle_geometric_envelope`, is bracketed by FREE support, is at most `maximum_inferred_gap_m`, avoids row band/no-go/hard/current OCCUPIED, and the two observed endpoint heights satisfy both configured maximum slope and maximum step limits.

- [ ] **Step 6: Apply precedence exactly**

```text
HARD_BLOCKED > NO_GO > SENSOR_OBSTACLE > INFERRED_TRAVERSABLE > OBSERVED_FREE > UNKNOWN
```

Candidate occupancy construction:

```python
result = np.full(shape, UNKNOWN, dtype=np.uint8)
result[observed_free | inferred] = FREE
result[sensor_obstacle | semantic_no_go | hard_blocked] = OCCUPIED
```

- [ ] **Step 7: Register/run tests and commit**

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

## Task 5: Candidate serialization and Workbench generation

**Files:**
- Modify `src/agt_offline_assets/agt_offline_assets/traversability.py`
- Modify `src/agt_offline_assets/test/test_traversability.py`
- Modify `src/agt_map_workbench/agt_map_workbench/agricultural_workbench.py`
- Modify `src/agt_map_workbench/agt_map_workbench/navigation_preview.py`
- Modify `src/agt_map_workbench/test/test_site_boundary_workbench.py`

- [ ] **Step 1: Write failing serialization round-trip tests**

Writer output is exactly:

```text
traversability_evidence.yaml
traversability_evidence.npz
navigation_map_12f.yaml
navigation_map_12f.pgm
navigation_map_12f_derivation.yaml
```

Test that bytes of canonical `navigation_map.yaml`, `navigation_map.pgm`, `derivation.yaml` are unchanged.

- [ ] **Step 2: Run; expect missing writer/loader**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_traversability.py
```

- [ ] **Step 3: Implement NPZ with exact keys**

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

- [ ] **Step 4: Implement candidate PGM/YAML/derivation metadata**

Derivation YAML records source Site Boundary hash, current-map source identity, grid geometry, state counts, `maximum_inferred_gap_m`, requested obstacle padding, effective padding cells, candidate counts, and hashes of candidate PGM/YAML/NPZ.

- [ ] **Step 5: Implement loader validation**

Loader checks schema, frame, NPZ mask shapes, candidate-grid dimensions, and all required keys before returning evidence.

- [ ] **Step 6: Add Workbench state and controls**

```text
V25-12F Candidate
最大遮挡补全缺口：0.60 m
[生成 12F Candidate]
[导出到当前 run 目录]
```

Required state:

```python
self._traversability_evidence = None
self._navigation_12f_result = None
```

Generate only when navigation, structure, corridor, and READY Site Boundary exist.

- [ ] **Step 7: Construct candidate without mutating current result**

```python
self._navigation_12f_result = replace(
    self._navigation_result,
    occupancy=self._traversability_evidence.candidate_occupancy(),
)
```

Never assign this object to `_navigation_result`.

- [ ] **Step 8: Add preview entries for candidate and rich states**

```text
12F Candidate 三态
12F OBSERVED_FREE
12F INFERRED_TRAVERSABLE
12F HARD_BLOCKED
12F SENSOR_OBSTACLE
12F UNKNOWN
12F Aisle Geometric Envelope
```

- [ ] **Step 9: Export to current PCD parent and require explicit overwrite confirmation only for 12F candidate files**

- [ ] **Step 10: Run tests and commit**

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

## Task 6: Site Boundary enforcement in lane/connector preview

**Files:**
- Modify `src/agt_offline_assets/agt_offline_assets/vehicle_safe_lane.py`
- Modify `src/agt_offline_assets/agt_offline_assets/forward_connector_navigation_gate.py`
- Modify `src/agt_offline_assets/agt_offline_assets/reverse_primitive_connector.py`
- Modify corresponding three pytest files

**Signature changes:** add keyword-only `site_boundary: SiteBoundary | None = None` to the three public derive functions while preserving current behavior when omitted.

- [ ] **Step 1: Add tests where pose center is inside but the footprint touches/crosses boundary**

All three modules must reject the pose even when underlying Navigation Grid cells are FREE.

- [ ] **Step 2: Run focused tests and confirm new keyword is missing**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_safe_lane.py \
  src/agt_offline_assets/test/test_forward_connector_navigation_gate.py \
  src/agt_offline_assets/test/test_reverse_primitive_connector.py
```

- [ ] **Step 3: Add `_preview_pose_status()` in reverse primitive module**

```python
def _preview_pose_status(x, y, yaw, navigation, local_footprint, site_boundary=None):
    sample = ForwardConnectorSample(x=float(x), y=float(y), z=0.0, yaw=float(yaw))
    polygon = _transform_polygon(local_footprint, sample)
    if site_boundary is not None:
        if not polygon_strictly_inside_site_boundary(site_boundary, polygon):
            return "SITE_BOUNDARY_CONFLICT"
    # Execute the existing grid bounds and all-FREE tests here unchanged.
    # Return NAVIGATION_GRID_OUT_OF_BOUNDS, NAVIGATION_NOT_FREE, or PREVIEW_POSE_FREE.
```

The implementation must literally move the existing grid-check code into this function; do not weaken it. `_preview_pose_free()` remains as a compatibility wrapper returning whether status equals `PREVIEW_POSE_FREE`.

- [ ] **Step 4: Vehicle-Safe Lane passes Site Boundary to every lateral pose probe**

Width rejection remains unchanged. If boundary rejection is the reason no candidate pose exists, include `SITE_BOUNDARY_CONFLICT` in lane reason.

- [ ] **Step 5: Forward gate checks every sampled transformed footprint**

A boundary-conflicting candidate cannot emit `PREVIEW_FOOTPRINT_FREE`.

- [ ] **Step 6: R6B checks start pose, every primitive sample, and goal-shot sample**

Boundary-conflicting start returns:

```text
status = SITE_BOUNDARY_CONFLICT
search_expansions = 0
```

No R6B config value changes.

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

## Task 7: Route Debug 12F A/B

**Files:**
- Modify `src/agt_offline_assets/agt_offline_assets/route_debug_dataset.py`
- Modify `src/agt_offline_assets/agt_offline_assets/route_debug_overlay.py`
- Modify their tests
- Modify `src/agt_map_workbench/agt_map_workbench/route_debug_view.py`
- Modify `src/agt_map_workbench/agt_map_workbench/route_debug_panel.py`
- Modify their tests

**Dataset additions:** `candidate_navigation`, `site_boundary`, `traversability`.

**New asset keys:** `site_boundary`, `navigation_12f`, `traversability_evidence`.

**New layer keys:**

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

- [ ] **Step 1: Add dataset tests for LOADED/MISSING/INVALID 12F assets**

12E data must continue loading when optional 12F data is absent or invalid.

- [ ] **Step 2: Add overlay test for one Site Boundary polygon**

Required properties:

```text
layer_key = semantics.site_boundary
feature_kind = SITE_BOUNDARY
status = READY
source_asset = site_boundary.yaml
boundary_semantics = VEHICLE_PERMITTED_INNER_BOUNDARY
```

- [ ] **Step 3: Load candidate map and traversability evidence**

Use:

```python
load_navigation_grid(run_dir / "navigation_map_12f.yaml")
load_traversability_evidence(run_dir, expected_frame_id=dataset.frame_id)
```

- [ ] **Step 4: Render rich masks as rasters and Site Boundary as vector outline**

Do not create one GeoJSON feature per cell.

- [ ] **Step 5: Add `12F A/B` preset**

Enable current map, candidate map, inferred, hard-blocked, geometric envelope, Site Boundary, NO_GO, and structural aisles. Current/candidate base rasters remain manually toggleable.

- [ ] **Step 6: Run Route Debug tests and commit**

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

## Task 8: Repeatable acceptance harness and documentation

**Files:**
- Create `tools/v25_12f_acceptance.py`
- Create `tests/test_v25_12f_traversability_contract.py`
- Modify `src/agt_map_workbench/README.md`
- Modify `docs/v2.5/V25_12E_CURRENT_STATE.md`

- [ ] **Step 1: Add cross-package contract test**

```python
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_v25_12f_contract_tokens():
    text = (
        ROOT / "docs/superpowers/specs/2026-08-15-v25-12f-traversability-hard-boundary-design.md"
    ).read_text(encoding="utf-8")
    for token in (
        "VEHICLE_PERMITTED_INNER_BOUNDARY",
        "SITE_BOUNDARY_CONFLICT",
        "maximum_inferred_gap_m",
        "navigation_map_12f.pgm",
        "traversability_evidence.npz",
    ):
        assert token in text


def test_qt_does_not_own_12f_planning_math():
    text = (
        ROOT / "src/agt_map_workbench/agt_map_workbench/agricultural_workbench.py"
    ).read_text(encoding="utf-8")
    assert "def derive_traversability_evidence" not in text
    assert "binary_closing" not in text
    assert "max_expansions" not in text
```

- [ ] **Step 2: Implement acceptance script argument parsing and fixed inputs**

```python
parser = argparse.ArgumentParser()
parser.add_argument("--run-dir", required=True)
parser.add_argument("--vehicle-profile", required=True)
args = parser.parse_args()

run_dir = Path(args.run_dir).resolve()
graph = load_agricultural_aisle_graph(run_dir / "aisle_graph.yaml")
current_nav = load_navigation_grid(run_dir / "navigation_map.yaml")
candidate_nav = load_navigation_grid(run_dir / "navigation_map_12f.yaml")
boundary = load_site_boundary(run_dir / "site_boundary.yaml", expected_frame_id="map")
vehicle = load_canonical_vehicle_profile(args.vehicle_profile)
```

- [ ] **Step 3: Compare Vehicle-Safe Lane with one identical config**

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
    graph,
    current_nav,
    vehicle,
    lane_cfg,
    site_boundary=boundary,
)
candidate_lane = derive_vehicle_safe_lane_plan(
    graph,
    candidate_nav,
    vehicle,
    lane_cfg,
    site_boundary=boundary,
)
```

Print JSON for current/candidate: ready, partial, unavailable, mean coverage, per-aisle status/coverage/reason, and count of reasons containing `SITE_BOUNDARY_CONFLICT`.

- [ ] **Step 4: Replay connector_015 and connector_017 with unchanged default R6B config**

```python
requests = load_coverage_connector_requests(run_dir / "coverage_order.yaml")
zones = load_turn_zones(run_dir / "turn_zones.yaml")
selected = tuple(
    request
    for request in requests
    if request.connector_id in {"connector_015", "connector_017"}
)
items = tuple(
    ReverseFallbackAdmissionItem(
        connector_id=request.connector_id,
        from_aisle_id=request.from_aisle_id,
        to_aisle_id=request.to_aisle_id,
        turn_zone_id=request.turn_zone_id,
        forward_audit_status="FROZEN_REGRESSION_REPLAY",
        decision="ELIGIBLE_REVERSE_FALLBACK",
        reason="V25-12F candidate-map regression replay of previously admitted connector",
    )
    for request in selected
)
admission = ReverseFallbackAdmissionPlan(
    frame_id=zones.frame_id,
    platform_id=vehicle.profile_id,
    platform_profile_sha256=vehicle.profile_sha256,
    items=items,
    source={"validation_scope": "V25_12F_REGRESSION_REPLAY_ONLY"},
)
config = ReversePrimitiveConnectorConfig()
current_r6b = derive_reverse_primitive_connector_plan(
    selected,
    admission,
    zones,
    current_nav,
    vehicle,
    config,
    site_boundary=boundary,
)
candidate_r6b = derive_reverse_primitive_connector_plan(
    selected,
    admission,
    zones,
    candidate_nav,
    vehicle,
    config,
    site_boundary=boundary,
)
```

Print status, path length, reverse length, cusp count, expansions, goal errors, and reason for both maps.

- [ ] **Step 5: Run all focused automated tests**

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

- [ ] **Step 6: Clean-build affected packages**

```bash
rm -rf build/agt_offline_assets build/agt_map_workbench \
       install/agt_offline_assets install/agt_map_workbench
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  agt_offline_assets agt_map_workbench \
  --event-handlers console_direct+
source install/setup.bash
```

- [ ] **Step 7: Author the real Site Boundary**

```bash
ros2 run agt_map_workbench agt_map_workbench
```

Open `/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run/processed.pcd`, draw the permitted inner perimeter, and export `/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run/site_boundary.yaml`.

- [ ] **Step 8: Generate/export candidate and preserve canonical hashes**

Before and after export run:

```bash
sha256sum \
  runtime/maps/agt_workbench_run/navigation_map.yaml \
  runtime/maps/agt_workbench_run/navigation_map.pgm \
  runtime/maps/agt_workbench_run/derivation.yaml
```

The three hashes must not change.

- [ ] **Step 9: Run the repeatable A/B harness**

```bash
python3 tools/v25_12f_acceptance.py \
  --run-dir /home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run \
  --vehicle-profile /home/yangxuan/agt_navigation_v2/profiles/platforms/mk_mini.yaml
```

- [ ] **Step 10: Route Debug visual acceptance**

Verify: Site Boundary distinct from NO_GO; no inferred cells outside boundary; short gaps recovered only inside structural envelope; no lateral crop-row bridge; aisle_003 remains blocked; aisle_005 remains width rejected; aisle_013 remains padding-diagnostic; connector_015/017 match harness output and never violate boundary.

- [ ] **Step 11: Update docs only with measured results and commit**

Before real review, docs say:

```text
V25-12F CORE IMPLEMENTED / REAL-DATA A/B PENDING
```

Only promote after Steps 7-10 succeed.

```bash
git add tools/v25_12f_acceptance.py \
        tests/test_v25_12f_traversability_contract.py \
        src/agt_map_workbench/README.md \
        docs/v2.5/V25_12E_CURRENT_STATE.md
git commit -m "test(v25-12f): lock traversability acceptance contract"
```

---

## Final Verification Checklist

```text
[ ] boundary YAML round-trip passes
[ ] self-intersection fails closed
[ ] touching footprint is boundary conflict
[ ] rasterizer works with NavigationMapResult without assuming frame_id exists
[ ] missing boundary prevents candidate generation
[ ] structural geometric envelope survives ground holes
[ ] structurally narrow aisle never enters recovery envelope
[ ] 0.4 m test gap recovers
[ ] 0.9 m test gap remains UNKNOWN
[ ] no generic 2D closing exists
[ ] current OCCUPIED is not recovered
[ ] NO_GO beats inference
[ ] hard boundary beats every other state
[ ] canonical map hashes stay unchanged
[ ] candidate files use fixed names
[ ] lane preview enforces continuous boundary
[ ] forward connector preview enforces continuous boundary
[ ] R6B enforces continuous boundary without config changes
[ ] Route Debug 12F assets are optional and fail closed per layer
[ ] current/candidate A/B view works
[ ] acceptance harness uses identical MK-mini config for both maps
[ ] connector_015/017 replay uses one unchanged ReversePrimitiveConnectorConfig
[ ] zero Site Boundary footprint violations in real acceptance
[ ] no REAL-DATA PASS before operator review
```
