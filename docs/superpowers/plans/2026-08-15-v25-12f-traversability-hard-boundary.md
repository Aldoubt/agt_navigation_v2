# AGT Navigation V25-12F Traversability + Hard Boundary Repair Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a manually authored vehicle-permitted greenhouse boundary, preserve structural aisle geometry before ground-valid filtering, recover only short vegetation-occluded longitudinal aisle gaps, export a non-destructive V25-12F candidate Navigation Map, enforce the site boundary at vehicle-footprint level, and expose all new evidence in Route Debug for real-data A/B acceptance.

**Architecture:** Keep deterministic production semantics in `agt_offline_assets` and keep Qt responsible only for authoring/rendering. `site_boundary.py` owns the frozen hard-boundary contract and geometry checks, `navigation_corridor.py` exposes the structural aisle envelope, and `traversability.py` owns rich evidence states plus candidate-map serialization. Existing vehicle-lane/connector preview gates receive an optional `SiteBoundary` and fail closed at footprint level; Route Debug reads the frozen 12F outputs and remains render-only.

**Tech Stack:** Python 3.10, ROS 2 Humble, `ament_cmake_python`, NumPy, SciPy, PyYAML, Shapely, PyQt5, pytest, existing AGT offline asset contracts and Route Debug infrastructure.

## Global Constraints

- Base implementation branch: `feat/v25-12f-traversability-hard-boundary`
- Approved design: `docs/superpowers/specs/2026-08-15-v25-12f-traversability-hard-boundary-design.md`
- `site_boundary` means the **vehicle-permitted inner boundary**, not a wall centerline
- A continuous vehicle footprint touching or crossing `site_boundary` is `SITE_BOUNDARY_CONFLICT`
- A READY `site_boundary.yaml` is mandatory before any 12F candidate map is generated
- Never infer a fake boundary from map bounds and never automatically fit walls in V25-12F
- Never implement global `UNKNOWN -> FREE`
- Bounded recovery is longitudinal along the agricultural aisle direction only
- Initial `maximum_inferred_gap_m = 0.60`
- Direct current-map OCCUPIED evidence is never recovered in the A-first candidate
- `aisle_003` remains a direct-obstacle regression probe
- `aisle_005` remains structurally width-blocked when its width gate fails
- `aisle_013` remains the padding-dominant diagnostic probe
- Main 12F candidate preserves the currently frozen map-padding setting; no-map-padding remains a diagnostic counterfactual, not an automatic promotion
- No R6B search parameter, search budget, admission gate, or R7 implementation changes in this increment
- Keep the current frozen `navigation_map.yaml`, `navigation_map.pgm`, and `derivation.yaml` untouched during acceptance
- Candidate outputs use the fixed names `site_boundary.yaml`, `traversability_evidence.yaml`, `traversability_evidence.npz`, `navigation_map_12f.yaml`, `navigation_map_12f.pgm`, and `navigation_map_12f_derivation.yaml`
- Rich compatibility mapping is fixed: `OBSERVED_FREE -> FREE`, `INFERRED_TRAVERSABLE -> FREE`, `HARD_BLOCKED -> OCCUPIED`, `SENSOR_OBSTACLE -> OCCUPIED`, `UNKNOWN -> UNKNOWN`, semantic `NO_GO -> OCCUPIED`
- Route Debug remains read-only for production truth and may only render/load the new frozen evidence
- Real-data acceptance directory: `/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run`
- No `REAL-DATA PASS` claim from synthetic tests alone

---

## File Structure

### New files

- `src/agt_offline_assets/agt_offline_assets/site_boundary.py`
  - Frozen schema, validation, YAML I/O, rasterization, strict continuous-polygon containment
- `src/agt_offline_assets/agt_offline_assets/traversability.py`
  - Rich traversability evidence, directional bounded-gap recovery, candidate map generation, YAML/NPZ/PGM serialization and reload
- `src/agt_offline_assets/test/test_site_boundary.py`
  - Pure geometry and contract tests
- `src/agt_offline_assets/test/test_traversability.py`
  - Rich-state, recovery, precedence, serialization tests
- `src/agt_map_workbench/test/test_site_boundary_workbench.py`
  - Headless Site Boundary authoring/reload tests
- `tests/test_v25_12f_traversability_contract.py`
  - Cross-package architecture/output/safety contract lock

### Modified files

- `src/agt_offline_assets/agt_offline_assets/navigation_corridor.py`
  - Preserve `aisle_geometric_envelope` before `safe_base`
- `src/agt_offline_assets/agt_offline_assets/vehicle_safe_lane.py`
  - Optional footprint-level `SiteBoundary` enforcement
- `src/agt_offline_assets/agt_offline_assets/forward_connector_navigation_gate.py`
  - Optional boundary gate for every swept preview footprint
- `src/agt_offline_assets/agt_offline_assets/reverse_primitive_connector.py`
  - Optional boundary gate for start pose, primitive samples, and goal-shot samples
- `src/agt_offline_assets/agt_offline_assets/__init__.py`
  - Export the new public contracts
- `src/agt_offline_assets/CMakeLists.txt`
  - Register new pytest files
- `src/agt_map_workbench/agt_map_workbench/agricultural_workbench.py`
  - Site Boundary authoring, candidate generation/export state, navigation-layer entries
- `src/agt_map_workbench/agt_map_workbench/navigation_preview.py`
  - Preview `aisle_geometric_envelope` and 12F candidate/evidence layers
- `src/agt_map_workbench/agt_map_workbench/route_debug_panel.py`
  - 12F A/B preset and availability labels
- `src/agt_map_workbench/agt_map_workbench/route_debug_view.py`
  - Candidate raster, rich-state masks, Site Boundary layer rendering
- `src/agt_offline_assets/agt_offline_assets/route_debug_dataset.py`
  - Load `site_boundary`, candidate Navigation Grid, traversability evidence
- `src/agt_offline_assets/agt_offline_assets/route_debug_overlay.py`
  - Site Boundary vector provenance and 12F summary features
- `src/agt_map_workbench/CMakeLists.txt`
  - Register Site Boundary Workbench test if needed as a standalone test
- `src/agt_map_workbench/README.md`
  - Authoring and A/B operator workflow
- `docs/v2.5/V25_12E_CURRENT_STATE.md`
  - Record V25-12F implementation/acceptance state without erasing frozen 12E evidence

---

### Task 1: Add the frozen Site Boundary contract and strict footprint geometry

**Files:**
- Create: `src/agt_offline_assets/agt_offline_assets/site_boundary.py`
- Create: `src/agt_offline_assets/test/test_site_boundary.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/__init__.py`
- Modify: `src/agt_offline_assets/CMakeLists.txt`

**Interfaces:**
- Produces:
  - `SITE_BOUNDARY_SCHEMA = "agt_site_boundary/v1"`
  - `SITE_BOUNDARY_SEMANTICS = "VEHICLE_PERMITTED_INNER_BOUNDARY"`
  - `SiteBoundary`
  - `validate_site_boundary(boundary, expected_frame_id=None) -> None`
  - `load_site_boundary(path, expected_frame_id=None) -> SiteBoundary`
  - `write_site_boundary(boundary, path, overwrite=False) -> Path`
  - `rasterize_site_boundary(boundary, grid) -> np.ndarray`
  - `polygon_strictly_inside_site_boundary(boundary, polygon_xy) -> bool`

- [ ] **Step 1: Write the failing Site Boundary contract tests**

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


def _grid():
    return NavigationGridEvidence(
        resolution_m=1.0,
        origin_x_m=-1.0,
        origin_y_m=-1.0,
        width=6,
        height=5,
        occupancy=np.full((5, 6), 205, dtype=np.uint8),
        frame_id="map",
    )


def test_site_boundary_round_trip_and_frame_check(tmp_path: Path):
    path = write_site_boundary(_boundary(), tmp_path / "site_boundary.yaml")
    loaded = load_site_boundary(path, expected_frame_id="map")
    assert loaded.boundary_semantics == "VEHICLE_PERMITTED_INNER_BOUNDARY"
    assert loaded.outer_boundary_xy == _boundary().outer_boundary_xy
    with pytest.raises(ValueError, match="frame_id mismatch"):
        load_site_boundary(path, expected_frame_id="odom")


def test_site_boundary_rejects_self_intersection():
    with pytest.raises(ValueError, match="simple"):
        SiteBoundary(
            frame_id="map",
            outer_boundary_xy=((0.0, 0.0), (2.0, 2.0), (0.0, 2.0), (2.0, 0.0)),
        ).validate()


def test_rasterization_keeps_only_permitted_inner_area():
    mask = rasterize_site_boundary(_boundary(), _grid())
    assert mask.shape == (5, 6)
    assert mask[1, 1]
    assert mask[3, 4]
    assert not mask[0, 0]
    assert not mask[-1, -1]


def test_footprint_touching_boundary_is_rejected():
    boundary = _boundary()
    inside = ((1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0))
    touching = ((0.0, 1.0), (1.0, 1.0), (1.0, 2.0), (0.0, 2.0))
    crossing = ((-0.1, 1.0), (1.0, 1.0), (1.0, 2.0), (-0.1, 2.0))
    assert polygon_strictly_inside_site_boundary(boundary, inside)
    assert not polygon_strictly_inside_site_boundary(boundary, touching)
    assert not polygon_strictly_inside_site_boundary(boundary, crossing)
```

- [ ] **Step 2: Run the new test and confirm the API is missing**

Run:

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest -q src/agt_offline_assets/test/test_site_boundary.py
```

Expected: collection/import failure because `SiteBoundary` and related functions do not yet exist.

- [ ] **Step 3: Implement the Site Boundary dataclass, validation, I/O, rasterization, and strict polygon containment**

```python
# src/agt_offline_assets/agt_offline_assets/site_boundary.py
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Sequence

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


def _polygon(boundary: SiteBoundary) -> Polygon:
    polygon = Polygon(boundary.outer_boundary_xy)
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
    points = np.asarray(boundary.outer_boundary_xy, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] < 3 or points.shape[1] != 2:
        raise ValueError("site_boundary requires at least three [x, y] vertices")
    if not np.all(np.isfinite(points)):
        raise ValueError("site_boundary vertices must be finite")
    if np.unique(points, axis=0).shape[0] < 3:
        raise ValueError("site_boundary requires at least three unique vertices")
    _polygon(boundary)


def polygon_strictly_inside_site_boundary(
    boundary: SiteBoundary,
    polygon_xy: Sequence[Sequence[float]],
) -> bool:
    validate_site_boundary(boundary)
    footprint = Polygon(np.asarray(polygon_xy, dtype=np.float64))
    if not footprint.is_valid or footprint.area <= 0.0:
        return False
    site = _polygon(boundary)
    return bool(site.contains(footprint) and site.boundary.disjoint(footprint))


def rasterize_site_boundary(boundary: SiteBoundary, grid) -> np.ndarray:
    validate_site_boundary(boundary, expected_frame_id=str(grid.frame_id))
    rows, cols = np.indices((int(grid.height), int(grid.width)), dtype=np.float64)
    x = float(grid.origin_x_m) + (cols + 0.5) * float(grid.resolution_m)
    y = float(grid.origin_y_m) + (rows + 0.5) * float(grid.resolution_m)
    return _points_inside_polygon(x, y, boundary.outer_boundary_xy).astype(bool)
```

Implement YAML read/write with the exact approved fields and refuse overwrite unless `overwrite=True`.

- [ ] **Step 4: Export the new API from `agt_offline_assets.__init__` and register the pytest target**

Add the new imports to `src/agt_offline_assets/agt_offline_assets/__init__.py` and:

```cmake
ament_add_pytest_test(test_site_boundary test/test_site_boundary.py)
```

to `src/agt_offline_assets/CMakeLists.txt`.

- [ ] **Step 5: Run the focused tests**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_site_boundary.py
```

Expected: PASS.

- [ ] **Step 6: Commit Task 1**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/site_boundary.py \
  src/agt_offline_assets/agt_offline_assets/__init__.py \
  src/agt_offline_assets/test/test_site_boundary.py \
  src/agt_offline_assets/CMakeLists.txt
git commit -m "feat(v25-12f): add site boundary contract"
```

---

### Task 2: Add Site Boundary authoring and sibling reload to the Workbench

**Files:**
- Modify: `src/agt_map_workbench/agt_map_workbench/agricultural_workbench.py`
- Create: `src/agt_map_workbench/test/test_site_boundary_workbench.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**
- Consumes: `SiteBoundary`, `load_site_boundary`, `write_site_boundary`
- Produces Workbench state:
  - `self._site_boundary: SiteBoundary | None`
  - `self._site_boundary_vertices: list[tuple[float, float]]`
  - `_start_site_boundary_authoring()`
  - `_append_site_boundary_vertex(x, y)`
  - `_undo_site_boundary_vertex()`
  - `_finish_site_boundary_authoring()`
  - `_clear_site_boundary()`
  - `_load_sibling_site_boundary()`
  - `_export_site_boundary()`

- [ ] **Step 1: Write headless tests for authoring state, strict validation, and sibling reload**

```python
# src/agt_map_workbench/test/test_site_boundary_workbench.py
import os
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtWidgets import QApplication

from agt_map_workbench.review_workbench import ReviewMapWorkbenchWindow
from agt_offline_assets import SiteBoundary, write_site_boundary


def _qapp():
    return QApplication.instance() or QApplication([])


def test_site_boundary_authoring_builds_ready_inner_polygon():
    app = _qapp()
    window = ReviewMapWorkbenchWindow()
    try:
        window._site_boundary_vertices = [
            (0.0, 0.0), (5.0, 0.0), (5.0, 4.0), (0.0, 4.0)
        ]
        assert window._finish_site_boundary_authoring(show_errors=False)
        assert window._site_boundary is not None
        assert window._site_boundary.status == "READY"
        assert window._site_boundary.boundary_semantics == "VEHICLE_PERMITTED_INNER_BOUNDARY"
        app.processEvents()
    finally:
        window.close()


def test_site_boundary_undo_and_clear_are_independent_from_navigation_overrides():
    app = _qapp()
    window = ReviewMapWorkbenchWindow()
    try:
        window._site_boundary_vertices = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
        window._navigation_overrides = [{"mode": "no_go", "polygon_xy": [[2, 2], [3, 2], [3, 3]]}]
        window._undo_site_boundary_vertex()
        assert len(window._site_boundary_vertices) == 2
        window._clear_site_boundary()
        assert window._site_boundary is None
        assert len(window._navigation_overrides) == 1
    finally:
        window.close()


def test_load_sibling_site_boundary_uses_processed_pcd_parent(tmp_path: Path):
    app = _qapp()
    window = ReviewMapWorkbenchWindow()
    try:
        pcd = tmp_path / "processed.pcd"
        pcd.write_text("placeholder", encoding="utf-8")
        write_site_boundary(
            SiteBoundary(
                frame_id="map",
                outer_boundary_xy=((0, 0), (4, 0), (4, 3), (0, 3)),
            ),
            tmp_path / "site_boundary.yaml",
        )
        window._source_path = pcd
        assert window._load_sibling_site_boundary()
        assert window._site_boundary is not None
    finally:
        window.close()
```

- [ ] **Step 2: Run the test and verify the authoring API is absent**

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/agt_map_workbench/test/test_site_boundary_workbench.py
```

Expected: FAIL on missing Site Boundary Workbench methods/state.

- [ ] **Step 3: Add Site Boundary UI controls inside the agricultural Navigation Map panel**

Add this bounded control group after the existing agricultural evidence explanation:

```text
Site Boundary（车辆允许区域内边界）
[开始绘制] [完成] [撤销顶点] [清除]
[导出 site_boundary.yaml]
状态：未定义 / 草稿 N 点 / READY / INVALID
```

Initialize all Site Boundary state **before** `super().__init__()` because `_build_navigation_tab()` is called from the base constructor.

- [ ] **Step 4: Reuse the existing scene/map click infrastructure without mixing Site Boundary with Navigation Override**

Override map click handling in `AgriculturalMapWorkbenchWindow`:

```python
def _on_map_clicked(self, x: float, y: float) -> None:
    if self._interaction_mode == "site_boundary":
        self._append_site_boundary_vertex(x, y)
        return
    super()._on_map_clicked(x, y)
```

Use a dedicated polygon item and dedicated vertex-item list. Do not reuse `_nav_vertices`, `_nav_polygon_item`, or `_navigation_overrides`.

- [ ] **Step 5: Make finish validation fail closed and render the READY polygon distinctly from NO_GO**

`_finish_site_boundary_authoring(show_errors: bool = True) -> bool` must construct `SiteBoundary(frame_id="map", ...)`, call `validate()`, and return `False` without replacing the last READY boundary if the draft is invalid.

Render:

```text
Site Boundary edge: bright solid line
Permitted polygon: low-opacity neutral/green fill
Authoring vertices: numbered handles
NO_GO remains the existing magenta/purple semantic override visualization
```

- [ ] **Step 6: Load `site_boundary.yaml` automatically when opening a sibling `processed.pcd`**

Override `_open_pcd()` in `AgriculturalMapWorkbenchWindow`, call `super()._open_pcd()`, then when `self._source_path` changed:

```python
candidate = self._source_path.parent / "site_boundary.yaml"
if candidate.is_file():
    self._site_boundary = load_site_boundary(candidate, expected_frame_id="map")
```

If the sibling file is invalid, show an explicit warning and leave `_site_boundary = None`; do not invent a fallback.

- [ ] **Step 7: Export only READY Site Boundary data**

Default export path must be:

```python
self._source_path.parent / "site_boundary.yaml"
```

when a PCD is loaded. `write_site_boundary(..., overwrite=True)` is allowed only after the user explicitly chooses the destination in the save dialog.

- [ ] **Step 8: Register and run the Workbench test**

Add:

```cmake
ament_add_pytest_test(
  test_site_boundary_workbench test/test_site_boundary_workbench.py
)
```

Then run:

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/agt_map_workbench/test/test_site_boundary_workbench.py \
  src/agt_map_workbench/test/test_route_debug_workbench_integration.py
```

Expected: PASS and existing Route Debug tab integration remains unchanged.

- [ ] **Step 9: Commit Task 2**

```bash
git add \
  src/agt_map_workbench/agt_map_workbench/agricultural_workbench.py \
  src/agt_map_workbench/test/test_site_boundary_workbench.py \
  src/agt_map_workbench/CMakeLists.txt
git commit -m "feat(v25-12f): author site boundary in workbench"
```

---

### Task 3: Preserve `aisle_geometric_envelope` before terrain filtering

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/navigation_corridor.py`
- Modify: `src/agt_offline_assets/test/test_navigation_corridor.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/agricultural_workbench.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/navigation_preview.py`

**Interfaces:**
- Produces: `CorridorRefinementResult.aisle_geometric_envelope: np.ndarray`
- Invariant: `aisle_candidate <= aisle_geometric_envelope`
- Invariant: structurally rejected pairs contribute zero cells to the geometric envelope

- [ ] **Step 1: Add a failing regression showing a short ground-valid hole disappears from `aisle_candidate` but not from structural geometry**

Append to `test_navigation_corridor.py`:

```python
def test_geometric_envelope_survives_ground_evidence_hole():
    navigation, structure = _fixture()
    navigation.ground_valid[14:17, 20:24] = False
    result = derive_corridor_refinement(navigation, structure, _feasible_config())

    assert np.any(result.aisle_geometric_envelope[14:17, 20:24])
    assert not np.any(result.aisle_candidate[14:17, 20:24])
    assert np.all(~result.aisle_candidate | result.aisle_geometric_envelope)


def test_too_narrow_pair_has_no_geometric_envelope():
    navigation, structure = _fixture()
    result = derive_corridor_refinement(navigation, structure)
    assert np.count_nonzero(result.aisle_geometric_envelope) == 0
```

- [ ] **Step 2: Run the focused test and verify the new field is missing**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_navigation_corridor.py
```

Expected: FAIL on missing `aisle_geometric_envelope`.

- [ ] **Step 3: Extend `CorridorRefinementResult` and make `evaluate_corridor()` return structural geometry separately from safe cells**

Change the internal return shape to:

```python
return geometry, safe, centerline, diagnostic
```

For width rejection, missing row support, or insufficient longitudinal overlap, `geometry` must be an all-false mask. Only after structural gates pass should:

```python
geometry = longitudinal & (vv >= corridor_min) & (vv <= corridor_max)
safe = geometry & safe_base
```

be evaluated.

Accumulate:

```python
aisle_geometric_envelope |= geometry
```

for accepted structural row-row and boundary-row pairs.

- [ ] **Step 4: Expose the envelope as a Workbench preview layer**

Add the key:

```text
aisle_geometric_envelope
```

with UI label:

```text
结构行道几何包络（未经过 Ground FREE 过滤）
```

Render it with a distinct low-opacity color. Do not call it FREE or safe.

- [ ] **Step 5: Run corridor and preview regressions**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_navigation_corridor.py \
  src/agt_offline_assets/test/test_vehicle_corridor.py
```

Expected: PASS.

- [ ] **Step 6: Commit Task 3**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/navigation_corridor.py \
  src/agt_offline_assets/test/test_navigation_corridor.py \
  src/agt_map_workbench/agt_map_workbench/agricultural_workbench.py \
  src/agt_map_workbench/agt_map_workbench/navigation_preview.py
git commit -m "feat(v25-12f): preserve structural aisle envelope"
```

---

### Task 4: Implement rich traversability states and bounded longitudinal occlusion recovery

**Files:**
- Create: `src/agt_offline_assets/agt_offline_assets/traversability.py`
- Create: `src/agt_offline_assets/test/test_traversability.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/__init__.py`
- Modify: `src/agt_offline_assets/CMakeLists.txt`

**Interfaces:**
- Consumes:
  - `NavigationMapResult`
  - `NavigationStructureResult`
  - `CorridorRefinementResult`
  - `SiteBoundary`
  - frozen/current Navigation Override metadata for semantic `NO_GO`
- Produces:
  - `TRAVERSABILITY_EVIDENCE_SCHEMA = "agt_traversability_evidence/v1"`
  - `TraversabilityConfig(maximum_inferred_gap_m=0.60)`
  - `TraversabilityEvidence`
  - `derive_traversability_evidence(...) -> TraversabilityEvidence`
  - `TraversabilityEvidence.candidate_occupancy() -> np.ndarray`

- [ ] **Step 1: Write failing synthetic tests for short-gap recovery, long-gap rejection, row blocking, direct OCCUPIED preservation, NO_GO precedence, and hard boundary precedence**

Use a small grid with row direction `+X` so the expected behavior is explicit:

```python
# src/agt_offline_assets/test/test_traversability.py

def test_short_longitudinal_unknown_gap_is_inferred(fixture):
    nav, structure, corridor, boundary = fixture
    nav.occupancy[10, 8:12] = UNKNOWN  # 0.4 m gap at 0.1 m resolution
    evidence = derive_traversability_evidence(
        nav, structure, corridor, boundary, overrides=[],
        config=TraversabilityConfig(maximum_inferred_gap_m=0.60),
    )
    assert np.all(evidence.inferred_traversable_mask[10, 8:12])
    assert np.all(evidence.candidate_occupancy()[10, 8:12] == FREE)


def test_large_gap_remains_unknown(fixture):
    nav, structure, corridor, boundary = fixture
    nav.occupancy[10, 5:14] = UNKNOWN  # 0.9 m
    evidence = derive_traversability_evidence(
        nav, structure, corridor, boundary, overrides=[],
        config=TraversabilityConfig(maximum_inferred_gap_m=0.60),
    )
    assert not np.any(evidence.inferred_traversable_mask[10, 5:14])
    assert np.all(evidence.candidate_occupancy()[10, 5:14] == UNKNOWN)


def test_recovery_never_crosses_row_structural_band(fixture):
    nav, structure, corridor, boundary = fixture
    corridor.row_structural_band[10, 8:12] = True
    nav.occupancy[10, 8:12] = UNKNOWN
    evidence = derive_traversability_evidence(nav, structure, corridor, boundary, overrides=[])
    assert not np.any(evidence.inferred_traversable_mask[10, 8:12])


def test_current_occupied_cell_is_not_recovered(fixture):
    nav, structure, corridor, boundary = fixture
    nav.occupancy[10, 9] = OCCUPIED
    evidence = derive_traversability_evidence(nav, structure, corridor, boundary, overrides=[])
    assert evidence.sensor_obstacle_mask[10, 9]
    assert evidence.candidate_occupancy()[10, 9] == OCCUPIED


def test_site_boundary_and_no_go_beat_recovery(fixture):
    nav, structure, corridor, boundary = fixture
    nav.occupancy[10, 8:12] = UNKNOWN
    overrides = [{
        "mode": "no_go",
        "polygon_xy": [[0.9, 0.9], [1.1, 0.9], [1.1, 1.2], [0.9, 1.2]],
    }]
    evidence = derive_traversability_evidence(
        nav, structure, corridor, boundary, overrides=overrides
    )
    assert not np.any(evidence.inferred_traversable_mask & evidence.semantic_no_go_mask)
    assert not np.any(evidence.inferred_traversable_mask & evidence.hard_blocked_mask)
```

- [ ] **Step 2: Run the tests and verify `traversability.py` is absent**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_traversability.py
```

Expected: import failure.

- [ ] **Step 3: Define the rich evidence dataclasses and fixed compatibility mapping**

```python
@dataclass(frozen=True)
class TraversabilityConfig:
    maximum_inferred_gap_m: float = 0.60

    def validate(self) -> None:
        if not math.isfinite(self.maximum_inferred_gap_m) or self.maximum_inferred_gap_m <= 0.0:
            raise ValueError("maximum_inferred_gap_m must be finite and > 0")


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

    def candidate_occupancy(self) -> np.ndarray:
        occupancy = np.full(self.observed_free_mask.shape, UNKNOWN, dtype=np.uint8)
        occupancy[self.observed_free_mask | self.inferred_traversable_mask] = FREE
        occupancy[
            self.sensor_obstacle_mask | self.semantic_no_go_mask | self.hard_blocked_mask
        ] = OCCUPIED
        return occupancy
```

Masks must have one shared grid shape and the final precedence must be asserted in `__post_init__`/validation.

- [ ] **Step 4: Build the semantic masks conservatively from the current map**

The A-first policy is intentionally conservative:

```python
base_free = np.asarray(navigation.occupancy) == FREE
base_occupied = np.asarray(navigation.occupancy) == OCCUPIED
base_unknown = np.asarray(navigation.occupancy) == UNKNOWN
inside_boundary = rasterize_site_boundary(boundary, navigation)
hard_blocked = ~inside_boundary
semantic_no_go = _rasterize_no_go(navigation, overrides)
```

Use current OCCUPIED as non-recoverable `sensor_obstacle_mask` for the main candidate. Preserve the existing occupancy-source audit separately for RAW/GEOMETRY/PADDING provenance; do not free a current OCCUPIED cell in this task.

- [ ] **Step 5: Implement directional recovery in row coordinates, not generic 2D closing**

Use the corridor row direction to project grid cell centers into longitudinal/lateral coordinates:

```python
u = x * direction[0] + y * direction[1]
v = x * perpendicular[0] + y * perpendicular[1]
v_bin = np.rint((v - v_min) / navigation.resolution_m).astype(np.int64)
```

For each `v_bin` independently:

1. select cells in `corridor.aisle_geometric_envelope`
2. sort by `u`
3. find runs of `base_unknown` bracketed by `base_free` on both sides
4. require the unsupported longitudinal span `<= maximum_inferred_gap_m`
5. reject any run containing `row_structural_band`, `semantic_no_go`, `hard_blocked`, or current OCCUPIED
6. require finite endpoint ground heights
7. require endpoint height difference and longitudinal separation to satisfy current maximum slope and maximum step limits
8. mark only that run `INFERRED_TRAVERSABLE`

Do not use `binary_closing()` or an isotropic structuring element.

- [ ] **Step 6: Lock precedence and derive final mutually interpretable masks**

Apply:

```text
HARD_BLOCKED
> semantic NO_GO
> SENSOR_OBSTACLE
> INFERRED_TRAVERSABLE
> OBSERVED_FREE
> UNKNOWN
```

before constructing the returned masks. Assert that no inferred cell intersects hard/no-go/occupied masks.

- [ ] **Step 7: Export the public API and register the test target**

Add:

```cmake
ament_add_pytest_test(test_traversability test/test_traversability.py)
```

and export the traversability classes/functions from `agt_offline_assets.__init__`.

- [ ] **Step 8: Run focused pure-Python tests**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_traversability.py \
  src/agt_offline_assets/test/test_navigation_corridor.py \
  src/agt_offline_assets/test/test_navigation_map_derivation.py
```

Expected: PASS.

- [ ] **Step 9: Commit Task 4**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/traversability.py \
  src/agt_offline_assets/agt_offline_assets/__init__.py \
  src/agt_offline_assets/test/test_traversability.py \
  src/agt_offline_assets/CMakeLists.txt
git commit -m "feat(v25-12f): derive occlusion-aware traversability"
```

---

### Task 5: Serialize the non-destructive 12F candidate and add Workbench candidate controls

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/traversability.py`
- Modify: `src/agt_offline_assets/test/test_traversability.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/agricultural_workbench.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/navigation_preview.py`
- Modify: `src/agt_map_workbench/test/test_site_boundary_workbench.py`

**Interfaces:**
- Produces:
  - `TRAVERSABILITY_CANDIDATE_DERIVATION_SCHEMA = "agt_traversability_candidate_navigation_map/v1"`
  - `write_traversability_candidate(evidence, navigation, run_dir, overwrite=False) -> Path`
  - `load_traversability_evidence(path, expected_frame_id=None) -> TraversabilityEvidence`
  - Workbench `self._traversability_evidence`
  - Workbench `self._navigation_12f_result`

- [ ] **Step 1: Add failing serialization tests for the exact six output files and immutable canonical map protection**

```python
def test_candidate_writer_uses_fixed_12f_names_and_does_not_touch_canonical(tmp_path, fixture):
    nav, structure, corridor, boundary = fixture
    (tmp_path / "navigation_map.pgm").write_bytes(b"old-pgm")
    (tmp_path / "navigation_map.yaml").write_text("image: navigation_map.pgm\n", encoding="utf-8")
    before_pgm = (tmp_path / "navigation_map.pgm").read_bytes()

    evidence = derive_traversability_evidence(nav, structure, corridor, boundary, overrides=[])
    write_traversability_candidate(evidence, nav, tmp_path)

    for name in (
        "traversability_evidence.yaml",
        "traversability_evidence.npz",
        "navigation_map_12f.yaml",
        "navigation_map_12f.pgm",
        "navigation_map_12f_derivation.yaml",
    ):
        assert (tmp_path / name).is_file()
    assert (tmp_path / "navigation_map.pgm").read_bytes() == before_pgm


def test_traversability_evidence_round_trip(tmp_path, fixture):
    nav, structure, corridor, boundary = fixture
    evidence = derive_traversability_evidence(nav, structure, corridor, boundary, overrides=[])
    write_traversability_candidate(evidence, nav, tmp_path)
    loaded = load_traversability_evidence(tmp_path, expected_frame_id="map")
    assert np.array_equal(loaded.inferred_traversable_mask, evidence.inferred_traversable_mask)
    assert loaded.config.maximum_inferred_gap_m == 0.60
```

- [ ] **Step 2: Run and verify writer/loader functions are missing**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_traversability.py
```

Expected: FAIL on missing writer/loader.

- [ ] **Step 3: Implement deterministic YAML/NPZ/PGM serialization**

Write:

```text
site_boundary.yaml                    # authored by Task 2, not rewritten here
traversability_evidence.yaml          # summary/provenance/hashes
traversability_evidence.npz           # grid masks
navigation_map_12f.yaml               # ROS trinary metadata
navigation_map_12f.pgm                # candidate occupancy
navigation_map_12f_derivation.yaml    # candidate lineage/config/counts
```

`traversability_evidence.npz` must include at least:

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

The derivation YAML must record:

```text
source_navigation_map
source_site_boundary
maximum_inferred_gap_m
current requested obstacle_padding_m
current effective padding cells = ceil(obstacle_padding_m / resolution_m)
counts for all rich states
candidate FREE/OCCUPIED/UNKNOWN counts
hashes of PGM/YAML/NPZ outputs
```

- [ ] **Step 4: Construct a candidate `NavigationMapResult` without mutating the base result**

In Workbench:

```python
self._traversability_evidence = derive_traversability_evidence(
    self._navigation_result,
    self._navigation_structure_result,
    self._corridor_refinement_result,
    self._site_boundary,
    overrides=self._navigation_overrides,
    config=TraversabilityConfig(
        maximum_inferred_gap_m=float(self._traversability_gap_limit.value())
    ),
)
self._navigation_12f_result = replace(
    self._navigation_result,
    occupancy=self._traversability_evidence.candidate_occupancy(),
)
```

Do not replace `self._navigation_result`.

- [ ] **Step 5: Add a bounded V25-12F candidate group to the Navigation Map page**

Controls:

```text
V25-12F Candidate
最大遮挡补全缺口：0.60 m
[生成 12F Candidate]
[导出到当前 run 目录]
状态：等待 Site Boundary / READY / INVALID
```

The generate button must refuse with an explicit message when any required input is missing:

```text
_navigation_result
_navigation_structure_result
_corridor_refinement_result
_site_boundary READY
```

- [ ] **Step 6: Add Workbench preview keys for candidate and rich evidence**

Add navigation preview entries:

```text
12F Candidate 三态
12F OBSERVED_FREE
12F INFERRED_TRAVERSABLE
12F HARD_BLOCKED
12F SENSOR_OBSTACLE
12F UNKNOWN
12F Aisle Geometric Envelope
```

`NavigationPreviewItem` may accept an optional `TraversabilityEvidence`; do not duplicate derivation logic in Qt.

- [ ] **Step 7: Export candidate files to the current processed PCD run directory**

Default run directory:

```python
self._source_path.parent
```

If candidate files already exist, ask the operator before passing `overwrite=True`. Never delete or overwrite the canonical current map files.

- [ ] **Step 8: Run serialization and headless Workbench tests**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_traversability.py
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/agt_map_workbench/test/test_site_boundary_workbench.py
```

Expected: PASS.

- [ ] **Step 9: Commit Task 5**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/traversability.py \
  src/agt_offline_assets/test/test_traversability.py \
  src/agt_map_workbench/agt_map_workbench/agricultural_workbench.py \
  src/agt_map_workbench/agt_map_workbench/navigation_preview.py \
  src/agt_map_workbench/test/test_site_boundary_workbench.py
git commit -m "feat(v25-12f): export candidate navigation map"
```

---

### Task 6: Enforce Site Boundary at preview-footprint level in lane and connector gates

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/reverse_primitive_connector.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/forward_connector_navigation_gate.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/vehicle_safe_lane.py`
- Modify: `src/agt_offline_assets/test/test_reverse_primitive_connector.py`
- Modify: `src/agt_offline_assets/test/test_forward_connector_navigation_gate.py`
- Modify: `src/agt_offline_assets/test/test_vehicle_safe_lane.py`

**Interfaces:**
- All existing callers remain source compatible because `site_boundary` is optional
- New optional parameters:
  - `derive_forward_connector_navigation_gate(..., site_boundary: SiteBoundary | None = None, ...)`
  - `derive_reverse_primitive_connector_plan(..., site_boundary: SiteBoundary | None = None, ...)`
  - `derive_vehicle_safe_lane_plan(..., site_boundary: SiteBoundary | None = None, ...)`
- New preview pose status helper in `reverse_primitive_connector.py`:
  - `_preview_pose_status(...) -> str`
  - status values include `PREVIEW_POSE_FREE`, `SITE_BOUNDARY_CONFLICT`, `NAVIGATION_GRID_OUT_OF_BOUNDS`, `NAVIGATION_NOT_FREE`

- [ ] **Step 1: Add failing footprint tests where center point is inside but the vehicle rectangle touches/crosses the Site Boundary**

For each gate, construct a boundary that cuts through the outer half of the canonical footprint while keeping the pose center inside. Assert the gate rejects with `SITE_BOUNDARY_CONFLICT` or a boundary-specific reason.

Example unit assertion:

```python
status = _preview_pose_status(
    x=0.45,
    y=1.0,
    yaw=0.0,
    navigation=navigation,
    local_footprint=local_footprint,
    site_boundary=boundary,
)
assert status == "SITE_BOUNDARY_CONFLICT"
```

- [ ] **Step 2: Run the three focused gate test files and verify the optional boundary API is missing**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_safe_lane.py \
  src/agt_offline_assets/test/test_forward_connector_navigation_gate.py \
  src/agt_offline_assets/test/test_reverse_primitive_connector.py
```

Expected: FAIL on missing parameters/status helper.

- [ ] **Step 3: Split boolean pose checking into a status-returning helper while preserving `_preview_pose_free()`**

Implement:

```python
def _preview_pose_status(
    x, y, yaw, navigation, local_footprint, site_boundary=None
) -> str:
    sample = ForwardConnectorSample(x=float(x), y=float(y), z=0.0, yaw=float(yaw))
    polygon = _transform_polygon(local_footprint, sample)
    if site_boundary is not None:
        if not polygon_strictly_inside_site_boundary(site_boundary, polygon):
            return "SITE_BOUNDARY_CONFLICT"
    # existing grid bounds + FREE checks
    ...


def _preview_pose_free(..., site_boundary=None) -> bool:
    return _preview_pose_status(...) == "PREVIEW_POSE_FREE"
```

Keep all existing occupancy checks unchanged after the boundary check.

- [ ] **Step 4: Use the optional boundary in Vehicle-Safe Lane lateral candidate evaluation**

Pass `site_boundary` into each pose check. If a lane has no feasible pose and at least one attempted candidate failed specifically on the boundary, include `SITE_BOUNDARY_CONFLICT` in the lane reason instead of reporting only generic occupancy failure.

Do not change structural-width logic.

- [ ] **Step 5: Gate every forward candidate sample footprint against Site Boundary before accepting it**

In `forward_connector_navigation_gate.py`, compute:

```python
boundary_conflict = any(
    not polygon_strictly_inside_site_boundary(
        site_boundary,
        _transform_polygon(local_footprint, sample),
    )
    for sample in samples
) if site_boundary is not None else False
```

A boundary-conflicting candidate must never be accepted even when its Navigation Grid footprint evidence is all FREE. If the selected/best candidate is boundary-conflicting, its final status/reason must expose `SITE_BOUNDARY_CONFLICT`.

- [ ] **Step 6: Gate R6B start pose, every primitive edge sample, and every forward goal-shot sample**

Pass `site_boundary` through `_preview_pose_status()` / `_edge_is_free()` / `_try_forward_goal_shot()`.

A start pose that touches/crosses the boundary must return a result with:

```text
status = SITE_BOUNDARY_CONFLICT
reason contains site boundary
search_expansions = 0
```

Do not increase search budget and do not alter motion primitive costs.

- [ ] **Step 7: Run gate tests and the known reverse regression tests**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_safe_lane.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane_width_gate.py \
  src/agt_offline_assets/test/test_forward_connector_navigation_gate.py \
  src/agt_offline_assets/test/test_reverse_primitive_connector.py
```

Expected: PASS, including existing behavior when `site_boundary=None`.

- [ ] **Step 8: Commit Task 6**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/reverse_primitive_connector.py \
  src/agt_offline_assets/agt_offline_assets/forward_connector_navigation_gate.py \
  src/agt_offline_assets/agt_offline_assets/vehicle_safe_lane.py \
  src/agt_offline_assets/test/test_reverse_primitive_connector.py \
  src/agt_offline_assets/test/test_forward_connector_navigation_gate.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane.py
git commit -m "feat(v25-12f): enforce site boundary on preview footprints"
```

---

### Task 7: Load and visualize V25-12F evidence in Route Debug A/B

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/route_debug_dataset.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/route_debug_overlay.py`
- Modify: `src/agt_offline_assets/test/test_route_debug_dataset.py`
- Modify: `src/agt_offline_assets/test/test_route_debug_overlay.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/route_debug_view.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/route_debug_panel.py`
- Modify: `src/agt_map_workbench/test/test_route_debug_view.py`
- Modify: `src/agt_map_workbench/test/test_route_debug_panel.py`

**Interfaces:**
- Extend `RouteDebugDataset` with:
  - `candidate_navigation: NavigationGridEvidence | None`
  - `site_boundary: SiteBoundary | None`
  - `traversability: TraversabilityEvidence | None`
- Add asset-state keys:
  - `site_boundary`
  - `navigation_12f`
  - `traversability_evidence`
- Add layer keys:
  - `base.navigation_12f`
  - `traversability.observed`
  - `traversability.inferred`
  - `traversability.hard_blocked`
  - `traversability.sensor_obstacle`
  - `traversability.unknown`
  - `traversability.aisle_geometric_envelope`
  - `semantics.site_boundary`
- Add preset: `traversability_ab`

- [ ] **Step 1: Add failing dataset tests for optional 12F evidence and fail-closed frame mismatch**

Create a synthetic run with the fixed 12F files and assert:

```python
assert dataset.asset_state("site_boundary").availability == ASSET_LOADED
assert dataset.asset_state("navigation_12f").availability == ASSET_LOADED
assert dataset.asset_state("traversability_evidence").availability == ASSET_LOADED
assert dataset.candidate_navigation is not None
assert dataset.site_boundary is not None
assert dataset.traversability is not None
```

Then write an `odom` Site Boundary and assert only that 12F boundary-dependent layer becomes `INVALID`; existing 12E Navigation/Aisle/Coverage evidence must still load.

- [ ] **Step 2: Add failing overlay tests for Site Boundary provenance**

The GeoJSON overlay should contain one Site Boundary polygon feature with:

```text
layer_key = semantics.site_boundary
feature_kind = SITE_BOUNDARY
status = READY
source_asset = site_boundary.yaml
boundary_semantics = VEHICLE_PERMITTED_INNER_BOUNDARY
```

Raster masks stay in `RouteDebugDataset` and are not exploded into one GeoJSON feature per cell.

- [ ] **Step 3: Extend `load_route_debug_dataset()` with optional 12F discovery**

Rules:

```text
site_boundary.yaml missing -> MISSING
navigation_map_12f.yaml missing -> MISSING
traversability_evidence.yaml/.npz missing -> MISSING
existing but schema/frame/grid mismatch -> INVALID for that layer
```

Do not prevent the 12E layers from opening when optional 12F evidence is absent or invalid.

- [ ] **Step 4: Render 12F candidate and rich masks as shared-scene rasters**

Use distinct alpha/color semantics, for example:

```text
12F Candidate             normal trinary palette
OBSERVED_FREE             low-opacity green
INFERRED_TRAVERSABLE      cyan/blue highlight
HARD_BLOCKED              strong red
SENSOR_OBSTACLE           orange/red
UNKNOWN                   gray
Aisle Geometric Envelope  thin/light cyan support
Site Boundary             high-contrast solid outline
```

Do not reuse the NO_GO color for Site Boundary.

- [ ] **Step 5: Add a `12F A/B` preset button to Route Debug**

The preset should enable:

```text
Current Navigation Map
12F Candidate Navigation Map
INFERRED_TRAVERSABLE
HARD_BLOCKED
Aisle Geometric Envelope
Site Boundary
NO_GO
Structural Aisles
```

The operator can then manually toggle current/candidate base rasters to compare them without regenerating data.

- [ ] **Step 6: Extend Inspector text for Site Boundary and traversability provenance**

When a Site Boundary is selected, show:

```text
boundary_semantics
frame_id
vertex count
source asset
status
```

When a 12F diagnostic feature is selected, show the candidate/evidence source and recovery config. Do not claim inferred cells are directly observed FREE.

- [ ] **Step 7: Run Route Debug pure and GUI tests**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_route_debug_dataset.py \
  src/agt_offline_assets/test/test_route_debug_overlay.py

QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/agt_map_workbench/test/test_route_debug_view.py \
  src/agt_map_workbench/test/test_route_debug_panel.py \
  src/agt_map_workbench/test/test_route_debug_workbench_integration.py
```

Expected: PASS.

- [ ] **Step 8: Commit Task 7**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/route_debug_dataset.py \
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

### Task 8: Lock the cross-package contract, build, and perform real greenhouse acceptance

**Files:**
- Create: `tests/test_v25_12f_traversability_contract.py`
- Modify: `src/agt_map_workbench/README.md`
- Modify: `docs/v2.5/V25_12E_CURRENT_STATE.md`

**Interfaces:**
- Cross-package contract verifies no 12F production logic moved into Qt
- Docs record exact output filenames, Site Boundary semantics, recovery default, and acceptance state

- [ ] **Step 1: Write the cross-package contract test**

```python
# tests/test_v25_12f_traversability_contract.py
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_v25_12f_fixed_assets_and_safety_contract_are_documented():
    spec = (ROOT / "docs/superpowers/specs/2026-08-15-v25-12f-traversability-hard-boundary-design.md").read_text()
    assert "VEHICLE_PERMITTED_INNER_BOUNDARY" in spec
    assert "SITE_BOUNDARY_CONFLICT" in spec
    assert "maximum_inferred_gap_m" in spec
    assert "0.60 m" in spec
    assert "navigation_map_12f.pgm" in spec
    assert "traversability_evidence.npz" in spec


def test_qt_does_not_own_traversability_derivation():
    qt = (ROOT / "src/agt_map_workbench/agt_map_workbench/agricultural_workbench.py").read_text()
    assert "def derive_traversability_evidence" not in qt
    assert "binary_closing" not in qt


def test_r6b_search_parameters_are_not_redefined_by_12f_ui():
    qt = (ROOT / "src/agt_map_workbench/agt_map_workbench/agricultural_workbench.py").read_text()
    assert "max_expansions" not in qt
    assert "cusp_penalty_m" not in qt
```

- [ ] **Step 2: Update README and Current State before real-data execution**

Document the status as:

```text
V25-12F CORE IMPLEMENTED / REAL-DATA A/B PENDING
```

Do not write `REAL-DATA PASS` yet.

Include the operator workflow:

```text
open processed.pcd
-> draw Site Boundary inner permitted polygon
-> export site_boundary.yaml
-> generate 12F Candidate
-> export fixed candidate/evidence files
-> Route Debug -> 12F A/B
```

- [ ] **Step 3: Run all focused unit tests before building**

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

Expected: all PASS.

- [ ] **Step 4: Clean-build the two affected ROS packages**

```bash
rm -rf \
  build/agt_offline_assets \
  build/agt_map_workbench \
  install/agt_offline_assets \
  install/agt_map_workbench

source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  agt_offline_assets \
  agt_map_workbench \
  --event-handlers console_direct+
source install/setup.bash
```

Expected: both packages build successfully.

- [ ] **Step 5: Author the real Site Boundary in the greenhouse run**

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

In `导航地图`:

1. draw the vehicle-permitted inner greenhouse perimeter, intentionally inside the physical walls by the desired stand-off
2. finish and verify `READY`
3. export exactly:

```text
/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run/site_boundary.yaml
```

- [ ] **Step 6: Generate and export the 12F candidate without touching the current frozen map**

Verify these new files appear:

```text
traversability_evidence.yaml
traversability_evidence.npz
navigation_map_12f.yaml
navigation_map_12f.pgm
navigation_map_12f_derivation.yaml
```

Verify these existing files retain their original hashes/contents:

```text
navigation_map.yaml
navigation_map.pgm
derivation.yaml
```

- [ ] **Step 7: Perform the Route Debug `12F A/B` visual acceptance**

Check all of the following:

```text
Site Boundary
- visibly distinct from NO_GO and physical OCCUPIED
- candidate map may never recover outside it

Occluded aisle gaps
- short gaps inside structural aisle envelope become INFERRED_TRAVERSABLE
- large gaps remain UNKNOWN
- no lateral bridge crosses crop-row structural bands

aisle_003
- direct obstacle remains blocked

aisle_005
- structural width rejection remains unchanged

aisle_013
- padding-dominant blockage remains explainable; do not silently free current OCCUPIED in the main candidate
```

- [ ] **Step 8: Run Vehicle-Safe Lane A/B with the same canonical MK-mini profile and record metrics**

For current vs candidate Navigation Grid, record:

```text
READY count
PARTIAL count
NO_VEHICLE_SAFE_LANE count
mean any-lateral footprint-free fraction
per-aisle coverage fraction
zero Site Boundary footprint violations
```

The same profile, footprint padding, lateral search, and validator settings must be used on both maps.

- [ ] **Step 9: Re-run connector regressions only after the candidate map is frozen**

Verify:

```text
connector_015
- remains an explainable known F -> R -> F regression or improves without safety regression
- Site Boundary is respected

connector_017
- remains distinguishable as searched-but-unsolved if still unsolved
- no search parameter change is used to manufacture success
```

- [ ] **Step 10: Update Current State with measured evidence, then commit**

Only after the operator completes Steps 5-9 may the document be upgraded from:

```text
V25-12F CORE IMPLEMENTED / REAL-DATA A/B PENDING
```

to a measured acceptance result. If any safety or semantic check fails, keep it pending and record the exact failure instead of claiming PASS.

Commit:

```bash
git add \
  tests/test_v25_12f_traversability_contract.py \
  src/agt_map_workbench/README.md \
  docs/v2.5/V25_12E_CURRENT_STATE.md
git commit -m "test(v25-12f): lock traversability acceptance contract"
```

---

## Final Verification Checklist

Before calling V25-12F complete, confirm every item below with fresh evidence:

```text
[ ] Site Boundary schema/YAML round trip passes
[ ] Self-intersecting/invalid boundary fails closed
[ ] Boundary-touching vehicle footprint is rejected
[ ] Missing Site Boundary cannot generate a 12F candidate
[ ] aisle_geometric_envelope exists independently of ground-valid holes
[ ] structurally rejected/narrow pair never enters recovery envelope
[ ] short longitudinal UNKNOWN gap can be inferred
[ ] long gap stays UNKNOWN
[ ] no lateral crop-row bridge is created
[ ] current OCCUPIED is not recovered in A-first main candidate
[ ] NO_GO beats inference
[ ] Site Boundary beats every other state
[ ] canonical current Navigation Map files are unchanged
[ ] candidate outputs use the fixed 12F names
[ ] Vehicle-Safe Lane can enforce Site Boundary
[ ] Forward connector preview can enforce Site Boundary
[ ] R6B primitive samples can enforce Site Boundary without changing search parameters
[ ] Route Debug loads 12F evidence as optional layers
[ ] Route Debug can visually compare current and candidate maps
[ ] aisle_003 remains direct-obstacle conservative
[ ] aisle_005 remains width-blocked
[ ] aisle_013 remains padding-diagnostic explainable
[ ] real greenhouse A/B metrics are recorded
[ ] zero Site Boundary footprint violations in acceptance run
```

## Execution Order Rationale

Do not reorder Tasks 1-7. The safety boundary contract must exist before the Workbench can author it; structural geometry must exist before recovery can be justified; rich traversability must exist before a candidate can be serialized; candidate semantics must be frozen before lane/connector A/B testing; Route Debug must read frozen outputs instead of duplicating derivation. Task 8 is the only stage allowed to make a real-data acceptance statement.
