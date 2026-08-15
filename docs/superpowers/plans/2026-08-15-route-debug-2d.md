# AGT Map Workbench 2D Route Debug Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a read-only 2D Route Debug tab to the existing AGT Map Workbench that loads frozen greenhouse route-production assets, renders coverage/order/planning/collision evidence in one map, and lets the operator inspect why an aisle or connector is accepted, rejected, blocked, or unsolved.

**Architecture:** `agt_offline_assets` owns all parsing, cross-asset validation, cause-chain joining, collision provenance, and generation of the render-only `route_debug_overlay.geojson`. `agt_map_workbench` only renders Navigation/source-mask rasters and GeoJSON features into the existing shared 2D `QGraphicsScene`, controls layer visibility/presets, and exposes a read-only Inspector. The GUI never reruns Dubins, R6, map derivation, or route admission logic.

**Tech Stack:** Python 3, dataclasses, pathlib, PyYAML, NumPy, SciPy, standard-library JSON/GeoJSON serialization, PyQt5 `QGraphicsScene/QGraphicsView`, ROS 2 Humble `ament_cmake_python` + `ament_cmake_pytest`, pytest.

## Global Constraints

- Implement the approved spec at `docs/superpowers/specs/2026-08-15-route-debug-2d-design.md` without expanding scope.
- Route Debug is integrated into the existing AGT Map Workbench; do not create a standalone viewer and do not add 3D route linkage in this increment.
- The Route Debug page is read-only for route-production truth assets. It may create/replace only the derived `route_debug_overlay.geojson`; it must not overwrite source YAML/PGM/NPY assets.
- The GUI must not rerun planner or collision math. Planner/collision replay required for visualization belongs in `agt_offline_assets` only.
- Missing optional assets degrade to an unavailable layer. An existing asset with invalid schema or incompatible `frame_id` fails closed for that layer and is not rendered.
- Never silently convert coordinate frames. GeoJSON stores normal world `(x, y)` coordinates; Qt rendering converts them to scene `(x, -y)` consistently with the existing Workbench.
- `WITH_ROW_DIRECTION` and `AGAINST_ROW_DIRECTION` remain normal FORWARD aisle traversal. Reverse motion is rendered only when a connector sample explicitly says `motion_direction=REVERSE`.
- Preserve the exact implemented R6 backend label `BOUNDED_REVERSE_PRIMITIVE_SEARCH_NOT_ANALYTIC_REEDS_SHEPP`; never relabel it analytic Reeds-Shepp.
- NO_GO is recovered and displayed as separate semantic evidence from `derivation.yaml.overrides`, but this MVP does not modify Coverage/R6/R7 NO_GO admission behavior.
- Do not change Navigation Map generation, obstacle padding, vehicle profile, Aisle Graph geometry, Coverage Ordering, connector requests, R6A admission, R6B search, R7 state, or Route READY promotion.
- Formal vehicle readiness remains blocked by the unverified real MK-mini `base_footprint` reference/final mounted envelope; Route Debug remains preview/explainability tooling.
- No new third-party dependency is required. Use standard `json` for GeoJSON.
- Every task follows RED → GREEN → focused regression → commit. Do not claim real-data visual acceptance until the operator runs the final greenhouse smoke checklist.

## File Structure

Create these focused production units:

```text
src/agt_offline_assets/agt_offline_assets/
  route_debug_dataset.py     # frozen-asset discovery, validation, joins, provenance
  route_debug_overlay.py     # GeoJSON feature generation + collision/source diagnostics

src/agt_map_workbench/agt_map_workbench/
  route_debug_view.py        # shared-scene renderer, layer groups, presets, selection
  route_debug_panel.py       # controls, run-directory load/reload, Inspector, PNG export
```

Modify only these integration surfaces:

```text
src/agt_offline_assets/agt_offline_assets/__init__.py
src/agt_offline_assets/CMakeLists.txt
src/agt_map_workbench/agt_map_workbench/app.py
src/agt_map_workbench/agt_map_workbench/review_workbench.py
src/agt_map_workbench/agt_map_workbench/__init__.py
src/agt_map_workbench/CMakeLists.txt
src/agt_map_workbench/README.md
docs/v2.5/V25_12E_CURRENT_STATE.md
```

Add tests:

```text
src/agt_offline_assets/test/route_debug_test_data.py
src/agt_offline_assets/test/test_route_debug_dataset.py
src/agt_offline_assets/test/test_route_debug_overlay.py
src/agt_map_workbench/test/test_route_debug_view.py
src/agt_map_workbench/test/test_route_debug_panel.py
src/agt_map_workbench/test/test_route_debug_workbench_integration.py
tests/test_v25_12e_route_debug_contract.py
```

---

### Task 1: Frozen RouteDebugDataset Core and Graceful Asset Loading

**Files:**
- Create: `src/agt_offline_assets/agt_offline_assets/route_debug_dataset.py`
- Create: `src/agt_offline_assets/test/route_debug_test_data.py`
- Create: `src/agt_offline_assets/test/test_route_debug_dataset.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/__init__.py`
- Modify: `src/agt_offline_assets/CMakeLists.txt`

**Interfaces:**
- Consumes: existing `load_navigation_grid(path)`, `load_agricultural_aisle_graph(path)`, `load_turn_zones(path)`, `load_canonical_vehicle_profile(path)`, YAML files in one run directory.
- Produces:

```python
ASSET_LOADED = "LOADED"
ASSET_MISSING = "MISSING"
ASSET_INVALID = "INVALID"

@dataclass(frozen=True)
class RouteDebugAssetState:
    key: str
    availability: str
    path: str | None
    schema: str | None
    frame_id: str | None
    status: str | None
    error: str = ""

@dataclass(frozen=True)
class RouteDebugNoGoRegion:
    region_id: str
    polygon_xy: tuple[tuple[float, float], ...]
    source_asset: str

@dataclass(frozen=True)
class RouteDebugCoverageTraversal:
    sequence: int
    aisle_id: str
    motion_direction: str
    graph_orientation: str
    entry_side: str
    exit_side: str
    geometric_width_m: float
    required_width_m: float

@dataclass(frozen=True)
class RouteDebugCoverageRejection:
    aisle_id: str
    reason: str
    geometric_width_m: float
    required_width_m: float

@dataclass(frozen=True)
class RouteDebugAisleRecord:
    aisle_id: str
    aisle: AislePrimitive
    traversal: RouteDebugCoverageTraversal | None
    rejection: RouteDebugCoverageRejection | None

@dataclass(frozen=True)
class RouteDebugDataset:
    run_dir: Path
    frame_id: str
    navigation: NavigationGridEvidence | None
    aisle_graph: AgriculturalAisleGraph | None
    turn_zones: TurnZoneSet | None
    vehicle_profile: CanonicalVehicleProfile | None
    no_go_regions: tuple[RouteDebugNoGoRegion, ...]
    aisles: tuple[RouteDebugAisleRecord, ...]
    connector_requests: tuple[ConnectorRequest, ...]
    asset_states: tuple[RouteDebugAssetState, ...]

    def asset_state(self, key: str) -> RouteDebugAssetState: ...
    def aisle_by_id(self, aisle_id: str) -> RouteDebugAisleRecord | None: ...


def load_route_debug_dataset(
    run_dir: str | Path,
    *,
    vehicle_profile_path: str | Path | None = None,
) -> RouteDebugDataset: ...
```

- Coverage parsing is debug-specific and must read `coverage_order.yaml` fully, including `traversals`, `connector_requests`, and `rejected_aisles`, because the existing `load_coverage_connector_requests()` intentionally exposes only requests.
- Dataset frame authority: first valid `navigation_map.yaml`, else valid `aisle_graph.yaml`, else valid `coverage_order.yaml`. Any later framed asset that disagrees is marked `INVALID` and excluded; do not fail the whole page.
- NO_GO recovery reads `derivation.yaml.overrides[*]` where `mode == "no_go"`; preserve polygon geometry and source path even though PGM rasterization is OCCUPIED.
- Profile resolution: if `vehicle_profile_path` is explicit, use it. Otherwise infer `platform_id` from valid coverage/planner assets and walk parents of `run_dir` until `profiles/platforms/<platform_id>.yaml` exists. If not found, mark `vehicle_profile` MISSING and keep non-vehicle layers usable.

- [ ] **Step 1: Write the failing dataset tests and fixture builder**

Create `route_debug_test_data.py` with real on-disk YAML/PGM fixtures, not mocks. The helper must write a 0.10 m trinary Navigation Map, `derivation.yaml`, two aisles, one coverage traversal pair/request, and an optional NO_GO polygon.

Key fixture behavior:

```python
def write_minimal_route_debug_run(
    root: Path,
    *,
    include_no_go: bool = False,
    coverage_frame: str = "map",
) -> tuple[Path, Path]:
    run_dir = root / "runtime" / "maps" / "debug_run"
    run_dir.mkdir(parents=True)
    # Write navigation_map.pgm/yaml, derivation.yaml, aisle_graph.yaml,
    # turn_zones.yaml, coverage_order.yaml and profiles/platforms/mk_mini.yaml.
    return run_dir, root / "profiles/platforms/mk_mini.yaml"
```

Add focused tests:

```python
def test_route_debug_dataset_recovers_no_go_and_coverage(tmp_path):
    run_dir, profile = write_minimal_route_debug_run(tmp_path, include_no_go=True)
    dataset = load_route_debug_dataset(run_dir, vehicle_profile_path=profile)

    assert dataset.frame_id == "map"
    assert dataset.asset_state("navigation").availability == "LOADED"
    assert dataset.asset_state("coverage_order").availability == "LOADED"
    assert len(dataset.no_go_regions) == 1
    assert dataset.no_go_regions[0].polygon_xy[0] == (1.0, -0.5)
    assert len(dataset.aisles) == 2
    assert dataset.aisle_by_id("aisle_001").traversal.sequence == 1
    assert dataset.aisle_by_id("aisle_002").traversal.motion_direction == "FORWARD"
    assert len(dataset.connector_requests) == 1


def test_invalid_optional_frame_fails_closed_for_only_that_layer(tmp_path):
    run_dir, profile = write_minimal_route_debug_run(tmp_path, coverage_frame="odom")
    dataset = load_route_debug_dataset(run_dir, vehicle_profile_path=profile)

    assert dataset.frame_id == "map"
    assert dataset.asset_state("aisle_graph").availability == "LOADED"
    assert dataset.asset_state("coverage_order").availability == "INVALID"
    assert dataset.connector_requests == ()
    assert len(dataset.aisles) == 2
```

- [ ] **Step 2: Run the tests and verify RED**

Run:

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_route_debug_dataset.py
```

Expected: collection/import failure because `agt_offline_assets.route_debug_dataset` does not exist.

- [ ] **Step 3: Implement the minimum dataset loader**

Implement the dataclasses/signatures above plus small private helpers:

```python
def _load_yaml(path: Path) -> Mapping[str, Any]: ...
def _asset_state(...): ...
def _frame_accepts(authoritative: str | None, candidate: str | None) -> bool: ...
def _load_debug_coverage(path: Path) -> tuple[
    tuple[RouteDebugCoverageTraversal, ...],
    tuple[ConnectorRequest, ...],
    tuple[RouteDebugCoverageRejection, ...],
    str,
    str,
]: ...
def _load_no_go_regions(derivation_path: Path) -> tuple[RouteDebugNoGoRegion, ...]: ...
```

Schema checks must use exact current schema constants:

```python
NAVIGATION_DERIVATION_SCHEMA = "agt_ground_relative_navigation_map/v1"
COVERAGE_ORDER_SCHEMA = "agt_agricultural_coverage_order/v1"
```

Join `AislePrimitive` records to traversal/rejection by `aisle_id`; never change geometry or eligibility.

Export the public dataset API from `agt_offline_assets/__init__.py` and register `test_route_debug_dataset` in `src/agt_offline_assets/CMakeLists.txt`.

- [ ] **Step 4: Run focused and neighboring tests**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_route_debug_dataset.py \
  src/agt_offline_assets/test/test_agricultural_route_io.py \
  src/agt_offline_assets/test/test_navigation_grid.py
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/route_debug_dataset.py \
  src/agt_offline_assets/agt_offline_assets/__init__.py \
  src/agt_offline_assets/test/route_debug_test_data.py \
  src/agt_offline_assets/test/test_route_debug_dataset.py \
  src/agt_offline_assets/CMakeLists.txt
git commit -m "feat(route-debug): load frozen route debug dataset"
```

---

### Task 2: Join Forward/R6 Cause Chain and Vehicle-Lane Diagnostics

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/route_debug_dataset.py`
- Modify: `src/agt_offline_assets/test/route_debug_test_data.py`
- Modify: `src/agt_offline_assets/test/test_route_debug_dataset.py`

**Interfaces:**
- Consumes Task 1 `RouteDebugDataset` and current schemas:

```text
agt_forward_connector_plan/v1
agt_forward_connector_zone_fit_diagnostic/v1
agt_forward_connector_candidate_audit/v1
agt_reverse_fallback_admission/v1
agt_reverse_primitive_connector_plan/v1
agt_vehicle_safe_aisle_lane/v1
agt_vehicle_safe_lane_diagnostic/v1
agt_vehicle_safe_lane_occupancy_source_audit/v1
agt_connector_anchor_plan/v1
```

- Produces these additional immutable records and lookup methods:

```python
@dataclass(frozen=True)
class RouteDebugMotionSample:
    x: float
    y: float
    z: float
    yaw: float
    motion_direction: str
    segment_index: int | None = None
    is_cusp: bool = False

@dataclass(frozen=True)
class RouteDebugForwardCandidateRecord:
    path_type: str
    length_m: float
    local_headland_candidate: bool
    preview_footprint_free: bool
    max_required_zone_extension_m: float

@dataclass(frozen=True)
class RouteDebugVehicleLaneRecord:
    aisle_id: str
    status: str
    coverage_fraction: float
    centerline_xyz: tuple[tuple[float, float, float], ...]
    reason: str

@dataclass(frozen=True)
class RouteDebugLaneDiagnosticRecord:
    aisle_id: str
    classification: str
    reference_free_fraction: float
    any_lateral_footprint_free_fraction: float

@dataclass(frozen=True)
class RouteDebugOccupancySourceRecord:
    aisle_id: str
    classification: str
    raw_obstacle_fraction: float
    geometry_fraction: float
    padding_fraction: float
    unexplained_fraction: float

@dataclass(frozen=True)
class RouteDebugConnectorRecord:
    connector_id: str
    from_aisle_id: str
    to_aisle_id: str
    turn_zone_id: str
    request: ConnectorRequest | None
    forward_status: str | None
    forward_backend: str | None
    forward_path_type: str | None
    forward_samples: tuple[RouteDebugMotionSample, ...]
    zone_fit_status: str | None
    forward_audit_status: str | None
    forward_candidates: tuple[RouteDebugForwardCandidateRecord, ...]
    r6a_decision: str | None
    r6b_status: str | None
    r6b_backend: str | None
    r6b_samples: tuple[RouteDebugMotionSample, ...]
    r6b_path_length_m: float | None
    r6b_forward_distance_m: float | None
    r6b_reverse_distance_m: float | None
    r6b_cusp_count: int | None
    r6b_search_expansions: int | None
    r6b_goal_position_error_m: float | None
    r6b_goal_yaw_error_rad: float | None

# Extend RouteDebugAisleRecord with:
vehicle_lane: RouteDebugVehicleLaneRecord | None
diagnostic: RouteDebugLaneDiagnosticRecord | None
occupancy_source: RouteDebugOccupancySourceRecord | None

# Extend RouteDebugDataset with:
connectors: tuple[RouteDebugConnectorRecord, ...]
connector_by_id(connector_id: str) -> RouteDebugConnectorRecord | None
```

- Optional asset discovery must inspect top-level YAML `schema` values rather than depend on one filename. For multiple `agt_reverse_primitive_connector_plan/v1` files use deterministic priority:

```text
reverse_primitive_connectors_anchored.yaml
reverse_primitive_connectors.yaml
then lexicographically first reverse_primitive_connectors*.yaml
```

- [ ] **Step 1: Extend the fixture builder and write RED cause-chain tests**

Add fixture options that write a solved `connector_015`-like R6B record and a searched-but-unsolved `connector_017`-like record. The test does not need the real greenhouse geometry; it must preserve the semantic distinction.

```python
def test_connector_cause_chain_distinguishes_solved_and_searched_unsolved(tmp_path):
    run_dir, profile = write_minimal_route_debug_run(tmp_path)
    write_route_debug_planner_assets(run_dir)

    dataset = load_route_debug_dataset(run_dir, vehicle_profile_path=profile)

    solved = dataset.connector_by_id("connector_015")
    assert solved.r6a_decision == "ELIGIBLE_REVERSE_FALLBACK"
    assert solved.r6b_status == "REVERSE_PRIMITIVE_PREVIEW_FREE"
    assert solved.r6b_backend == "BOUNDED_REVERSE_PRIMITIVE_SEARCH_NOT_ANALYTIC_REEDS_SHEPP"
    assert [s.motion_direction for s in solved.r6b_samples] == ["FORWARD", "REVERSE", "FORWARD"]
    assert solved.r6b_cusp_count == 2

    unsolved = dataset.connector_by_id("connector_017")
    assert unsolved.request is not None
    assert unsolved.r6a_decision == "ELIGIBLE_REVERSE_FALLBACK"
    assert unsolved.r6b_status == "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION"
    assert unsolved.r6b_search_expansions == 67
    assert unsolved.r6b_samples == ()
```

Also lock lane/diagnostic joins:

```python
def test_aisle_record_joins_vehicle_lane_and_blocker_source(tmp_path):
    run_dir, profile = write_minimal_route_debug_run(tmp_path)
    write_route_debug_lane_assets(run_dir)
    dataset = load_route_debug_dataset(run_dir, vehicle_profile_path=profile)

    aisle = dataset.aisle_by_id("aisle_001")
    assert aisle.vehicle_lane.status == "VEHICLE_SAFE_LANE_PARTIAL"
    assert aisle.diagnostic.classification == "FOOTPRINT_OCCUPIED_DOMINANT"
    assert aisle.occupancy_source.classification == "PADDING_ONLY_DOMINANT"
```

- [ ] **Step 2: Run tests and verify RED**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_route_debug_dataset.py
```

Expected: FAIL because the new connector/lane fields and schema discovery are absent.

- [ ] **Step 3: Implement schema-based optional discovery and joins**

Implement:

```python
def _discover_yaml_by_schema(run_dir: Path) -> dict[str, tuple[Path, ...]]: ...
def _choose_reverse_primitive_asset(paths: tuple[Path, ...]) -> Path | None: ...
def _load_forward_debug(...): ...
def _load_reverse_debug(...): ...
def _load_vehicle_lane_debug(...): ...
def _join_connector_records(...): ...
```

For each optional file:
- validate the exact schema before parsing
- validate its `frame_id` against dataset authority
- mark only that asset `INVALID` if parsing/frame validation fails
- preserve source filename in its `RouteDebugAssetState`
- do not infer planner success from absence of a path; carry the frozen status string.

- [ ] **Step 4: Run route-debug plus source producer regressions**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_route_debug_dataset.py \
  src/agt_offline_assets/test/test_forward_connector.py \
  src/agt_offline_assets/test/test_forward_connector_candidate_audit.py \
  src/agt_offline_assets/test/test_reverse_fallback_admission.py \
  src/agt_offline_assets/test/test_reverse_primitive_connector.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane_diagnostics.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane_occupancy_sources.py
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/route_debug_dataset.py \
  src/agt_offline_assets/test/route_debug_test_data.py \
  src/agt_offline_assets/test/test_route_debug_dataset.py
git commit -m "feat(route-debug): join planner and lane diagnostics"
```

---

### Task 3: Build Render-Only Route Debug GeoJSON

**Files:**
- Create: `src/agt_offline_assets/agt_offline_assets/route_debug_overlay.py`
- Create: `src/agt_offline_assets/test/test_route_debug_overlay.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/__init__.py`
- Modify: `src/agt_offline_assets/CMakeLists.txt`

**Interfaces:**
- Consumes Task 2 `RouteDebugDataset`.
- Produces:

```python
ROUTE_DEBUG_OVERLAY_SCHEMA = "agt_route_debug_overlay/v1"

@dataclass(frozen=True)
class RouteDebugOverlayConfig:
    forward_sample_step_m: float = 0.05
    collision_sample_spacing_m: float = 0.10
    lateral_search_step_m: float = 0.05
    maximum_lateral_shift_m: float = 0.50
    preview_footprint_padding_m: float = 0.05


def build_route_debug_overlay(
    dataset: RouteDebugDataset,
    config: RouteDebugOverlayConfig | None = None,
) -> dict[str, Any]: ...


def write_route_debug_overlay(
    overlay: Mapping[str, Any],
    path: str | Path,
    *,
    overwrite: bool = False,
) -> Path: ...
```

Top-level serialization is fixed:

```python
{
    "type": "FeatureCollection",
    "agt_schema": "agt_route_debug_overlay/v1",
    "frame_id": dataset.frame_id,
    "validation_scope": "DEBUG_RENDER_ONLY",
    "features": [...],
}
```

Every feature property must contain:

```text
feature_id
layer_group
layer_key
feature_kind
frame_id
status
source_asset
source_id
source_field
is_failure
inspector
```

- [ ] **Step 1: Write RED overlay structure/coverage/motion tests**

```python
def test_overlay_separates_structural_coverage_request_and_motion(tmp_path):
    run_dir, profile = write_minimal_route_debug_run(tmp_path)
    write_route_debug_planner_assets(run_dir)
    dataset = load_route_debug_dataset(run_dir, vehicle_profile_path=profile)

    overlay = build_route_debug_overlay(dataset)
    assert overlay["type"] == "FeatureCollection"
    assert overlay["agt_schema"] == "agt_route_debug_overlay/v1"
    kinds = {f["properties"]["feature_kind"] for f in overlay["features"]}
    assert "AISLE_CENTERLINE" in kinds
    assert "COVERAGE_TRAVERSAL" in kinds
    assert "CONNECTOR_REQUEST" in kinds
    assert "R6_MOTION_SEGMENT" in kinds
    assert "CUSP" in kinds

    request = next(f for f in overlay["features"] if f["properties"]["feature_kind"] == "CONNECTOR_REQUEST")
    assert request["properties"]["layer_group"] == "coverage"
    motion = next(f for f in overlay["features"] if f["properties"]["feature_kind"] == "R6_MOTION_SEGMENT")
    assert motion["properties"]["layer_group"] == "motion"
```

Add writer immutability:

```python
def test_overlay_writer_refuses_accidental_overwrite(tmp_path):
    path = tmp_path / "route_debug_overlay.geojson"
    write_route_debug_overlay({"type": "FeatureCollection", "features": []}, path)
    with pytest.raises(FileExistsError):
        write_route_debug_overlay({"type": "FeatureCollection", "features": []}, path)
```

- [ ] **Step 2: Run and verify RED**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_route_debug_overlay.py
```

Expected: import failure because `route_debug_overlay.py` does not exist.

- [ ] **Step 3: Implement base GeoJSON features and forward-candidate replay**

Create helper constructors rather than hand-building property dictionaries repeatedly:

```python
def _feature(
    *, feature_id: str, layer_group: str, layer_key: str,
    feature_kind: str, frame_id: str, geometry: Mapping[str, Any],
    status: str, source_asset: str, source_id: str,
    source_field: str, is_failure: bool,
    inspector: Mapping[str, Any],
) -> dict[str, Any]: ...
```

Generate:
- NO_GO Polygon from dataset semantics
- Aisle `LineString`
- Vehicle-safe lane `LineString` when non-empty
- Turn Zone Polygon
- coverage traversal `LineString` with sequence/orientation in inspector
- ConnectorRequest `LineString` between frozen start/goal
- frozen forward selected samples when present
- frozen R6 samples split into separate FORWARD/REVERSE `LineString`s at motion-direction changes
- `Point` cusp markers at `is_cusp=True`
- failed connector `Point` at request midpoint when a frozen failure status exists.

Candidate audit geometry has no samples. Replay geometry **only inside `agt_offline_assets`** from the frozen request + canonical Rmin using the existing `_dubins_candidates()` and `_sample_candidate()` helpers. Attach:

```text
derived_geometry_method = REPLAY_ANALYTIC_DUBINS_FROM_FROZEN_REQUEST_AND_PROFILE
```

and join audit status by `path_type`; do not change which candidate was accepted/rejected.

- [ ] **Step 4: Run focused tests**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_route_debug_overlay.py \
  src/agt_offline_assets/test/test_route_debug_dataset.py \
  src/agt_offline_assets/test/test_forward_connector.py \
  src/agt_offline_assets/test/test_reverse_primitive_connector.py
```

Expected: PASS.

- [ ] **Step 5: Export overlay API and register test, then commit**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/route_debug_overlay.py \
  src/agt_offline_assets/agt_offline_assets/__init__.py \
  src/agt_offline_assets/test/test_route_debug_overlay.py \
  src/agt_offline_assets/CMakeLists.txt
git commit -m "feat(route-debug): build render-only GeoJSON overlay"
```

---

### Task 4: Add Collision-Location and Source-Provenance Features

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/route_debug_dataset.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/route_debug_overlay.py`
- Modify: `src/agt_offline_assets/test/test_route_debug_overlay.py`

**Interfaces:**
- Extend `RouteDebugDataset` with:

```python
occupancy_source_masks: NavigationOccupancySourceMasks | None
```

Load it through existing `load_navigation_occupancy_source_masks(navigation, navigation_asset)` only when required frozen sidecars are present and valid.

- Collision feature policy: do not serialize every global occupied cell to GeoJSON. Emit one `COLLISION_STATION` point for every sampled aisle station with no fully FREE bounded-lateral vehicle pose. Store the selected best candidate's footprint polygon in properties so Qt can highlight it without recomputing collision math.

- [ ] **Step 1: Write RED synthetic collision-provenance tests**

Construct a small direct `RouteDebugDataset` in the test with:
- 0.10 m Navigation Grid
- one 0.70 m-wide aisle, so lateral shift is zero with a 0.60 m vehicle + 0.05 m preview padding
- a direct raw obstacle cell just outside the footprint edge
- a neighboring PADDING_ONLY occupied cell inside the footprint
- explicit `NavigationOccupancySourceMasks`.

Test:

```python
def test_collision_station_reports_padding_only_and_footprint_polygon():
    dataset = synthetic_padding_collision_dataset()
    overlay = build_route_debug_overlay(dataset)
    conflicts = [
        f for f in overlay["features"]
        if f["properties"]["feature_kind"] == "COLLISION_STATION"
    ]
    assert conflicts
    first = conflicts[0]["properties"]
    assert first["dominant_source"] == "PADDING_ONLY"
    assert first["occupied_count"] > 0
    assert len(first["footprint_polygon_xy"]) >= 4
    assert first["source_asset"] == "navigation_map.yaml"
```

Also add an UNKNOWN-only case:

```python
def test_collision_station_keeps_unknown_distinct_from_occupied():
    dataset = synthetic_unknown_collision_dataset()
    overlay = build_route_debug_overlay(dataset)
    first = next(f for f in overlay["features"] if f["properties"]["feature_kind"] == "COLLISION_STATION")
    assert first["properties"]["dominant_source"] == "UNKNOWN"
    assert first["properties"]["unknown_count"] > 0
```

- [ ] **Step 2: Run and verify RED**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_route_debug_overlay.py \
  -k "collision_station"
```

Expected: FAIL because collision station features are absent.

- [ ] **Step 3: Implement collision station generation by reusing existing offline helpers**

Use, inside `route_debug_overlay.py` only:

```python
from .forward_connector_navigation_gate import _preview_local_footprint, _transform_polygon
from .vehicle_safe_lane import _normalize, _offset_candidates, _resample_polyline
from .vehicle_safe_lane_occupancy_sources import _pose_mask, _candidate_key, _fully_free
```

Algorithm per aisle:

```text
resample structural centerline at collision_sample_spacing_m
→ compute allowed lateral shift from structural width and preview vehicle width
→ evaluate all bounded offsets
→ if any full pose is FREE: no collision marker
→ else choose candidate using existing least-total-non-FREE ordering
→ classify selected footprint cells:
   RAW_OBSTACLE_DIRECT / GEOMETRY_DIRECT / PADDING_ONLY /
   UNEXPLAINED_OCCUPIED / UNKNOWN / OUT_OF_GRID
→ emit collision Point + footprint_polygon_xy + source counts
```

Do not relax occupancy, do not treat UNKNOWN as FREE, and do not write any modified map.

- [ ] **Step 4: Run collision/source regressions**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_route_debug_overlay.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane_diagnostics.py \
  src/agt_offline_assets/test/test_vehicle_safe_lane_occupancy_sources.py
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/route_debug_dataset.py \
  src/agt_offline_assets/agt_offline_assets/route_debug_overlay.py \
  src/agt_offline_assets/test/test_route_debug_overlay.py
git commit -m "feat(route-debug): expose collision locations and provenance"
```

---

### Task 5: Shared-Scene Route Debug Renderer and Layer Presets

**Files:**
- Create: `src/agt_map_workbench/agt_map_workbench/route_debug_view.py`
- Create: `src/agt_map_workbench/test/test_route_debug_view.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/__init__.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**
- Consumes `RouteDebugDataset` and GeoJSON mapping from Tasks 1-4.
- Produces:

```python
ROUTE_DEBUG_PRESETS: Mapping[str, frozenset[str]]

class RouteDebugSceneController(QObject):
    featureSelected = pyqtSignal(object)

    def __init__(self, scene: QGraphicsScene, parent: QObject | None = None): ...
    def set_content(self, dataset: RouteDebugDataset, overlay: Mapping[str, Any]) -> None: ...
    def clear(self) -> None: ...
    def set_active(self, active: bool) -> None: ...
    def set_layer_visible(self, layer_key: str, visible: bool) -> None: ...
    def layer_visible(self, layer_key: str) -> bool: ...
    def apply_preset(self, preset_name: str) -> None: ...
    def set_failure_focus(self, enabled: bool) -> None: ...
    def route_bounds(self) -> QRectF: ...
    def select_feature(self, feature_id: str) -> bool: ...
```

Layer keys are fixed for MVP:

```text
base.navigation
base.no_go
structure.turn_zones
structure.aisles
structure.vehicle_safe_lane
coverage.order
coverage.requests
motion.forward_candidates
motion.forward_selected
motion.reverse
diagnostics.raw
diagnostics.geometry
diagnostics.padding
diagnostics.unknown
diagnostics.conflicts
diagnostics.failed
```

Preset membership:

```python
ROUTE_DEBUG_PRESETS = {
    "coverage": frozenset({
        "base.navigation", "base.no_go", "structure.aisles",
        "structure.vehicle_safe_lane", "coverage.order", "coverage.requests",
    }),
    "planning": frozenset({
        "structure.aisles", "coverage.requests", "motion.forward_candidates",
        "motion.forward_selected", "motion.reverse", "diagnostics.failed",
    }),
    "collision": frozenset({
        "base.navigation", "base.no_go", "structure.aisles",
        "diagnostics.raw", "diagnostics.geometry", "diagnostics.padding",
        "diagnostics.unknown", "diagnostics.conflicts", "diagnostics.failed",
    }),
}
```

- [ ] **Step 1: Write RED Qt controller tests**

Use `QT_QPA_PLATFORM=offscreen` and instantiate a real `QApplication`, `QGraphicsScene`, and controller. Avoid scripted mouse coordinates.

```python
def test_route_debug_controller_applies_presets_and_world_y_flip(qapp):
    scene = QGraphicsScene()
    controller = RouteDebugSceneController(scene)
    dataset, overlay = synthetic_route_debug_content()
    controller.set_content(dataset, overlay)
    controller.apply_preset("coverage")

    assert controller.layer_visible("coverage.order")
    assert controller.layer_visible("coverage.requests")
    assert not controller.layer_visible("motion.reverse")

    assert controller.select_feature("aisle:aisle_001")
    item = next(item for item in scene.selectedItems() if item.data(0) == "aisle:aisle_001")
    assert item.sceneBoundingRect().center().y() < 0.0  # positive world Y renders negative scene Y
```

Selection dispatch test:

```python
def test_route_debug_controller_emits_inspector_payload(qapp):
    scene = QGraphicsScene()
    controller = RouteDebugSceneController(scene)
    dataset, overlay = synthetic_route_debug_content()
    selected = []
    controller.featureSelected.connect(selected.append)
    controller.set_content(dataset, overlay)
    assert controller.select_feature("connector:connector_015:r6:0")
    assert selected[-1]["source_id"] == "connector_015"
```

- [ ] **Step 2: Run and verify RED**

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/agt_map_workbench/test/test_route_debug_view.py
```

Expected: import failure because `route_debug_view.py` does not exist.

- [ ] **Step 3: Implement renderer without planner math**

Implementation rules:

```python
def _scene_xy(x: float, y: float) -> QPointF:
    return QPointF(float(x), float(-y))
```

- Render `NavigationGridEvidence` as a `QGraphicsPixmapItem` using the existing navigation-preview flip/placement convention: image rows flip for display; item origin is `(origin_x, -(origin_y + height*resolution))`; item scale is `resolution`.
- Render source masks as separate translucent pixmaps for raw/geometry/padding. Do not serialize every mask cell as a vector item.
- Render GeoJSON vectors into per-layer `QGraphicsItemGroup`s. Set `item.setData(0, feature_id)` and `item.setData(1, inspector_dict)` on selectable vector/conflict objects.
- Clicking/selecting a conflict may create a temporary footprint outline directly from its stored `footprint_polygon_xy`; do not recompute footprint collision.
- `set_failure_focus(True)` reduces opacity of features whose `is_failure` is false and never changes data membership.
- `clear()` removes only items owned by the controller; never clear the whole shared Workbench scene.

Export `RouteDebugSceneController` from `agt_map_workbench/__init__.py` and register the test in CMake.

- [ ] **Step 4: Run focused Qt and existing view regressions**

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/agt_map_workbench/test/test_route_debug_view.py \
  src/agt_map_workbench/test/test_workbench_model.py \
  src/agt_map_workbench/test/test_frame_calibration.py
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add \
  src/agt_map_workbench/agt_map_workbench/route_debug_view.py \
  src/agt_map_workbench/agt_map_workbench/__init__.py \
  src/agt_map_workbench/test/test_route_debug_view.py \
  src/agt_map_workbench/CMakeLists.txt
git commit -m "feat(route-debug): render layered route debug scene"
```

---

### Task 6: Route Debug Panel, Layer Tree, Inspector, Presets, and PNG Export

**Files:**
- Create: `src/agt_map_workbench/agt_map_workbench/route_debug_panel.py`
- Create: `src/agt_map_workbench/test/test_route_debug_panel.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/__init__.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**
- Consumes `load_route_debug_dataset()`, `build_route_debug_overlay()`, `write_route_debug_overlay()`, `RouteDebugSceneController`.
- Produces:

```python
class RouteDebugPanel(QWidget):
    def __init__(self, scene: QGraphicsScene, view: QGraphicsView, parent=None): ...
    def load_run_directory(self, path: str | Path) -> RouteDebugDataset: ...
    def reload_current_directory(self) -> RouteDebugDataset | None: ...
    def set_active(self, active: bool) -> None: ...
    def apply_preset(self, preset_name: str) -> None: ...
    def set_failure_focus(self, enabled: bool) -> None: ...
    def export_current_view(self, path: str | Path) -> bool: ...
    def current_run_directory(self) -> Path | None: ...
```

The panel loads synchronously for MVP because the frozen 2D YAML/PGM/NPY assets are small compared with the already-separate full PCD processing pipeline. Do not add a worker thread unless a measured UI stall later justifies it.

- [ ] **Step 1: Write RED panel behavior tests**

```python
def test_panel_loads_directory_writes_only_debug_overlay_and_populates_inspector(qapp, tmp_path):
    run_dir, profile = write_minimal_route_debug_run(tmp_path, include_no_go=True)
    write_route_debug_planner_assets(run_dir)
    scene = QGraphicsScene()
    view = QGraphicsView(scene)
    panel = RouteDebugPanel(scene, view)

    dataset = panel.load_run_directory(run_dir)
    assert dataset.run_dir == run_dir.resolve()
    assert (run_dir / "route_debug_overlay.geojson").is_file()
    assert panel.current_run_directory() == run_dir.resolve()

    assert panel.controller.select_feature("aisle:aisle_001")
    text = panel.inspector_text()
    assert "aisle_001" in text
    assert "STRUCTURE" in text
```

Preset/layer availability test:

```python
def test_panel_disables_missing_optional_layers_without_blocking_coverage(qapp, tmp_path):
    run_dir, _ = write_minimal_route_debug_run(tmp_path)
    scene = QGraphicsScene()
    view = QGraphicsView(scene)
    panel = RouteDebugPanel(scene, view)
    panel.load_run_directory(run_dir)
    panel.apply_preset("coverage")

    assert panel.layer_enabled("coverage.order")
    assert not panel.layer_enabled("motion.reverse")
```

- [ ] **Step 2: Run and verify RED**

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/agt_map_workbench/test/test_route_debug_panel.py
```

Expected: import failure because `route_debug_panel.py` does not exist.

- [ ] **Step 3: Implement the panel**

Build the approved controls:

```text
[选择运行目录] [重载] [适配全图]
[Coverage 总览] [规划结果] [碰撞诊断]
[仅失败] [导出 PNG]
Layers tree
Inspector
```

Use a two-column `QTreeWidget` for layer group/availability and a read-only `QTextBrowser` or two-column `QTreeWidget` for Inspector. Layer children are checkable only when the corresponding source is available/valid.

`load_run_directory()` must execute exactly:

```python
dataset = load_route_debug_dataset(path)
overlay = build_route_debug_overlay(dataset)
write_route_debug_overlay(
    overlay,
    dataset.run_dir / "route_debug_overlay.geojson",
    overwrite=True,
)
self.controller.set_content(dataset, overlay)
```

The only file the panel may replace is `route_debug_overlay.geojson`.

Inspector formatting must group known keys:

```text
STRUCTURE
COVERAGE
VEHICLE
NAVIGATION / VEHICLE FEASIBILITY
CONFLICT
RELATED
SOURCE
```

For connector features show cause-chain fields in order:

```text
Coverage Request
Forward
R5.6
R6A
R6B
```

Use frozen fields; do not invent an R7 status.

`export_current_view(path)` uses `self._view.grab().save(str(path), "PNG")` and returns the boolean result.

- [ ] **Step 4: Run panel/view tests**

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/agt_map_workbench/test/test_route_debug_panel.py \
  src/agt_map_workbench/test/test_route_debug_view.py
```

Expected: PASS.

- [ ] **Step 5: Export API and commit**

```bash
git add \
  src/agt_map_workbench/agt_map_workbench/route_debug_panel.py \
  src/agt_map_workbench/agt_map_workbench/__init__.py \
  src/agt_map_workbench/test/test_route_debug_panel.py \
  src/agt_map_workbench/CMakeLists.txt
git commit -m "feat(route-debug): add route debug controls and inspector"
```

---

### Task 7: Mount the New 路径调试 Tab into the Existing Workbench

**Files:**
- Modify: `src/agt_map_workbench/agt_map_workbench/app.py` around the right-side controls `QTabWidget`
- Modify: `src/agt_map_workbench/agt_map_workbench/review_workbench.py` in `ReviewMapWorkbenchWindow.__init__`
- Create: `src/agt_map_workbench/test/test_route_debug_workbench_integration.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**
- Consumes Task 6 `RouteDebugPanel`.
- Produces no new planner API. It exposes the existing right-side control tabs as `self._control_tabs` and adds one child tab named exactly `路径调试`.

- [ ] **Step 1: Write RED integration test**

```python
def test_review_workbench_contains_route_debug_control_tab(qapp):
    window = ReviewMapWorkbenchWindow()
    labels = [window._control_tabs.tabText(i) for i in range(window._control_tabs.count())]
    assert labels == ["点云编辑", "坐标系标定", "导航地图", "路径调试"]
    assert isinstance(window._route_debug_panel, RouteDebugPanel)
```

Also verify route layers are active only on that tab:

```python
def test_route_debug_activation_does_not_leave_overlay_on_authoring_tabs(qapp):
    window = ReviewMapWorkbenchWindow()
    route_index = window._route_debug_tab_index
    window._control_tabs.setCurrentIndex(route_index)
    assert window._route_debug_panel.controller.is_active()
    window._control_tabs.setCurrentIndex(0)
    assert not window._route_debug_panel.controller.is_active()
```

Add `is_active() -> bool` to the controller if not already present.

- [ ] **Step 2: Run and verify RED**

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/agt_map_workbench/test/test_route_debug_workbench_integration.py
```

Expected: FAIL because `MapWorkbenchWindow` does not expose `_control_tabs` and `ReviewMapWorkbenchWindow` does not mount Route Debug.

- [ ] **Step 3: Make the smallest integration changes**

In `app.py`, replace only the local control-tab ownership:

```python
self._control_tabs = QTabWidget()
self._control_tabs.addTab(self._build_edit_tab(), "点云编辑")
self._control_tabs.addTab(self._build_frame_tab(), "坐标系标定")
self._control_tabs.addTab(self._build_navigation_tab(), "导航地图")
layout.addWidget(self._control_tabs, 1)
```

Do not move editing/navigation logic into the new panel.

In `ReviewMapWorkbenchWindow.__init__` after the existing 3D/offline-action installation:

```python
self._install_route_debug()
```

Implement:

```python
def _install_route_debug(self) -> None:
    panel = RouteDebugPanel(self._scene, self._view, self)
    self._route_debug_tab_index = self._control_tabs.addTab(panel, "路径调试")
    self._route_debug_panel = panel
    self._control_tabs.currentChanged.connect(self._route_debug_control_tab_changed)
    panel.set_active(self._control_tabs.currentIndex() == self._route_debug_tab_index)


def _route_debug_control_tab_changed(self, index: int) -> None:
    active = index == self._route_debug_tab_index
    self._route_debug_panel.set_active(active)
    if active:
        self._navigation_preview_item.setVisible(False)
    else:
        self._update_navigation_overlay()
```

This preserves the actual launcher lineage through `review_workbench.main` and keeps one shared 2D scene/view.

- [ ] **Step 4: Run all Workbench tests offscreen**

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/agt_map_workbench/test
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add \
  src/agt_map_workbench/agt_map_workbench/app.py \
  src/agt_map_workbench/agt_map_workbench/review_workbench.py \
  src/agt_map_workbench/test/test_route_debug_workbench_integration.py \
  src/agt_map_workbench/CMakeLists.txt
git commit -m "feat(route-debug): mount route debug in map workbench"
```

---

### Task 8: Freeze Route-Debug Contracts, Documentation, and Local-Acceptance Gate

**Files:**
- Create: `tests/test_v25_12e_route_debug_contract.py`
- Modify: `src/agt_map_workbench/README.md`
- Modify: `docs/v2.5/V25_12E_CURRENT_STATE.md`

**Interfaces:**
- No new runtime interface. This task locks architectural boundaries and provides exact operator smoke commands.

- [ ] **Step 1: Write the RED repository contract test**

```python
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DATASET = ROOT / "src/agt_offline_assets/agt_offline_assets/route_debug_dataset.py"
OVERLAY = ROOT / "src/agt_offline_assets/agt_offline_assets/route_debug_overlay.py"
PANEL = ROOT / "src/agt_map_workbench/agt_map_workbench/route_debug_panel.py"
SPEC = ROOT / "docs/superpowers/specs/2026-08-15-route-debug-2d-design.md"
STATE = ROOT / "docs/v2.5/V25_12E_CURRENT_STATE.md"


def _read(path):
    return path.read_text(encoding="utf-8")


def test_route_debug_remains_render_only_and_not_a_planner():
    overlay = _read(OVERLAY)
    panel = _read(PANEL)
    assert 'agt_route_debug_overlay/v1' in overlay
    assert 'DEBUG_RENDER_ONLY' in overlay
    assert 'route_debug_overlay.geojson' in _read(SPEC)
    assert 'derive_reverse_primitive_connector_plan' not in panel
    assert 'derive_forward_connector_plan' not in panel
    assert 'derive_ground_relative_navigation_map' not in panel
    assert 'force_free' not in panel


def test_route_debug_preserves_no_go_and_reverse_semantics():
    dataset = _read(DATASET)
    state = _read(STATE)
    assert 'no_go' in dataset
    assert 'BOUNDED_REVERSE_PRIMITIVE_SEARCH_NOT_ANALYTIC_REEDS_SHEPP' in state
    assert 'Route Debug' in state
    assert 'LOCAL ACCEPTANCE PENDING' in state
```

- [ ] **Step 2: Run contract test and verify RED**

```bash
python3 -m pytest -q tests/test_v25_12e_route_debug_contract.py
```

Expected: FAIL until README/current-state text is updated.

- [ ] **Step 3: Update README and current-state without overclaiming**

Add a Route Debug section to `src/agt_map_workbench/README.md` covering:

```text
路径调试 is 2D and read-only
load a run directory, not individual YAML files
Coverage 总览 / 规划结果 / 碰撞诊断 presets
NO_GO is displayed separately from physical OCCUPIED
route_debug_overlay.geojson is derived render evidence only
missing optional layer vs invalid contract behavior
```

Update `docs/v2.5/V25_12E_CURRENT_STATE.md` to:

```text
Route Debug 2D
  CORE IMPLEMENTED / LOCAL ACCEPTANCE PENDING

Purpose
  visualize structural aisles + coverage + connector motion + conflict provenance
  before further R6/R7 work

No production map/route semantics changed
```

Keep the existing padding/direct-source diagnostic conclusion intact. Do not write that the greenhouse Route Debug visual acceptance passed.

- [ ] **Step 4: Run the complete focused test matrix**

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_route_debug_dataset.py \
  src/agt_offline_assets/test/test_route_debug_overlay.py \
  src/agt_map_workbench/test/test_route_debug_view.py \
  src/agt_map_workbench/test/test_route_debug_panel.py \
  src/agt_map_workbench/test/test_route_debug_workbench_integration.py \
  tests/test_v25_12e_route_debug_contract.py \
  tests/test_v25_12e_reverse_primitive_connector_contract.py \
  tests/test_v25_12e_vehicle_safe_lane_contract.py \
  tests/test_v25_12e_vehicle_safe_lane_occupancy_source_contract.py \
  tests/test_v25_12e_vehicle_safe_lane_padding_sensitivity_contract.py \
  tests/test_v25_12e_vehicle_safe_lane_direct_source_sensitivity_contract.py
```

Expected: PASS.

Then build and run package-level ament tests:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select agt_offline_assets agt_map_workbench
source install/setup.bash
colcon test --packages-select agt_offline_assets agt_map_workbench --event-handlers console_direct+
colcon test-result --verbose
```

Expected: no failed tests.

- [ ] **Step 5: Commit contract/docs**

```bash
git add \
  tests/test_v25_12e_route_debug_contract.py \
  src/agt_map_workbench/README.md \
  docs/v2.5/V25_12E_CURRENT_STATE.md
git commit -m "docs(route-debug): freeze MVP acceptance boundary"
```

- [ ] **Step 6: Generate the real greenhouse debug overlay without modifying truth assets**

After the implementation commits are available on the operator machine:

```bash
cd ~/agt_navigation_v2
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 - <<'PY'
from pathlib import Path
from agt_offline_assets import (
    build_route_debug_overlay,
    load_route_debug_dataset,
    write_route_debug_overlay,
)

run_dir = Path("runtime/maps/agt_workbench_run").resolve()
dataset = load_route_debug_dataset(run_dir)
overlay = build_route_debug_overlay(dataset)
output = write_route_debug_overlay(
    overlay,
    run_dir / "route_debug_overlay.geojson",
    overwrite=True,
)

print("frame:", dataset.frame_id)
print("aisles:", len(dataset.aisles))
print("connector requests:", len(dataset.connector_requests))
print("connectors:", len(dataset.connectors))
print("NO_GO regions:", len(dataset.no_go_regions))
print("features:", len(overlay["features"]))
print("overlay:", output)
print()
for state in dataset.asset_states:
    print(state.key, state.availability, state.path or "-", state.error or "")
PY
```

Expected structural counts for the current frozen greenhouse inputs, if the same run assets are still present:

```text
19 structural aisles
18 width accepted / 1 width rejected (aisle_005)
17 ConnectorRequests
```

Treat any mismatch as an asset-version change to investigate; do not hard-code these counts into the loader.

- [ ] **Step 7: Operator visual smoke in AGT Map Workbench**

Launch:

```bash
ros2 run agt_map_workbench agt_map_workbench
```

In `路径调试`, load:

```text
~/agt_navigation_v2/runtime/maps/agt_workbench_run
```

Perform this exact review:

```text
Coverage 总览
  verify whole greenhouse order is visible
  verify aisle traversal arrows do not imply reverse gear
  verify ConnectorRequest is visually different from solved motion

 ais le_005
  inspect structural width 0.658 m vs required preview width 0.700 m
  verify it is visibly rejected

 aisle_003
  enable collision-source view
  verify RAW obstacle evidence can be viewed with structural aisle and footprint conflict

 aisle_013
  verify PADDING_ONLY evidence can be viewed with centerline and footprint conflict

 connector_015
  verify FORWARD → REVERSE → FORWARD, 2 cusp markers
  Inspector should expose 4.499 m total, 0.600 m reverse, 1490 expansions,
  and 0.154 m / 8.11° goal error if the current frozen R6B asset is loaded

 connector_017
  verify request exists + R6A admitted + R6B searched +
  NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION is distinguishable from a missing asset

 NO_GO
  if the current run contains no NO_GO override, create a separate test derivation using the existing Navigation Map authoring page rather than editing the production run
  verify the semantic polygon is purple/separate from physical OCCUPIED
```

Do not mark `REAL-DATA OPERATOR ACCEPTANCE PASS` until this checklist is actually completed on the operator machine.

---

## Final Verification Gate Before Claiming Completion

The implementation worker must run, in order:

```bash
git status --short
python3 -m pytest -q \
  src/agt_offline_assets/test/test_route_debug_dataset.py \
  src/agt_offline_assets/test/test_route_debug_overlay.py \
  src/agt_map_workbench/test/test_route_debug_view.py \
  src/agt_map_workbench/test/test_route_debug_panel.py \
  src/agt_map_workbench/test/test_route_debug_workbench_integration.py \
  tests/test_v25_12e_route_debug_contract.py
colcon build --symlink-install --packages-select agt_offline_assets agt_map_workbench
colcon test --packages-select agt_offline_assets agt_map_workbench
colcon test-result --verbose
git status --short
```

Completion may be described only as:

```text
Route Debug 2D CORE IMPLEMENTED / AUTOMATED TESTS PASS
REAL-DATA OPERATOR VISUAL ACCEPTANCE PENDING
```

until the final greenhouse checklist is returned by the user.
