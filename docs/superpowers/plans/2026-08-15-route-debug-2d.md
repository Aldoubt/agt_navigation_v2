# AGT Map Workbench 2D Route Debug Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a read-only 2D Route Debug tab to the existing AGT Map Workbench that loads frozen greenhouse route-production assets, renders coverage/order/planning/collision evidence in one map, and lets the operator inspect why an aisle or connector is accepted, rejected, blocked, or unsolved.

**Architecture:** `agt_offline_assets` owns frozen-asset parsing, cross-asset validation, cause-chain joins, collision provenance, and generation of the render-only `route_debug_overlay.geojson`. `agt_map_workbench` only renders Navigation/source-mask rasters and GeoJSON features into the existing shared 2D `QGraphicsScene`, controls layer visibility/presets, and exposes a read-only Inspector. The GUI never reruns Dubins, R6, Navigation Map derivation, or route-admission logic.

**Tech Stack:** Python 3, dataclasses, pathlib, PyYAML, NumPy, SciPy, standard-library JSON for GeoJSON, PyQt5 `QGraphicsScene/QGraphicsView`, ROS 2 Humble `ament_cmake_python` + `ament_cmake_pytest`, pytest.

## Global Constraints

- Implement the approved spec at `docs/superpowers/specs/2026-08-15-route-debug-2d-design.md` without expanding scope.
- Route Debug is integrated into the existing AGT Map Workbench; do not create a standalone viewer and do not add 3D route linkage in this increment.
- The Route Debug page is read-only for route-production truth assets. It may create/replace only the derived `route_debug_overlay.geojson`; it must not overwrite source YAML/PGM/NPY assets.
- The GUI must not rerun planner or collision math. Planner/collision replay required for visualization belongs in `agt_offline_assets` only.
- Missing optional assets degrade to an unavailable layer. An existing asset with invalid schema or incompatible `frame_id` fails closed for that layer and is not rendered.
- Never silently convert coordinate frames. GeoJSON stores normal world `(x, y)` coordinates; Qt rendering converts them to scene `(x, -y)` consistently with the existing Workbench.
- `WITH_ROW_DIRECTION` and `AGAINST_ROW_DIRECTION` remain normal FORWARD aisle traversal. Reverse motion is rendered only when a connector sample explicitly says `motion_direction=REVERSE`.
- Preserve the exact R6 backend label `BOUNDED_REVERSE_PRIMITIVE_SEARCH_NOT_ANALYTIC_REEDS_SHEPP`; never relabel it analytic Reeds-Shepp.
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

Add tests/test helpers:

```text
src/agt_offline_assets/test/route_debug_test_data.py
src/agt_offline_assets/test/test_route_debug_dataset.py
src/agt_offline_assets/test/test_route_debug_overlay.py

src/agt_map_workbench/test/conftest.py
src/agt_map_workbench/test/route_debug_test_data.py
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
- Consumes: existing `load_navigation_grid(path)`, `load_agricultural_aisle_graph(path)`, `load_turn_zones(path)`, `load_canonical_vehicle_profile(path)`, plus frozen YAML files in one run directory.
- Public production types are immutable dataclasses:

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
```

Required public signatures:

```text
RouteDebugDataset.asset_state(key: str) -> RouteDebugAssetState
RouteDebugDataset.aisle_by_id(aisle_id: str) -> RouteDebugAisleRecord | None
load_route_debug_dataset(run_dir: str | Path, *, vehicle_profile_path: str | Path | None = None) -> RouteDebugDataset
```

- Coverage parsing is debug-specific and reads `coverage_order.yaml` fully: `traversals`, `connector_requests`, and `rejected_aisles`. Do not widen the existing `load_coverage_connector_requests()` contract just for the GUI.
- Dataset frame authority is first valid `navigation_map.yaml`, else valid `aisle_graph.yaml`, else valid `coverage_order.yaml`. Any later framed asset that disagrees is marked `INVALID` and excluded.
- NO_GO recovery reads `derivation.yaml.overrides[*]` where `mode == "no_go"`; preserve polygon geometry and source path even though the PGM cell state is OCCUPIED.
- Profile resolution: explicit `vehicle_profile_path` wins. Otherwise infer `platform_id` from valid route assets and walk parents of `run_dir` until `profiles/platforms/<platform_id>.yaml` exists. If it cannot be found, mark `vehicle_profile` MISSING and keep non-vehicle layers usable.

- [ ] **Step 1: Write the failing dataset tests and a real-file fixture builder**

In `route_debug_test_data.py`, implement:

```text
write_minimal_route_debug_run(root: Path, *, include_no_go: bool = False, coverage_frame: str = "map") -> tuple[Path, Path]
```

The helper writes real files under a temporary repo-like tree:

```text
runtime/maps/debug_run/navigation_map.pgm
runtime/maps/debug_run/navigation_map.yaml
runtime/maps/debug_run/derivation.yaml
runtime/maps/debug_run/aisle_graph.yaml
runtime/maps/debug_run/turn_zones.yaml
runtime/maps/debug_run/coverage_order.yaml
profiles/platforms/mk_mini.yaml
```

Use these exact fixture facts:

```text
frame_id = map
resolution = 0.10 m
2 structural aisles at positive world Y so Qt Y-flip can be tested later
aisle_001 width = 1.20 m
aisle_002 width = 1.20 m
both traversal motion_direction = FORWARD
sequence = 1, 2
one ConnectorRequest from aisle_001 to aisle_002
optional NO_GO polygon starts at [1.0, -0.5]
```

The profile fixture must contain `platform.name=mk_mini`, `kinematics=ackermann`, navigation footprint `[(0.42,0.30),(0.42,-0.30),(-0.42,-0.30),(-0.42,0.30)]`, and verified `minimum_turning_radius=1.5` so `load_canonical_vehicle_profile()` returns preview-ready.

Add tests:

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

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest -q src/agt_offline_assets/test/test_route_debug_dataset.py
```

Expected: collection/import failure because `agt_offline_assets.route_debug_dataset` does not exist.

- [ ] **Step 3: Implement the minimum dataset loader**

Implement these private responsibilities with exact inputs/outputs:

```text
_load_yaml(path: Path) -> Mapping[str, Any]
_frame_accepts(authoritative: str | None, candidate: str | None) -> bool
_load_debug_coverage(path: Path) -> coverage frame/status + traversal/request/rejection tuples
_load_no_go_regions(derivation_path: Path) -> tuple[RouteDebugNoGoRegion, ...]
_discover_vehicle_profile(run_dir: Path, platform_id: str) -> Path | None
```

Schema checks use the existing constants, including:

```python
NAVIGATION_DERIVATION_SCHEMA = "agt_ground_relative_navigation_map/v1"
COVERAGE_ORDER_SCHEMA = "agt_agricultural_coverage_order/v1"
```

Join `AislePrimitive` to traversal/rejection by `aisle_id`; never change geometry or eligibility. Export the public dataset API from `agt_offline_assets/__init__.py` and register `test_route_debug_dataset` in `src/agt_offline_assets/CMakeLists.txt`.

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
- Consume schemas exactly as currently frozen:

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

Add immutable records:

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
```

Extend `RouteDebugAisleRecord` with `vehicle_lane`, `diagnostic`, `occupancy_source`; extend `RouteDebugDataset` with `connectors` and:

```text
RouteDebugDataset.connector_by_id(connector_id: str) -> RouteDebugConnectorRecord | None
```

Optional asset discovery scans top-level YAML `schema` values. If multiple `agt_reverse_primitive_connector_plan/v1` assets exist, select in this deterministic priority:

```text
1. reverse_primitive_connectors_anchored.yaml
2. reverse_primitive_connectors.yaml
3. lexicographically first reverse_primitive_connectors*.yaml
```

- [ ] **Step 1: Extend the fixture builder and write RED cause-chain tests**

Implement two helper functions in `route_debug_test_data.py`:

```text
write_route_debug_planner_assets(run_dir: Path) -> None
write_route_debug_lane_assets(run_dir: Path) -> None
```

`write_route_debug_planner_assets()` rewrites the fixture coverage payload to contain requests `connector_015` and `connector_017`, then writes matching forward/audit/R6A/R6B files. Freeze these synthetic R6B facts:

```text
connector_015
  decision ELIGIBLE_REVERSE_FALLBACK
  status REVERSE_PRIMITIVE_PREVIEW_FREE
  backend BOUNDED_REVERSE_PRIMITIVE_SEARCH_NOT_ANALYTIC_REEDS_SHEPP
  three representative samples: FORWARD, REVERSE, FORWARD
  cusp_count 2

connector_017
  decision ELIGIBLE_REVERSE_FALLBACK
  status NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION
  search_expansions 67
  samples empty
```

Tests:

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
python3 -m pytest -q src/agt_offline_assets/test/test_route_debug_dataset.py
```

Expected: FAIL because the new connector/lane fields and schema discovery are absent.

- [ ] **Step 3: Implement schema-based discovery and joins**

Implement exact helper responsibilities:

```text
_discover_yaml_by_schema(run_dir: Path) -> dict[str, tuple[Path, ...]]
_choose_reverse_primitive_asset(paths: Sequence[Path]) -> Path | None
_load_forward_debug(path: Path) -> connector-id keyed frozen forward records
_load_reverse_debug(path: Path) -> connector-id keyed frozen R6 records
_load_vehicle_lane_debug(path: Path) -> aisle-id keyed lane records
_join_connector_records(requests, forward, audit, r6a, r6b) -> tuple[RouteDebugConnectorRecord, ...]
```

For every optional file: validate schema, validate `frame_id`, mark only that asset INVALID on failure, preserve source filename, and carry frozen status text even when no path samples exist.

- [ ] **Step 4: Run route-debug plus producer regressions**

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

```text
ROUTE_DEBUG_OVERLAY_SCHEMA = agt_route_debug_overlay/v1
build_route_debug_overlay(dataset: RouteDebugDataset, config: RouteDebugOverlayConfig | None = None) -> dict[str, Any]
write_route_debug_overlay(overlay: Mapping[str, Any], path: str | Path, *, overwrite: bool = False) -> Path
```

Config fields/defaults:

```python
@dataclass(frozen=True)
class RouteDebugOverlayConfig:
    forward_sample_step_m: float = 0.05
    collision_sample_spacing_m: float = 0.10
    lateral_search_step_m: float = 0.05
    maximum_lateral_shift_m: float = 0.50
    preview_footprint_padding_m: float = 0.05
```

Top-level GeoJSON must be:

```python
{
    "type": "FeatureCollection",
    "agt_schema": "agt_route_debug_overlay/v1",
    "frame_id": dataset.frame_id,
    "validation_scope": "DEBUG_RENDER_ONLY",
    "features": feature_list,
}
```

Every feature carries `feature_id`, `layer_group`, `layer_key`, `feature_kind`, `frame_id`, `status`, `source_asset`, `source_id`, `source_field`, `is_failure`, and `inspector`.

- [ ] **Step 1: Write RED structure/coverage/motion tests**

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
    motion = next(f for f in overlay["features"] if f["properties"]["feature_kind"] == "R6_MOTION_SEGMENT")
    assert request["properties"]["layer_group"] == "coverage"
    assert motion["properties"]["layer_group"] == "motion"


def test_overlay_writer_refuses_accidental_overwrite(tmp_path):
    path = tmp_path / "route_debug_overlay.geojson"
    write_route_debug_overlay({"type": "FeatureCollection", "features": []}, path)
    with pytest.raises(FileExistsError):
        write_route_debug_overlay({"type": "FeatureCollection", "features": []}, path)
```

- [ ] **Step 2: Run and verify RED**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_route_debug_overlay.py
```

Expected: import failure because `route_debug_overlay.py` does not exist.

- [ ] **Step 3: Implement base GeoJSON features and forward-candidate geometry replay**

Implement a single `_feature` constructor accepting every required property plus a GeoJSON geometry mapping. Generate:

```text
NO_GO Polygon
Aisle LineString
Vehicle-safe-lane LineString when non-empty
Turn Zone Polygon
Coverage traversal LineString with order/orientation Inspector data
ConnectorRequest LineString between frozen start/goal
Frozen selected forward samples
Frozen R6 samples split into FORWARD/REVERSE LineStrings at motion-direction changes
Cusp Point markers
Failed connector Point at request midpoint when frozen status is a failure
```

Candidate-audit records do not contain path samples. Replay candidate geometry only inside `agt_offline_assets` from the frozen ConnectorRequest + canonical Rmin using existing `_dubins_candidates()` and `_sample_candidate()`. Attach:

```text
derived_geometry_method = REPLAY_ANALYTIC_DUBINS_FROM_FROZEN_REQUEST_AND_PROFILE
```

Join audit evidence by `path_type`; do not alter frozen planner decisions.

- [ ] **Step 4: Run focused tests**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_route_debug_overlay.py \
  src/agt_offline_assets/test/test_route_debug_dataset.py \
  src/agt_offline_assets/test/test_forward_connector.py \
  src/agt_offline_assets/test/test_reverse_primitive_connector.py
```

Expected: PASS.

- [ ] **Step 5: Export API/register test and commit**

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
- Extend `RouteDebugDataset` with `occupancy_source_masks: NavigationOccupancySourceMasks | None`.
- Load masks through `load_navigation_occupancy_source_masks(navigation, navigation_asset)` only when required frozen sidecars exist and validate.
- Do not serialize every global OCCUPIED cell to GeoJSON. Emit one `COLLISION_STATION` point for every sampled aisle station with no fully FREE bounded-lateral vehicle pose, and store the selected candidate's `footprint_polygon_xy` in properties.

- [ ] **Step 1: Write RED synthetic provenance tests**

Construct a direct synthetic dataset with 0.10 m grid, 0.70 m aisle width, MK-mini 0.60 m navigation width + 0.05 m preview padding, and explicit `NavigationOccupancySourceMasks`. Place a raw direct cell immediately outside the footprint and a PADDING_ONLY neighbor inside it so the best zero-lateral pose fails specifically on padding.

```python
def test_collision_station_reports_padding_only_and_footprint_polygon():
    dataset = synthetic_padding_collision_dataset()
    overlay = build_route_debug_overlay(dataset)
    conflicts = [f for f in overlay["features"] if f["properties"]["feature_kind"] == "COLLISION_STATION"]
    assert conflicts
    first = conflicts[0]["properties"]
    assert first["dominant_source"] == "PADDING_ONLY"
    assert first["occupied_count"] > 0
    assert len(first["footprint_polygon_xy"]) >= 4
    assert first["source_asset"] == "navigation_map.yaml"


def test_collision_station_keeps_unknown_distinct_from_occupied():
    dataset = synthetic_unknown_collision_dataset()
    overlay = build_route_debug_overlay(dataset)
    first = next(f for f in overlay["features"] if f["properties"]["feature_kind"] == "COLLISION_STATION")
    assert first["properties"]["dominant_source"] == "UNKNOWN"
    assert first["properties"]["unknown_count"] > 0
```

- [ ] **Step 2: Run and verify RED**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_route_debug_overlay.py -k collision_station
```

Expected: FAIL because collision station features are absent.

- [ ] **Step 3: Implement collision station generation using existing offline helpers**

Inside `route_debug_overlay.py`, reuse:

```python
from .forward_connector_navigation_gate import _preview_local_footprint, _transform_polygon
from .vehicle_safe_lane import _normalize, _offset_candidates, _resample_polyline
from .vehicle_safe_lane_occupancy_sources import _pose_mask, _candidate_key, _fully_free
```

Per aisle:

```text
resample structural centerline at collision_sample_spacing_m
→ compute allowed lateral shift from structural width and preview vehicle width
→ evaluate all bounded offsets
→ if any full pose is FREE, emit no collision marker
→ otherwise choose the candidate with existing least-total-non-FREE ordering
→ classify selected footprint cells as RAW_OBSTACLE_DIRECT / GEOMETRY_DIRECT /
  PADDING_ONLY / UNEXPLAINED_OCCUPIED / UNKNOWN / OUT_OF_GRID
→ emit collision Point + footprint_polygon_xy + source counts
```

Do not relax occupancy, do not turn UNKNOWN into FREE, and do not write any modified map.

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

### Task 5: Shared-Scene Renderer and Layer Presets

**Files:**
- Create: `src/agt_map_workbench/agt_map_workbench/route_debug_view.py`
- Create: `src/agt_map_workbench/test/conftest.py`
- Create: `src/agt_map_workbench/test/route_debug_test_data.py`
- Create: `src/agt_map_workbench/test/test_route_debug_view.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/__init__.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**

```text
RouteDebugSceneController(scene: QGraphicsScene, parent: QObject | None = None)
.set_content(dataset: RouteDebugDataset, overlay: Mapping[str, Any]) -> None
.clear() -> None
.set_active(active: bool) -> None
.is_active() -> bool
.set_layer_visible(layer_key: str, visible: bool) -> None
.layer_visible(layer_key: str) -> bool
.apply_preset(preset_name: str) -> None
.set_failure_focus(enabled: bool) -> None
.route_bounds() -> QRectF
.select_feature(feature_id: str) -> bool
featureSelected = pyqtSignal(object)
```

Layer keys:

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

- [ ] **Step 1: Add an explicit headless QApplication fixture and Workbench-local test data**

In `src/agt_map_workbench/test/conftest.py`:

```python
import pytest
from PyQt5.QtWidgets import QApplication


@pytest.fixture(scope="session")
def qapp():
    app = QApplication.instance()
    if app is None:
        app = QApplication([])
    yield app
```

Do not add `pytest-qt`.

In `src/agt_map_workbench/test/route_debug_test_data.py`, create two helpers owned by this package's tests:

```text
synthetic_route_debug_content() -> tuple[RouteDebugDataset, dict]
write_workbench_route_debug_run(root: Path, *, include_no_go: bool = False, include_reverse: bool = False) -> Path
```

`synthetic_route_debug_content()` must place `aisle_001` at positive world Y and include an `aisle:aisle_001` GeoJSON LineString. When `include_reverse=True`, the disk fixture includes `connector_015` F/R/F evidence.

- [ ] **Step 2: Write RED controller tests**

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
    assert item.sceneBoundingRect().center().y() < 0.0


def test_route_debug_controller_emits_inspector_payload(qapp):
    scene = QGraphicsScene()
    controller = RouteDebugSceneController(scene)
    dataset, overlay = synthetic_route_debug_content()
    selected = []
    controller.featureSelected.connect(selected.append)
    controller.set_content(dataset, overlay)
    assert controller.select_feature("aisle:aisle_001")
    assert selected[-1]["source_id"] == "aisle_001"
```

- [ ] **Step 3: Run and verify RED**

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q src/agt_map_workbench/test/test_route_debug_view.py
```

Expected: import failure because `route_debug_view.py` does not exist.

- [ ] **Step 4: Implement renderer without planner math**

Use exactly:

```python
def _scene_xy(x: float, y: float) -> QPointF:
    return QPointF(float(x), float(-y))
```

Rendering rules:
- Navigation raster uses the same flip/placement convention as existing `NavigationPreviewItem`.
- Raw/geometry/padding source masks are translucent raster pixmaps, not thousands of GeoJSON cells.
- GeoJSON vectors go into per-layer `QGraphicsItemGroup`s.
- Selectable items store `feature_id` in `item.data(0)` and Inspector mapping in `item.data(1)`.
- Selecting a `COLLISION_STATION` draws a temporary footprint outline from stored `footprint_polygon_xy`; no collision recomputation.
- Failure focus only adjusts opacity.
- `clear()` removes only Route Debug-owned items, never the shared scene wholesale.

Export `RouteDebugSceneController` from `agt_map_workbench/__init__.py` and register `test_route_debug_view` in CMake.

- [ ] **Step 5: Run focused Qt regressions and commit**

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/agt_map_workbench/test/test_route_debug_view.py \
  src/agt_map_workbench/test/test_workbench_model.py \
  src/agt_map_workbench/test/test_frame_calibration.py
```

Expected: PASS.

```bash
git add \
  src/agt_map_workbench/agt_map_workbench/route_debug_view.py \
  src/agt_map_workbench/agt_map_workbench/__init__.py \
  src/agt_map_workbench/test/conftest.py \
  src/agt_map_workbench/test/route_debug_test_data.py \
  src/agt_map_workbench/test/test_route_debug_view.py \
  src/agt_map_workbench/CMakeLists.txt
git commit -m "feat(route-debug): render layered route debug scene"
```

---

### Task 6: Route Debug Panel, Inspector, Presets, and PNG Export

**Files:**
- Create: `src/agt_map_workbench/agt_map_workbench/route_debug_panel.py`
- Create: `src/agt_map_workbench/test/test_route_debug_panel.py`
- Modify: `src/agt_map_workbench/agt_map_workbench/__init__.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**

```text
RouteDebugPanel(scene: QGraphicsScene, view: QGraphicsView, parent=None)
.load_run_directory(path: str | Path) -> RouteDebugDataset
.reload_current_directory() -> RouteDebugDataset | None
.set_active(active: bool) -> None
.apply_preset(preset_name: str) -> None
.set_failure_focus(enabled: bool) -> None
.export_current_view(path: str | Path) -> bool
.current_run_directory() -> Path | None
.layer_enabled(layer_key: str) -> bool
.inspector_text() -> str
.controller: RouteDebugSceneController
```

The panel loads synchronously for MVP; do not add a worker thread without measured need.

- [ ] **Step 1: Write RED panel behavior tests using the Workbench-local disk fixture**

```python
def test_panel_loads_directory_writes_only_debug_overlay_and_populates_inspector(qapp, tmp_path):
    run_dir = write_workbench_route_debug_run(tmp_path, include_no_go=True, include_reverse=True)
    scene = QGraphicsScene()
    view = QGraphicsView(scene)
    panel = RouteDebugPanel(scene, view)

    dataset = panel.load_run_directory(run_dir)
    assert dataset.run_dir == run_dir.resolve()
    assert (run_dir / "route_debug_overlay.geojson").is_file()
    assert panel.current_run_directory() == run_dir.resolve()
    assert panel.controller.select_feature("aisle:aisle_001")
    assert "aisle_001" in panel.inspector_text()
    assert "STRUCTURE" in panel.inspector_text()


def test_panel_disables_missing_optional_layers_without_blocking_coverage(qapp, tmp_path):
    run_dir = write_workbench_route_debug_run(tmp_path)
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
QT_QPA_PLATFORM=offscreen python3 -m pytest -q src/agt_map_workbench/test/test_route_debug_panel.py
```

Expected: import failure because `route_debug_panel.py` does not exist.

- [ ] **Step 3: Implement the panel**

Approved controls:

```text
[选择运行目录] [重载] [适配全图]
[Coverage 总览] [规划结果] [碰撞诊断]
[仅失败] [导出 PNG]
Layers tree
Inspector
```

Use a two-column `QTreeWidget` for layer availability/check state and a read-only `QTextBrowser` or two-column `QTreeWidget` for Inspector. A layer is checkable only when its source is LOADED.

`load_run_directory()` executes this data path:

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

The panel may replace only `route_debug_overlay.geojson`.

Inspector sections:

```text
STRUCTURE
COVERAGE
VEHICLE
NAVIGATION / VEHICLE FEASIBILITY
CONFLICT
RELATED
SOURCE
```

Connector Inspector cause-chain order:

```text
Coverage Request
Forward
R5.6
R6A
R6B
```

Use frozen fields only and do not invent R7 state. `export_current_view(path)` calls `self._view.grab().save(str(path), "PNG")` and returns that boolean result.

- [ ] **Step 4: Run panel/view tests and commit**

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/agt_map_workbench/test/test_route_debug_panel.py \
  src/agt_map_workbench/test/test_route_debug_view.py
```

Expected: PASS.

```bash
git add \
  src/agt_map_workbench/agt_map_workbench/route_debug_panel.py \
  src/agt_map_workbench/agt_map_workbench/__init__.py \
  src/agt_map_workbench/test/test_route_debug_panel.py \
  src/agt_map_workbench/CMakeLists.txt
git commit -m "feat(route-debug): add route debug controls and inspector"
```

---

### Task 7: Mount 路径调试 into the Existing Workbench

**Files:**
- Modify: `src/agt_map_workbench/agt_map_workbench/app.py` around the right-side controls `QTabWidget`
- Modify: `src/agt_map_workbench/agt_map_workbench/review_workbench.py` in `ReviewMapWorkbenchWindow.__init__`
- Create: `src/agt_map_workbench/test/test_route_debug_workbench_integration.py`
- Modify: `src/agt_map_workbench/CMakeLists.txt`

**Interfaces:**
- Expose the current right control tabs as `self._control_tabs`.
- Add one tab named exactly `路径调试` containing `RouteDebugPanel`.
- Store `self._route_debug_panel` and `self._route_debug_tab_index`.

- [ ] **Step 1: Write RED integration tests**

```python
def test_review_workbench_contains_route_debug_control_tab(qapp):
    window = ReviewMapWorkbenchWindow()
    labels = [window._control_tabs.tabText(i) for i in range(window._control_tabs.count())]
    assert labels == ["点云编辑", "坐标系标定", "导航地图", "路径调试"]
    assert isinstance(window._route_debug_panel, RouteDebugPanel)


def test_route_debug_activation_does_not_leave_overlay_on_authoring_tabs(qapp):
    window = ReviewMapWorkbenchWindow()
    route_index = window._route_debug_tab_index
    window._control_tabs.setCurrentIndex(route_index)
    assert window._route_debug_panel.controller.is_active()
    window._control_tabs.setCurrentIndex(0)
    assert not window._route_debug_panel.controller.is_active()
```

- [ ] **Step 2: Run and verify RED**

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q src/agt_map_workbench/test/test_route_debug_workbench_integration.py
```

Expected: FAIL because the existing control `QTabWidget` is local and no Route Debug panel is mounted.

- [ ] **Step 3: Make the smallest integration changes**

In `app.py` change only ownership of the existing control tabs:

```python
self._control_tabs = QTabWidget()
self._control_tabs.addTab(self._build_edit_tab(), "点云编辑")
self._control_tabs.addTab(self._build_frame_tab(), "坐标系标定")
self._control_tabs.addTab(self._build_navigation_tab(), "导航地图")
layout.addWidget(self._control_tabs, 1)
```

In `ReviewMapWorkbenchWindow.__init__`, call `self._install_route_debug()` after current 3D/offline-action installation. Implement:

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

This preserves the real launcher lineage through `review_workbench.main` and one shared 2D scene/view.

- [ ] **Step 4: Run all Workbench tests and commit**

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q src/agt_map_workbench/test
```

Expected: PASS.

```bash
git add \
  src/agt_map_workbench/agt_map_workbench/app.py \
  src/agt_map_workbench/agt_map_workbench/review_workbench.py \
  src/agt_map_workbench/test/test_route_debug_workbench_integration.py \
  src/agt_map_workbench/CMakeLists.txt
git commit -m "feat(route-debug): mount route debug in map workbench"
```

---

### Task 8: Freeze Contracts, Documentation, and Local-Acceptance Gate

**Files:**
- Create: `tests/test_v25_12e_route_debug_contract.py`
- Modify: `src/agt_map_workbench/README.md`
- Modify: `docs/v2.5/V25_12E_CURRENT_STATE.md`

**Interfaces:**
- No new runtime interface. This task locks architecture and leaves real-data visual acceptance pending.

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

- [ ] **Step 2: Run the contract test and verify RED**

```bash
python3 -m pytest -q tests/test_v25_12e_route_debug_contract.py
```

Expected: FAIL until README/current-state text is updated.

- [ ] **Step 3: Update README/current-state without overclaiming**

README Route Debug section must state:

```text
路径调试 is 2D and read-only
load a run directory rather than individual YAML files
Coverage 总览 / 规划结果 / 碰撞诊断 presets
NO_GO is visually separate from physical OCCUPIED
route_debug_overlay.geojson is derived render evidence only
missing optional asset degrades; invalid contract fails closed for that layer
```

Current state must say exactly:

```text
Route Debug 2D
  CORE IMPLEMENTED / LOCAL ACCEPTANCE PENDING

No production map/route semantics changed
```

Keep the existing padding/direct-source diagnostic conclusion intact. Do not claim greenhouse visual acceptance.

- [ ] **Step 4: Run the complete focused matrix**

```bash
source /opt/ros/humble/setup.bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
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

Then:

```bash
colcon build --symlink-install --packages-select agt_offline_assets agt_map_workbench
source install/setup.bash
QT_QPA_PLATFORM=offscreen colcon test \
  --packages-select agt_offline_assets agt_map_workbench \
  --event-handlers console_direct+
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

After the implementation commits are present on the operator machine:

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

For the same frozen greenhouse asset version previously tested, expect structurally:

```text
19 structural aisles
18 width accepted / 1 width rejected (aisle_005)
17 ConnectorRequests
```

Treat a mismatch as an asset-version change to inspect; never hard-code these counts in the loader.

- [ ] **Step 7: Operator visual smoke in AGT Map Workbench**

Launch:

```bash
ros2 run agt_map_workbench agt_map_workbench
```

Load:

```text
~/agt_navigation_v2/runtime/maps/agt_workbench_run
```

Review exactly:

```text
Coverage 总览
  whole greenhouse order visible
  aisle arrows do not imply reverse gear
  ConnectorRequest visually differs from solved motion

 aisle_005
  structural width 0.658 m vs required preview width 0.700 m
  visibly rejected

 aisle_003
  RAW obstacle evidence + structural aisle + footprint conflict visible together

 aisle_013
  PADDING_ONLY evidence + centerline + footprint conflict visible together

 connector_015
  FORWARD → REVERSE → FORWARD
  2 cusp markers
  Inspector shows frozen 4.499 m total / 0.600 m reverse / 1490 expansions /
  0.154 m / 8.11° goal error when that frozen R6B asset is loaded

 connector_017
  request exists
  R6A admitted
  R6B searched
  NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION
  visibly distinct from a missing asset

 NO_GO
  if production run has none, make a separate test derivation in the existing Navigation Map authoring page
  semantic polygon renders separately from physical OCCUPIED
```

Do not mark `REAL-DATA OPERATOR ACCEPTANCE PASS` until the operator actually completes this checklist.

---

## Final Verification Gate Before Claiming Completion

Run in order:

```bash
git status --short
source /opt/ros/humble/setup.bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/agt_offline_assets/test/test_route_debug_dataset.py \
  src/agt_offline_assets/test/test_route_debug_overlay.py \
  src/agt_map_workbench/test/test_route_debug_view.py \
  src/agt_map_workbench/test/test_route_debug_panel.py \
  src/agt_map_workbench/test/test_route_debug_workbench_integration.py \
  tests/test_v25_12e_route_debug_contract.py
colcon build --symlink-install --packages-select agt_offline_assets agt_map_workbench
source install/setup.bash
QT_QPA_PLATFORM=offscreen colcon test --packages-select agt_offline_assets agt_map_workbench
colcon test-result --verbose
git status --short
```

Until the operator returns the final visual checklist, completion wording is limited to:

```text
Route Debug 2D CORE IMPLEMENTED / AUTOMATED TESTS PASS
REAL-DATA OPERATOR VISUAL ACCEPTANCE PENDING
```
