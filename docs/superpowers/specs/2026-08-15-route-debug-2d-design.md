# AGT Map Workbench — 2D Route Debug Design

Date: 2026-08-15
Status: DESIGN APPROVED IN CHAT / IMPLEMENTATION NOT STARTED
Scope: V25-12E route-production explainability and operator review

## 1. Purpose

Add a fourth tab to the existing AGT Map Workbench:

```text
AGT Map Workbench
├─ 点云编辑
├─ 坐标系标定
├─ 导航地图
└─ 路径调试
```

The first version is a pure 2D overview. It does not create a standalone Route Viewer and does not add 3D linkage yet

The page exists to answer one question clearly:

> Why is a route segment accepted, rejected, blocked, or unsolved at this location?

It must make road extraction, coverage order, vehicle feasibility, connector planning, semantic NO_GO, and collision provenance visible in one map instead of forcing the operator to infer state from multiple YAML files and terminal summaries

This page is an explainability and debugging view, not a planner and not an alternative route truth owner

## 2. Current route-production semantics preserved

Current greenhouse coverage is not a direct Fields2Cover implementation

The current route flow is:

```text
real greenhouse PCD
↓
Row / Aisle extraction
↓
Agricultural Aisle Graph
↓
vehicle-width filtering
↓
deterministic Boustrophedon / snake ordering
↓
ConnectorRequest
↓
forward Dubins evidence
↓
reverse-aware local fallback
↓
future R7 search fallback
```

The Route Debug page should provide a Fields2Cover-like whole-field coverage presentation, but the displayed swaths are the real detected aisles from the Aisle Graph rather than ideal swaths regenerated from a field polygon

The page must preserve the distinction:

```text
Aisle Graph
= where structural roads exist

Coverage Ordering
= which accepted aisles to visit and in what order

ConnectorRequest
= which pair of aisle endpoints needs a connection

Connector Planner
= how that connection is actually driven
```

`WITH_ROW_DIRECTION` and `AGAINST_ROW_DIRECTION` remain traversal orientation only. Both are normal FORWARD aisle traversal. True `motion_direction=REVERSE` may only come from connector planning

## 3. High-level architecture

The design uses four bounded components:

```text
agt_offline_assets/
  route_debug_dataset.py
  route_debug_overlay.py

agt_map_workbench/
  route_debug_view.py
  route_debug_panel.py
```

The architectural boundary is:

```text
agt_offline_assets
= parse frozen assets, validate contracts, build explainable overlay evidence

agt_map_workbench
= render, toggle layers, select objects, inspect provenance, export screenshots
```

The GUI must not duplicate planner math

Data flow:

```text
runtime/maps/<run_dir>/
        ↓
RouteDebugDataset
        ↓
Route Debug Overlay Builder
        ↓
render-only geometry + provenance
        ↓
AGT Map Workbench / 路径调试
```

The debug overlay is derived evidence only. It does not replace Navigation Map, Aisle Graph, Coverage Order, Connector plans, or final Route Asset as truth owners

## 4. Main UI layout

The page uses one full 2D map with a layer tree and an object inspector

```text
┌────────────────────────────────────────────────────────────┐
│ 路径调试                                                   │
│ [运行目录] [重载] [适配全图] [Coverage] [仅失败]           │
│ [碰撞来源] [车辆净空] [导出 PNG]                           │
├──────────────────────────────────────┬─────────────────────┤
│                                      │ Layers              │
│                                      │ ▼ Base              │
│                                      │ ▼ Structure         │
│              2D 总览图               │ ▼ Coverage          │
│                                      │ ▼ Motion            │
│                                      │ ▼ Diagnostics       │
│                                      ├─────────────────────┤
│                                      │ Inspector           │
│                                      │ aisle / connector   │
│                                      │ status / reason     │
└──────────────────────────────────────┴─────────────────────┘
```

The page reuses the existing Workbench map coordinate system and scene/view infrastructure

## 5. Layer groups

### 5.1 Base

```text
Navigation FREE / OCCUPIED / UNKNOWN
NO_GO semantic polygons
optional PCD top-view sample if available in the active Workbench session
```

Recommended visual treatment:

```text
FREE       light neutral fill
UNKNOWN    gray hatched / textured fill
OCCUPIED   dark neutral fill
NO_GO      purple hatch + border
```

NO_GO must never look identical to physical OCCUPIED

### 5.2 Agricultural Structure

```text
Row Structural Band                 optional when frozen geometry exists
Vegetation / Row envelope           optional when frozen geometry exists
Aisle Graph centerline              required when aisle_graph.yaml exists
Aisle ID labels
Structural width annotation
Accepted / rejected state
Boundary aisle identity
```

Visual semantics:

```text
Structural aisle centerline   thin cyan-like line
Vehicle-safe lane             thicker solid line
Rejected aisle                red/gray dashed line
Boundary aisle                solid lane + boundary endpoint marker
```

A structural aisle is not automatically a vehicle-safe lane

### 5.3 Coverage

```text
Coverage visit order
Aisle traversal direction arrows
Vehicle-width accepted/rejected state
ConnectorRequest links
```

Coverage order should display large sequence labels such as:

```text
① aisle_001 ↓
② aisle_002 ↑
③ aisle_003 ↓
```

The direction arrows represent Aisle Graph traversal orientation, not reverse driving

ConnectorRequest should use a visually lighter dashed link because it expresses requested connectivity, not a solved vehicle trajectory

### 5.4 Planned Motion

```text
Forward Dubins candidates
selected / representative forward candidate
R6 bounded F/R/F solution
FORWARD / REVERSE segment distinction
cusp / direction-change poses
future R7 solved trajectory
```

Visual semantics:

```text
forward selected segment        thick solid line
reverse segment                 thick dashed line
cusp                            diamond marker
failed / non-selected candidate thin low-opacity line
selected connector              highest route visual priority
```

The current reverse backend must be shown by its exact implemented name when inspected:

```text
BOUNDED_REVERSE_PRIMITIVE_SEARCH_NOT_ANALYTIC_REEDS_SHEPP
```

The UI must not relabel this as analytic Reeds-Shepp

### 5.5 Diagnostics

```text
vehicle footprint outline
swept footprint when frozen evidence exists
collision cells / conflict regions
RAW_OBSTACLE_DIRECT
GEOMETRY_DIRECT
PADDING_ONLY
UNKNOWN
NO_GO conflict
invalid / unsolved connector markers
```

Recommended conflict symbols:

```text
RAW_OBSTACLE_DIRECT   red ×
GEOMETRY_DIRECT       yellow triangle
PADDING_ONLY          orange translucent grid/fill
UNKNOWN               gray ? / hatch
NO_GO                 purple semantic conflict marker
```

Conflict rendering should highlight the cells or regions that cause the block rather than painting the entire vehicle footprint red

## 6. Preset views

The layer system remains fully user-controllable, but the toolbar provides three presets

### 6.1 Coverage 总览

Shows:

```text
Navigation Map
NO_GO
Aisles
Coverage order
Traversal arrows
ConnectorRequest
```

This is the Fields2Cover-like whole-greenhouse presentation

### 6.2 规划结果

Shows:

```text
Aisles
ConnectorRequest
Forward candidates
selected forward path
R6 F/R/F path
forward/reverse segment markers
cusps
failed connector state
```

### 6.3 碰撞诊断

Dims normal content and emphasizes:

```text
vehicle footprint
RAW obstacle
geometry evidence
padding-only conflict
UNKNOWN
NO_GO
failed aisle
failed connector
```

A separate `仅失败` control may reuse the same visibility model and reduce opacity of accepted content

## 7. RouteDebugDataset loading contract

The user selects a run directory, for example:

```text
runtime/maps/agt_workbench_run/
```

The loader discovers supported frozen assets rather than asking the user to select each YAML separately

Expected current inputs include when available:

```text
navigation_map.yaml / navigation_map.pgm
derivation.yaml

aisle_graph.yaml
turn_zones.yaml
coverage_order.yaml

forward connector plans / diagnostics
forward_connector_candidate_audit.yaml

reverse_fallback_admission.yaml
reverse_primitive_connectors*.yaml

vehicle_safe_aisles.yaml
connector_anchors.yaml

vehicle_safe_lane_diagnostics.yaml
vehicle_safe_lane_occupancy_sources.yaml
vehicle_safe_lane_padding_sensitivity.yaml
```

The in-memory dataset is read-only and grouped by meaning:

```text
RouteDebugDataset
├─ navigation
├─ semantics
├─ agricultural_structure
├─ coverage
├─ forward_planning
├─ reverse_planning
├─ vehicle_lane
├─ diagnostics
└─ source_registry
```

Every loaded entity should retain source provenance where available:

```text
source path
schema
status
frame_id
asset identity / hash when present
```

## 8. Missing assets and contract failures

Missing optional assets use graceful degradation

Example:

```text
navigation_map.yaml + aisle_graph.yaml + coverage_order.yaml exist
reverse_primitive_connectors.yaml missing
```

The page still opens and displays the available coverage information. The Reverse Motion layer is disabled and reports why it is unavailable

An existing asset with invalid schema, incompatible frame, or other contract error must fail closed for that layer

```text
missing file
→ unavailable layer

existing but invalid file
→ explicit contract error, layer not rendered
```

No silent coordinate conversion or schema guessing is allowed

## 9. Selection and inspector behavior

### 9.1 Selecting an aisle

Selecting an aisle highlights:

```text
selected aisle
adjacent structural rows/bands when available
vehicle-safe lane evidence
incoming connector
outgoing connector
associated conflict evidence
```

The inspector should expose groups such as:

```text
STRUCTURE
aisle_id
kind
structural width
adjacent structures

COVERAGE
visit order
traversal orientation
accepted/rejected

VEHICLE
platform
required width

NAVIGATION / VEHICLE FEASIBILITY
reference FREE fraction
any-lateral vehicle-pose FREE fraction

CONFLICT
dominant source
raw / geometry / padding / unknown evidence

RELATED
incoming connector
outgoing connector
```

### 9.2 Selecting a connector

The inspector should present the connector cause chain rather than only the final status:

```text
Coverage Request
↓
Forward planning result
↓
R5.6 audit result
↓
R6A admission decision
↓
R6B result
↓
future R7 result
```

For a solved R6 connector, show motion pattern, cusp count, path length, reverse length, search expansions, goal error, and validation scope when those fields exist

### 9.3 Selecting a collision / conflict

The inspector should identify:

```text
world x / y
Navigation state
conflict source
related aisle
related connector
source asset
source feature / field
```

The goal is that an operator can click a red/orange/purple conflict and answer `why is this blocked?`

## 10. Route Debug Overlay

The GUI should not recompute full planning or collision logic

A dedicated derived overlay is recommended:

```text
route_debug_overlay.geojson
or equivalent serialized overlay representation
```

The exact serialization can be finalized during implementation planning, but the semantic contract is fixed

Overlay features may include:

```text
Aisle geometry
Coverage arrows / labels
ConnectorRequest geometry
Forward candidate geometry
Reverse solution geometry
cusp markers
vehicle footprint polygons
swept footprint polygons
conflict cells / polygons
NO_GO polygons
```

Each derived feature must point back to its frozen source, for example:

```text
source_asset: forward_connector_candidate_audit.yaml
source_id: connector_015
source_field: preview_footprint_evidence
```

The overlay is render-only derived evidence and cannot become a new route truth owner

## 11. NO_GO semantic contract

Current Workbench Navigation overrides support:

```text
force_free
force_occupied
unknown
no_go
```

Current Navigation Map rasterization maps both `force_occupied` and `no_go` to OCCUPIED. Therefore a PGM alone cannot preserve the semantic distinction

The Route Debug page must reconstruct NO_GO polygons from frozen override metadata such as `derivation.yaml.overrides`

Long-term route semantics must distinguish:

```text
Physical Navigation Evidence
FREE / OCCUPIED / UNKNOWN

AND

Semantic Exclusion
NO_GO
```

Conceptual drive permission:

```text
drivable = physical FREE AND not semantic NO_GO
```

Aisle Graph may still preserve a structural aisle that crosses a NO_GO area because Aisle Graph answers whether a structural road exists

Coverage and motion-planning stages must be able to reject such an aisle or connector semantically without pretending the road itself does not exist

Future statuses should preserve the reason, for example:

```text
STRUCTURALLY_VALID
SEMANTICALLY_BLOCKED_BY_NO_GO
```

and connector conflict such as:

```text
SEMANTIC_NO_GO_CONFLICT
```

R6 / R7 must never use stronger search to bypass semantic exclusion

The first Route Debug MVP only visualizes and inspects the frozen NO_GO semantics. It does not implement a new NO_GO authoring workflow in the Route Debug tab

## 12. Explicit MVP exclusions

The first version does not:

```text
edit aisle geometry
move connector endpoints
modify Navigation Map cells
force collision cells FREE
rerun Dubins inside the GUI
rerun R6 inside the GUI
implement R7 Smac search
change map padding
promote a Route Asset to READY
add 3D route linkage
```

The Route Debug tab is read-only for route-production evidence

If an operator discovers a map authoring issue, they return to the appropriate existing authoring page, regenerate the frozen asset, and reload the Route Debug dataset

## 13. MVP acceptance cases

### 13.1 Whole greenhouse coverage overview

The current greenhouse dataset must show:

```text
19 structural aisles
18 vehicle-width accepted
1 width rejected
17 ConnectorRequests
```

Coverage order labels and traversal arrows must be visible at whole-map scale

### 13.2 aisle_005 width rejection

The UI must clearly show:

```text
aisle_005
structural width 0.658 m
required preview width 0.700 m
REJECTED
```

It must not be promoted just because local Navigation Map cells appear free

### 13.3 aisle_003 direct-obstacle diagnostic probe

The map must be able to show the structural aisle, vehicle footprint conflict, and RAW obstacle evidence together

### 13.4 aisle_013 padding diagnostic probe

The map must be able to show structural centerline, complete vehicle footprint, and PADDING_ONLY conflict together

### 13.5 connector_015 solved reverse regression

The UI must visualize:

```text
FORWARD → REVERSE → FORWARD
2 cusps
```

and inspect the frozen known result:

```text
4.499 m total
0.600 m reverse
1490 expansions
0.154 m / 8.11 deg goal error
```

This connector is the positive R6 visual regression sample

### 13.6 connector_017 searched-but-unsolved regression

The UI must distinguish:

```text
ConnectorRequest exists
R6A admitted
R6B searched
NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION
```

from a connector that simply has no generated asset

### 13.7 NO_GO semantic recovery

A NO_GO region created in the existing Navigation Map authoring flow may rasterize to OCCUPIED in the PGM, but Route Debug must additionally display the preserved semantic polygon as NO_GO and inspect it as semantic exclusion rather than physical obstacle

## 14. Testing strategy

Most correctness tests belong in pure Python rather than fragile GUI automation

### 14.1 agt_offline_assets tests

Test:

```text
RouteDebugDataset loading
schema validation
frame consistency
optional asset graceful degradation
NO_GO semantic recovery
aisle ↔ connector linkage
collision-source linkage
overlay feature generation
source provenance retention
```

### 14.2 agt_map_workbench tests

Keep GUI tests focused on display behavior:

```text
layer → graphics-item mapping
layer visibility toggle
preset visibility states
selection dispatch
Inspector model content
```

Avoid extensive tests that depend on literal screen pixel coordinates or scripted mouse gestures

Target testing split:

```text
~90% data / semantic correctness in pure Python
~10% Qt rendering / interaction behavior
```

## 15. Implementation boundaries

The implementation should prefer focused modules instead of adding the Route Debug logic directly to the already-large `app.py`

The main window should only instantiate and mount the new panel/tab, pass scene or map-view dependencies through a small interface, and react to high-level selection or status signals

The Route Debug implementation must remain compatible with the current Workbench principle:

```text
full-resolution / frozen processing evidence
!= display sampling / display-only visualization
```

## 16. Definition of success

The MVP is successful when an operator can load the current greenhouse run directory, see the entire coverage route at once, click a problematic aisle or connector, and determine whether the dominant issue belongs to:

```text
road extraction
vehicle-width gate
Navigation Map physical evidence
padding / clearance semantics
semantic NO_GO
forward connector planning
reverse connector planning
missing / invalid upstream evidence
```

without having to manually correlate several terminal outputs and YAML files

The page should therefore be understood as:

> an explainable offline agricultural route-production debugger

not as a second planner or a second route editor
