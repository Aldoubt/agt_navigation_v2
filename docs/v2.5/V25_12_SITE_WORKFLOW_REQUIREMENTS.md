# V25-12 Site Workflow & BT Architecture Requirements Baseline

Status: FROZEN REQUIREMENTS BASELINE

Date: 2026-08-11

## 1. Goal

V25-12 freezes the target workflow and architecture before new runtime features are added

The project shall evolve from a navigation repository into a reusable agricultural navigation development and deployment platform with two properties

1. A new site can be prepared offline through a repeatable asset-production and benchmark workflow without changing runtime navigation code
2. Runtime navigation is exposed upward as project-level capabilities orchestrated by BehaviorTree.CPP, while important algorithms and backends remain replaceable behind stable project interfaces

This baseline does not claim that all target modules are already implemented

## 2. Frozen end-to-end workflow

```text
Site acquisition
    ↓
Raw dataset / bag / scan import
    ↓
Map construction
    ↓
Point-cloud cleaning and editing
    ↓
Localization prior authoring
    ↓
Navigation map derivation
    ↓
Semantic task annotation
    ↓
Offline route planning and vehicle feasibility preview
    ↓
Benchmark dataset and truth binding
    ↓
Offline localization / degradation / system evaluation
    ↓
READY Site Package export
    ↓
Vehicle runtime loading
    ↓
Mission / BT execution
    ↓
Runtime bag + diagnostics + experiment report
    ↓
Long-duration and GNSS/global-navigation validation
```

The workflow is intentionally split into an Offline Plane and Runtime Plane

Runtime shall not modify a READY site asset in place to make a task continue

## 3. Site Package as the deployment unit

A site shall be versioned as one auditable package rather than a loose PCD file

Recommended layout

```text
sites/<site_id>/
├── manifest.yaml
├── calibration/
├── raw/
├── maps/
│   ├── cleaned_map.pcd
│   ├── localization_map.pcd
│   ├── navigation_map.pgm
│   └── navigation_map.yaml
├── localization/
│   ├── prior.yaml
│   ├── stable_regions.yaml
│   └── degeneracy_regions.yaml
├── semantic/
│   └── semantic_map.yaml
├── routes/
│   └── *.yaml
├── benchmark/
│   ├── dataset metadata
│   ├── truth binding
│   └── reports
└── vehicle_package/
```

`manifest.yaml` must bind IDs, versions, hashes, coordinate-frame identity, calibration version, vehicle/sensor profile, map products, semantic map, route revisions and benchmark references

Official downstream assets must be selected by explicit ID/version/hash rather than by latest file in a directory

## 4. Map products remain separated

The following products are separate contracts

### 4.1 Localization Map

Purpose: global localization / relocalization / map matching

Expected properties

- 3D point cloud or tiled submaps
- cleaned of temporary objects and acquisition artifacts
- retains stable structures and discriminative geometry
- may preserve higher geometric resolution than the navigation map

### 4.2 Localization Prior Layer

Purpose: provide explicit site knowledge about localization reliability

The first schema shall support spatial regions or voxels with coarse semantic weight classes

```text
EXCLUDE
LOW
NORMAL
HIGH
LANDMARK
```

The prior is metadata consumed by localization algorithms or relocalization backends; merely changing PCD intensity must not be treated as algorithmic weighting unless the selected backend explicitly consumes it

The final effective measurement confidence should combine static prior, current geometric observability and current match quality

### 4.3 Navigation Map

Purpose: traversability and global/local planning

Examples include 2D occupancy, ground/traversability products and optional local ESDF products

### 4.4 Semantic Map

Purpose: persistent agricultural/task knowledge

Examples include rows, corridors, work zones, no-go zones, entrances, parking, charging, inspection points and task anchors

### 4.5 Route Asset

Purpose: validated geometric navigation intent produced from semantic task intent, map data, route policy and vehicle profile

A Route is not a Mission and is not a controller path

## 5. AGT Map Workbench target

A dedicated visual tool is a target capability, not a prerequisite for the first CLI pipeline

MVP functions

1. 3D crop/select/delete of irrelevant or temporary point-cloud regions
2. voxel/downsample/outlier/height/ground processing tools
3. localization-prior brush or region authoring with visible weight categories
4. semantic annotation editing
5. offline route preview with footprint and kinematic feasibility evidence
6. deterministic export of a new immutable site/map revision

The editor must not silently overwrite READY assets

## 6. Benchmark workflow

Each target site shall support reproducible benchmark datasets

Benchmark capture should bind, when available

- LiDAR
- IMU
- camera
- wheel/chassis state
- canonical odometry
- canonical localization status
- TF / TF static
- Safety and SensorHealth diagnostics
- GNSS/RTK truth or other independent truth source
- CPU/RSS/runtime diagnostics

Offline evaluation targets

### Localization / odometry

- ATE / RPE or equivalent trajectory error
- drift per distance/time
- relocalization success rate and latency
- false relocalization / jump evidence
- localization confidence and correction history

### Degradation

- effective correspondence / feature evidence
- geometry observability metrics exposed by the selected estimator
- covariance / innovation / information-condition diagnostics where available
- explicit degraded-state intervals

### Runtime stability

- CPU
- RSS memory
- map/submap memory
- processing latency
- topic rate / frame drops
- map search or submap retrieval latency

The benchmark framework must distinguish measured evidence from heuristics or manually assigned labels

## 7. Long-duration standard experiment

The project shall define a repeatable figure-eight or equivalent closed-loop route for long-duration tests

The same route and truth binding should be reused for comparisons between estimator/fusion configurations, including future combinations of

- FAST-LIVO2
- wheel constraints
- localization-prior weighting
- GNSS/global correction

Long-duration validation shall include repeated laps plus time-based runs and memory/latency monitoring

## 8. GNSS architecture requirement

GNSS shall be integrated as global correction/fusion evidence rather than replacing continuous local odometry

The target ownership remains

```text
continuous estimator -> odom -> base
localization/global-fusion authority -> map -> odom
```

GNSS, relocalization, loop/place recognition and backend optimization are correction producers/factors and shall not compete as parallel authoritative `map -> odom` publishers

## 9. Large-map / long-term requirement

The architecture must not require one unbounded PCD to remain fully resident and globally searched for all site sizes

Future large-map support shall allow

- map tiling / submaps
- spatial index
- candidate-place retrieval before fine registration
- local active submap windows
- bounded cache / eviction policy
- explicit memory and search-latency telemetry

The first V25-12 implementation may use a single map but must preserve interfaces that do not block later tiling

## 10. Navigation as a BT-composable capability

The final Mission architecture is

```text
Operator / Web / Scheduler / Higher-level Robot Task
                    ↓
             ExecuteMission
                    ↓
          agt_mission_manager
        state / audit / cancel owner
                    ↓
           BehaviorTree.CPP
                    ↓
         project-level BT nodes
                    ↓
       project capability Actions
                    ↓
        navigation runtime services
                    ↓
         safety / chassis boundary
```

BT is an orchestration backend, not an algorithm container

BT nodes shall not

- publish velocity
- publish TF
- inspect raw LiDAR/IMU to implement localization logic
- directly own maps
- directly call arbitrary algorithm-native interfaces when a project capability interface exists

## 11. Replaceable-component requirement

The following domains must be replaceable behind project-owned contracts

### Continuous odometry backend

Examples: FAST-LIO2, FAST-LIVO2, future wheel/LiDAR/vision estimator

Stable project output: canonical odometry + registered cloud + estimator health

### Global localization backend

Examples: NDT/ICP, Scan Context/place recognition, weighted prior localization, GNSS/fusion backend

Stable project output: localization/correction evidence consumed by the single Localization Authority

### Global planner backend

Examples: NavFn, Smac, Hybrid-A*, farmland/coverage planner

Stable project boundary: project route/path capability

### Local controller backend

Examples: MPPI, RPP, DWB, custom tracker

Stable project boundary: runtime path input and navigation command output before Safety

### Map backend

Examples: single PCD, tiled PCD/submap store, occupancy products

Stable project boundary: versioned Site Package + map identity

Replacing one backend must not require rewriting Mission semantics or business-level BT trees

## 12. Capability model

Mission trees should compose semantic capabilities such as

```text
CheckTaskReadiness
EnsureLocalization
Relocalize
LoadSitePackage
ExecuteRoute
NavigateToSemanticTarget
ExecuteCoverageTask
WaitForCondition
RecordBenchmark
ReturnToSafePoint
```

A capability can internally use multiple ROS nodes, Actions and algorithms

BT should depend on capability result semantics such as SUCCESS / FAILURE / CANCELED and structured project error codes rather than backend-specific topics

## 13. Safety and ownership invariants

- `agt_mission_manager` remains the single Mission state/action/audit owner
- BehaviorTree.CPP remains an execution backend under Mission Manager
- continuous odometry owns `odom -> base_footprint`
- Localization Authority uniquely owns authoritative `map -> odom`
- controller output must pass through the project Safety boundary before chassis actuation
- BT cancellation propagates down through project capabilities but final physical stop is guaranteed by the Safety domain
- Offline tools never own runtime TF or chassis command

## 14. Frozen development order

### V25-12A — Site Manifest & Asset Lineage

Define Site Package schema, IDs, hashes and validation tooling

### V25-12B — Point Cloud Processing CLI

Create deterministic import/clean/filter/derive pipeline before GUI work

### V25-12C — AGT Map Workbench MVP

Visual map cleaning, prior annotation, semantic editing and route preview

### V25-12D — Localization Prior Contract

Define region/voxel prior schema and adapter interface; do not force estimator changes yet

### V25-12E — Offline Semantic + Route Preview

Connect existing semantic map / task / coverage components to immutable Site Package assets

### V25-12F — Benchmark Dataset & Evaluator

Standardize bag metadata, truth binding, trajectory/degradation/runtime reports

### V25-12G — Vehicle Package Export

Produce a validated deployment package from READY site assets

### V25-13 — Real BUNKER + MID360 + FAST-LIVO2 validation

Replace simulation authorities with real adapters while preserving V25-11 acceptance semantics where possible

### V25-14 — Long-duration Figure-eight Benchmark

Run repeated and time-based real-site comparisons with truth

### V25-15 — GNSS Global Navigation

Integrate RTK/global correction and GNSS-denied transitions

### V25-16 — Large-map & Long-term Runtime

Introduce tiled/submap storage, cache policies and memory/search-latency acceptance

## 15. Definition of architecture success

The architecture is considered successful when a new agricultural site can be prepared through the standard offline workflow, exported as a validated Site Package, loaded by the vehicle without runtime code changes, and executed through the same Mission/BT capability semantics while core localization/planning/control backends can be replaced independently
