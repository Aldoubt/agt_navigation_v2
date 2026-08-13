# V25-12C AGT Map Workbench MVP

Status: IMPLEMENTED, UPDATED LOCAL ACCEPTANCE PENDING

Date: 2026-08-13

## Goal

V25-12C provides one Offline Plane editor for three related but separate authoring products

```text
PCD editing        → V25-12B processing Recipe
Map frame          → map_frame.yaml
Navigation map     → ground-relative PGM/YAML + evidence
```

It does not publish runtime ROS topics, own TF, command the vehicle, or modify READY assets in place

## Current real-data baseline

Primary full scene

```text
runtime/maps/green house full.pcd
```

The earlier `greenhouse_aligned_full2.pcd` is an already-cropped historical V25-12B smoke asset

## 1. Preview / asset separation

The point-cloud renderer remains 2.5D and deterministically sampled

Preview controls

- dark/light background
- height/intensity/mono color
- 1–4 px point size
- 60k/150k/300k/600k display sample
- display-only Z window

Formal PCD processing and Navigation Map derivation use the complete loaded PCD

```text
Preview resolution != Asset resolution
```

Display sampling never becomes processing input

## 2. Point-cloud editing

The visual editor still serializes to the frozen V25-12B contract

```text
polygon × Z
    ↓
delete_polygon / crop_polygon
    ↓
agt_pointcloud_processing_recipe/v1
    ↓
immutable full-resolution processing
```

Visible feedback includes fixed-size numbered vertices, coordinates and explicit Recipe execution order

## 3. Map Frame Calibration

### Z semantics are now separated

The first real smoke showed that one shared Z control was ambiguous

The Workbench now has independent values for

```text
Display Z       only changes visualization
Origin Z        nearest-point origin snapping
X-wall Z        wall-corridor fitting
Z-pillar Z      pillar-cylinder fitting
```

A convenience button copies the current Display Z into all three calibration ranges, after which they can be edited independently

Completed X/Z fits report the actual `z_window_m` captured in their fit evidence

### Structural fitting

```text
origin click
    ↓ nearest 3D point inside Origin Z

X wall endpoints
    ↓ XY corridor + X-wall Z
    ↓ PCA principal line

Z pillar center
    ↓ XY cylinder + Z-pillar Z
    ↓ 3D PCA principal line
```

The noisy X/Z candidates are orthogonalized

```text
X ← X projected perpendicular to Z
Y = Z × X
X = Y × Z
```

The result is an orthonormal right-handed map frame

### Preview-label correction

Axis lines use map coordinates, but marker/label size and offsets are screen-space UI properties

The label is now a child of the fixed marker item so offsets such as 8 px / 20 px are no longer interpreted as metres in the scene

The solved origin is displayed once as `O / +Z↑`; X1/X2 and Z-pillar labels use different local offsets to reduce overlap

### Geographic boundary

`map_frame.yaml` remains separate from RTK/ENU/UTM georeference

```text
source map
    ↓ map_frame.yaml
site-friendly map
    ↕ future georeference.yaml
ENU / UTM
```

## 4. Ground-relative Navigation Map

### Reason for the change

A fixed global Z slice is not a reliable way to derive a greenhouse PGM because the floor is not perfectly level

A locally higher but traversable floor must not become occupied merely because its absolute Z is larger

### Offline derivation contract

Implemented in

```text
src/agt_offline_assets/agt_offline_assets/navigation_map_derivation.py
```

Schema

```text
agt_ground_relative_navigation_map/v1
```

Pipeline

```text
full finite XYZ
    ↓
XY grid
    ↓
per-cell low-quantile ground seed
    ↓
finite-distance gap filling
    ↓
local median ground smoothing
    ↓
z_ground(x,y)
    ↓
h_rel = z - z_ground(x,y)
    ↓
relative obstacle evidence
+ ground support
+ slope
+ step
    ↓
FREE / OCCUPIED / UNKNOWN
```

Important semantics

- FREE requires explicit ground support
- OCCUPIED can come from obstacle evidence, excessive slope, or excessive local step
- cells without enough evidence remain UNKNOWN
- no global absolute-height obstacle slice is used

The current algorithm is an elevation-grid MVP, not a claim that the final greenhouse DTM method is frozen

Later PMF/CSF/progressive morphology candidates can be compared behind the same derivation/output contract

### Workbench Navigation Map tab

Parameters exposed in the MVP

- grid resolution
- ground low quantile
- ground gap-fill distance
- smoothing radius
- relative obstacle min/max height
- maximum slope
- maximum step

Preview layers

```text
Final trinary PGM
Ground height
Obstacle evidence
Slope
Step
```

The semi-transparent layer is aligned to the currently loaded PCD coordinates

The editor intentionally does not apply `map_frame.yaml` to an untransformed source cloud behind the operator's back

A transformed immutable PCD revision will be materialized in the next increment; loading that product will make Navigation Map authoring naturally occur in the new map frame

### Manual override contract

Manual repair is world-coordinate polygon metadata, not pixel painting

```text
FORCE_FREE
FORCE_OCCUPIED
UNKNOWN
NO_GO
```

Therefore the repair can be replayed after changing map resolution

### Exported evidence

```text
navigation_map.pgm
navigation_map.yaml
derivation.yaml
ground_height.npy
slope_deg.npy
step_m.npy
obstacle_count.npy
ground_support_count.npy
```

## 5. Updated local acceptance gate

Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to agt_map_workbench
source install/setup.bash
```

Tests

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_navigation_map_derivation.py \
  src/agt_map_workbench/test/test_workbench_model.py \
  src/agt_map_workbench/test/test_frame_calibration.py \
  tests/test_v25_12c_map_workbench_contract.py
```

V25-12B regression remains

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_pcd_compressed.py \
  src/agt_offline_assets/test/test_pointcloud_profile.py \
  src/agt_offline_assets/test/test_pointcloud_processing.py \
  tests/test_v25_12b_pointcloud_contract.py
```

Launch

```bash
ros2 run agt_map_workbench agt_map_workbench
```

### GUI smoke

Open

```text
runtime/maps/green house full.pcd
```

Verify

1. coordinate labels stay near O/X/Y endpoints at different zoom levels
2. changing Display Z does not alter completed X/Z fit evidence
3. Origin/X-wall/Z-pillar each use their dedicated calibration Z range
4. Navigation Map generation does not depend on Display Z
5. switch Final/Ground/Obstacle/Slope/Step layers and verify overlays remain aligned with the loaded PCD
6. inspect whether raised but smooth ground remains free instead of becoming occupied from absolute height alone
7. draw one FORCE_FREE or FORCE_OCCUPIED polygon and verify it appears in the override list and final overlay
8. export a new Navigation Map run and verify all PGM/YAML/evidence files exist

Do not promote V25-12C to PASS until these tests are run on the real full greenhouse PCD

## 6. Next increment

Priority after this acceptance

```text
map_frame.yaml
    ↓
materialize transformed PCD
    ↓
new immutable Map Revision
    ↓
Ground-relative Navigation Map in final map frame
```

Then continue with Localization Prior, Semantic Map, Route Preview and RTK georeference
