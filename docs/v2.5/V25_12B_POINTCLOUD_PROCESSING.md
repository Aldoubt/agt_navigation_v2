# V25-12B Point Cloud Processing CLI

Status: IMPLEMENTED, REAL PCD SMOKE PASS, UPDATED LOCAL ACCEPTANCE PENDING

## Goal

Provide the deterministic non-destructive point-cloud processing kernel that V25-12C AGT Map Workbench will call

The stage is complete only when the same input PCD and recipe produce a validated immutable processing run and the existing V25-12A/offline contracts remain green

## Implemented

### PCD I/O

`src/agt_offline_assets/agt_offline_assets/pcd_io.py`

Supports

- PCD v0.7-style schema parsing
- arbitrary scalar/vector field preservation
- `DATA ascii`
- uncompressed `DATA binary`
- PCL `DATA binary_compressed` input through bounds-checked LZF decoding and field-major/SoA restoration
- deterministic ASCII/BINARY writing
- required scalar x/y/z geometry fields

Formal processing output remains normalized to ASCII or uncompressed binary; V25-12B does not emit `binary_compressed`

### Read-only profiling

`inspect-pcd` provides exact finite-XYZ counts and bounds

Optional profiling

```bash
ros2 run agt_offline_assets agt_offline_assets_cli.py inspect-pcd \
  --input <input.pcd> \
  --profile \
  --sample-limit 200000
```

adds deterministic evenly spaced scalar-field sampling and reports

```text
p1 / p5 / p25 / p50 / p75 / p95 / p99
```

for each scalar PCD field

The profile is diagnostic only and does not modify the source or become a formal map-quality result

Exact bounds use all finite XYZ points; sampled percentiles are explicitly labeled sampled so they are not confused with exact extrema

### Recipe executor

`src/agt_offline_assets/agt_offline_assets/pointcloud_processing.py`

Recipe schema

```text
agt_pointcloud_processing_recipe/v1
```

Processing record

```text
agt_pointcloud_processing/v1
```

Report

```text
agt_pointcloud_processing_report/v1
```

Supported operations

```text
remove_nonfinite
crop_box
crop_polygon
delete_polygon
height_range
voxel_downsample
sor
radius_outlier
ground_separation / plane_ransac
```

### CLI

```text
inspect-pcd
process-pointcloud
validate-pointcloud-processing
```

### Dependencies

- numpy
- scipy for KD-tree SOR/radius queries

No Open3D dependency is required by the canonical processing core

V25-12C may use Open3D or another renderer for visualization without changing the processing contract

## Processing artifact

```text
<run_dir>/
├── recipe.yaml
├── processed.pcd
├── processing.yaml
└── processing_report.json
```

The source is never overwritten

Existing run directories are not reused

The processing artifact is not automatically copied into a Map Version and does not become READY navigation/localization truth by itself

## Ground backend boundary

The first ground backend is seeded near-horizontal plane RANSAC

It is intentionally a baseline only

The agricultural production pipeline may later add PMF, CSF, terrain-adaptive or learned ground backends behind the same operation/evidence boundary

## Real greenhouse smoke evidence

The real source

```text
runtime/maps/greenhouse_ground/pcd/greenhouse_aligned_full2.pcd
```

was successfully consumed after `binary_compressed` support was added

Observed source identity and scale

```text
sha256:619d51351ad0da8762d455af3b7135db55725e99b35efbefbb5ba8d9834660ed
points: 5,192,062
DATA: binary_compressed
```

The minimal `remove_nonfinite` processing smoke retained all 5,192,062 points, emitted canonical binary PCD

```text
sha256:709478134cb38ee3f9c0f5e6542412fb0870d3764b80df1e28ba9f7759d348d1
```

and `validate-pointcloud-processing` returned `valid=true` with all integrity checks true

The exact experiment record is

```text
docs/experiments/v25_12b_greenhouse_pcd_smoke_20260811.md
```

This proves real-format processing compatibility, not localization-map quality

## V25-12B Acceptance Gate

### Build

```bash
rm -rf build/agt_offline_assets install/agt_offline_assets
colcon build --symlink-install --packages-up-to agt_offline_assets
source install/setup.bash
```

If numpy/scipy are missing, resolve package dependencies first with the project rosdep workflow

### Package tests

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_pcd_compressed.py \
  src/agt_offline_assets/test/test_pointcloud_profile.py \
  src/agt_offline_assets/test/test_pointcloud_processing.py
```

Then

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test
```

### Repository contracts

```bash
python3 -m pytest -q \
  tests/test_v25_12b_pointcloud_contract.py \
  tests/test_navigation_offline_contracts.py \
  tests/test_offline_asset_contract.py \
  tests/test_navigation_architecture_contract.py
```

### CLI profile on real data

```bash
ros2 run agt_offline_assets agt_offline_assets_cli.py inspect-pcd \
  --input runtime/maps/greenhouse_ground/pcd/greenhouse_aligned_full2.pcd \
  --profile \
  --sample-limit 200000
```

Use the resulting distributions, especially x/y/z percentiles, before choosing scene-specific crop and height gates

### Processing smoke

The 2026-08-11 real greenhouse compressed-input smoke is already PASS and does not need to be repeated unless PCD I/O or processing identity semantics change

For a new source dataset, run

```bash
ros2 run agt_offline_assets agt_offline_assets_cli.py process-pointcloud \
  --input <input.pcd> \
  --recipe <recipe.yaml> \
  --output-dir <new_run_dir>
```

followed by

```bash
ros2 run agt_offline_assets agt_offline_assets_cli.py validate-pointcloud-processing \
  --run-dir <new_run_dir>
```

Required

```text
valid = true
processing_report.status = PASS
source SHA256 unchanged
processing output SHA256 stable across repeated run with same input/recipe
```

## Not yet implemented

The following are intentionally outside V25-12B

- AGT Map Workbench GUI
- interactive 3D picking/brush rendering
- Localization Prior weight authoring
- PMF/CSF production ground backend
- automatic stable-structure classification
- automatic map-quality/relocalization acceptance from processing PASS
- direct modification of READY Map Versions
- tiled/out-of-core point-cloud processing

Large-map out-of-core/chunked processing belongs to later V25-16 work; V25-12B currently processes one PCD in memory

## Next stage

After this Gate passes, V25-12C builds AGT Map Workbench as a visual client over the frozen V25-12B recipe executor
