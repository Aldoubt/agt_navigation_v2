# Point-cloud Processing Recipe and Run Contract

V25-12B defines the deterministic point-cloud processing core used before AGT Map Workbench authoring and before a cleaned Localization Map is promoted into a READY Map Version

The processing core is pure offline logic

It does not publish ROS topics, TF or velocity, does not mutate READY Map Versions, and does not make a processing result executable merely because processing succeeded

## 1. Canonical flow

```text
Raw / mapped PCD
+ Point-cloud Processing Recipe
        ↓
process-pointcloud
        ↓
immutable processing run
        ├── recipe.yaml
        ├── processed.pcd
        ├── processing.yaml
        └── processing_report.json
        ↓
validate-pointcloud-processing
        ↓
review / quality gate
        ↓
explicit later selection/materialization into a PROCESSING Map Version
```

The source PCD remains read-only

A GUI edit that changes accepted map content must be represented as a replayable recipe operation or patch before it can produce formal downstream assets

## 2. Recipe schema

```yaml
schema: agt_pointcloud_processing_recipe/v1
recipe_id: greenhouse_localization_clean_v1
frame_id: map
output_data: binary
random_seed: 42

operations:
  - type: remove_nonfinite

  - type: crop_box
    parameters:
      min: [-10.0, -8.0, -1.0]
      max: [35.0, 12.0, 4.0]
      keep_inside: true

  - type: delete_polygon
    parameters:
      polygon_xy:
        - [4.0, 1.0]
        - [7.0, 1.0]
        - [7.0, 3.0]
        - [4.0, 3.0]
      z_min: -0.5
      z_max: 3.0

  - type: voxel_downsample
    parameters:
      leaf_size_m: 0.05

  - type: sor
    parameters:
      mean_k: 20
      stddev_mul: 1.0
```

`frame_id` is the declared coordinate-frame identity of the PCD product; PCD itself does not encode a ROS frame

`output_data` currently accepts `ascii` or uncompressed `binary`

## 3. Supported operations

### `remove_nonfinite`

Remove points whose x/y/z contains NaN or infinity

Operations requiring geometry fail closed on non-finite XYZ unless this operation has removed them first

### `crop_box`

Parameters

```yaml
min: [x, y, z]
max: [x, y, z]
keep_inside: true
```

`keep_inside: false` is a deterministic 3D delete-box primitive

### `crop_polygon`

Select an XY polygon with optional Z bounds

```yaml
polygon_xy: [[x0, y0], [x1, y1], [x2, y2]]
z_min: 0.0
z_max: 3.0
keep_inside: true
```

This is appropriate for retaining an irregular site boundary

### `delete_polygon`

Same polygon/Z representation as `crop_polygon`, but points in the selected volume are removed

This is the canonical V25-12B primitive for a future Workbench polygon-delete operation

### `height_range`

```yaml
min_z: -0.2
max_z: 2.5
```

Both bounds are explicit in the V25-12B recipe baseline

### `voxel_downsample`

Either isotropic

```yaml
leaf_size_m: 0.05
```

or per-axis

```yaml
leaf_size: [0.05, 0.05, 0.05]
```

The output point order is deterministic by voxel key

XYZ is replaced by the voxel centroid while other PCD fields are retained from a deterministic representative point

### `sor`

Statistical outlier removal

```yaml
mean_k: 20
stddev_mul: 1.0
```

The V25-12B backend uses a KD-tree nearest-neighbour implementation

### `radius_outlier`

```yaml
radius_m: 0.20
min_neighbors: 3
```

`min_neighbors` excludes the query point itself

### `ground_separation`

First backend

```yaml
method: plane_ransac
distance_threshold_m: 0.08
max_iterations: 120
max_tilt_deg: 20.0
fit_sample_limit: 50000
keep: nonground
random_seed: 42       # optional; falls back to recipe random_seed
```

This is a deterministic near-horizontal plane baseline, not the final agricultural ground model

PMF, CSF, terrain models or future learned backends may be added later without changing the processing-run identity model

## 4. Unsupported operations fail closed

Unknown operation names are rejected

`DATA binary_compressed` PCD is currently rejected rather than silently decompressed or converted

If a source tool creates compressed PCD, convert it explicitly to `ascii` or uncompressed `binary` before formal V25-12B processing

## 5. Processing run artifact

Canonical layout

```text
<run_dir>/
├── recipe.yaml
├── processed.pcd
├── processing.yaml
└── processing_report.json
```

`processing.yaml` uses

```text
schema: agt_pointcloud_processing/v1
```

and binds

- recipe ID and SHA256
- declared frame
- input source display name and SHA256
- input PCD point count / field schema / DATA mode
- ordered operation list and parameters
- per-operation input/output/removed counts and metrics
- output relative path and SHA256
- output point count / field schema / DATA mode
- stable `processing_content_sha256`

Absolute source paths are not part of the stable processing identity

The source PCD itself remains external immutable evidence referenced by content hash

## 6. Report

`processing_report.json` uses

```text
schema: agt_pointcloud_processing_report/v1
status: PASS
```

and records at least

- input point count
- output point count
- removed point count
- retained ratio
- input SHA256
- output SHA256
- processing content SHA256
- operation count

`PASS` means the recipe executed and produced internally consistent evidence

It does not mean the resulting point cloud has passed localization quality, map quality or vehicle-deployment acceptance

## 7. Determinism requirement

For the same

```text
input PCD bytes
+ recipe bytes
+ declared processing backend/environment
```

the V25-12B core is expected to produce the same processed PCD bytes and SHA256

Randomized operations must consume a recorded seed

Later research comparisons should also record dependency/runtime versions when strict cross-machine byte reproducibility matters

## 8. Immutability

`process-pointcloud` refuses an already existing output directory

A new edit creates a new processing run

The command never overwrites the source PCD and never edits a READY Map Version in place

## 9. Workbench boundary

AGT Map Workbench V25-12C is a visual authoring client over this contract

Expected mapping

```text
3D crop box       -> crop_box
polygon retain    -> crop_polygon
polygon delete    -> delete_polygon
height slider     -> height_range
voxel UI          -> voxel_downsample
outlier UI        -> sor / radius_outlier
ground UI         -> ground_separation
```

The Workbench may preview an operation interactively, but a formal save/export must execute or replay the canonical recipe core and freeze its hashes

## 10. Localization Prior boundary

Stable-structure weighting such as

```text
EXCLUDE / LOW / NORMAL / HIGH / LANDMARK
```

belongs to V25-12D Localization Prior and is not silently encoded by V25-12B filtering

Deleting a point and assigning a low localization weight are different semantics
