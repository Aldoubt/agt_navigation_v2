# V25-12B Real Greenhouse PCD Smoke — 2026-08-11

Status: PASS

## Purpose

Validate the V25-12B point-cloud processing path against an existing real greenhouse map rather than only synthetic fixtures

This is a processing/runtime compatibility smoke and data-profile record, not a localization-quality acceptance

## Source

```text
runtime/maps/greenhouse_ground/pcd/greenhouse_aligned_full2.pcd
```

Source identity

```text
sha256:619d51351ad0da8762d455af3b7135db55725e99b35efbefbb5ba8d9834660ed
```

PCD characteristics observed by `inspect-pcd`

```text
point_count: 5,192,062
DATA: binary_compressed
FIELDS:
  Coord._Z
  curvature
  intensity
  normal_x
  normal_y
  normal_z
  x
  y
  z
```

XYZ bounds

```text
min: [-2.517016887664795, -31.68592643737793, -2.697890520095825]
max: [31.410165786743164, 10.037065505981445, 8.045159339904785]
```

Approximate axis spans from the exact bounds

```text
X: 33.927 m
Y: 41.723 m
Z: 10.743 m
```

All 5,192,062 points have finite x/y/z values

## Smoke recipe

The first real-data smoke intentionally made no scene-specific crop/filter assumptions

```yaml
schema: agt_pointcloud_processing_recipe/v1
recipe_id: greenhouse_compressed_input_smoke_v1
frame_id: map
output_data: binary
random_seed: 42
operations:
  - type: remove_nonfinite
```

## Smoke result

Processing output

```text
input_points:  5,192,062
output_points: 5,192,062
removed_points: 0
retained_ratio: 1.0
```

Output identity

```text
sha256:709478134cb38ee3f9c0f5e6542412fb0870d3764b80df1e28ba9f7759d348d1
```

Processing content identity

```text
sha256:901ce675dd8c2ea1d212840c872b3a833d333edafa26c896b50d2e9bef31aebd
```

`validate-pointcloud-processing` returned

```text
valid: true
record_readable: true
recipe_hash_valid: true
output_hash_valid: true
output_pcd_readable: true
output_point_count_valid: true
processing_content_identity_valid: true
```

## Deterministic profile

The real map was profiled with

```text
inspect-pcd --profile --sample-limit 200000
```

The profile uses deterministic even-spacing sampling for percentiles while finite counts and XYZ bounds are exact

### X

```text
p1   -1.897999
p5   -0.870461
p25   3.478747
p50  11.894555
p75  22.040533
p95  28.555266
p99  29.890684
```

### Y

```text
p1  -29.168531
p5  -27.546118
p25 -20.750445
p50 -11.479524
p75  -5.021789
p95   4.689084
p99   7.999018
```

### Z

```text
p1  -2.178724
p5  -1.765981
p25 -0.418319
p50  0.437934
p75  2.059991
p95  3.994271
p99  4.516166
```

The p1-to-p99 Z range is approximately 6.695 m, so the large vertical extent is not explained by only a few extreme points

A global height crop is therefore not justified from statistics alone; greenhouse roof/frame structure may be valuable stable localization geometry

### Field observations

`Coord._Z` is numerically almost identical to `z` across exact min/max and sampled percentiles, so it is redundant as a localization-product field

`curvature` is exactly 0 for every point and therefore contains no information in this asset

`intensity` is non-degenerate

```text
min   0
p25   4
p50   8
p75  27
p95 124
p99 152
max 255
```

The source/master point cloud still retains all original fields because destructive field removal is not required for cleaning provenance

A later derived Localization Map should use an explicit field policy rather than inheriting every CloudCompare analysis field by accident

Initial localization-product candidate policy

```text
keep: x, y, z, intensity
not required by default: Coord._Z, curvature, normal_x, normal_y, normal_z
```

This policy is a product-derivation decision and does not mutate the source/master map

## Cleaning decisions after profile

The profile is sufficient to reject several premature automatic choices

### Global XY crop

Do not infer a site polygon from percentiles

The next stage should author the true site boundary visually in AGT Map Workbench and serialize it as `crop_polygon`/`delete_polygon` recipe operations

### Global Z crop

Do not apply one yet

The observed p1-to-p99 vertical range contains a substantial fraction of the point cloud and may include stable greenhouse frame/roof geometry useful for relocalization

### Voxel size

Do not freeze 0.05 m or 0.10 m only from bounding-box statistics

The Workbench/quality workflow should compare retained structure and localization performance before choosing the production value

### Outlier removal

SOR and radius-outlier backends are available but no production threshold is frozen by this smoke

### Ground separation

Do not remove the ground from the master/localization map merely because a ground backend exists

Ground extraction is primarily a navigation-map/traversability derivation concern unless localization experiments demonstrate a benefit

## Conclusion

The V25-12B reader/executor successfully consumed a real PCL/CloudCompare `binary_compressed` PCD with more than five million points, restored its structured fields, emitted a canonical uncompressed binary PCD, froze processing provenance and passed the read-only integrity validator

The deterministic profile also demonstrates that V25-12B can inspect real scene distributions without modifying the asset

This closes the real-format compatibility and read-only profiling risks for the V25-12B processing core

It does not prove that the current PCD is an optimal Localization Map

Scene-boundary editing and visual authoring should now move to V25-12C AGT Map Workbench while the V25-12B processing kernel remains the canonical save/export backend
