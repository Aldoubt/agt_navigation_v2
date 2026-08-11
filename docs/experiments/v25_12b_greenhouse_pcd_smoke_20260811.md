# V25-12B Real Greenhouse PCD Smoke — 2026-08-11

Status: PASS

## Purpose

Validate the V25-12B point-cloud processing path against an existing real greenhouse map rather than only synthetic fixtures

This is a processing/runtime compatibility smoke, not a localization-quality acceptance

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

## Result

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

## Conclusion

The V25-12B reader/executor successfully consumed a real PCL/CloudCompare `binary_compressed` PCD with more than five million points, restored its structured fields, emitted a canonical uncompressed binary PCD, froze processing provenance and passed the read-only integrity validator

This closes the real-format compatibility risk discovered during the first smoke

It does not prove that the current PCD is an optimal Localization Map

Before selecting crop, height, voxel, outlier or ground parameters, the next step is to profile the real field distributions and then produce a scene-specific cleaning recipe
