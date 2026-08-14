# V25-12C Agricultural Structure Evidence

Status: IMPLEMENTED, LOCAL ACCEPTANCE PENDING

Date: 2026-08-14

## Goal

Extend the generic Ground-relative Navigation Map with agricultural scene-scale evidence without replacing or hiding raw obstacle evidence

```text
Ground-relative result
        ↓
Ground Confidence
        ↓
0.5 m Robust Local-plane Slope
        ↓
Agricultural Row Structure
        ↓
Row Regularization
        ↓
Aisle Candidate
```

This stage does **not** yet rewrite the Final PGM with row priors

The operator must first review the independent evidence layers on the real greenhouse map

## Architecture boundary

Algorithm ownership

```text
agt_offline_assets/navigation_structure.py
```

Workbench ownership

```text
agt_map_workbench/agricultural_workbench.py
```

The Workbench only configures, invokes and previews the offline algorithm

It does not implement SciPy fitting or row detection privately in the GUI

## Ground Confidence

Ground Confidence combines

- real ground-support point count
- local robust-plane residual
- Ground-relative ground-valid mask

High confidence means the cell is supported by multiple points and locally behaves like a consistent terrain surface

Low confidence does not automatically mean occupied

It is evidence for later FREE-space policy

## Robust Local-plane Slope

The original Ground-relative slope is a diagnostic derivative of the 0.10 m elevation grid

The agricultural structure layer adds a separate scene-scale slope estimate

Default

```text
window = 0.50 m
small median prefilter = 1 cell
local model = z = ax + by + c
slope = atan(sqrt(a² + b²))
```

The fit is vectorized over the regular grid and tolerates missing ground cells

The output also records local plane RMS residual

The original 0.10 m slope layer remains available for comparison

## Row direction

Preferred source

```text
Map Frame Calibration +X
```

This is appropriate when +X has been aligned to the greenhouse wall / crop-row direction

If no Map Frame calibration is currently available, the Workbench falls back to automatic dominant-row direction estimation

The automatic estimator searches candidate directions and scores how strongly obstacle evidence forms repeated bands in the perpendicular coordinate

## Row Structure

For row direction unit vector `d`

```text
u = along-row coordinate
v = cross-row coordinate
```

Raw relative-height obstacle evidence is accumulated across `u` to build a cross-row support profile over `v`

Peaks become row candidates only when they satisfy minimum spacing and prominence constraints

For each accepted row

- evidence is projected along `u`
- short gaps may be repaired
- short isolated segments are removed
- the result is rasterized back to the original XY grid using the configured row half-width

Default authoring parameters

```text
row half width = 0.22 m
maximum repaired gap = 0.60 m
minimum continuous segment = 1.50 m
minimum row spacing = 0.55 m
```

These are experiment defaults, not frozen greenhouse product values

## Aisle Candidate

A cell is only an Aisle Candidate when

- Ground Confidence is sufficient
- Robust Local-plane Slope is within the current navigation slope limit
- it is outside the regularized crop-row structure
- it is not a raw relative-height obstacle cell

Aisle Candidate is not yet authoritative FREE

Later final FREE evidence should combine

```text
Aisle Candidate
+ bag ray-traced observed free space
+ vehicle swept footprint
```

## Workbench layers

The Navigation Map layer selector now includes

```text
Base evidence
- Final PGM trinary
- Local Ground Height
- Raw Obstacle Evidence
- Original 0.10 m Slope
- Step Height

Agricultural structure evidence
- Ground Confidence
- Robust Plane Slope
- Local Plane Residual
- Row Support
- Regularized Crop Rows
- Aisle Candidate
```

The operator can enable

```text
仅看分析层（隐藏点云底图）
```

to distinguish actual derived evidence from structures that are only visible in the underlying PCD preview

## Acceptance

Build from a clean ROS underlay

```bash
cd ~/agt_navigation_v2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select agt_offline_assets agt_map_workbench
source install/setup.bash
```

Tests

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_navigation_map_derivation.py \
  src/agt_offline_assets/test/test_navigation_structure.py \
  src/agt_map_workbench/test/test_workbench_model.py \
  src/agt_map_workbench/test/test_frame_calibration.py \
  tests/test_v25_12c_map_workbench_contract.py
```

Launch

```bash
ros2 run agt_map_workbench agt_map_workbench
```

Recommended real-map review sequence

1. Generate the same Ground-relative map used in the previous successful comparison
2. Keep the previous navigation thresholds unchanged for A/B comparison
3. Review `Ground Confidence`
4. Review original `坡度`
5. Review `0.5m Robust Plane 坡度`
6. Review `局部平面残差`
7. Review `种植行支持强度`
8. Review `规则化种植行`
9. Review `行道候选`
10. Toggle `仅看分析层` to remove PCD-display ambiguity

For the first row-structure smoke, prefer the previously validated Map Frame +X direction when available

Record

- detected row angle
- number of accepted rows
- whether row centers visually coincide with crop rows
- whether short vegetation gaps are repaired
- whether cross-aisles / long gaps remain open
- whether isolated non-row obstacles remain outside Aisle Candidate
- Robust Slope versus original 0.10 m slope behavior

## Promotion boundary

Do not use Row Regularization to modify the exported Final PGM until the real greenhouse acceptance demonstrates that

- row angle is stable
- row count is plausible
- row half-width is physically meaningful
- short-gap repair does not bridge true cross-aisles
- Aisle Candidate does not erase independent obstacles

After that gate, the next stage may add an explicit final-policy fusion rather than silently overwriting raw evidence
