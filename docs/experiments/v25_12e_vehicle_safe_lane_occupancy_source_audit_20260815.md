# V25-12E Vehicle-Safe Lane OCCUPIED Source Audit — 2026-08-15

Real greenhouse audit using the frozen Navigation Map derivation sidecars,
Agricultural Aisle Graph, and canonical MK-mini preview footprint

## Summary

Source exactness:

```text
APPROX_GEOMETRY_THRESHOLD_SOURCE_NO_POINT_COUNT_GROUND_VALID
```

Global Navigation Map OCCUPIED attribution:

```text
actual occupied  85468
raw obstacle     46981  0.550
geometry         15813  0.185
padding only     22674  0.265
unexplained          0  0.000
```

Best bounded-lateral vehicle-pose OCCUPIED source classification across the 19
structural aisles:

```text
17 PADDING_ONLY_DOMINANT
 1 RAW_OBSTACLE_DIRECT_DOMINANT   aisle_003
 1 STRUCTURAL_WIDTH_BLOCKED       aisle_005
```

No aisle is geometry-dominant, UNKNOWN-dominant, out-of-grid-dominant, or
unexplained-occupied-dominant

## Representative route evidence

```text
aisle_001
raw obstacle 0.436
geometry     0.069
padding only 0.495

 aisle_011
raw obstacle 0.344
geometry     0.122
padding only 0.534

 aisle_013
raw obstacle 0.208
geometry     0.031
padding only 0.761

 aisle_015
raw obstacle 0.263
geometry     0.037
padding only 0.700

 aisle_020
raw obstacle 0.057
geometry     0.161
padding only 0.781
```

The late greenhouse aisles that are mostly configuration-space FREE still show
padding as the dominant source of their remaining occupied pose-cell conflicts

## Important discretization fact

The frozen derivation configuration uses the ground-relative Navigation Map
implementation whose default obstacle padding is:

```text
obstacle_padding_m = 0.05 m
resolution_m       = 0.10 m
```

The current derivation converts metric padding to integer cells using:

```text
ceil(obstacle_padding_m / resolution_m)
```

Therefore a requested 0.05 m padding becomes one full grid cell at 0.10 m
resolution and the existing square maximum-filter dilation expands into the
8-neighborhood. The route preview separately checks the complete MK-mini
footprint and currently adds another 0.05 m preview-footprint padding

This creates a credible double-margin hypothesis, but the source audit alone is
not enough to justify changing the production map

## Frozen decision

Do not overwrite the frozen Navigation Map

Do not continue R6B or R7 yet

Required next experiment is a counterfactual padding sensitivity audit using the
same frozen direct obstacle/geometry evidence and ground-support sidecars:

```text
map padding radius      0 / 1 / 2 cells
footprint padding       0 / 0.05 m
```

The experiment must quantify per-aisle bounded-lateral vehicle-pose feasibility
for all six combinations before any production parameter is changed

Interpretation policy:

```text
0-cell map padding strongly restores aisle feasibility
while 1-cell padding destroys it
→ map/footprint safety margin is being duplicated or discretized too coarsely

0-cell map padding still leaves most early aisles blocked
→ raw obstacle / geometry evidence itself remains the limiting source

footprint padding 0.05 m alone causes the main drop
→ preview vehicle-envelope margin is too conservative for this resolution

only 2-cell differs materially from 1-cell
→ current 1-cell padding is not the dominant route-feasibility cause
```

The sensitivity layer is diagnostic only and must not mutate the production
Navigation Map, Aisle Graph, vehicle profile, or R6/R7 admission
