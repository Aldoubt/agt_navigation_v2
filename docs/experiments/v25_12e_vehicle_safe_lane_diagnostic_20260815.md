# V25-12E Vehicle-Safe Lane Diagnostic — 2026-08-15

Real greenhouse diagnostic using the frozen Aisle Graph, Navigation Grid, and
canonical MK-mini preview footprint

## Summary

```text
19 aisles
13 FOOTPRINT_OCCUPIED_DOMINANT
 2 MIXED_OCCUPIED_DOMINANT
 2 MOSTLY_CONFIGURATION_SPACE_FREE
 1 CENTER_REFERENCE_OCCUPIED_DOMINANT
 1 STRUCTURAL_WIDTH_BLOCKED
```

No aisle was UNKNOWN-dominant or map-coverage-limited

## Key interpretation

Most failures are not caused by the structural centerline reference point being
outside the Navigation Map or UNKNOWN

Representative evidence:

```text
aisle_013
reference FREE              0.970
zero-offset footprint FREE  0.003
any lateral footprint FREE  0.007
best candidate mean         FREE 0.652 / OCC 0.335 / UNKNOWN 0.013

aisle_015
reference FREE              0.956
zero-offset footprint FREE  0.000
any lateral footprint FREE  0.000
best candidate mean         FREE 0.644 / OCC 0.356 / UNKNOWN 0.000

aisle_011
structural width            1.856 m
allowed lateral shift       0.500 m
reference FREE              0.663
any lateral footprint FREE  0.000
best candidate mean         FREE 0.386 / OCC 0.608 / UNKNOWN 0.006
```

`aisle_003` is the only `CENTER_REFERENCE_OCCUPIED_DOMINANT` aisle

```text
reference FREE 0.397
reference OCC  0.603
```

`aisle_016` and `aisle_020` are mostly configuration-space FREE

```text
aisle_016 any lateral FREE 0.951
aisle_020 any lateral FREE 0.945
```

`aisle_017` and `aisle_018` are mixed but mostly recoverable locally

`aisle_005` remains structurally rejected because 0.658 m < 0.700 m preview
vehicle-width gate

## Frozen conclusion

The dominant unresolved question is now the provenance of `OCCUPIED` cells that
overlap the MK-mini preview footprint

Do not continue R6B / R7 and do not relax occupancy thresholds yet

Required next audit:

```text
Navigation Map OCCUPIED
├─ raw obstacle evidence
├─ slope / step geometry evidence
├─ obstacle-padding-only cells
└─ unexplained / override / legacy occupied cells
```

The audit must compare blocked early/middle aisles against the mostly-free late
aisles so the repair can be assigned to the correct upstream semantic layer
