# V25-12E Vehicle-Safe Lane Padding Sensitivity — 2026-08-15

Real greenhouse counterfactual audit using the frozen Navigation Map derivation
sidecars, Agricultural Aisle Graph, and canonical MK-mini preview footprint

## Frozen map parameters

```text
requested obstacle padding  0.050 m
resolution                  0.100 m
effective padding radius    1 cell
axis dilation               0.100 m
diagonal reach              0.141 m
```

The requested 0.05 m metric padding therefore becomes one full square grid-cell
radius in the current integer-cell maximum-filter implementation

## Six-case summary

```text
map pad   footprint pad   zero-feasible aisles   mostly-free aisles   mean any-lateral FREE
0 cell    0.00 m          0                       5                    0.368
0 cell    0.05 m          2                       5                    0.305
1 cell    0.00 m          8                       5                    0.245
1 cell    0.05 m          10                      2                    0.229   CURRENT-LIKE
2 cells   0.00 m          14                      2                    0.112
2 cells   0.05 m          15                      1                    0.075
```

Removing map padding and preview-footprint padding both improve route-pose
feasibility. The interaction confirms that safety margin is being compounded at
0.10 m map resolution

However this is not the dominant root cause of the early greenhouse failures

Even in the least conservative counterfactual:

```text
map padding       0 cells
footprint padding 0.00 m
```

representative early/middle aisle feasibility remains:

```text
aisle_001 0.091
aisle_002 0.081
aisle_003 0.008
aisle_006 0.203
aisle_007 0.141
aisle_008 0.027
aisle_011 0.133
aisle_012 0.067
```

while late aisles remain strongly feasible:

```text
aisle_016 0.986
aisle_017 0.978
aisle_018 0.958
aisle_019 0.921
aisle_020 0.960
```

`aisle_013` and `aisle_015` partially recover when all extra padding is removed:

```text
aisle_013 0.500
aisle_015 0.491
```

This demonstrates that padding is an important secondary conservatism but cannot
explain the strong early-vs-late spatial split

## Frozen conclusion

Do not set production `obstacle_padding_m` to zero solely from this experiment

Do not continue R6B or R7 yet

The next diagnostic must isolate the direct occupied sources after map padding is
removed:

```text
NO_DIRECT_OCCUPIED
RAW_OBSTACLE_ONLY
GEOMETRY_ONLY
RAW_PLUS_GEOMETRY
```

All four cases use zero map padding and the same frozen ground-support evidence

The purpose is to determine whether the remaining early-aisle bottleneck is
primarily raw obstacle projection, slope/step geometry evidence, or an interaction
between the two

The production Navigation Map, Aisle Graph, canonical vehicle profile, and
R6/R7 admission remain immutable during the experiment
