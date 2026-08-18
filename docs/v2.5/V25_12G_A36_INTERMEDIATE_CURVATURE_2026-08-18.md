# V25-12G A3.6 Intermediate-Curvature Evidence Freeze

Date: 2026-08-18

Status: **A3.6 FROZEN — PARTIAL RESULT — A4 BLOCKED**

This document freezes the evidence for the V25-12G A3.6 Intermediate-Curvature Primitive Amendment. It records only target-machine results supplied by the operator plus repository-verifiable implementation provenance. It does not reconstruct unsupplied per-connector details and does not make route-ready, optimal, field-ready, globally connected, or global-infeasibility claims.

## 1. Repository / Provenance

```text
Repository: Aldoubt/agt_navigation_v2
Branch: feat/v25-12g-maximum-feasible-coverage
ROS: ROS2 Humble
Target workspace: ~/agt_navigation_v2
```

Frozen A3.5 diagnostic implementation:

```text
692b88cce215dba25df5952c89fdb49258e68687
feat(v25-12g): persist A3.5 connector diagnostics
```

Frozen A3.5 evidence:

```text
cb0935bb23ba6981d3c6d06067f2417267981d9a
docs(v25-12g): freeze A3.5 connector diagnosis evidence
```

A3.6 tests-only RED HEAD:

```text
c2af3784c78a000a8902dde3795f6a361f07811f
```

A3.6 production amendment:

```text
13031ee094a0557d35195d0100cfca823e2f73bf
feat(v25-12g): add intermediate R6B curvature primitives
```

A3.6 read-only acceptance harness:

```text
a1f8d77719d155962ae08b7156f64c4292b7c54e
feat(v25-12g): add A3.6 intermediate-curvature acceptance harness
```

Target-machine G3 repository HEAD:

```text
d6d35a0817ca33ead6e04970fcdd7f5ef3b231e7
```

Formal design and implementation plan:

```text
docs/superpowers/specs/2026-08-17-v25-12g-a36-intermediate-curvature-primitive-design.md
docs/superpowers/plans/2026-08-17-v25-12g-a36-intermediate-curvature-primitive.md
```

## 2. A3.6 Hypothesis and Only Behavior Change

A3.6 tested the falsifiable hypothesis that the original three-curvature R6B lattice was too sparse to express some locally feasible Ackermann connector paths while all existing physical, map, safety and bounded-search gates remained fixed.

The only intended planner behavior amendment was:

```text
before:
{-1.0/R, 0.0, +1.0/R}

after:
{-1.0/R, -0.5/R, 0.0, +0.5/R, +1.0/R}
```

The intermediate `0.5/R` primitives have lower curvature magnitude than the existing saturated `1/R` primitives. The amendment expands the discrete reachable trajectory family without relaxing the physical curvature bound `abs(curvature) <= 1/R`.

No curvature-fraction tuning parameter or parameter sweep was introduced.

## 3. Frozen Constraints

The following remained unchanged throughout A3.6:

```text
canonical platform                  mk_mini
kinematics                          Ackermann
minimum turning radius             1.50 m
canonical footprint                unchanged
preview footprint padding          0.05 m
Site Boundary semantics            unchanged / hard
Navigation Grid semantics          FREE-only / hard
primitive length                   0.30 m
collision sampling                 0.10 m
state XY resolution                0.15 m
state yaw resolution               15 deg
goal position tolerance            0.18 m
goal yaw tolerance                 12 deg
goal-shot distance                 3.0 m
goal-shot behavior                 forward-only Dubins
max cusps                          2
max expansions                     30000
max path length                    18.0 m
longitudinal search padding        0.80 m
lateral search padding             1.25 m
heuristic                          unchanged
queue ordering                     unchanged
cost model                         unchanged
state dominance                    unchanged
A1 assets                          unchanged
A2 assets                          unchanged
motion-graph schema                unchanged
```

A3.6 did not add `±0.25/R`, change terminal semantics, replace R6B with Reeds-Shepp, or weaken any map/safety gate.

## 4. G1 — VALID RED

Operator-supplied target-machine RED evidence:

```text
Collected:  26
Selected:   16
Passed:      0
Failed:     16
Errors:      0
Deselected: 10
```

Intended failure families:

```text
curvature API missing                          6
synthetic +0.5/R reachable-state failure      1
five-primitive path-length accounting         1
A3.6 acceptance harness missing               8
```

Synthetic `+0.5/R` fixture before production:

```text
status = NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION
```

The operator confirmed that this fixture was not failing because of Site Boundary, Navigation Grid, invalid config, R6A admission, or a goal-shot shortcut.

Pre-amendment path-length accounting:

```text
search_expansions = 3
primitive_edges_considered = 9
primitive_edges_rejected_path_length = 9
```

This proved the old search was still using three primitive categories.

**G1 conclusion:** VALID RED. Production work was allowed to begin only after this evidence.

## 5. G2 — GREEN / Regression PASS

### 5.1 Focused A3.6 tests

```text
Collected:  26
Selected:   16
Passed:     16
Failed:      0
Errors:      0
Deselected: 10
```

Synthetic `+0.5/R` fixture after production:

```text
status = FORWARD_PRIMITIVE_PREVIEW_FREE
path_length_m = 0.29998611130401104
goal_position_error_m = 5.55e-17
goal_yaw_error_rad = 4.44e-16
contains +0.5/R curvature sample = True
```

Five-category accounting:

```text
search_expansions = 3
primitive_edges_considered = 15
primitive_edges_rejected_path_length = 15
15 = 5 * 3
```

### 5.2 A3.5 regression subset

```text
Collected: 89
Passed:    89
Failed:     0
Errors:     0
```

### 5.3 Package build / CTest

```text
colcon build --packages-select agt_offline_assets --symlink-install: PASS
CTest total:   43
Passed:        43
Failed:         0
Skipped:        0
```

### 5.4 Frozen-parameter check

```text
fractions = (-1.0, -0.5, 0.0, 0.5, 1.0)
max curvature = 0.6666666666666666 1/m
1/R limit = 0.6666666666666666 1/m
A36_FROZEN_PARAMETER_CHECK=PASS
```

Protected local paths remained unstaged and untouched by A3.6.

**G2 conclusion:** PASS.

## 6. G3 — One-and-Only-One Real-Data Experiment

The operator executed the A3.6 real-data derivation **exactly once** against the frozen real run and reported:

```text
A36_RC = 0
G3 HEAD = d6d35a0817ca33ead6e04970fcdd7f5ef3b231e7
```

The saved target-machine evidence was:

```text
/tmp/v25_12g_a36_intermediate_curvature.json
/tmp/v25_12g_a36_intermediate_curvature.time
```

No rerun was performed after observing the result.

## 7. Real Connector Outcome

Frozen A3.5 baseline:

```text
EXECUTABLE:  0
REJECTED:   40
```

A3.6 amended result:

```text
EXECUTABLE:  2
REJECTED:   38
```

Backend status counts:

```text
REVERSE_PRIMITIVE_PREVIEW_FREE:          2
NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION:  38
```

Therefore A3.6 recovered **2 previously rejected inter-segment connector transitions** while all integrity gates below remained satisfied.

The operator supplied abbreviated recovered-connector descriptions:

```text
...aisle_016...to.aisle_017...dead_end_forward_in_reverse_out
...aisle_016...to.aisle_017...service_high_to_low
```

These strings contain ellipses and are **not recorded here as canonical full connector IDs**. The exact full IDs and the complete 38 rejected IDs remain in the saved target-machine JSON. This evidence document deliberately does not reconstruct missing identifier text from naming conventions.

## 8. G3 Integrity Gates

All reported integrity gates passed:

```text
service_behavior_equal = True
connector_universe_preserved = True
protected_input_hashes_unchanged = True
forbidden_semantic_key_count = 0
unexpectedly_regressed_connector_ids = []
connector_count = 40
```

The working tree after G3 still contained only the pre-existing protected local paths:

```text
M  tools/rosbag_sensor_trimmer
?? src/agt_route_benchmark/
?? tools/map_tools/render_pcd_top_views.py
```

Staged files:

```text
none
```

**G3 integrity conclusion:** PASS.

## 9. Search-Termination Evidence

A3.6 real-data search statistics:

```text
queue_exhausted_count = 35
expansion_budget_reached_count = 3
```

Operator-supplied expansion histogram entries:

```text
0-9:           4
100-499:      16
500-999:       5
1000-4999:     5
10000-19999:   5
20000-30000:   5
```

The `10-99` bin was not explicitly included in the operator report and is therefore not independently asserted here.

The lowest-expansion rejected connectors reported by the operator included:

```text
3 expansions:       FOOTPRINT_GRID_LIMITED x2
7 / 9 expansions:   SITE_BOUNDARY_LIMITED
121-198 expansions: MIXED_LIMITATION examples
```

A3.5 had terminated all 40 searches by queue exhaustion with zero expansion-budget hits. A3.6's larger five-category lattice therefore changed the bounded-search workload enough that the frozen 30,000-expansion cap became an observed termination condition for 3 connectors. This observation does **not** authorize increasing `max_expansions` inside A3.6.

## 10. Resource Evidence

Harness-internal metrics:

```text
wall_seconds        = 3066.34 s
user_seconds        = 3064.05 s
system_seconds      = 0.295 s
cpu_percent_approx  = 99.94 %
max_rss_kb          = 93748 kB
```

External `/usr/bin/time -v` metrics:

```text
elapsed             = 51:06.61
user                = 3065.64 s
system              = 2.50 s
CPU                 = 100 %
maximum RSS         = 93748 kB
```

For context, the frozen A3.5 all-40 diagnosis previously reported `20:06.31` elapsed and `82908 kB` maximum RSS. The A3.6 five-category search therefore carried a materially larger observed compute cost on this real dataset. This is an observational engineering trade-off, not a causal attribution to any single connector or rejection class.

## 11. A4 Entry Gate

G3 reported:

```text
a4_entry_gate_passed = False
witness = empty
```

The frozen A4 minimum engineering entry predicate still requires one continuous chain:

```text
segment A
 -> EXECUTABLE transition 1
segment B
 -> EXECUTABLE transition 2
segment C
```

with:

```text
>= 3 distinct segment_ids
>= 2 distinct aisle_ids
>= 2 continuous EXECUTABLE inter-segment transitions
```

The two recovered A3.6 transitions do **not** satisfy this predicate in the frozen real graph.

**A4 conclusion: BLOCKED.**

This does not imply global infeasibility; it means only that the minimum engineering entry predicate for the next maximum-feasible-coverage stage is not yet present.

## 12. Frozen Result Classification

A3.6 result classification:

```text
PARTIAL
```

Reason:

```text
2 newly executable transitions recovered
but
a4_entry_gate_passed = False
```

This is positive evidence for the narrow A3.6 hypothesis in the sense that intermediate curvature categories recovered real connector motion that the frozen three-category lattice did not recover. It is **not** sufficient evidence to enter A4.

A3.6 does not continue by automatically adding more curvature fractions, increasing expansion/path budgets, refining state resolution, relaxing goal tolerances, changing Site Boundary/Grid semantics, or adding a second backend.

## 13. Evidence Boundary / Next Development State

The following statements are frozen:

```text
A3.5: complete and frozen
A3.6 G1: VALID RED
A3.6 G2: PASS
A3.6 G3: integrity PASS, executed exactly once
A3.6 real result: PARTIAL (2 EXECUTABLE / 38 REJECTED)
A4 minimum-chain predicate: False
A4: BLOCKED
```

No A3.6 rerun is required or permitted merely to improve the outcome.

Any further connector-backend behavioral amendment must begin as a **new evidence-backed brainstorm/spec cycle**. This evidence freeze itself does not select that next amendment.
