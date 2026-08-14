# V25-12E Agricultural Route Production

Status: STARTED — R1/R4 REAL-DATA ACCEPTED; R2-R3 IMPLEMENTED CORE; R5 CONNECTOR-SPECIFIC NAVIGATION PREVIEW GATE IMPLEMENTED

Date: 2026-08-14

本文件是 V25-12E 的连续性台账

后续对话、Codex 任务或分支续作，应先读取

- `docs/v2.5/V25_12C_MAP_WORKBENCH.md`
- `docs/architecture/agricultural_route_production.md`
- `docs/interfaces/mk_mini_chassis_route_profile.md`
- `docs/interfaces/route_asset_contract.md`
- `docs/experiments/v25_12e_r5_turn_zone_fit_20260814.md`
- 本文件

## 1. V25-12C handoff baseline

真实温室 PCD operator review 已确认

```text
Ground-relative surface         usable
Ground Confidence               usable
Robust local-plane slope        usable
Hybrid Row Skeleton             usable
Row Structural Band             usable
Interior Aisle                  usable
Boundary Aisle                  usable
Aisle Centerline                usable
Vehicle Corridor                reacts to vehicle width as expected
2D / 3D Review                  usable
3D dynamic sampling             responsive
```

Workbench 仍是 Offline authoring/review client，不成为 Route truth owner

## 2. Frozen scene/platform ownership

```text
Greenhouse / 温室
  → MK-mini
  → profiles/platforms/mk_mini.yaml
  → Ackermann agricultural aisle route

GAAS open field / 农科院开阔场地
  → BUNKER
  → profiles/platforms/bunker.yaml
  → future GNSS / RTK truth benchmark + outdoor global navigation
```

`greenhouse_ackermann.yaml` 保留历史兼容，不再作为新 V25-12E greenhouse Route Asset 的 canonical platform

## 3. V25-12E production chain

```text
Navigation Map
+ Row / Aisle evidence
        ↓
Aisle Graph
        ↓
Turn Zones
+ canonical Vehicle Profile
+ Route Policy
        ↓
Vehicle-compatible Coverage Ordering
        ↓
Connector Requests
        ↓
Forward Connector
        ↓ Turn Zone reject
Zone-Fit Diagnostic
        ↓
Connector-specific Navigation Preview Gate
        ↓
  ┌─────┴──────────────┐
forward path free      forward path conflict
  ↓                    ↓
minimal revised        Reverse-aware Fallback
Turn Zone              ↓ fail
  ↓                    Smac Search Fallback
rerun/freeze R5
        ↓
Full-footprint Feasibility
        ↓
Existing READY Route Asset
        ↓
2D / 3D Route Preview
```

Shared Turn-Zone refinement remains a geometry proposal/evidence tool, but the FREE ratio of one enlarged whole-zone rectangle must not be used as a proxy for one connector path's drivable footprint

## 4. Frozen product boundaries

### Navigation Map

`navigation_map.pgm/yaml`

静态 occupancy / collision / unknown / clearance 约束

冻结 PGM/YAML 可通过 `NavigationGridEvidence` 轻量重新读取，不需要重新处理原始 PCD

### Agricultural Aisle Graph

`aisle_graph.yaml`

结构化道路 identity、centerline XYZ、宽度、endpoint、相邻结构、诊断 evidence

`aisle_count` 是实际导出的 aisle 数量；`aisle_id` 绑定 corridor diagnostic pair index，因此允许稳定 ID 缺口

### Turn Zones

`turn_zones.yaml`

只定义 connector search / turn / reverse / direction-change envelope，不是 FREE-space truth

### Turn Zone Refinement Proposal

`agt_turn_zone_refinement_proposal/v1`

只提出 revised envelope 几何和新增区域 Navigation Grid evidence，不原地修改 Turn Zone，不是 drive permission

共享矩形的新增区域可能自然包含大量垄端 OCCUPIED，因此其整体 FREE 比例只用于 operator review，不能替代 connector-specific path gate

### Vehicle Profile

唯一正式来源

`profiles/platforms/<platform>.yaml`

Workbench 临时车宽只用于 Review

### Coverage Order

`agt_agricultural_coverage_order/v1`

职责：vehicle filtering + deterministic traversal order + connector request，不生成曲线

### Forward Connector Plan

`agt_forward_connector_plan/v1`

R5 只冻结 forward-only Dubins centerline kinematic + Turn Zone evidence

它不是 footprint-safe route，也不能直接用于 READY promotion

### Forward Connector Zone-Fit Diagnostic

`agt_forward_connector_zone_fit_diagnostic/v1`

当 R5 因 Turn Zone 拒绝 forward candidate 时，量化最容易容纳的 Dubins candidate 在农业 row frame 中还需要多少 outward / inward / lateral envelope

这是 Turn Zone 几何诊断，不是 collision truth

### Forward Connector Navigation Preview Gate

`agt_forward_connector_navigation_gate/v1`

逐 connector 枚举 forward Dubins candidate，并直接对冻结 Navigation Grid 做 centerline + canonical navigation-footprint preview swept evidence

它使用当前 MK-mini `geometric_center_for_offline_preview` 参考假设，因此可以决定 forward candidate 是否值得继续，但仍不能绕过 R8 formal vehicle-READY gate

### Route Asset

继续使用既有

```text
route.yaml
route.csv
policy.yaml
feasibility_report.json
preview.geojson
```

## 5. MK-mini manufacturer baseline

Operator supplied `MK-mini User Manual V1.1.0`

Frozen manufacturer facts

```text
overall dimensions        0.840 x 0.600 x 0.310 m
mass                      50 kg
kinematics                front Ackermann + rear dual hub drive
wheelbase                 0.600 m
track width               0.517 m
wheel diameter            0.240 m
ground clearance          0.111 m
minimum turning radius    1.500 m
VCU steering soft limit   +/-34 deg
steering resolution       0.01 deg/bit
manufacturer max speed    9.7 km/h
CAN                       2.0B extended / Intel / 500 kbit/s
control + feedback        10 ms
```

Route connector uses manufacturer vehicle-level `minimum_turning_radius=1.5 m`

Do not derive radius from `34 deg`; the manual does not establish equivalent bicycle-model steering semantics

Current operational preview limits remain conservative

```text
forward 1.0 m/s
reverse 0.5 m/s
```

The exact base_footprint reference inside the 840 x 600 mm envelope and final mounted envelope still require physical measurement

Therefore

```text
planning preview          allowed
formal vehicle READY      fail-closed until reference measurement
```

## 6. Real greenhouse acceptance records

### 6.1 R1 Aisle Graph

```text
schema                    agt_agricultural_aisle_graph/v1
status                    DRAFT
frame                     map
navigation resolution     0.10 m
corridor pair count       20
aisle count               19
row direction             [0.963456, 0.267866]
nominal row spacing       1.797185 m
```

Representative interior

```text
aisle_001
length                    27.458922 m
geometric width           1.057342 m
minimum derivation width  0.45 m
longitudinal overlap      25.083555 m
status                    ACCEPTED
```

Representative boundary

```text
aisle_020
kind                      boundary
pair                      BOUNDARY_HIGH
length                    25.136239 m
geometric width           1.456873 m
longitudinal overlap      23.873834 m
status                    ACCEPTED
```

Operator confirms exported channel count matches Workbench review

`centerline_xyz` remains structural route evidence, not final controller path; downstream Route must regularize/smooth and validate footprint

### 6.2 R4 coverage-order smoke

Canonical vehicle adapter operator output

```text
platform                  mk_mini
kinematics                ackermann
navigation width          0.600 m
wheelbase                 0.600 m
minimum turning radius    1.500 m
steering limit            34 deg
planning preview ready    true
formal route ready        false
```

Real Aisle Graph ordering

```text
input aisles              19
accepted                  18
rejected                   1
connector requests        17
```

Rejected aisle

```text
aisle_005
geometric width           0.658 m
required width            0.700 m
reason                    VEHICLE_WIDTH_INFEASIBLE
```

Narrow accepted aisle requiring later R8 attention

```text
aisle_002
geometric width           0.758 m
required width            0.700 m
surplus                    0.058 m
```

The stable aisle ID gap is intentional; coverage topology may connect `aisle_003 → aisle_006` after rejected structural pairs without renumbering asset identities

R4 real-data smoke PASS for filtering / deterministic snake / connector-request generation

### 6.3 R5 forward-only greenhouse smoke

Real MK-mini result with canonical `Rmin=1.5 m`

```text
connector requests        17
ACCEPTED_CENTERLINE        0
NO_FORWARD_DUBINS_IN_TURN_ZONE 17
analytic candidates       4 or 5 per connector
```

Every connector had analytic forward Dubins candidates, but none were fully contained by the original auto Turn Zone envelopes

This was not treated as evidence that reverse motion is mandatory

### 6.4 R5 real Turn-Zone fit diagnostic

Operator real-data result

```text
connectors                17
fits current zone          0
need expansion            17
inside fraction mean       0.682
inside fraction median     0.695
inside fraction range      0.404 .. 0.906
```

Aggregate row-frame deficits

```text
outward mean              0.433 m
outward median            0.468 m
outward maximum           0.700 m
inward median             0.000 m
inward maximum            0.846 m
maximum one-axis deficit median  0.470 m
maximum one-axis deficit P90     0.686 m
```

HIGH_U is predominantly outward-limited

```text
max outward               0.700 m
max inward                0.055 m
max lateral-low           0.277 m
```

LOW_U is also mostly outward-limited, except one major inward outlier

```text
max outward               0.589 m
max inward                0.846 m
```

Key diagnostic drivers

```text
connector_001
HIGH_U
RLR
outward                   0.677 m
lateral-low               0.277 m

connector_016
LOW_U
RSR
outward                   0.361 m
inward                    0.846 m
```

Interpretation

- most connectors support the hypothesis that original `outward_extension_m=0.80` is conservative
- `connector_016` cannot be hidden by one global outward-only increase because it drives a large inward expansion
- `connector_001` also requires explicit lateral review
- do not overwrite current Turn Zones yet

Full record

`docs/experiments/v25_12e_r5_turn_zone_fit_20260814.md`

### 6.5 Shared Turn-Zone refinement real-data result

Operator ran `agt_turn_zone_refinement_proposal/v1` against the frozen greenhouse Navigation Grid

```text
turn_low_u
status                    REVIEW_REQUIRED_NAVIGATION_CONFLICT
added cells               5896
FREE                       968   / 0.164
OCCUPIED                  4904   / 0.832
UNKNOWN                     24   / 0.004
grid coverage             1.000
proposal delta            outward 0.689 / inward 0.946 m

turn_high_u
status                    REVIEW_REQUIRED_NAVIGATION_CONFLICT
added cells               3632
FREE                      1251   / 0.344
OCCUPIED                  2357   / 0.649
UNKNOWN                     24   / 0.007
grid coverage             1.000
proposal delta            outward 0.800 / inward 0.155 / lateral-low 0.377 m
```

Interpretation correction

These numbers do not prove the forward Dubins paths collide

The shared LOW_U/HIGH_U Turn Zone rectangles span many aisle endpoints; enlarging the whole rectangle naturally includes crop-row ends that the Navigation Map correctly marks OCCUPIED

Therefore whole-zone added FREE ratio is too conservative to decide one connector's path feasibility

The next evidence gate must evaluate each Dubins path and its preview vehicle envelope directly, not the whole shared rectangle

### 6.6 Connector-specific Navigation Preview Gate

Implemented schema

```text
agt_forward_connector_navigation_gate/v1
```

For every R4 connector request it

```text
enumerates LSL / RSR / LSR / RSL / RLR / LRL
→ samples each candidate
→ restores MK-mini canonical navigation-footprint preview envelope
→ sweeps the preview footprint over frozen navigation_map.pgm/yaml
→ measures FREE / OCCUPIED / UNKNOWN / out-of-grid evidence
→ prefers a preview-free candidate
→ uses zone expansion and length only after navigation evidence
```

Default preview adds explicit 0.05 m footprint padding

The output remains DRAFT because the exact real `base_footprint` reference is not yet measured

## 7. Planner policy

Coverage baseline

```text
Deterministic Boustrophedon / Snake
```

R4 normal aisle traversal semantics

```text
motion_direction = FORWARD

graph_orientation =
  WITH_ROW_DIRECTION
  or
  AGAINST_ROW_DIRECTION
```

`AGAINST_ROW_DIRECTION` does not mean reverse gear

True `REVERSE` is reserved for R6 connector fallback

Frozen planner fallback wording

```text
Dubins / Dubins-CC forward first
        ↓ fail
Reeds-Shepp reverse fallback
        ↓ fail
Smac Hybrid-A* / State Lattice search fallback
```

Current R5 implementation is analytic classical Dubins forward-only

Dubins-CC remains a replaceable forward backend candidate and is not claimed implemented by the current R5 core

Before a R5 Turn-Zone reject is sent into R6, connector-specific Navigation Grid evidence must distinguish search-envelope insufficiency from an actually blocked forward path

Only unresolved physically forward-infeasible connectors enter R6 Reeds-Shepp reverse fallback

Every candidate must eventually pass formal full-footprint / occupancy / semantic / turn-zone / kinematic gates

## 8. Implementation ledger

### R1 — Aisle Graph deterministic export

Status: IMPLEMENTED / REAL-DATA ACCEPTANCE PASS / AUTOMATED GATE PENDING FINAL RERUN

Schema: `agt_agricultural_aisle_graph/v1`

### R2 — Turn Zone core

Status: IMPLEMENTED CORE / REAL ASSET GENERATED / SHARED-RECTANGLE FREE-RATIO NOT USED AS PATH GATE

```text
src/agt_offline_assets/agt_offline_assets/turn_zones.py
src/agt_offline_assets/test/test_turn_zones.py
```

Schema: `agt_turn_zones/v1`

LOW_U / HIGH_U endpoint envelopes are search envelopes, not free-space truth

### R3 — Canonical Vehicle Profile adapter

Status: IMPLEMENTED CORE / MK-MINI MANUFACTURER SPEC FROZEN / REAL ADAPTER OUTPUT CONFIRMED

```text
profiles/platforms/mk_mini.yaml
src/agt_offline_assets/agt_offline_assets/vehicle_profile.py
src/agt_offline_assets/test/test_vehicle_profile.py
docs/interfaces/mk_mini_chassis_route_profile.md
```

Current behavior

- reads canonical platform YAML and hashes exact file
- exposes navigation footprint, wheelbase, track, wheel diameter, ground clearance, minimum turning radius, steering interface limit and speed policy when present
- separates `planning_preview_ready` from formal `route_feasibility_ready`
- Ackermann remains no-in-place-rotation
- MK-mini now has verified manufacturer `Rmin=1.5 m`

### R4 — Vehicle-aware deterministic coverage ordering

Status: IMPLEMENTED CORE / GREENHOUSE REAL-DATA SMOKE PASS / AUTOMATED GATE PENDING FINAL RERUN

```text
src/agt_offline_assets/agt_offline_assets/agricultural_coverage_ordering.py
src/agt_offline_assets/test/test_agricultural_coverage_ordering.py
tests/test_v25_12e_coverage_ordering_contract.py
```

Schema: `agt_agricultural_coverage_order/v1`

Current behavior

- filters aisle by canonical vehicle navigation width + explicit side clearance
- optional boundary-aisle inclusion policy
- deterministic lateral ordering
- Boustrophedon alternating orientation
- every normal aisle traversal remains forward vehicle motion
- emits `turn_low_u` / `turn_high_u` ConnectorRequest objects
- does not synthesize connector curves

### R5 — Forward connector + connector-specific Navigation decision gate

Status: IMPLEMENTED CORE / REAL ZONE-FIT + SHARED-ZONE NAVIGATION RESULT RECORDED / CONNECTOR NAVIGATION GATE LOCAL ACCEPTANCE PENDING

```text
src/agt_offline_assets/agt_offline_assets/forward_connector.py
src/agt_offline_assets/agt_offline_assets/forward_connector_diagnostics.py
src/agt_offline_assets/agt_offline_assets/navigation_grid.py
src/agt_offline_assets/agt_offline_assets/turn_zone_refinement.py
src/agt_offline_assets/agt_offline_assets/forward_connector_navigation_gate.py
src/agt_offline_assets/agt_offline_assets/route_diagnostic_io.py
```

Schemas

```text
agt_forward_connector_plan/v1
agt_forward_connector_zone_fit_diagnostic/v1
agt_turn_zone_refinement_proposal/v1
agt_forward_connector_navigation_gate/v1
```

Current behavior

- Ackermann forward-only analytic Dubins backend
- enumerates LSL / RSR / LSR / RSL / RLR / LRL
- uses canonical MK-mini `Rmin=1.5 m`
- current original Turn Zones reject all 17 connector requests
- real zone deficit distribution is frozen in experiment record
- frozen navigation PGM/YAML can be loaded without rerunning PCD
- whole shared-zone refinement result is recorded but not treated as connector path collision truth
- connector-specific gate sweeps canonical navigation-footprint preview along every Dubins candidate
- candidate selection prioritizes navigation evidence before zone deficit / path length
- no automatic Turn Zone mutation
- no formal vehicle-READY safety claim

### R6 — Reverse fallback

Status: NOT STARTED — WAITING FOR CONNECTOR-SPECIFIC FORWARD NAVIGATION GATE

### R7 — Smac search fallback adapter

Status: NOT STARTED

### R8 — Full-footprint / kinematic feasibility

Status: FOUNDATION EXISTS

### R9 — Existing Route Asset generation

Status: FOUNDATION EXISTS, AGRICULTURAL INPUT NOT WIRED

### R10 — 2D / 3D Route Preview

Status: 3D REVIEW SUBSTRATE EXISTS, ROUTE OVERLAY NOT WIRED

## 9. Continuity rule

每完成一个 increment 必须同时

1. 更新本文件 Status / ledger
2. 更新直接受影响 architecture/contract 文档
3. 增加 unit/repository contract test
4. 区分 IMPLEMENTED / LOCAL ACCEPTANCE PENDING / PASS
5. 记录 real-data operator review 与剩余问题
6. 场景切换只能切 canonical platform/profile，不允许复制车辆真值进 Mission/BT

## 10. Current next action

```text
Run forward_connector_navigation_gate automated tests
→ load frozen greenhouse coverage_order.yaml + turn_zones.yaml + navigation_map.pgm/yaml
→ evaluate all 17 connectors with MK-mini preview swept footprint
→ inspect PREVIEW_FOOTPRINT_FREE vs NO_FORWARD_PREVIEW_FREE_CANDIDATE
→ for preview-free forward connectors, derive the minimal revised Turn Zone that contains the selected safe candidate
→ only unresolved physically forward-infeasible connectors enter R6 Reeds-Shepp reverse fallback
```
