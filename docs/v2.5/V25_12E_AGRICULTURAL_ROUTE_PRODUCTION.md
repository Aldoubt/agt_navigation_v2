# V25-12E Agricultural Route Production

Status: STARTED — R1 REAL-DATA ACCEPTED / AUTOMATED GATE PENDING; R2-R4 IMPLEMENTED CORE / LOCAL ACCEPTANCE PENDING

Date: 2026-08-14

本文件是 V25-12E 的连续性台账

后续对话、Codex 任务或分支续作，应先读取

- `docs/v2.5/V25_12C_MAP_WORKBENCH.md`
- `docs/architecture/agricultural_route_production.md`
- `docs/interfaces/mk_mini_chassis_route_profile.md`
- `docs/interfaces/route_asset_contract.md`
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
        ↓ fail
Reverse-aware Fallback
        ↓ fail
Smac Search Fallback
        ↓
Full-footprint Feasibility
        ↓
Existing READY Route Asset
        ↓
2D / 3D Route Preview
```

## 4. Frozen product boundaries

### Navigation Map

`navigation_map.pgm/yaml`

静态 occupancy / collision / unknown / clearance 约束

### Agricultural Aisle Graph

`aisle_graph.yaml`

结构化道路 identity、centerline XYZ、宽度、endpoint、相邻结构、诊断 evidence

`aisle_count` 是实际导出的 aisle 数量；`aisle_id` 绑定 corridor diagnostic pair index，因此允许稳定 ID 缺口

### Turn Zones

`turn_zones.yaml`

只定义 connector search / turn / reverse / direction-change envelope，不是 FREE-space truth

### Vehicle Profile

唯一正式来源

`profiles/platforms/<platform>.yaml`

Workbench 临时车宽只用于 Review

### Coverage Order

`agt_agricultural_coverage_order/v1`

职责：vehicle filtering + deterministic traversal order + connector request，不生成曲线

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

## 6. Real greenhouse R1 acceptance record

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

R1 real-data acceptance PASS; automated regression still waits explicit confirmation

`centerline_xyz` remains structural route evidence, not final controller path; downstream Route must regularize/smooth and validate footprint

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

Connector fallback target

```text
Dubins / Dubins-CC forward first
        ↓ fail
Reeds-Shepp reverse fallback
        ↓ fail
Smac Hybrid-A* / State Lattice search fallback
```

Every candidate must pass full-footprint / occupancy / semantic / turn-zone / kinematic gates

## 8. Implementation ledger

### R1 — Aisle Graph

Status: IMPLEMENTED / REAL-DATA ACCEPTANCE PASS / AUTOMATED GATE PENDING

Schema: `agt_agricultural_aisle_graph/v1`

### R2 — Turn Zone core

Status: IMPLEMENTED CORE / LOCAL ACCEPTANCE PENDING

```text
src/agt_offline_assets/agt_offline_assets/turn_zones.py
src/agt_offline_assets/test/test_turn_zones.py
```

Schema: `agt_turn_zones/v1`

LOW_U / HIGH_U endpoint envelopes are search envelopes, not free-space truth

### R3 — Canonical Vehicle Profile adapter

Status: IMPLEMENTED CORE / MK-MINI MANUFACTURER SPEC FROZEN / LOCAL ACCEPTANCE PENDING

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

Status: IMPLEMENTED CORE / LOCAL ACCEPTANCE PENDING

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

### R5 — Forward connector

Status: NOT STARTED

Must use MK-mini `Rmin=1.5 m` in greenhouse

### R6 — Reverse fallback

Status: NOT STARTED

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
Run R1-R4 automated tests
→ inspect mk_mini canonical adapter output
→ derive greenhouse Turn Zones
→ run R4 against real greenhouse aisle_graph
→ inspect eligible/rejected aisle list + snake order + connector requests
→ then start R5 forward connector
```
