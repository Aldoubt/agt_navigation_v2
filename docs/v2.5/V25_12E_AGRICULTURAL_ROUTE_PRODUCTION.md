# V25-12E Agricultural Route Production

Status: STARTED — R1 REAL-DATA ACCEPTED / AUTOMATED GATE PENDING; R2-R3 IMPLEMENTED CORE / LOCAL ACCEPTANCE PENDING

Date: 2026-08-14

本文件是 V25-12E 的连续性台账

任何后续对话、Codex 任务或分支续作，都应先读取

- `docs/v2.5/V25_12C_MAP_WORKBENCH.md`
- `docs/architecture/agricultural_route_production.md`
- `docs/interfaces/route_asset_contract.md`
- 本文件

再继续实现

## 1. V25-12C handoff baseline

真实温室 PCD 的当前 operator review 已确认

```text
Ground-relative surface         usable
Ground Confidence               usable
Robust local-plane slope        usable
Hybrid Row Skeleton             usable
Row Structural Band             usable
Interior Aisle                  usable
Boundary Aisle                  usable in current greenhouse review
Aisle Centerline                usable
Vehicle Corridor                reacts to vehicle width as expected
2D / 3D Review                  usable
3D interaction preview          responsive with dynamic sampling
```

当前 Map Workbench 继续是 Offline authoring/review client，不成为 Route truth owner

## 2. V25-12E goal

把 V25-12C 已恢复的农业结构从“可视化证据”升级为可审计、可规划、可验证的 Route Asset 上游输入

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
Coverage Ordering
        ↓
Kinematic Connector Planning
        ↓
Full-footprint Feasibility
        ↓
Existing READY Route Asset
        ↓
2D / 3D Route Preview
```

## 3. Non-goals

V25-12E 不做

- 运行时绕过 Route Asset 直接从 Workbench 发 Nav2 goal
- Workbench 直接发布速度
- 创建第二份 vehicle geometry truth
- 用一张漂亮的中心线图替代 footprint / kinematic validation
- 把 Fields2Cover、Smac 或任一 planner 名称写死到 Mission / BT 语义
- 修改 READY Route revision 原文件

## 4. Frozen product boundaries

### Navigation Map

```text
navigation_map.pgm
navigation_map.yaml
```

职责：静态 occupancy / collision / unknown / clearance 约束

### Agricultural Aisle Graph

```text
aisle_graph.yaml
```

职责：结构化道路 identity、centerline XYZ、宽度、endpoint、相邻结构、诊断证据

`aisle_count` 是实际导出的 aisle 数量

`aisle_id` 绑定 corridor diagnostic pair index，因此允许存在稳定 ID 缺口，例如 19 条有效 aisle 中最后一个 ID 为 `aisle_020`

这不是计数错误，而是为了避免某一个前序 aisle 失效后其余 identity 集体漂移

### Turn Zones

```text
turn_zones.yaml
```

职责：限制 connector 允许搜索、调头、倒车和方向切换的区域

Turn Zone 是 search envelope，不是 FREE-space truth

所有 connector 仍必须重新通过 Navigation Map + full-footprint feasibility

### Vehicle Profile

正式来源只有

```text
profiles/platforms/<platform>.yaml
```

Workbench 输入的临时车宽只用于 Review

Ackermann 未验证 minimum turning radius 时必须 fail-closed

### Route Asset

继续使用既有正式合同

```text
route.yaml
route.csv
policy.yaml
feasibility_report.json
preview.geojson
```

## 5. Planner policy

Coverage Ordering baseline

```text
Deterministic Boustrophedon / Snake
```

Backend target

```text
Fields2Cover / OR-tools ordering
```

Connector fallback chain

```text
Dubins / Dubins-CC forward first
        ↓ fail
Reeds-Shepp reverse fallback
        ↓ fail
Smac Hybrid-A* / State Lattice search fallback
```

每个 candidate 都必须再过 full-footprint / occupancy / semantic / turn-zone / kinematic gate

## 6. Real greenhouse R1 acceptance record

Operator export from the current greenhouse `processed.pcd`

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

Representative interior aisle

```text
aisle_001
kind                      interior
pair                      ROW_ROW
left/right                row_01 / row_02
length                    27.458922 m
geometric width           1.057342 m
minimum derivation width  0.45 m
longitudinal overlap      25.083555 m
centerline cells          225
status                    ACCEPTED
```

Representative boundary aisle

```text
aisle_020
kind                      boundary
pair                      BOUNDARY_HIGH
left/right                row_20 / boundary_high
length                    25.136239 m
geometric width           1.456873 m
minimum derivation width  0.45 m
longitudinal overlap      23.873834 m
centerline cells          250
status                    ACCEPTED
```

Operator confirms exported channel count matches the Workbench channel review

The XYZ ground heights and start/end poses are plausible against the current 2D/3D review

R1 real-data acceptance is therefore PASS

The final R1 stage status remains pending until the automated regression command is explicitly confirmed PASS

### Important centerline semantics

The exported `centerline_xyz` is structural route evidence, not final controller path

The greenhouse example contains occasional larger longitudinal sample gaps / lateral steps caused by grid evidence discontinuities

Therefore the downstream Route stage must perform deterministic longitudinal regularization/smoothing and full-footprint validation before generating a controller-consumable Runtime Path

## 7. Canonical BUNKER implication

Current canonical BUNKER profile records

```text
physical width             0.778 m
navigation footprint width 0.938 m
kinematics                 tracked_differential
```

For representative `aisle_001`

```text
1.057342 - 0.938 ≈ 0.119 m total navigation-footprint surplus
≈ 0.060 m per side before any additional route-policy clearance
```

Therefore Workbench temporary width settings must not be used as formal route acceptance

R3 canonical vehicle-profile binding is required before route ordering/connection is promoted

## 8. Implementation ledger

### R1 — Aisle Graph deterministic export

Status: IMPLEMENTED / REAL-DATA ACCEPTANCE PASS / AUTOMATED GATE PENDING

Implemented

```text
src/agt_offline_assets/agt_offline_assets/agricultural_aisle_graph.py
src/agt_offline_assets/test/test_agricultural_aisle_graph.py
tests/test_v25_12e_aisle_graph_contract.py
Workbench: 离线资产 → 导出 Aisle Graph YAML
```

Schema

```text
agt_agricultural_aisle_graph/v1
status: DRAFT
```

### R2 — Turn Zone candidate derivation/export core

Status: IMPLEMENTED CORE / LOCAL ACCEPTANCE PENDING

Implemented

```text
src/agt_offline_assets/agt_offline_assets/turn_zones.py
src/agt_offline_assets/test/test_turn_zones.py
tests/test_v25_12e_turn_zone_vehicle_profile_contract.py
```

Schema

```text
agt_turn_zones/v1
status: DRAFT
```

Current baseline derives deterministic LOW_U / HIGH_U endpoint envelopes

They are explicitly labeled `SEARCH_ENVELOPE_NOT_FREE_SPACE_TRUTH`

Manual polygon authoring / Workbench overlay is still pending

### R3 — Canonical Vehicle Profile adapter

Status: IMPLEMENTED CORE / LOCAL ACCEPTANCE PENDING

Implemented

```text
src/agt_offline_assets/agt_offline_assets/vehicle_profile.py
src/agt_offline_assets/test/test_vehicle_profile.py
tests/test_v25_12e_turn_zone_vehicle_profile_contract.py
```

Current behavior

- reads existing canonical platform YAML
- hashes exact profile file
- uses navigation footprint when present
- normalizes route-relevant kinematics/velocity fields
- tracked/differential zero-radius rotation is represented explicitly
- Ackermann route acceptance fails closed until positive minimum turning radius is verified
- does not create a second vehicle geometry YAML

### R4 — Coverage ordering

Status: NOT STARTED

首版 deterministic boustrophedon

### R5 — Forward connector

Status: NOT STARTED

### R6 — Reverse fallback

Status: NOT STARTED

### R7 — Smac search fallback adapter

Status: NOT STARTED

### R8 — Full-footprint / kinematic feasibility

Status: FOUNDATION EXISTS

复用现有 `agt_coverage_planning.path_validator` 与 route feasibility contract

### R9 — Existing Route Asset generation

Status: FOUNDATION EXISTS, NEW AGRICULTURAL INPUT NOT WIRED

### R10 — 2D / 3D Route Preview

Status: 3D REVIEW SUBSTRATE EXISTS, ROUTE OVERLAY NOT WIRED

## 9. Continuity rule

每完成一个 R increment 必须同时

1. 更新本文件 Status / ledger
2. 更新直接受影响的 architecture/contract 文档
3. 增加 unit 或 repository contract test
4. 明确 `IMPLEMENTED`、`LOCAL ACCEPTANCE PENDING`、`PASS` 三种状态，不提前宣称
5. 记录 operator real-data review 的结论与剩余问题

## 10. Current next action

```text
Run R1/R2/R3 automated tests
→ inspect canonical bunker adapter output
→ derive greenhouse turn_zones.yaml
→ add Workbench Turn Zone overlay/authoring if candidate envelopes look reasonable
→ then start R4 deterministic coverage ordering
```
