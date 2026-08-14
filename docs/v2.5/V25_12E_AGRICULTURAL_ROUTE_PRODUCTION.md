# V25-12E Agricultural Route Production

Status: STARTED — R1 IMPLEMENTED / LOCAL ACCEPTANCE PENDING

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
Boundary Aisle                  implemented / local review ongoing
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

### Turn Zones

```text
turn_zones.yaml
```

职责：限制允许调头、倒车和方向切换的空间区域

### Vehicle Profile

正式来源只有

```text
profiles/platforms/<platform>.yaml
```

Workbench 输入的临时车宽只用于 Review

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

## 6. Implementation ledger

### R1 — Aisle Graph deterministic export

Status: IMPLEMENTED / LOCAL ACCEPTANCE PENDING

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

Current behavior

- Interior aisle 与 Boundary aisle identity 可区分
- accepted corridor centerline 从 grid evidence 恢复为 ordered XYZ polyline
- polyline 沿 Hybrid Row 正向 `u` 确定性排序
- start/end pose 使用相同 row-direction yaw
- aisle identity 绑定 corridor diagnostic pair index，避免前序 aisle 状态变化导致整体重编号
- width / length / longitudinal overlap / safe-cell / centerline-cell evidence 进入 YAML
- 同一输入按固定 rounding / sampling 生成 deterministic output
- Workbench 导出不写入用户 workspace 绝对路径
- Aisle Graph 明确保持 DRAFT，不修改 Navigation Map、Corridor evidence 或 READY Route Asset

Local acceptance still required

- unit/repository tests PASS
- 在真实 greenhouse `processed.pcd` 上导出 YAML
- 对照 2D / 3D aisle centerline 检查 aisle 数量、Boundary aisle、start/end、XYZ 高度
- 检查 DRAFT YAML 后再进入 R2/R3

### R2 — Turn Zone authoring/export

Status: NOT STARTED

### R3 — Canonical Vehicle Profile adapter

Status: NOT STARTED

只读取 `profiles/platforms/<platform>.yaml`，不复制 vehicle truth

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

## 7. Continuity rule

每完成一个 R increment 必须同时

1. 更新本文件 Status / ledger
2. 更新直接受影响的 architecture/contract 文档
3. 增加 unit 或 repository contract test
4. 明确 `IMPLEMENTED`、`LOCAL ACCEPTANCE PENDING`、`PASS` 三种状态，不提前宣称
5. 记录 operator real-data review 的结论与剩余问题

## 8. Current next action

```text
Local pytest for R1
→ Workbench real greenhouse aisle_graph.yaml export
→ inspect aisle count / boundary aisles / XYZ / start-end poses
→ mark R1 PASS only after operator review
→ start R2 Turn Zone + R3 Vehicle Profile adapter
```
