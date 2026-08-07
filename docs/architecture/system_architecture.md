# v2.5 系统架构与交付路线

本文是当前架构边界和 V25-08 语义基线。运行时接口的名称、类型和 owner 以
[`topic_contract.md`](../interfaces/topic_contract.md) 为准；未来能力不因出现在
架构图中而成为已实现能力。

## 四层架构总图

```mermaid
flowchart TD
  subgraph M[Mission Plane]
    UI[Qt / Web / CLI]
    MM[agt_mission_manager\n唯一 Mission state owner]
    BT[BT.CPP backend\nproject capability Actions only]
    UI --> MM --> BT
  end

  subgraph N[Navigation Capability Plane]
    CAP[Navigation Capability]
    MAP[MAP\nNav2 global planner + controller]
    ROUTE[ROUTE\nreserved: route resolver + path follower]
    LOCAL[LOCAL\nreserved: local target + controller]
    CAP --> MAP
    CAP --> ROUTE
    CAP --> LOCAL
    BT -->|ExecuteWaypointTask| CAP
  end

  subgraph S[State Estimation / Map Knowledge Plane]
    G[Global Navigation Map\n2D OccupancyGrid]
    P[Localization Prior\nlocalization_map.pcd]
    SEM[Semantic Map\nGeoJSON / named anchors]
    LIO[FAST-LIVO2 adapter\n唯一 odom -> base_footprint]
    AUTH[Localization Authority\nagt_localization only]
    CORR[NDT/ICP/GTSAM/GNSS/loop evidence\nnot TF owners]
    LIO --> AUTH
    CORR --> AUTH
    AUTH -->|authoritative map -> odom| MAP
    G --> MAP
    SEM --> CAP
    P --> CORR
  end

  subgraph C[Local Perception / Control Plane]
    CLOUD[/agt/mapping/registered_points]
    OCC[Local Occupancy\nrolling, transient]
    ESDF[Optional ESDF]
    SAFE[Collision Monitor -> agt_safety]
    CH[agt_chassis]
    CLOUD --> OCC --> ESDF
    MAP --> SAFE --> CH
  end

  LIO --> CLOUD
  OCC -. future local cost .-> MAP
```

架构中只有 `agt_localization` 可以发布 authoritative `map -> odom`，只有
FAST-LIVO2 adapter 可以发布 `odom -> base_footprint`。NDT/ICP relocalization、
GTSAM/iSAM2、GNSS backend、loop closure 和 place recognition 只能提供 evidence、
factor、candidate transform 或 pose constraint。

Global Navigation Map、Localization Prior 和 Semantic Map 是三个不同产品。
Local Occupancy 向 Optional ESDF 的关系是派生关系；ESDF 不是 P1 默认主链。

## 当前与目标状态

| Capability / product | Status | 说明 |
| --- | --- | --- |
| MAP-oriented Nav2 navigation | SYSTEM-INTEGRATED | 当前稳定导航基线，通过 `ExecuteWaypointTask` 使用 |
| P0 BT Mission | SYSTEM-INTEGRATED | BT 是 Mission Manager backend，不是第二个 Mission owner |
| ROUTE | RESERVED | V25-09 目标；当前没有 route runtime |
| LOCAL | RESERVED | 当前没有 local-target runtime |
| Local Environment Mapping | RESERVED | V25-11 目标；当前 local occupancy 不是已实现 runtime 产品 |
| Sparse Global Correction | RESERVED | V25-10 目标；当前 correction evidence 不拥有 TF |
| ESDF | OPTIONAL | 仅在 Local Occupancy 之后按需派生 |

## 交付路线

```text
P0   First BT Mission                         PASS software integration; vehicle validation separate
V25-08 Architecture & Semantics Baseline
V25-09 Robust Route Navigation Core
V25-10 Sparse Global Correction / Anchor Recovery
V25-11 Local Environment Mapping
V25-12 Optional ESDF / Advanced Local Planning
```

GNSS、Wheel、GTSAM 保持 P1 state-estimation track；STD、Scan Context 和 seasonal
maps 保持 P2 research track。

## Runtime ownership summary

- `agt_localization` uniquely owns authoritative `map -> odom` correction.
- `agt_mapping_fast_livo2_adapter` uniquely publishes `odom -> base_footprint`.
- `robot_state_publisher` owns the robot/sensor description chain below `base_footprint`.
- `agt_mission_manager` remains the single project Mission Action/state owner.
- BT nodes do not publish velocity or TF and do not call Nav2 native Actions directly.
- Motion remains `Nav2 -> collision/safety chain -> agt_safety -> agt_chassis`.
- `/agt/mapping/registered_points` is the sole canonical registered-cloud topic;
  historical names remain forbidden in runtime configuration.
