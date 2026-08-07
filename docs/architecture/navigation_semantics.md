# Navigation Semantics Baseline (V25-08)

本文冻结 V25-08 的导航架构和正式语义。它只定义概念、所有权和边界，
不实现 ROUTE/LOCAL runtime，不新增 ROS Action、Service 或 topic，也不改变
现有 `ExecuteWaypointTask.action`。

## Navigation Capability

**Navigation Capability** 是业务层可以调用的导航能力。它负责把一个受约束的
导航意图转换为受安全、定位、地图和任务合同保护的执行过程。Nav2 是其中一个
内部 backend，而不是 Navigation 的同义词。

Mission、Behavior Tree 和前端只访问 project capability Action；它们不得直接
依赖 `NavigateToPose`、`NavigateThroughPoses`、`FollowPath` 或 `cmd_vel`。BT
节点不发布速度或 TF，也不实现规划、定位、感知和控制算法。

当前稳定的 project capability 是 `ExecuteWaypointTask`。WaypointTask 是任务
输入，Route 是将来的内部 resolved representation；本阶段不公开
`ExecuteRouteTask` 或 `ExecuteNavigationTask`。

## Navigation Mode

### MAP

MAP 使用：

```text
global map + continuous/available global localization
             + global planner + local controller
```

它适合任意目标导航、稳定人工环境和需要在线全局路径规划的任务。当前已实现
的 waypoint/Nav2 基线属于 MAP-oriented navigation。

### ROUTE

ROUTE 使用：

```text
prior semantic/geometric route + continuous robust odometry
                                + sparse global correction
                                + local obstacle handling
```

全局地图主要用于初始化、路线定义、localization anchor 和稀疏恢复。正常运动
不要求当前点云持续高频匹配完整旧地图，更不能设计为每个 controller tick
都用旧 PCD 匹配当前植被。ROUTE 是 V25-09 及以后实现的目标能力。

### LOCAL

LOCAL 使用：

```text
relative/local target + odometry + local perception + local controller
```

它不要求 Global Navigation Map。LOCAL 是预留能力，当前未实现。

## Map products

三个产品必须保持独立：

| Product | 定义与用途 | 不承担的职责 |
| --- | --- | --- |
| Global Navigation Map | 默认是 `2D OccupancyGrid`；表达全局自由/占用几何，供全局规划和 Semantic Map 空间参照 | 实时动态障碍真值、持续 scan-to-map localization 原始数据 |
| Localization Prior | 与全局导航栅格分离；当前可为 `localization_map.pcd`，优先表达墙、立柱、温室结构、地面、树干和固定设施 | 依赖叶片、软枝、杂草和临时物体作为稳定特征 |
| Semantic Map | 持久化领域知识，包含 field boundary、row centerline、access lane、headland、keepout、named waypoint、localization anchor | OccupancyGrid、Runtime Path 或 Mission |

Global Navigation Map 的短期动态障碍不得写回权威地图。Semantic Map 也不等于
可执行任务。

**Local Environment Map** 是短期在线感知产品，默认推荐 rolling 2D occupancy：

```text
registered cloud -> ground/obstacle separation -> raycast
                 -> log-odds -> observation timeout/decay
                 -> Local Occupancy
                 -> Optional ESDF
```

它服务局部碰撞规避、局部代价和动态障碍证据，不能成为 versioned global map
truth。ESDF 是可选派生表达，不是 P1 默认必经地图产品。

## State-estimation ownership

- FAST-LIVO2 adapter 是唯一 `odom -> base_footprint` owner。
- `agt_localization` 是唯一 authoritative `map -> odom` owner。
- NDT/ICP relocalization、GTSAM/iSAM2、GNSS backend、loop closure 和 place
  recognition 只能产生 global correction evidence、factor、candidate transform
  或 pose constraint；它们不得并列发布 authoritative `map -> odom`。

所有候选修正必须进入唯一 Localization Authority，再由 `agt_localization`
更新 `map -> odom`。

## Waypoint, task, route and path

| Term | 正式含义 |
| --- | --- |
| SemanticWaypoint | 持久化、命名的 map anchor；没有执行顺序 |
| WaypointTask / TaskGroup | 有序任务意图，定义本次目标、顺序、loop/revision/hash/map binding |
| Route | 由 TaskGroup + Semantic Map + Navigation Policy 解析出的导航意图 |
| Route Segment | Route 中可独立解析、校正和跟踪的一段 |
| Runtime Path | controller 可消费的几何轨迹，可表达在 map 或 odom；ROUTE 优先在 odom 中运行活动 segment |
| Localization Anchor | 用于初始化、稀疏校正或恢复的稳定语义/几何参照 |

因此必须保持：

```text
SemanticWaypoint != WaypointTask != Route != Runtime Path
```

ROUTE 的目标运行链为：

```text
map-frame route -> resolve current segment -> transform to odom
                -> controller tracks odom-frame Runtime Path
                -> anchor/confidence event -> sparse relocalization
                -> authoritative map->odom update -> resolve next segment
```

这只是冻结的目标语义，不代表当前 runtime 已实现。

## Backend boundary

```text
Navigation Capability
        |
        +-- MAP   -> Nav2 global planner + Nav2 controller
        +-- ROUTE -> Route Resolver + controller/path follower
        +-- LOCAL -> local target resolver + local controller
```

ROUTE backend 可以内部复用 Nav2 Controller Server、`FollowPath`、RPP 或 MPPI，
但这些不是 Mission/BT 的 public boundary。

## Readiness semantics

V25-08 只定义 mode-aware readiness 语义，不修改 `EvaluateTaskReadiness.srv`。
现有 `TASK_EXECUTION` 和 `RELOCALIZATION` 主要对应当前 MAP navigation baseline。
未来至少需要区分：

```text
MAP_START_READY       MAP_CONTINUE_READY
ROUTE_START_READY     ROUTE_CONTINUE_READY
GLOBAL_CORRECTION_READY
LOCAL_READY
```

`ROUTE_CONTINUE_READY` 不应持续要求 global map matching healthy 或近期刚更新
`map -> odom`；它必须要求 odometry、local control、安全和所需 local perception
健康。`GLOBAL_CORRECTION_READY` 单独判断是否具备重定位条件。
