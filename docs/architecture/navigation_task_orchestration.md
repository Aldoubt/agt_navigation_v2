# 可解释导航与 Mission 编排边界

## 目标

导航应是一个有明确输入、结果、失败原因和取消语义的能力模块，而不是 GUI 内的一段流程。
Qt、Web、自动任务和未来机械臂流程都复用项目级 Action，避免各前端分别用 topic、距离阈值或
定时器猜测任务状态。

```text
Qt / Web / autostart
        |
        v
agt_mission_manager
Mission Action / state / audit owner
        |
        +---- current finite FSM steps
        |
        +---- P0 BehaviorTree.CPP execution engine
        |        (implementation layer, not a second owner)
        |
        +-------------------------+
        v                         v
navigation project Actions   future arm project Actions
        |
        v
Nav2 -> agt_safety -> chassis
```

当前 `agt_mission_manager` 已实现有限顺序编排，只支持版本化 waypoint task、有限时长等待和带
timeout 的结构化事件等待；底层 `/agt/navigation/execute_waypoint_task` 继续作为稳定导航能力边界。
V2.5 P0 的 BehaviorTree.CPP/Groot2 工作应接入该 manager 后端，不创建第二个 Mission Action/state
owner。

## 模式合同

| 模式 | 默认前端 | 坐标系 | 允许能力 |
| --- | --- | --- | --- |
| mapping | RViz；Qt 默认关闭 | `odom` | 采集、点云/二维地图监视、底图编辑兼容、手动输入；禁止导航任务执行 |
| navigation | Qt 默认开启 | `map` | READY 底图只读、Task Library、重定位、项目导航 Action、状态与诊断 |
| annotation | 项目语义编辑器 | `map` | 编辑、校验、离线预览；禁止运动 |
| mission | Qt/Web/自动前端均可 | `map` + future arm frames | 只通过版本化项目 Action 编排导航、等待、未来作业和恢复 |

建图需要 Qt 监视时显式传入 `start_mapping_gui:=true`。mapping profile 同时在 UI 和 ROS2
channel 检查 `EnableTaskExecution=false`；这用于能力隔离，但不替代 `agt_safety`。
导航点的人工输入是一个带位置和 yaw 的完整 pose：Qt 先点位置、再点朝向，未完成的两点交互
不得进入任务文件或 Action。代价地图仍由 Nav2 拥有；Qt 大地图 profile 默认不全量渲染，规划
调试通过 RViz 或显式开启。

任务中相邻 pose 表示有序规划目标，不表示机器人必须沿两点直线运动。保存阶段只验证端点所在的
基础地图栅格；点间可达性由 Task Library 的 planner-only 预览或运行时 Nav2 Action 判断，因此
障碍物遮挡显示连线不能成为任务文件保存失败的理由。

navigation/offline profile 的任务中心只保留版本化 Task Library，并隐藏旧拓扑任务保存入口。
两者也隐藏底图编辑、保存和另存为，避免把加载动作误当成对 READY PGM/YAML 的写权限。
任务保存只写入当前版本的 `tasks/`；需要修图时回到 mapping 数据链，完成后登记新的不可变地图版本。

## 当前导航能力

- 单点兼容入口：`/goal_pose`，由桥接节点转换为 Nav2 `NavigateToPose`；当前不经过完整
  TaskReadiness/map-binding 门禁，只能作为兼容/调试入口；
- 多点稳定入口：`/agt/navigation/execute_waypoint_task`；
- 多点执行权：项目 server 调用 Nav2 `FollowWaypoints`，以 Action 状态和 missed waypoint
  决定结果；
- 任务权威：`agt_navigation` Task Registry 根据 map/task ID 读取
  `runtime/maps/<map_id>/versions/<map_version_id>/tasks/`，Qt/Web 不提交执行文件路径；
- 会话权威：`/agt/navigation/session_status` 与 `/agt/navigation/session/get` 保留当前或最近一次
  waypoint 会话，GUI 断联不取消任务，重连后按机器人端状态恢复；
- 运动输出：只能经过 Nav2、Collision Monitor、`agt_safety` 和底盘 watchdog；
- 解释信息：Action feedback/result、结构化 blocker code、operator/technical message 与 deprecated
  `/agt/navigation/task_status`，必须保留拒绝、取消、安全状态丢失、地图越界和 Nav2 失败原因。

后续应逐步淘汰 `/goal_pose` 作为自动流程接口，但在人工单点操作兼容期继续保留。

## 当前 Mission Manager 与 P0 BT 边界

`agt_mission_manager` 已经是项目 Mission Action/state owner。P0 BT 不是重新做一个 manager，而是
增加一种可视化、可组合、可监控的执行引擎：

```text
ExecuteMission goal
       ↓
agt_mission_manager
       ↓
mission validation / audit / cancellation ownership
       ↓
BehaviorTree executor
       ↓
BT Conditions + project Action wrappers
       ↓
ExecuteWaypointTask / Relocalize / ChangeSystemMode / future actions
```

第一棵 BT 只需要有限、可取消的 waypoint mission：

1. 检查 TaskReadiness；
2. 切换/确认 navigation mode；
3. 定位有效则继续，否则调用项目 `Relocalize`；
4. 调用项目 `ExecuteWaypointTask`；
5. 将 child Action 的 SUCCESS/FAILURE/CANCELED 映射回 Mission result。

BT 不订阅 raw sensor 决策，不直接调用 Nav2 native Action，不发布速度或 TF。Groot2 只负责树编辑、
运行可视化和调试，不成为运行时业务状态 owner。

## Mission 步骤演进原则

步骤类型逐步增加，不一次设计任意 DSL：

1. `NAVIGATE_WAYPOINTS`：调用现有项目导航 Action；
2. `WAIT`：有上限、可取消的等待；
3. `ARM_TASK`：未来调用机械臂项目 Action；
4. `VERIFY`：检查传感器或作业结果，不直接控制执行器；
5. `RECOVERY`：显式、有限且可审计的恢复分支。

编排器不得发布底盘或机械臂速度。它拥有父 Mission Action 状态，必须等待子 Action 接受取消后再
进入下一状态。进程重启后的策略必须明确为“恢复前人工确认”或“从安全检查点恢复”，不能默认
重复执行可能有副作用的机械臂步骤。

## 底盘与机械臂互锁预留

机械臂接入前至少定义以下跨域条件：

- 基座任务已成功到位，且定位/TF 在规定时间内有效；
- Nav2 子目标已结束或取消确认，底盘速度持续低于阈值；
- 底盘和机械臂分别有独立安全状态、急停和 watchdog；
- 机械臂展开后对底盘 footprint、重心和允许运动状态的影响显式建模；
- 任一安全状态丢失时，编排器取消活动子 Action，但最终停车仍由各执行域安全链保证；
- 重试不得重复抓取、喷洒或开关等有副作用动作，除非步骤声明了幂等策略。

## 升级顺序

1. 完成当前 waypoint Action 的成功、失败、取消和急停实车验收；
2. 完成 V2.5 P0 sensor health、semantic waypoint 等首个 BT 前置能力；
3. 在现有 `agt_mission_manager` 后端接入最小 BT executor，只编排 readiness、重定位、模式切换和 waypoint task；
4. 使用 Groot2 完成运行状态观测和失败/取消验收；
5. 定义机械臂独立 Action 和安全合同，先用模拟器接入；
6. 增加跨域互锁、断电恢复和任务审计，再进入实机联合流程。

这样前端可以替换、导航算法可以独立升级、机械臂可以独立演进，同时 Mission、执行和安全责任仍然清晰。
