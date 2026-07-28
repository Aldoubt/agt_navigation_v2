# 可解释导航与未来任务编排边界

## 目标

导航应是一个有明确输入、结果、失败原因和取消语义的能力模块，而不是 GUI 内的一段流程。
当前 Qt、多点任务、未来 Web 前端、自启动任务以及机械臂流程都应复用同一组项目 Action，
避免各前端分别用 topic、距离阈值或定时器猜测任务状态。

```text
Qt / Web / autostart
        |
        v
future mission orchestrator (workflow state + audit)
        |                         |
        v                         v
navigation Actions          future arm Actions
        |
        v
Nav2 -> agt_safety -> chassis
```

第一版 `agt_mission_manager` 实现有限顺序编排。它只支持版本化 waypoint task、有限时长等待
和带 timeout 的结构化事件等待；底层 `/agt/navigation/execute_waypoint_task` 继续作为稳定能力边界。

## 模式合同

| 模式 | 默认前端 | 坐标系 | 允许能力 |
| --- | --- | --- | --- |
| mapping | RViz；Qt 默认关闭 | `odom` | 采集、点云/二维地图监视、底图编辑兼容、手动输入；禁止导航任务执行 |
| navigation | Qt 默认开启 | `map` | READY 底图只读、Task Library 任务编排、重定位、单点/多点 Nav2 Action、状态与诊断 |
| annotation | 项目语义编辑器 | `map` | 编辑、校验、离线预览；禁止运动 |
| future mission | 可替换前端 | `map` + arm frames | 只通过版本化 Action 编排导航、作业和恢复 |

建图需要 Qt 监视时显式传入 `start_mapping_gui:=true`。mapping profile 同时在 UI 和 ROS2
channel 检查 `EnableTaskExecution=false`；这用于能力隔离，但不替代 `agt_safety`。
导航点的人工输入是一个带位置和 yaw 的完整 pose：Qt 先点位置、再点朝向，未完成的
两点交互不得进入任务文件或 Action。代价地图仍由 Nav2 拥有；Qt 大地图 profile 默认不全量渲染，
规划调试通过 RViz 或显式开启。

任务中相邻 pose 表示有序规划目标，不表示机器人必须沿两点直线运动。保存阶段只验证端点所在的
基础地图栅格；点间可达性由 Task Library 的 planner-only 预览或运行时 Nav2 Action 判断，因此
障碍物遮挡显示连线不能成为任务文件保存失败的理由。

navigation/offline profile 的任务中心只保留版本化 Task Library，并隐藏旧拓扑任务保存入口。
两者也隐藏底图编辑、保存和另存为，避免把加载动作误当成对 READY PGM/YAML 的写权限。
任务保存只写入当前版本的 `tasks/`；需要修图时回到 mapping 数据链，完成后登记新的不可变地图版本。

## 当前导航能力

- 单点兼容入口：`/goal_pose`，由桥接节点转换为 Nav2 `NavigateToPose`；当前不经过
  `TaskReadiness`、active map identity 或 chassis-connected 共享门禁，只能作为兼容/调试入口；
- 多点稳定入口：`/agt/navigation/execute_waypoint_task`；
- 多点执行权：项目 server 调用 Nav2 `FollowWaypoints`，以 Action 状态和 missed waypoint
  决定结果；
- 运动输出：只能经过 Nav2、Collision Monitor、`agt_safety` 和底盘 watchdog；
- 解释信息：Action feedback/result 与 `/agt/navigation/task_status`，必须保留拒绝、取消、
  安全状态丢失、地图越界和 Nav2 失败原因。

后续应逐步淘汰 `/goal_pose` 作为自动流程接口，但在人工单点操作兼容期继续保留。

## 未来任务编排原则

未来 mission goal 应引用版本化任务文件或内嵌步骤，并为每一步记录：任务 ID、步骤 ID、输入
快照哈希、开始/结束时间、Action 结果、重试次数、安全状态和人工干预。第一版只支持有限、
顺序步骤，不实现任意脚本或无限循环。

推荐步骤类型逐步增加，而不是一次设计完整 DSL：

1. `NAVIGATE_WAYPOINTS`：调用现有项目导航 Action；
2. `WAIT`：有上限、可取消的等待；
3. `ARM_TASK`：未来调用机械臂项目 Action；
4. `VERIFY`：检查传感器或作业结果，不直接控制执行器；
5. `RECOVERY`：显式、有限且可审计的恢复分支。

编排器不得发布底盘或机械臂速度。它只拥有父 Action 状态，必须等待子 Action 接受取消后再
进入下一状态。进程重启后的策略必须明确为“恢复前人工确认”或“从安全检查点恢复”，不能
默认重复执行可能有副作用的机械臂步骤。

## 底盘与机械臂互锁预留

机械臂接入前至少定义以下跨域条件：

- 基座任务已成功到位，且定位/TF 在规定时间内有效；
- Nav2 子目标已结束或取消确认，底盘速度持续低于阈值；
- 底盘和机械臂分别有独立安全状态、急停和 watchdog；
- 机械臂展开后对底盘 footprint、重心和允许运动状态的影响显式建模；
- 任一安全状态丢失时，编排器取消活动子 Action，但最终停车仍由各执行域安全链保证；
- 重试不得重复抓取、喷洒或开关等有副作用动作，除非步骤声明了幂等策略。

## 升级顺序

1. 完成 Qt 多点 Action 的成功、失败、取消和急停实车验收。
2. 给导航结果增加统一运行记录，而不改变 Action 字段。
3. 新增最小 mission orchestrator，仅编排导航与有限等待。
4. 定义机械臂独立 Action 和安全合同，先用模拟器接入。
5. 增加跨域互锁、断电恢复和任务审计，再进入实机联合流程。

这样前端可以替换、导航算法可以从 Nav2 当前配置升级、机械臂可以独立演进，同时执行和
安全责任仍然清晰。
