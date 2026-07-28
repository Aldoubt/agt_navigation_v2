# RobotState interface

`/agt/system/robot_state` 是界面的统一只读快照，`/agt/system/get_robot_state` 返回同一模型的
一次性读取。它包含模式、受管进程计数、`SystemHealth`、`TaskReadiness`、活动地图、
`LocalizationStatus`、`MissionStatus`、Nav2、safety、chassis 和 Bag 会话。

核心规则：

- 每个快照有 ROS 时间戳和单调 revision；默认 2 Hz，并在权威输入变化时立即发布。
- 未收到或已过 freshness 窗口的数据保持 unknown/stale，不能伪造成健康。
- 活动地图来自 `/agt/maps/active`，聚合器不读取 `active_map.yaml` 或 manifest。
- safety 的 motion enabled、急停和 navigation ready 来自权威 safety diagnostics。
- chassis connected 与 odometry freshness 分别保留；连接 topic 不代表里程计新鲜。
- Nav2 active 必须来自 lifecycle 状态证据，不以 node/topic 存在代替。
- blocker code 稳定供自动客户端使用，message 供人阅读。

状态 topic 使用 reliable transient-local depth 1，便于后启动 Qt/Web 立即绘制明确状态；客户端
仍必须检查 `header.stamp` 和对应 freshness。
