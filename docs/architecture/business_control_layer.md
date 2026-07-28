# 业务总控层

业务总控层是多个 ROS 2 manager 的逻辑组合：

| owner | 权威状态与操作 | 正式接口 |
| --- | --- | --- |
| `agt_system_manager` | 模式、进程、健康、readiness、RobotState | `/agt/system/*` |
| `agt_mission_manager` | 有限顺序 Mission、暂停恢复、事件和审计 | `/agt/missions/*` |
| `agt_map_manager` | 不可变地图 registry、活动地图上下文 | `/agt/maps/*` |
| `agt_experiment_manager` | 实验、显式 bag profile、录制和回放 | `/agt/data/bags/*` |

`RobotState` 是可替换界面的统一读模型，不是新的写入 owner。聚合器只消费权威输出；重启后
未收到新证据的字段保持 `UNKNOWN`，不以 topic 发现或节点存在推断健康。地图身份只消费
`agt_map_manager` 发布的活动上下文。

Mission manager 只编排项目 `ExecuteWaypointTask` 和有限等待，不调用 Nav2 原生 Action，不发布
速度，不启动 launch。暂停导航时先请求取消子 Action并等待确认；恢复前重新检查活动地图、
定位和 `TaskReadiness`。进程重启时持久化的活动执行变为 `INTERRUPTED`，不会自动重放。

状态 topic 使用 reliable、transient-local、depth 1，使重启后的前端能立即得到最后一个明确
快照；每个快照仍携带时间戳和 freshness，latched 数据本身不表示仍然新鲜。事件 topic 使用
reliable volatile，避免旧外部事件被新 WAIT_EVENT 步骤消费。
