# V25-11E Observability Validation

V25-11E 在已经通过的 V25-11C happy path 与 V25-11D failure matrix 基础上增加统一运行证据层

本阶段不新增定位器、规划器、控制器或任务系统，不改变 `map -> odom` authority，也不绕过 `agt_safety`

## 1. V25-11E-1 Observability Baseline

第一阶段目标是把以下运行状态统一采集并可视化

- canonical localization `/agt/localization/status`
- route state `/agt/simulation/navigation/route_state`
- sensor health `/diagnostics`
- safety state `/agt/safety/status`
- post-safety command `/agt/safety/cmd_vel`
- Gazebo ground truth `/simulation/bunker/ground_truth`

统一 observer

```text
v25_11e_observability.py
```

输出

```text
/agt/validation/status_markers
/agt/validation/ground_truth_path
/tmp/agt_v25_11e_timeline.jsonl
/tmp/agt_v25_11e_summary.json
```

## 2. Timeline

`timeline.jsonl` 使用 schema `agt_v25_11e_timeline/v1` 的状态变化事件

记录核心事件

- observer_started
- localization
- route_state
- sensor_health
- safety
- safety_cmd_motion
- observer_stopped

只在状态变化时追加事件，不以高频传感器采样刷满日志

所有事件使用 ROS simulation time

## 3. Summary

`summary.json` 使用 schema `agt_v25_11e_summary/v1`

主要字段

- route final state
- canonical localization final state / generation / map identity
- SensorHealth final state
- Safety final state
- ground-truth pose count
- total traveled distance
- max displacement
- max post-safety command norm
- first TRACKING time
- route RUNNING time
- first non-zero command time
- route terminal time
- event counts

这些字段后续直接用于 happy path 与 V25-11D failure cases 的统一比较

## 4. RViz

专用配置

```text
rviz/gazebo_observability_validation.rviz
```

默认显示

- deterministic validation map
- Gazebo LiDAR
- AGT runtime route
- ground-truth trail
- TF
- Validation Status MarkerArray

状态文本绑定在 `base_footprint` 上方，显示 Localization / Route / Sensor / Safety / cmd_vel 与累计路程

## 5. Launch

```bash
ros2 launch agt_simulation gazebo_observability_validation.launch.py
```

该 launch 会

1. 先启动 observability observer
2. 复用 `gazebo_navigation_validation.launch.py`
3. 禁用嵌套 navigation RViz，保证单 RViz owner
4. 默认保留 V25-11C navigation acceptance
5. 启动 V25-11E observability acceptance
6. 启动 V25-11E 专用 RViz

因此 V25-11E PASS 不能掩盖 V25-11C 导航回归

## 6. Gate

结果文件

```text
/tmp/agt_v25_11c_navigation_result.json
/tmp/agt_v25_11e_observability_result.json
/tmp/agt_v25_11e_summary.json
/tmp/agt_v25_11e_timeline.jsonl
```

V25-11E-1 要求

- V25-11C navigation result = PASS
- V25-11E observability result = PASS
- MarkerArray 至少 5 个状态 marker
- ground-truth path 可用
- route = SUCCEEDED
- final canonical localization = TRACKING + accepted + pose valid
- max displacement >= 0.5 m
- post-safety non-zero command 被观察到
- first TRACKING / route RUNNING / first command / route terminal 时间全部存在
- timeline 包含六类核心事件

## 7. 后续

V25-11E-1 PASS 后再进入 V25-11E-2 comparison harness

V25-11E-2 将复用同一 summary / timeline schema 对 happy path 与选定 V25-11D failure case 做统一指标比较，不在 E-1 阶段引入算法替换或复杂 GUI
