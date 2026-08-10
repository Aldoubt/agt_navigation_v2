# V25-11D Gazebo Failure Matrix

V25-11D 在 V25-11C 的 SOFTWARE_ONLY Gazebo + canonical localization + Nav2 闭环基础上验证失败路径

目标不是增加新的定位器或规划器，而是证明关键前置条件异常时 route execution 会 fail-closed，并且真正的速度链必须经过 `agt_safety`

## 1. 验证矩阵

当前固定六个确定性场景

| fault_case | 注入方式 | 期望结果 |
|---|---|---|
| `map_identity_mismatch` | route 使用错误 `map_hash` | route 在规划前失败，车辆不执行 |
| `localization_lost` | 执行过程中调用 synthetic localization LOST service | active Nav2 goal 被取消，route 失败且运动受限 |
| `planner_invalid` | 使用不存在的 planner ID | planner action 失败，车辆不执行 |
| `controller_invalid` | 使用不存在的 controller ID | controller action 失败，车辆不执行 |
| `lidar_dropout` | Gazebo sensor adapter 停止转发 LaserScan | sensor monitor ERROR，safety 立即归零，Nav2 最终失败 |
| `imu_dropout` | Gazebo sensor adapter 停止转发 IMU | sensor monitor ERROR，safety 立即归零，Nav2 最终失败 |

V25-11D 不新增 AMCL，不新增第二个 `map -> odom` authority，不绕过 `GlobalCorrectionManager`

## 2. 正式速度链

V25-11D 第二阶段把 Gazebo 导航切回生产拓扑

```text
Nav2 controller_server
        |
        v
/agt/navigation/cmd_vel
        |
        v
agt_safety/tracked_safety_controller
        |
        v
/agt/safety/cmd_vel
        |
        v
ros_gz_bridge -> BUNKER simulation
```

Nav2 不再直接发布 `/agt/safety/cmd_vel`

`agt_safety` 对导航输入执行以下 runtime gate

- emergency stop
- explicit motion enable
- navigation input freshness
- canonical localization validity
- required sensor input health
- velocity / acceleration / track-speed limits

手动输入仍保持软件仲裁优先级，传感器/定位失效不会替代硬件急停

## 3. SensorHealth 链

Gazebo 使用正式 `agt_sensor_monitor`

```text
/agt/sensors/lidar/scan ----+
                            +--> agt_sensor_monitor --> /diagnostics
/agt/sensors/imu/data ------+          |
                                       v
                          agt_sensor_monitor/summary
                                       |
                                       v
                                  agt_safety
```

SOFTWARE_ONLY 配置位于

```text
src/agt_simulation/config/v25_11d_sensor_monitor.yaml
```

其中

- LiDAR 使用 `sensor_msgs/LaserScan`
- LiDAR 标称 10 Hz，Gate 下限 8 Hz
- IMU 标称 100 Hz，Gate 下限 80 Hz
- filtered LiDAR 在该仿真场景禁用

生产 `sensor_monitor.yaml` 仍默认 `livox_custom`

`livox_ros_driver2` 改为可选 CMake backend：未安装 Livox 驱动时仍可构建 LaserScan/IMU 监控；若真机配置请求 `livox_custom` 而构建时没有 Livox 支持，节点会明确 fail-fast

`required_streams_healthy` 与启动期 diagnostic severity 分离：required stream 未实际 healthy 时该字段始终为 `false`，即使 startup grace 内 per-stream level 只是 `WARN`

## 4. Route runner 持续 localization guard

V25-11C/11D 共用 `v25_11c_route_runner.py`

route 开始前及 planner/controller action 等待期间持续要求

- canonical localization = `TRACKING`
- `localization_accepted == true`
- `pose_valid == true`
- `correction_generation >= 1`
- `map_id` 与 route 一致
- `map_hash` 与 route 一致

失效时取消当前 Nav2 goal 并进入 `FAILED`

主要 failure class

- `LOCALIZATION_NOT_READY`
- `LOCALIZATION_GUARD_FAILED`
- `PLANNER_*`
- `CONTROLLER_*`

## 5. Sensor dropout 故障注入

`gazebo_sensor_adapter.py` 只负责 SOFTWARE_ONLY 转发和故障开关，不实现健康策略

服务

```text
/agt/simulation/sensors/set_lidar_drop
/agt/simulation/sensors/set_imu_drop
```

`v25_11d_fault_injector.py` 在默认约 9 s 后调用对应服务

Dropout PASS 必须同时看到

- fault injector = `FIRED`
- route 已经进入 `RUNNING`
- 故障前出现过非零 `/agt/safety/cmd_vel`
- 对应 sensor stream diagnostic = `ERROR`
- `agt_sensor_monitor/summary` = `ERROR`
- `agt_safety/status.message == sensor_input_unhealthy`
- `sensor_input_ready=false`
- `/agt/safety/cmd_vel` 在故障后真实归零
- localization 没有被错误改成 LOST
- route 从未 `SUCCEEDED`
- route 最终 `FAILED`
- 总运动保持在验收上限内

因此测试不会把“Nav2 本来就没启动”或“只停了传感器但底盘继续走”误判为 PASS

## 6. 构建

```bash
cd ~/agt_navigation_v2
source /opt/ros/humble/setup.bash

rm -rf \
  build/agt_sensor_monitor build/agt_safety build/agt_simulation \
  install/agt_sensor_monitor install/agt_safety install/agt_simulation

colcon build --symlink-install --packages-up-to \
  agt_sensor_monitor agt_safety agt_simulation

source install/setup.bash
```

## 7. 静态 contract

```bash
python3 -m pytest -q \
  src/agt_sensor_monitor/test/test_sensor_monitor_contract.py \
  src/agt_safety/test/test_safety_controller.py \
  src/agt_simulation/test/test_v25_11_simulation_contract.py \
  src/agt_simulation/test/test_v25_11c_navigation_contract.py \
  src/agt_simulation/test/test_v25_11d_failure_contract.py
```

## 8. Happy-path regression

由于 V25-11D 第二阶段改变了命令拓扑，新增 dropout case 前必须重新确认 V25-11C

```bash
rm -f /tmp/agt_v25_11c_navigation_result.json
ros2 launch agt_simulation gazebo_navigation_validation.launch.py \
  run_acceptance:=true
```

结果

```bash
cat /tmp/agt_v25_11c_navigation_result.json
```

必须仍为 `PASS`

## 9. 六个 failure cases

```bash
ros2 launch agt_simulation gazebo_failure_validation.launch.py fault_case:=map_identity_mismatch
ros2 launch agt_simulation gazebo_failure_validation.launch.py fault_case:=localization_lost
ros2 launch agt_simulation gazebo_failure_validation.launch.py fault_case:=planner_invalid
ros2 launch agt_simulation gazebo_failure_validation.launch.py fault_case:=controller_invalid
ros2 launch agt_simulation gazebo_failure_validation.launch.py fault_case:=lidar_dropout
ros2 launch agt_simulation gazebo_failure_validation.launch.py fault_case:=imu_dropout
```

新增结果文件

```text
/tmp/agt_v25_11d_lidar_dropout_result.json
/tmp/agt_v25_11d_imu_dropout_result.json
```

调试时可改变主动注入时刻

```bash
ros2 launch agt_simulation gazebo_failure_validation.launch.py \
  fault_case:=lidar_dropout \
  trigger_delay_s:=10.0
```

## 10. V25-11D Gate

当前 Gate 分两层

1. V25-11C happy-path 在正式 Safety/SensorHealth 命令链上仍为 PASS
2. 六个 V25-11D failure case 全部 PASS

任一 case 为 FAIL 时优先看 JSON 中具体 `checks`，不要仅凭 RViz 画面判断

V25-11E 再进入可视化与算法/配置对比，不在 11D 引入 Hybrid A*、Reeds-Shepp、BT Navigator、动态障碍或覆盖规划
