# V25-11D Gazebo Failure Matrix

V25-11D 在 V25-11C 的 SOFTWARE_ONLY Gazebo + canonical localization + Nav2 闭环基础上验证失败路径

目标不是增加新的定位器或规划器，而是证明关键前置条件异常时 route execution 会 fail-closed，且验收脚本不会把“没有真正执行”误判成成功

## 1. 范围

本阶段固定四个确定性故障场景

| fault_case | 注入方式 | 期望结果 |
|---|---|---|
| `map_identity_mismatch` | route 使用错误 `map_hash` | route 在规划前失败，车辆不执行 |
| `localization_lost` | 执行过程中调用现有 synthetic localization LOST service | active Nav2 goal 被取消，route 失败且运动受限 |
| `planner_invalid` | 使用不存在的 planner ID | planner action 失败，车辆不执行 |
| `controller_invalid` | 使用不存在的 controller ID | controller action 失败，车辆不执行 |

V25-11D 不新增 AMCL，不新增第二个 `map -> odom` authority，不绕过 `GlobalCorrectionManager`

激光断流暂不作为本 Gate 的正式场景

原因是单纯停止 `/agt/sensors/lidar/scan` 并不能保证 Nav2 `ObstacleLayer` 立即拒绝控制

该场景应在 `agt_sensor_monitor -> safety/mission gate` 的运行时 fail-closed 链路接入 Gazebo 后再加入，避免制造“看起来有故障注入但系统仍可能继续运行”的假安全测试

## 2. 关键改动

### 2.1. Route runner 持续定位 guard

`v25_11c_route_runner.py` 仍是 V25-11C/11D 共用的 SOFTWARE_ONLY route-to-Nav2 adapter

V25-11D 增加执行期 guard

- route 开始前要求 canonical localization 为 `TRACKING`
- `localization_accepted == true`
- `pose_valid == true`
- `correction_generation >= 1`
- `map_id` 与 route 一致
- `map_hash` 与 route 一致
- planner/controller action 等待期间持续检查同一 guard
- guard 失效时取消当前 Nav2 goal
- route state 进入 `FAILED`

稳定失败类别包括

- `LOCALIZATION_NOT_READY`
- `LOCALIZATION_GUARD_FAILED`
- `PLANNER_GOAL_REJECTED`
- `PLANNER_RESULT_FAILED`
- `CONTROLLER_GOAL_REJECTED`
- `CONTROLLER_RESULT_FAILED`

### 2.2. Fault injector

`v25_11d_fault_injector.py` 不发布 canonical localization，也不发布 TF

它只调用已有服务

```text
/agt/simulation/localization/publish_lost
```

由 `synthetic_localization_evidence.py` 发布 LOST evidence，再由 `GlobalCorrectionManager` 发布 canonical `/agt/localization/status`

因此故障路径仍经过正式 authority

### 2.3. Failure acceptance

每个 case 输出独立 JSON

```text
/tmp/agt_v25_11d_map_identity_mismatch_result.json
/tmp/agt_v25_11d_localization_lost_result.json
/tmp/agt_v25_11d_planner_invalid_result.json
/tmp/agt_v25_11d_controller_invalid_result.json
```

共同 Gate 要求

- observer 输入存在
- planner action server 存在
- controller action server 存在
- route 最终进入 terminal state
- terminal state 必须是 `FAILED`
- route 从未进入 `SUCCEEDED`
- failure class 与 fault case 对应
- 未完成全部 route segments

对于开始前失败的 case，还要求车辆位移不超过 0.50 m

对于 `localization_lost`，额外要求

- fault injector 确认 `FIRED`
- canonical LOST 被观察到
- route 已经进入 RUNNING
- 已经观察到非零导航控制命令
- abort 后总位移保持在 2.50 m 以内

这组条件用于区分“真正执行后安全中止”和“系统根本没启动”

## 3. 构建

```bash
cd ~/agt_navigation_v2
source /opt/ros/humble/setup.bash

rm -rf build/agt_simulation install/agt_simulation
colcon build --symlink-install --packages-up-to agt_simulation
source install/setup.bash
```

## 4. 逐项运行

建议一次终端只运行一个 case，结束后 `Ctrl-C` 退出 Gazebo 再开始下一项

### 4.1. Map identity mismatch

```bash
ros2 launch agt_simulation gazebo_failure_validation.launch.py \
  fault_case:=map_identity_mismatch
```

结果

```bash
cat /tmp/agt_v25_11d_map_identity_mismatch_result.json
```

### 4.2. Localization LOST during execution

```bash
ros2 launch agt_simulation gazebo_failure_validation.launch.py \
  fault_case:=localization_lost
```

默认在 launch 后约 9 s 注入 LOST

如需调试时改变注入时刻

```bash
ros2 launch agt_simulation gazebo_failure_validation.launch.py \
  fault_case:=localization_lost \
  trigger_delay_s:=10.0
```

结果

```bash
cat /tmp/agt_v25_11d_localization_lost_result.json
```

### 4.3. Invalid planner ID

```bash
ros2 launch agt_simulation gazebo_failure_validation.launch.py \
  fault_case:=planner_invalid
```

结果

```bash
cat /tmp/agt_v25_11d_planner_invalid_result.json
```

### 4.4. Invalid controller ID

```bash
ros2 launch agt_simulation gazebo_failure_validation.launch.py \
  fault_case:=controller_invalid
```

结果

```bash
cat /tmp/agt_v25_11d_controller_invalid_result.json
```

## 5. 静态 contract

```bash
python3 -m pytest -q \
  src/agt_simulation/test/test_v25_11_simulation_contract.py \
  src/agt_simulation/test/test_v25_11c_navigation_contract.py \
  src/agt_simulation/test/test_v25_11d_failure_contract.py
```

## 6. V25-11D Gate

四个 result JSON 均为 `PASS` 后，本阶段才算完成

如果任一 case 为 `FAIL`，优先根据 JSON 中 `checks` 的单项失败定位，不用凭 RViz 画面猜测

V25-11E 再进入可视化与算法/配置对比，不在 11D 引入 Hybrid A*、Reeds-Shepp、BT Navigator、动态障碍或覆盖规划
