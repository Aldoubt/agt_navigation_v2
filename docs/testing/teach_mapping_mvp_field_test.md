# Teach Mapping MVP 现场测试说明

本流程用于验证一个保守的示教、低速复扫、离线 Candidate Map 和对比报告闭环。它不会批准 Candidate 用于导航，也不会替换 Bootstrap Map。

## A. 构建并选择输入资产

```bash
cd /path/to/agt_navigation_v2
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select \
    agt_interfaces \
    agt_system_manager \
    agt_teach_repeat \
    agt_bringup \
    agt_navigation \
    agt_ui_bridge \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

source install/setup.bash

export SESSION_ID=greenhouse_001
export TEACH_BAG="$PWD/runtime/rosbag/Benchmark-BAG-260725"
export BOOTSTRAP_MAP_YAML=/absolute/path/to/map.yaml
export BOOTSTRAP_PCD=/absolute/path/to/localization_map.pcd
export BOOTSTRAP_RECORD=/absolute/path/to/localization_map.processing.yaml
```

示教 Bag 必须包含类型为 `nav_msgs/msg/Odometry` 的 `/agt/mapping/odometry`。PCD 处理记录必须为 `state: ready`，必须指向所选 PCD；如果记录中包含哈希值，则必须与 PCD 的实际哈希一致。

## B. 创建 Session

即使使用单位变换，也必须提供全部四个变换参数。

```bash
ros2 run agt_system_manager teach_mapping_workflow.py init \
  --session-id "$SESSION_ID" \
  --runtime-root "$PWD/runtime/teach_mapping" \
  --platform-profile "$PWD/profiles/platforms/bunker.yaml" \
  --map-id greenhouse_bootstrap \
  --bootstrap-map-yaml "$BOOTSTRAP_MAP_YAML" \
  --bootstrap-localization-pcd "$BOOTSTRAP_PCD" \
  --bootstrap-processing-record "$BOOTSTRAP_RECORD" \
  --teach-bag "$TEACH_BAG" \
  --map-from-teach-odom-x 0.0 \
  --map-from-teach-odom-y 0.0 \
  --map-from-teach-odom-z 0.0 \
  --map-from-teach-odom-yaw 0.0

ros2 run agt_system_manager teach_mapping_workflow.py status \
  --session "$PWD/runtime/teach_mapping/$SESSION_ID/session.yaml"
```

预期阶段为 `BOOTSTRAP_READY`。除非操作员明确提供 `--overwrite`，否则已有 Session 会被拒绝覆盖。

## C. 提取并预览路线

```bash
ros2 run agt_system_manager teach_mapping_workflow.py extract \
  --session "$PWD/runtime/teach_mapping/$SESSION_ID/session.yaml"

ros2 launch agt_teach_repeat teach_preview.launch.py \
  manifest:="$PWD/runtime/teach_mapping/$SESSION_ID/teach_route/manifest.yaml" \
  start_rviz:=false \
  start_qt:=true
```

在第二个终端中执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /agt/teach/status --once
ros2 topic echo /agt/teach/validation_report --once
ros2 topic echo /agt/teach/corridor_report --once
ros2 topic echo /agt/teach/route_annotations --once
```

Qt 中应看到绑定地图上的蓝色参考路线、方向箭头和转弯/掉头标签。标注可见只证明只读数据链正常；
如果验证报告为 false，仍不得进入实车步骤。

只有验证报告明确包含 `eligible_for_execution=true`，并且已检查走廊显示后，才可以继续连接实车。

## D. 断车 Dry Run

```bash
ros2 launch agt_bringup teach_mapping_rescan.launch.py \
  session:="$PWD/runtime/teach_mapping/$SESSION_ID/session.yaml" \
  runtime_dir:="$PWD/runtime/teach_mapping/$SESSION_ID/rescan" \
  start_chassis:=false \
  execution_enabled:=false \
  record_bag:=true
```

确认 Bootstrap Map、定位、示教 Manifest、参考路径、验证器、障碍物链路、GUI 和录包进程均已启动。此时不应发布任何底盘指令，也不会自动调用 `/agt/teach/start`。

## E. 实车低速复扫

必须在封闭测试区域内进行。操作员应留在固定安全位置，随时可使用遥控器和物理急停；路线范围内不得有其他人员进入。

```bash
ros2 launch agt_bringup teach_mapping_rescan.launch.py \
  session:="$PWD/runtime/teach_mapping/$SESSION_ID/session.yaml" \
  runtime_dir:="$PWD/runtime/teach_mapping/$SESSION_ID/rescan" \
  start_chassis:=true \
  execution_enabled:=true \
  record_bag:=true \
  rescan_max_speed_mps:=0.10
```

运动前按顺序确认：

1. `/agt/localization/status` 是新鲜的 `TRACKING` 状态，定位已接受且位姿有效。
2. Nav2 的地图、规划器、控制器和行为树生命周期节点均处于 Active 状态。
3. `/agt/teach/validation_report` 仍满足执行条件。
4. 局部障碍物链路和 Collision Monitor 数据新鲜。
5. Bunker 通信和 `/agt/chassis/status` 状态正常。
6. 已检查急停行为。
7. 已通过现有安全操作控制明确启用运动。

只有完成以上检查后，才能启动路线：

```bash
ros2 service call /agt/teach/start std_srvs/srv/Trigger '{}'
```

通过以下命令取消，不得绕过 Nav2 或安全链路：

```bash
ros2 service call /agt/teach/cancel std_srvs/srv/Trigger '{}'
```

执行器采用 `min(manifest_limit, rescan_max_speed_mps)` 作为速度限制，并在成功、失败或取消后将 Nav2 `SpeedLimit` 重置为无限制（`0.0`）。正常停止 Launch 会完成 rosbag 元数据写入。

## F. 注册复扫 Bag

在复扫运行目录下找到已完成的 Bag，并注册其准确目录：

```bash
export RESCAN_BAG=/absolute/path/to/completed/rescan_bag

ros2 run agt_system_manager teach_mapping_workflow.py register-rescan \
  --session "$PWD/runtime/teach_mapping/$SESSION_ID/session.yaml" \
  --bag "$RESCAN_BAG"
```

注册过程会拒绝空 Bag 或消息类型错误的 Bag，并要求其中包含原始 MID360、IMU、建图里程计、定位、已执行路径、安全状态和底盘状态。预期阶段为 `RESCAN_RECORDED`。

## G. 构建独立 Candidate Map

在离线回放前停止实时复扫 Launch，然后执行：

```bash
ros2 run agt_system_manager teach_mapping_workflow.py build-candidate \
  --session "$PWD/runtime/teach_mapping/$SESSION_ID/session.yaml" \
  --candidate-map-name "${SESSION_ID}_candidate_v1"
```

构建器以仿真时间启动现有建图链路，不启动传感器或底盘进程，并且只回放原始 MID360、IMU 和时钟。它会在建图进程仍运行时保存 2D 地图，然后发送 `SIGINT`，使 FAST-LIVO2 写出 PCD。输出始终保存在 Session 的 Candidate 目录下，不能覆盖 Bootstrap。失败目录会被保留；重试时应使用新的 Candidate 名称，或者仅在明确检查后手动删除失败的 Candidate 目录。

## H. 生成对比报告

```bash
ros2 run agt_system_manager teach_mapping_workflow.py report \
  --session "$PWD/runtime/teach_mapping/$SESSION_ID/session.yaml"

ros2 run agt_system_manager teach_mapping_workflow.py status \
  --session "$PWD/runtime/teach_mapping/$SESSION_ID/session.yaml" \
  --json
```

检查 `reports/map_comparison.json` 和 `reports/map_comparison.md`。报告支持分辨率、尺寸和旋转地图原点不同的地图，并只报告栅格、PCD、路径和足迹差异及警告；它绝不会选择、激活或发布 Candidate。

## 验收记录

- 已检查 Bootstrap 重定位和路径预览。
- 断车 Launch 未产生运动。
- 实车执行同时要求明确启用安全运动权限和明确调用启动服务。
- 速度保持在 `0.10 m/s` 或以下；取消和急停均有效。
- 完成的 Bag 保留了原始 MID360、IMU、已执行路径、定位、安全和底盘证据。
- Candidate 资产相互独立且状态为 Ready；Bootstrap 哈希保持不变。
- 已生成对比报告，且 Candidate 未被自动选中。
