# BUNKER 首次上车测试备忘

本备忘用于 BUNKER、MID360、FAST-LIVO2、Nav2 和项目内 Qt5 GUI 的首次低速联调。
它不是参数验收记录，也不能替代物理急停、遥控器接管和现场安全负责人。
完整数据链与设计说明见
[`bunker_qt5_fastlio_baseline.md`](bunker_qt5_fastlio_baseline.md)。

## 0. 紧急停车先记住

优先按下车辆物理急停或使用遥控器接管。软件禁用命令为：

```bash
ros2 service call /agt/safety/set_motion_enabled \
  std_srvs/srv/SetBool '{data: false}'
```

仓库目前只订阅 `/agt/safety/emergency_stop`，没有项目节点把车辆物理急停转换成该 topic；
不要把软件服务当成硬件急停。

## 快速顺序

```text
物理急停/遥控器就位
  -> CAN UP 且有报文
  -> MID360 点云和 IMU 持续更新
  -> 架空履带确认前后/转向符号
  -> 低速落地建图
  -> 保存同源 PGM/YAML/PCD
  -> 启动导航但保持 motion disabled
  -> Qt5 设置初始位姿
  -> 定位、TF、Nav2、障碍点云、底盘全部就绪
  -> 显式使能运动
  -> 0.5–1.0 m 短目标
  -> 障碍/CAN/急停失效测试
  -> 禁用运动并 Ctrl+C 正常收车
```

## 1. 当天测试信息

开始前填写，避免混用地图和现场配置：

```text
日期/场地：
车辆/固件：
测试负责人：
物理急停操作员：
CAN 接口：
MID360 主机 IP / 雷达 IP：
地图 ID：
PGM/YAML：
同源 PCD：
定位后端：NDT / ICP
测试 bag：
```

## 2. 只做一次的软件准备

在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/agt_coverage_ws/install/setup.bash"

colcon build --symlink-install --packages-up-to agt_bringup
source install/setup.bash

tools/build_ros_qt5_gui_app.sh
test -x build/ros_qt5_gui_app/ros_qt5_gui_app
```

基础单点导航不启动语义地图和覆盖规划；Coverage underlay 仅用于保持统一的终端环境。

## 3. 每个新终端

```bash
cd "$(git rev-parse --show-toplevel)"
source /opt/ros/humble/setup.bash
source "$HOME/agt_coverage_ws/install/setup.bash"
source install/setup.bash
```

## 4. 上电前检查：任何一项失败都不运动

- 车辆四周隔离，首次方向测试时履带架空。
- 物理急停有效，遥控器已配对并由专人持有。
- 电池电压、CAN 线、CAN-USB、MID360 网线和机械固定正常。
- 雷达外参、车体 footprint、履带正反方向已人工核对。
- 测试人员知道软件禁用命令和总控终端位置。
- 不使用尚未验收的温室狭窄通道作为首次测试场地。

2026-07-19 已把粗测的雷达中心位置写入 BUNKER 描述：车体纵向中心线、车头外沿向后
`0.250 m`、离地 `0.500 m`；按当前几何中心 `base_link` 换算的平移为
`[0.2615, 0.0, 0.3000] m`。雷达 `+Z` 轴向车头 `+X` 前倾 `30 deg`，对应 ROS
`rpy=[0.0, 0.5235987756, 0.0] rad`。`base_link` 高度、外参精度和有效履带中心距仍含
provisional 成分；未经精测验收，不得把首次测试结果解释为量产参数验证。

## 5. CAN 门禁

首次安装工具：

```bash
sudo apt-get install -y can-utils libasio-dev
sudo modprobe gs_usb
```

本仓库总控当前使用底盘 launch 的默认接口 `can0`。建立 500 kbit/s 接口：

```bash
CAN_IF=can0
sudo ip link set "$CAN_IF" down 2>/dev/null || true
sudo ip link set "$CAN_IF" up type can bitrate 500000
ip -details link show "$CAN_IF"
timeout 5s candump "$CAN_IF"
```

通过条件：接口为 `UP`，车辆上电后能持续收到 CAN 帧。无报文时检查车辆上电、终端电阻、
线序和 CAN-USB；不要通过发送运动命令诊断总线。

## 6. MID360 网络门禁

默认配置为主机 `192.168.1.5/24`、MID360 `192.168.1.12`。选择实际雷达网卡：

```bash
ip -br link
LIDAR_NIC=<实际有线网卡>
sudo ip link set "$LIDAR_NIC" up
sudo ip addr replace 192.168.1.5/24 dev "$LIDAR_NIC"
ping -c 4 192.168.1.12
```

如现场地址不同，复制网络模板到 runtime 后编辑，不提交现场 IP：

```bash
mkdir -p runtime/config
cp src/agt_sensor_adapters/config/mid360_network.json \
  runtime/config/mid360_network.json
```

自定义网络配置需要单独启动传感器：

```bash
ros2 launch agt_sensor_adapters mid360.launch.py \
  user_config_path:="$(pwd)/runtime/config/mid360_network.json"
```

后续总控命令相应增加 `start_sensor:=false`，避免启动第二个 MID360 驱动。

通过条件：

```bash
ros2 topic hz /agt/sensors/lidar/custom
ros2 topic hz /agt/sensors/imu/data
```

两个 topic 均应持续更新，frame 和时间戳无明显异常。

## 7. 阶段 A：架空履带与 Qt5 手动控制

使用默认 MID360 配置时启动：

```bash
ros2 launch agt_bringup system.launch.py \
  mode:=mapping \
  map_name:=first_vehicle_test \
  start_mapping_gui:=true \
  start_rviz:=true \
  record_bag:=true
```

传感器已用自定义配置单独启动时，增加 `start_sensor:=false`。

先检查底盘与安全状态：

```bash
ros2 topic echo /agt/chassis/status --once
ros2 topic echo /agt/chassis/connected --once
ros2 topic echo /agt/safety/status --once
ros2 topic hz /agt/chassis/odometry
```

确认架空且物理急停操作员就位后再使能：

```bash
ros2 service call /agt/safety/reset_emergency_stop \
  std_srvs/srv/Trigger '{}'
ros2 service call /agt/safety/set_motion_enabled \
  std_srvs/srv/SetBool '{data: true}'
```

在 Qt5 中只给极小、短时手动命令，依次确认：

- 正线速度：两侧履带同向，车辆语义为前进；
- 负线速度：两侧履带同向，车辆语义为后退；
- 正/负角速度：左右转方向与 ROS 右手系一致；
- 松开控制：`/agt/safety/cmd_vel` 和 `/agt/chassis/cmd_vel` 快速归零；
- Qt5 手动速度只发布到 `/agt/cmd_vel_manual`，不直连厂商 `/cmd_vel`。

任一方向错误立即禁用运动，记录现象，不通过修改已验证参数临时掩盖。

## 8. 阶段 B：低速建图

履带落地后，在空旷隔离区以低速直行、转弯，观察：

```bash
ros2 topic hz /agt/mapping/odometry
ros2 topic hz /agt/mapping/registered_points_lidar
ros2 run tf2_ros tf2_echo odom base_footprint
```

Qt5 mapping profile 应以 `odom` 为固定坐标系，显示
`/agt/map/mapping_occupancy`。确认点云、轨迹和二维地图稳定后保存：

```bash
tools/save_mapping_outputs.sh first_vehicle_test
```

确认 PGM/YAML 已生成，再在总控终端按 `Ctrl+C`，让 FAST-LIVO2 正常保存 PCD：

```bash
ls -lh runtime/maps/first_vehicle_test/first_vehicle_test.{pgm,yaml}
test -s runtime/maps/first_vehicle_test/pcd/localization_map.pcd
grep -E '^(state|input_points|accepted_points|rejected_nonfinite|rejected_coordinate_range|output_points|min_xyz|max_xyz):' \
  runtime/maps/first_vehicle_test/pcd/localization_map.processing.yaml
```

处理记录必须显示 `state: ready`。PGM/YAML 与导航使用的全局 PCD 必须来自本次同一建图任务；
总控录制的 bag 名称以 `<map_name>_mapping_<时间戳>` 开头，移动或归档时应把该 bag、
`runtime/maps/<map_name>/` 整个目录和当次日志作为一个 session 保存，不要只复制 PCD。
不要使用曾因 PCL 整数网格溢出而与 raw 文件逐字节相同的 `all_downsampled_points.pcd`。

## 9. 阶段 C：导航只读检查，保持运动禁用

```bash
REPO_ROOT="$(git rev-parse --show-toplevel)"
MAP_YAML="$REPO_ROOT/runtime/maps/first_vehicle_test/first_vehicle_test.yaml"
GLOBAL_PCD="$REPO_ROOT/runtime/maps/first_vehicle_test/pcd/localization_map.pcd"

test -f "$MAP_YAML"
test -s "$GLOBAL_PCD"
```

启动基础导航：

```bash
ros2 launch agt_bringup system.launch.py \
  mode:=navigation \
  map:="$MAP_YAML" \
  global_map_pcd:="$GLOBAL_PCD" \
  backend:=ndt \
  start_gui:=true \
  start_semantic_map_server:=false \
  start_coverage_planning:=false \
  record_bag:=true
```

自定义 MID360 驱动已单独启动时增加 `start_sensor:=false`。

在 Qt5 中设置准确的初始位姿，然后检查：

```bash
ros2 topic echo /agt/localization/status --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint

ros2 lifecycle get /map_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /collision_monitor

ros2 topic hz /agt/perception/obstacle_cloud
ros2 topic echo /agt/chassis/status --once
ros2 topic echo /agt/safety/status --once
```

通过条件：

- 定位状态为成功、`converged=true`，fitness 通过当前门限；
- `ndt_num_threads` 为正整数；Bunker 已验证基线为 `4`，禁止使用会使 NDT-OMP
  工作数组越界的 `0`；
- `map -> odom -> base_footprint` 连续且无跳变；
- 五个 lifecycle 节点均为 `active [3]`；
- 障碍点云持续更新；
- 底盘在线且无错误码；
- 安全状态仍为 `motion_enabled=false`。

## 10. 阶段 D：首次 Nav2 短目标

测试区域清空，物理急停操作员就位后才使能：

```bash
ros2 service call /agt/safety/set_motion_enabled \
  std_srvs/srv/SetBool '{data: true}'
```

在 Qt5 中发送车前方 `0.5–1.0 m` 的单点目标。自动导航时不要触碰手动速度控件，因为
手动命令优先于导航命令。观察：

```bash
ros2 topic echo /agt/navigation/status
ros2 topic echo /agt/safety/status
ros2 topic hz /agt/navigation/cmd_vel_raw
ros2 topic hz /agt/navigation/cmd_vel
ros2 topic hz /agt/chassis/cmd_vel
```

按顺序逐级放行：

1. 直行 `0.5–1.0 m`；
2. 小角度转向；
3. 空旷区 L 形短路线；
4. 静态障碍前减速与停止；
5. 只有前一级通过后才进入下一级。

## 10.1 Qt 手工多点 Demo

维护版 Qt 的 **Start Task Chain** 已接到
`/agt/navigation/execute_waypoint_task`，可以直接用于低速 Demo。按钮状态来自 Nav2 Action；
**Stop Task Chain** 会取消任务，“Repeat twice (finite)”只用于明确的两遍测试，首次实车不要勾选。

1. 在 Qt 中打开当前 Nav2 YAML，编辑并保存同名 `.topology`；
2. 添加任务点，确认顺序和朝向；
3. 确认重定位、Nav2、底盘和安全状态后显式使能运动；
4. 点击 **Start Task Chain**，观察 Pending/Running/Finish 或 Failed 状态。

离线编辑时先在 Task 表中选中目标行，再点击地图上的已有点位，该点会写入当前行；
新增、改名或删除点位后下拉框会自动刷新。鼠标中键拖动可在任意工具下平移，普通查看
模式也可在地图空白处左键拖动；滚轮或右下角 `+/-` 以光标/视图中心缩放。手动平移或
缩放会自动取消“跟随机器人”，避免离线地图被拉回机器人位置。窗口默认使用系统边框，
可从左右边缘调整宽度。顶栏可选择中文或 English，保存后重启 Qt 应用生效。

只查看离线路径时应关闭其他导航启动进程，再使用专用入口：

```bash
MAP_YAML="$(realpath runtime/maps/<map_id>/<map_id>.yaml)"
ros2 launch agt_navigation waypoint_preview.launch.py \
  map:="$MAP_YAML" \
  platform_profile:="$(realpath profiles/platforms/bunker.yaml)"
```

Task 第一行作为预览起点，至少添加两行，然后点击“预览离线路径”；路径显示在 `/plan`。
offline profile 中“开始多点任务”保持禁用，避免把离线查看误当真实执行。

需要脱离 GUI 回放已保存 JSON 时，仍可使用：

```bash
TASK_JSON="$(realpath runtime/maps/first_vehicle_test/demo_task_01.json)"
test -f "$TASK_JSON"

ros2 run agt_navigation execute_waypoint_task.py "$TASK_JSON"
```

另开终端观察项目状态：

```bash
ros2 topic echo /agt/navigation/task_status
```

项目服务会拒绝重复追加、非有限坐标、超出当前 OccupancyGrid 的旧拓扑点和未就绪的
`agt_safety`，并以 Nav2 `FollowWaypoints` 的成功状态及 `missed_waypoints` 作为完成依据。
按 `Ctrl+C` 会请求取消当前任务。只有单次任务实车验收通过后，才能尝试显式有限循环，
例如 `--loop-count 2`；不提供无限循环。

## 11. 阶段 E：失效安全测试

每项单独执行，执行前确认人员和车辆安全：

- Qt5 停止手动输入后，手动命令超时归零；
- 取消 Nav2 目标后，速度链归零；
- 障碍进入 stop zone 后，Collision Monitor 输出零速；
- CAN 通讯断开后，底盘 watchdog 停车并报告离线；
- MID360/障碍点云中断时不得继续自动导航；
- 定位失败、TF 跳变或 LIO 退出时立即人工禁用运动；
- 物理急停能够独立于 ROS 停车。

当前基础单点导航没有把所有定位/传感器故障自动接入硬件急停，因此现场人员必须持续监视，
不能进行无人值守测试。

## 12. 正常收车

先禁用运动：

```bash
ros2 service call /agt/safety/set_motion_enabled \
  std_srvs/srv/SetBool '{data: false}'
```

然后：

1. 确认 `/agt/chassis/cmd_vel` 为零；
2. 在总控终端按 `Ctrl+C`，等待所有节点正常退出；
3. 不使用 `kill -9`；
4. 检查 rosbag `metadata.yaml`、地图和 PCD 完整；
5. 车辆断电，再断开 CAN 和雷达连接。

## 13. 当天结果记录

```text
[ ] CAN 报文正常
[ ] MID360 点云/IMU 正常
[ ] 履带前后/转向符号正确
[ ] 手动超时归零
[ ] 地图与 PCD 保存成功
[ ] NDT/ICP 收敛，fitness：________
[ ] TF 唯一且连续
[ ] Nav2 lifecycle 全部 active
[ ] 0.5–1.0 m 目标成功
[ ] 项目多点 Action 完成且 missed_waypoints 为空
[ ] 多点取消后车辆停车
[ ] 急停导致多点子 Action 取消
[ ] 障碍停车成功，距离：________
[ ] CAN 断连停车成功，时间：________
[ ] 物理急停成功，距离：________
[ ] 正常 Ctrl+C 退出且 bag 完整

未通过项/日志时间戳：

下一步：
```

只有当天全部门禁通过，才进入更高速度、狭窄通道或温室路线验收。
