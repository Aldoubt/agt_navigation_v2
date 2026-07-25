# AGT Navigation Data Stream

本文记录当前 Bunker Qt5/FAST-LIO 导航基线的两种启动方式：命令行直接启动，以及 Web 控制台通过系统管理器启动。文档只描述当前代码已经实现的链路，不把语义地图、覆盖规划和实车运动验收描述成默认已开启能力。

## 1. 启动前环境

所有 ROS 2 命令都应在同一个工作区环境中执行。`AGT_WS` 只是示例变量，不要把用户名、工作区或设备路径写死到脚本中。

```bash
export AGT_WS=/absolute/path/to/agt_navigation_v2
cd "$AGT_WS"
source /opt/ros/humble/setup.bash
source "$AGT_WS/install/setup.bash"
```

Web 使用 `.venv-web` 中的 Python，但仍然需要先 source ROS 工作区。否则会出现：

```text
ModuleNotFoundError: No module named 'agt_experiment_manager'
```

FastAPI、Starlette 和 Uvicorn 只负责 Web HTTP/WebSocket 运行时；Node.js 不是当前前端运行依赖。

## 2. 命令行核心入口

### 2.1 系统管理器

如果要让 Web、白名单 profile 或 `ChangeSystemMode` Action 管理主链，先单独启动系统管理器：

```bash
ros2 launch agt_system_manager system_manager.launch.py \
  active_mode:=IDLE \
  runtime_dir:="$AGT_WS/runtime"
```

该 launch 会启动三个节点：

- `agt_system_manager_health`：发布 `/agt/system/health` 和 `/agt/system/task_readiness`。
- `agt_system_mode_manager`：提供 `/agt/system/change_mode` Action，管理 profile 进程组。
- `agt_relocalization_mode_controller`：管理 `MANUAL_ONLY`、`AUTO_ON_START` 和 `AUTO_RECOVERY`。

系统管理器的实现入口是 [system_manager.launch.py](/home/yangxuan/agt_navigation_v2/src/agt_system_manager/launch/system_manager.launch.py:10) 和 [system_mode_manager.py](/home/yangxuan/agt_navigation_v2/src/agt_system_manager/scripts/system_mode_manager.py:21)。

### 2.2 直接启动建图链

实时 MID360 建图的推荐命令：

```bash
ros2 launch agt_bringup system.launch.py \
  mode:=mapping \
  runtime_dir:="$AGT_WS/runtime" \
  user_config_path:="$AGT_WS/src/agt_sensor_adapters/config/mid360_network.json" \
  map_name:=mid360_map \
  start_sensor:=true \
  start_lidar_self_filter:=true \
  start_chassis:=false \
  start_chassis_monitor:=false \
  use_sim_time:=false \
  record_bag:=false
```

历史 bag 建图时不启动真实 MID360，建图算法使用仿真时间：

```bash
ros2 launch agt_bringup system.launch.py \
  mode:=mapping \
  runtime_dir:="$AGT_WS/runtime" \
  map_name:=mid360_map \
  start_sensor:=false \
  start_lidar_self_filter:=true \
  start_chassis:=false \
  start_chassis_monitor:=false \
  use_sim_time:=true \
  record_bag:=false
```

随后在另一个终端回放受限的建图输入：

```bash
ros2 bag play --clock --rate 1.0 \
  "$AGT_WS/runtime/rosbag/<bag_id>" \
  --topics \
  /clock \
  /tf_static \
  /agt/sensors/lidar/custom \
  /agt/sensors/imu/data
```

建图模式不能把 bag 中已有的 FAST-LIVO2 输出、注册点云、二维地图或 `/tf` 再回放一遍，否则会产生重复数据和 TF 竞争。原始
`/agt/sensors/lidar/custom` 保留为 bag 输入；`agt_livox_self_filter` 默认独立于
`start_sensor` 启动并发布 `/agt/sensors/lidar/custom_filtered`，随后才进入 FAST-LIVO2。
`start_lidar_self_filter:=false` 仅用于显式 A/B 基线，此时 FAST-LIVO2 回退到原始 topic。

`system.launch.py` 是总分流入口；`mode:=mapping` 实际包含 [mapping_mode.launch.py](/home/yangxuan/agt_navigation_v2/src/agt_bringup/launch/mapping_mode.launch.py:36) 的以下节点：

1. `agt_description/bunker_description.launch.py`：机器人和传感器静态描述。
2. `agt_sensor_adapters/mid360.launch.py`：MID360 驱动，原始输入为 `/agt/sensors/lidar/custom` 和 `/agt/sensors/imu/data`。
3. `agt_mapping/fast_livo2_mapping.launch.py`：前置 `agt_livox_self_filter`、FAST-LIVO2 后端和 `fast_livo2_adapter.py`。
4. `agt_map_processing/octomap_projection.launch.py`：注册点云经过独立节流器后进入全局 OctoMap 投影。
5. `nav2_map_server/map_saver_server` 和 lifecycle manager：提供二维地图保存服务。
6. 可选的 BUNKER 控制链、只读 CAN 监测、RViz、建图 Qt 前端和录包进程。

建图数据流如下：

```text
MID360 CustomMsg + IMU
        |
        v
agt_livox_self_filter
        |
        v
FAST-LIVO2 (custom_filtered)
        |-- /agt/mapping/backend/registered_points
        |       |
        |       +--> fast_livo2_adapter --> /agt/mapping/registered_points
        |       |                              |
        |       |                              +--> Web/Qt/RViz 观察
        |       |
        |       +--> octomap_cloud_throttle -- 0.2 Hz 默认 --+--> OctoMap
        |
        +--> /agt/mapping/odometry

OctoMap --> /agt/map/mapping_occupancy --> map_saver_server --> PGM/YAML
FAST-LIVO2 退出保存 --> localization_map.pcd + processing.yaml
```

其中局部障碍链继续使用未节流的注册点云；0.2 Hz 只作用于全局地图投影输入。直接命令行启动时，PCD 默认目录是：

```text
$AGT_WS/runtime/maps/<map_name>/pcd/
```

命令行 launch 只启动 map saver 服务，不会自动替操作者调用保存服务。Web 的保留流程会自动调用它。

### 2.3 直接启动导航链

导航必须使用已经准备好的 OccupancyGrid YAML、定位 PCD 和 ready processing record。路径应来自 READY 地图版本的 manifest：

```bash
MAP_VERSION=<map_version_id>
MAP_YAML="$AGT_WS/runtime/maps/<map_id>/versions/$MAP_VERSION/navigation/map.yaml"
MAP_PCD="$AGT_WS/runtime/maps/<map_id>/versions/$MAP_VERSION/pointcloud/localization_map.pcd"
MAP_RECORD="$AGT_WS/runtime/maps/<map_id>/versions/$MAP_VERSION/pointcloud/localization_map.processing.yaml"

ros2 launch agt_bringup system.launch.py \
  mode:=navigation \
  runtime_dir:="$AGT_WS/runtime" \
  map_id:=<map_id> \
  map:="$MAP_YAML" \
  global_map_pcd:="$MAP_PCD" \
  global_map_processing_record:="$MAP_RECORD" \
  start_sensor:=true \
  start_chassis:=false \
  start_chassis_monitor:=false \
  start_gui:=true \
  use_sim_time:=false \
  auto_relocalize_on_start:=false
```

启动前可检查地图版本：

```bash
ros2 run agt_map_manager map_registry.py \
  --root "$AGT_WS/runtime/maps" list

ros2 run agt_map_manager map_registry.py \
  --root "$AGT_WS/runtime/maps" validate <map_version_id>
```

导航模式的主要内容来自 [navigation_system.launch.py](/home/yangxuan/agt_navigation_v2/src/agt_bringup/launch/navigation_system.launch.py:122)：

- 描述、MID360、FAST-LIVO2 adapter 和局部障碍过滤器。
- `agt_localization/relocalization.launch.py`：读取 `global_map_pcd` 和 processing record，唯一负责 `map -> odom`。
- Nav2 map server、planner、controller、behavior、BT navigator、waypoint follower、collision monitor 和任务服务。
- `agt_bringup/localization_navigation_gate.py`：定位无效或状态过期时阻断导航任务。
- 可选的语义地图服务器、Keepout Filter、覆盖规划和语义编辑器。它们默认关闭。
- 可选 Qt 导航前端、底盘安全链、BUNKER 驱动和录包。

导航速度链必须保持：

```text
Nav2 controller
    --> /agt/navigation/cmd_vel_raw
    --> collision monitor / agt_safety
    --> /agt/chassis/cmd_vel
    --> BUNKER driver
```

`start_chassis:=false` 是无车或离线验证默认值。只有实车验收时才显式打开 `start_chassis:=true`；Qt、Web 和导航节点都不能直接发布最终底盘速度。

注意：当前导航 launch 的 Nav2 lifecycle 默认由 `agt_navigation/launch/navigation.launch.py` 以 `autostart:=false` 启动。进程存在不等于 Nav2 已经 ACTIVE，必须另外检查 lifecycle、地图、定位和安全门禁。

## 3. Web 端启动方式

### 3.1 Web 服务器本身

Web 服务器不负责启动系统管理器，所以推荐两个终端：

```bash
# 终端 1：系统管理器
cd "$AGT_WS"
source /opt/ros/humble/setup.bash
source "$AGT_WS/install/setup.bash"
ros2 launch agt_system_manager system_manager.launch.py \
  active_mode:=IDLE runtime_dir:="$AGT_WS/runtime"
```

```bash
# 终端 2：Web 控制台
cd "$AGT_WS"
source /opt/ros/humble/setup.bash
source "$AGT_WS/install/setup.bash"
"$AGT_WS/.venv-web/bin/python" \
  "$AGT_WS/install/agt_web_console/lib/agt_web_console/web_console.py" \
  --config "$AGT_WS/src/agt_web_console/config/web_console.yaml" \
  --backend ros
```

当前配置文件 [web_console.yaml](/home/yangxuan/agt_navigation_v2/src/agt_web_console/config/web_console.yaml:1) 默认：

```yaml
host: 127.0.0.1
port: 8080
runtime_dir: runtime
backend: ros
```

`runtime_dir: runtime` 是相对 Web 进程当前工作目录解析的；从工作区根目录启动时对应 `$AGT_WS/runtime`。Web 使用 instance lock，重复启动同一 runtime 会在创建第二个 ROS bridge 节点前失败。

### 3.2 Web 启动代码

关键入口和职责：

| 代码 | 作用 |
| --- | --- |
| [web_console.py](/home/yangxuan/agt_navigation_v2/src/agt_web_console/scripts/web_console.py:18) | 读取 YAML，建立 instance lock、MapRegistry、ExperimentManager、ROS bridge、FastAPI 和 Uvicorn。 |
| [ros_bridge.py](/home/yangxuan/agt_navigation_v2/src/agt_web_console/agt_web_console/ros_bridge.py:31) | 创建 ROS Action/SRV/client、订阅健康/地图/点云/里程计/底盘状态。 |
| [service.py](/home/yangxuan/agt_navigation_v2/src/agt_web_console/agt_web_console/service.py:35) | 独立于 HTTP 的业务层，负责 profile、会话、地图登记、bag 回放和状态校验。 |
| [app.py](/home/yangxuan/agt_navigation_v2/src/agt_web_console/agt_web_console/app.py:8) | FastAPI REST/WebSocket 路由，只调用 `WebConsoleService`。 |
| [mode_profiles.yaml](/home/yangxuan/agt_navigation_v2/src/agt_system_manager/config/mode_profiles.yaml:1) | profile argv 白名单；Web 不能传入任意 shell 命令。 |
| [system_mode_manager.py](/home/yangxuan/agt_navigation_v2/src/agt_system_manager/scripts/system_mode_manager.py:159) | 接收 `ChangeSystemMode`，创建独立进程组，切换主链并写入进程状态。 |

Web 的启动调用路径是：

```text
浏览器 POST /api/v1/system/mode
        |
        v
WebConsoleService.set_mode()
        |
        v
RosConsoleBridge.start()
        |
        v
ChangeSystemMode Action: /agt/system/change_mode
        |
        v
agt_system_mode_manager
        |
        v
ProfileRegistry -> ProcessManager -> ros2 launch agt_bringup system.launch.py
```

`RosConsoleBridge.start()` 会等待 Action server、将 profile 和键值参数写入 generated Action goal，并等待明确的 Action result；找不到系统管理器时会提示：

```text
请先启动 agt_system_manager，再检查 ros2 action info /agt/system/change_mode
```

### 3.3 Web 建图逻辑

Web 建图不是直接执行一个浏览器传来的命令，而是以下固定流程：

1. 页面调用 `/api/v1/mapping/session/prepare`，服务创建 `runtime/mapping_sessions/<map_name>/<session_id>/`，并生成受管 `pcd_output_dir`。
2. 页面调用 `/api/v1/system/mode`，`profile=mapping`。Web 把 `map_name`、`mapping_output_dir`、`start_sensor`、`use_sim_time` 等声明参数传入 Action。
3. 系统管理器发现 `start_sensor:=true` 时，先启动独立的 `sensor_only` 进程组，再以 `start_sensor:=false` 启动 mapping 主链；历史 bag 模式本身传入 `start_sensor:=false`。
4. 页面调用 `/api/v1/bags/play` 时，`ExperimentManager` 只允许相对 `runtime/rosbag` 根目录下的完整 bag，并按状态强制选择 `mapping_inputs`：`/clock`、`/tf_static`、MID360 CustomMsg 和 IMU。
5. 页面点“完成建图”后先读取会话状态，要求确认采集完成并输入最终地图名。选择保留时，Web 调用 `/agt_mapping_map_saver/save_map` 保存 PGM/YAML，随后通过 `ChangeSystemMode` 正常停止 mapping。
6. 服务等待 PGM/YAML 和 `state: ready` 的 FAST-LIVO2 PCD record。受管会话中如果 record 缺少 `pcd_sha256`，服务在进程停止后计算并补写实际 PCD 哈希；已有哈希不匹配则失败闭环。
7. `agt_map_manager.import_legacy()` 将资产复制到不可变 `runtime/maps/<map_id>/versions/<version_id>/`。只有 READY 版本才可以进一步激活和用于导航。
8. 选择删除时，只删除受管临时会话；如果保留流程创建了 INVALID 版本，删除动作同时清理该失败版本。

对应实现集中在 [service.py](/home/yangxuan/agt_navigation_v2/src/agt_web_console/agt_web_console/service.py:301)、[ros_bridge.py](/home/yangxuan/agt_navigation_v2/src/agt_web_console/agt_web_console/ros_bridge.py:352) 和 [app.js](/home/yangxuan/agt_navigation_v2/src/agt_web_console/static/app.js:781)。

### 3.4 Web 导航逻辑

Web 导航启动前不会接受浏览器任意地图路径。页面只选择地图版本，`profileArguments()` 从当前 `READY + active` manifest 派生：

```text
map                         -> navigation/map.yaml
global_map_pcd              -> pointcloud/localization_map.pcd
global_map_processing_record -> pointcloud/localization_map.processing.yaml
```

服务端再次校验版本状态、active 标志和三条资产路径，然后调用 `/agt/system/change_mode` 的 `navigation` profile。导航模式禁止 Web bag 回放；Web 只显示系统健康、TaskReadiness、定位和进程状态，不替代 Nav2、项目 Action、`agt_safety` 或底盘 watchdog。

## 4. 常用状态检查

```bash
ros2 action info /agt/system/change_mode
ros2 node list
ros2 topic hz /agt/sensors/lidar/custom
ros2 topic hz /agt/sensors/imu/data
ros2 topic hz /agt/mapping/registered_points_lidar
curl http://127.0.0.1:8080/api/v1/system/status
curl http://127.0.0.1:8080/api/v1/mapping/session
```

如果 Web 返回“未发现 `/agt/system/change_mode` Action server”，先检查系统管理器，而不是重复启动 Web。若系统模式已经是 `MAPPING` 或 `NAVIGATION`，不要再次启动相同 profile；由系统管理器的受管进程组负责停止和切换。

正常停止优先使用 Web 的停止/完成动作或 `ChangeSystemMode` 的 IDLE 请求。不要用 `kill -9` 作为普通关闭流程，也不要手工停止不属于当前系统管理器的进程组。


https://open.cherryin.ai/?utm_source=chatgpt.com
