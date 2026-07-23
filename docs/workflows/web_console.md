# Web 控制台工作流

构建并加载工作区环境：

```bash
export AGT_WS="${AGT_WS:-$HOME/agt_navigation_v2}"
cd "$AGT_WS"
source /opt/ros/humble/setup.bash
colcon build --packages-select agt_interfaces agt_system_manager agt_map_manager agt_experiment_manager agt_web_console --symlink-install
source install/setup.bash
```

在另一个终端启动 ROS 管理器。它负责健康状态发布、运行模式 Action
以及有界重定位模式服务：

```bash
export AGT_WS="${AGT_WS:-$HOME/agt_navigation_v2}"
cd "$AGT_WS"
export AGT_RUNTIME_DIR="$PWD/runtime"
ros2 launch agt_system_manager system_manager.launch.py \
  active_mode:=IDLE runtime_dir:="$AGT_RUNTIME_DIR"
```

确认管理器已经提供运行模式 Action，再启动 Web 或点击建图：

```bash
ros2 action info /agt/system/change_mode
```

输出中必须包含 `Action servers: 1` 和 `/agt_system_mode_manager`。只有
`Action clients` 而没有 server 时，Web 会提示 `system_manager` 未运行，建图按钮不会启动任何算法。

FastAPI、Starlette 和 Uvicorn 不是基础 ROS 镜像的必需依赖。在目标环境中
安装这些依赖后，使用配置 YAML 启动 Web 进程：

```bash
export AGT_WS="${AGT_WS:-$HOME/agt_navigation_v2}"
cd "$AGT_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
source .venv-web/bin/activate
python install/agt_web_console/lib/agt_web_console/web_console.py \
  --config "$PWD/src/agt_web_console/config/web_console.yaml" \
  --backend ros
```

同一个 `runtime_dir` 只允许一个 Web 进程。ROS 真实后端和离线后端不能通过
两个终端同时启动；第二个实例会在创建 ROS bridge 之前退出。关闭 Web 时使用
启动它的终端发送 `Ctrl-C`，或先检查 `ss -ltnp | grep 8080`，不要同时打开
多个 Web 服务进程。

也可以使用 `ros2 run`，但必须先把 FastAPI、Starlette 和 Uvicorn 安装到
`ros2 run` 实际使用的 Python 环境；只激活 `.venv-web` 不会改变已安装
ROS 入口脚本的 shebang。

如果当前没有 MID360、车辆或 CAN，可以直接启动离线后端。它不要求真实
`agt_system_manager`，只用于检查网页按钮、流程状态和模拟重定位反馈：

```bash
export AGT_WS="${AGT_WS:-$HOME/agt_navigation_v2}"
cd "$AGT_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
source .venv-web/bin/activate
python install/agt_web_console/lib/agt_web_console/web_console.py \
  --config "$PWD/src/agt_web_console/config/web_console.offline.yaml" \
  --backend offline
```

浏览器访问 `http://127.0.0.1:8080/` 打开中文总控台；不要直接访问
`/api/v1/overview` 作为页面地址。总控台顶部按“系统管理器 -> 传感器 -> 建图或导航链
-> 地图与定位 -> 任务执行 -> 重定位”显示流程，每个可由 Web 安全触发的步骤都有对应按钮。
重定位页可以设置本次 Action 的候选模式、最大候选数和总超时，并只发送项目
`/agt/localization/relocalize` Action。

建图和导航链共享一个独立的 `sensor_only` 传感器进程组。切换主链时，系统管理器只停止旧的
建图/导航组并复用 MID360，不会先关闭再重新初始化雷达。网页的“启动底盘”选项同时作用于
建图和导航：默认不启动底盘控制。建图如果只需要轮速/CAN 状态，可打开“建图时只读监测 CAN”；
它不启动 `agt_safety`，也不发送任何底盘命令。真实导航才打开“启动底盘”，并由
`Nav2 -> agt_safety -> agt_chassis` 输出。

CAN 网卡不是 Web 可以代办的权限操作。首次部署由管理员使用驱动脚本或固定的
systemd/NetworkManager 配置完成接口初始化，之后在网页中只检查接口状态、BUNKER 连接和状态帧。
网页不会执行 `sudo`、`ip link`、`modprobe` 或任意 shell；因此“能否越过 sudo”的答案是不能也不应当，
应把受控初始化放在主机启动服务中。

建图状态不能只看 profile 启动成功。页面的“实时建图状态”依次检查 FAST-LIVO2 里程计、注册点云和
二维栅格地图；状态会显示“等待里程计”“等待注册点云”“等待二维地图”或“建图中”，并显示频率、
消息新鲜度和有界二维地图预览。预览来自 `/agt/map/mapping_occupancy`，不是可执行导航地图。

建图页还提供“降采样点云地图预览”。它消费注册点云并在 Web 进程内按固定体素保留有限数量的
`x/y/z` 点，仅用于观察建图是否在积累；画布支持 X-Y 俯视、X-Z/Y-Z 侧视、拖动、缩放和有限角度旋转，并显示消息的
`frame_id`、米制网格、原点和当前投影方向辅助线。它不是最终导航 PCD，也不会写入地图、定位或 Nav2。

建图链和导航链在控制台中是两个独立面板。传感器已经通过受管理进程、健康合同或当前主链证明启动时，
“启动传感器”按钮会变成禁用的“传感器已启动”，避免重复初始化。建图或导航运行时，另一个主链按钮也会
被锁定；建图状态不能通过普通停止按钮直接结束，必须点击“完成建图”并选择后续操作。

点击“完成建图”后，Web 会先显示当前会话状态、最终地图名称和“我确认本次采集已经完成”确认项；这一步不会因为误触就立即停止建图。
选择“保留并写入地图版本”时，Web 会先调用建图 launch 内的 `nav2_map_server/map_saver_server` 保存
`PGM/YAML`，随后正常停止建图进程，让 FAST-LIVO2 完成 `localization_map.pcd` 和
`localization_map.processing.yaml` 的落盘。页面持续显示 PGM/YAML、PCD 及 processing record 的状态，
只有 PCD record 为 `state: ready` 且全部资产存在时，才会调用 `agt_map_manager` 登记不可变地图版本。若受管 Web 会话中的 FAST-LIVO2 record 缺少 `pcd_sha256`，Web 会在建图进程正常停止后按实际 PCD 计算并补写摘要；已有摘要不一致时仍然失败闭环。
登记结果会提示版本 ID；“删除本次建图”只删除 `runtime/mapping_sessions/` 下的临时会话，不登记版本。
如果保留流程在登记阶段失败，会话会进入 `ERROR` 并保留证据供检查；此时建图页仍提供“删除残留文件”，它会在未生成 READY 版本的前提下清理临时会话及本次失败产生的无效版本目录。
离线后端可以先启动离线模拟建图、再在实验页选择完整 bag 执行“模拟回放”；建图预览会显示带有“模拟”标记的二维栅格和点云，便于检查拖动、缩放、机器人居中和结束建图按钮，但不会读取 bag 消息。离线保留最多占用一个模拟地图槽位，不能导出真实 PGM/YAML/PCD；删除后才能重新创建离线模拟地图。需要语义撰写、实车导航或重定位使用的真实资产，必须切换 ROS 2 后端按上一段流程生成并登记。

启动导航前，必须在“导航链”面板选择地图版本。下拉框只列出资产完整、状态为 `READY` 且已激活的版本，
启动请求中的 YAML、PCD 和 processing record 路径由版本 manifest 生成，浏览器不能自行替换这些路径。
服务器端还会再次校验版本状态、激活状态和三项资产的一致性。

页面顶部的“运行后端”可以在 `ROS 2 真实模式` 和 `离线测试模式`之间切换；
切换前必须先点击“停止受管理模块”。离线模式的“任务门禁”始终禁止执行，
离线重定位结果会明确标记为模拟结果，不能用于判断真实 PCD 定位精度。

局域网访问时，把配置中的 `host` 改为 `0.0.0.0`，并设置非空的随机
`token`；Web 页面打开后在“局域网访问令牌”输入框填写相同令牌。不要在
局域网配置中保留空 token：

```yaml
host: 0.0.0.0
port: 8080
token: "replace-with-a-long-random-token"
```

在其他设备上访问工控机的局域网 IP，例如
`http://192.168.x.x:8080/`。工控机防火墙还必须允许来自受信任局域网网段的
TCP 8080；不建议直接暴露到公共网络。

Web 进程只调用 `ChangeSystemMode`、`SetLocalizationMode` 和现有的有界
`Relocalize` Action，不自行执行 launch 命令。Localization 页面中的一次性
操作调用现有的 `/agt/localization/relocalize` Action。所有 launch 命令只能
来自系统管理器的 profile 配置。监听地址不是 loopback 时必须配置 token，
并通过 `X-AGT-Token` 访问；不要在局域网中暴露空 token 默认配置。

实车点击“仅启动传感器”时，Web 会等待 MID360 的健康合同；如果驱动初始化失败，
按钮会返回失败原因和受管理日志路径。MID360 的网络 JSON 必须使用实际连接雷达的
网卡地址，不能把示例地址直接当成当前工控机地址。启动前先确认：

```bash
ip -br addr
ping -c 4 <mid360-ip>
tail -f runtime/logs/system_manager/sensor_only.log
```

当前驱动输出 `/agt/sensors/lidar/custom`，健康检查也以该原始 `CustomMsg` 话题为准；
不要只看 `ros2 topic list`，还要确认 `ros2 topic hz /agt/sensors/lidar/custom` 和
`ros2 topic hz /agt/sensors/imu/data` 能收到数据。控制台的 MID360 网络配置输入框可以
传入白名单 `user_config_path`，用于选择与实车网卡匹配的 JSON。

激活地图前，必须先注册已经明确打包的地图版本：

```bash
ros2 run agt_map_manager map_registry.py --root /absolute/path/to/runtime/maps \
  register /absolute/path/to/runtime/maps/<map_id>/versions/<version>/manifest.yaml
ros2 run agt_map_manager map_registry.py --root /absolute/path/to/runtime/maps list
ros2 run agt_map_manager map_registry.py --root /absolute/path/to/runtime/maps \
  validate <map_version_id>
ros2 run agt_map_manager map_registry.py --root /absolute/path/to/runtime/maps \
  activate <map_version_id>
```

Map Library 通过 Web API 提供相同的受保护操作，包括 pin、归档、软删除和
显式 purge。active、pinned、processing、被子版本依赖以及被实验引用的版本，
始终由 registry 保护，不会被自动清理。

为保持兼容，`agt_bringup/launch/bag_record.launch.py` 仍然有效，并支持：
`bag_profile:=minimal|mapping|localization|navigation|full_experiment`。
录包始终使用版本化 profile 文件中的显式 topic 列表。

实验页可以在 ROS 2 后端列出 `runtime/rosbag/` 下带 `metadata.yaml` 的完整 bag，并使用固定的
`ros2 bag play --clock --rate <bounded-rate> <bag>` 回放。浏览器不能输入命令行。建图模式会自动追加
受控 `mapping_inputs` topic 白名单，只回放 `/clock`、`/tf_static`、`/agt/sensors/lidar/custom` 和
`/agt/sensors/imu/data`，不会重复回放 `/agt/mapping/odometry`、注册点云、二维地图或 `/tf`。导航模式
禁止 bag 回放。回放前应让被测节点使用 `use_sim_time:=true`，并单独确认传感器、定位、地图和安全门禁；
bag 回放不等于实车可执行。
离线测试后端现在可以选择一个完整 bag bundle 执行“离线模拟回放”，并在处于模拟建图模式时显示
明确标记的模拟地图预览；它仍只验证网页按钮、状态和流程提示，不会读取 bag 消息、发布 ROS topic、
录包或启动 `ros2 bag play`。若要用历史 bag 测试 ROS 节点，
应切换 ROS 2 后端，再由 ROS 后端回放入口启动真实 bag。

建图输入源可以在“实时 MID360 传感器”和“历史 rosbag”之间选择。选择历史 bag 时，Web 启动建图
算法链但传入 `start_sensor:=false`，并强制 `use_sim_time:=true`；FAST-LIVO2 会等待 bag 提供的输入话题，
因此传感器启动和算法启动不是强制的先后关系。建议先启动建图链，再点击实验页的指定 bag 回放，避免
回放已经结束后算法才启动。没有处于 `MAPPING` 主模式时，即使 bag 中包含点云或二维地图话题，两个 Web
预览也保持为空；停止建图会清除缓存预览。

二维栅格和降采样点云预览都支持鼠标/触控拖动，画布滚轮可缩放，“回到机器人位置”会使用当前
`/agt/mapping/odometry` 的位姿作为视图中心。该操作只改变浏览视图，不修改地图坐标或导航数据。

底盘通讯需要管理员先在工控机终端初始化 CAN，普通用户再在另一个终端启动 ROS 状态桥接。网页只生成
并复制命令，不执行 sudo 或 CAN 初始化：

```bash
# 终端 1：管理员
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0

# 终端 2：普通用户，只读 BUNKER 状态监测
source /opt/ros/humble/setup.bash
export AGT_WS=/absolute/path/to/agt_navigation_v2
source "$AGT_WS/install/setup.bash"
ros2 launch agt_chassis bunker.launch.py \
  can_interface:=can0 operation_mode:=monitor start_safety:=false \
  command_topic:=/agt/chassis/monitor_cmd_vel
```

真实导航控制时使用 `operation_mode:=control` 和明确的 `start_chassis:=true`，但仍必须经过
`Nav2 -> agt_safety -> agt_chassis`；不能把监测终端当作运动控制终端。

当前任务执行链不是 Qt5 直接下发底盘命令：Qt5 组织并提交 `map` 坐标系的 Pose 数组到
`/agt/navigation/execute_waypoint_task`，项目任务服务校验地图、定位、安全状态和取消语义后，
只调用 Nav2 `FollowWaypoints`。Nav2 输出经过 Collision Monitor、`agt_safety` 和底盘 watchdog，
最终才到 BUNKER。Web 当前只显示门禁和启动状态，不替代 Qt5 或项目任务 Action。

例如只用历史 bag 验证建图链时，先让测试节点不启动真实传感器和底盘，再在 Web 实验页回放：

```bash
ros2 launch agt_bringup system.launch.py mode:=mapping \
  start_sensor:=false start_chassis:=false start_chassis_monitor:=false \
  use_sim_time:=true record_bag:=false
```

导航链回放也应保持 `start_chassis:=false`，并且绝不在 bag 回放期间使能运动。需要验证真实
CAN/底盘通讯时不能把历史 bag 当作实车替代，必须切换到 `start_chassis:=true` 并进行现场安全验收。

基础验证不需要硬件：

```bash
PYTHONPATH=src/agt_system_manager python3 -m pytest -q src/agt_system_manager/test
PYTHONPATH=src/agt_map_manager python3 -m pytest -q src/agt_map_manager/test
PYTHONPATH=src/agt_experiment_manager python3 -m pytest -q src/agt_experiment_manager/test
PYTHONPATH=src/agt_web_console python3 -m pytest -q src/agt_web_console/test
```

FastAPI smoke 和浏览器检查需要额外安装 Web 依赖；离线合同测试不需要真实
CAN、MID360、机器人运动或其他硬件。

执行 ROS graph 级健康 smoke 时，先使用测试 runtime 以 `NAVIGATION` 模式
启动管理器，再运行已构建的模拟发布器：

```bash
ros2 launch agt_system_manager system_manager.launch.py active_mode:=NAVIGATION runtime_dir:=/absolute/path/to/runtime
ros2 run agt_system_manager health_smoke.py
ros2 param set /agt_relocalization drop_topic /agt/sensors/imu/data
ros2 param set /agt_relocalization drop_topic ""
```

smoke 节点会发布模拟传感器、底盘、安全、定位、地图、costmap 和 TF 数据，
并提供状态为 active 的模拟 Nav2 lifecycle 服务。丢弃一个必需 topic 后，
应能观察到消息过期或错误状态；恢复发布后可以验证健康状态恢复。部署环境
在可选前端或 rosbag 进程尚未运行时，仍可能显示 `WARN`，这是预期行为。
