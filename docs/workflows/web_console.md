# Web 控制台工作流

构建并加载工作区环境：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select agt_interfaces agt_system_manager agt_map_manager agt_experiment_manager agt_web_console --symlink-install
source install/setup.bash
```

在另一个终端启动 ROS 管理器。它负责健康状态发布、运行模式 Action
以及有界重定位模式服务：

```bash
ros2 launch agt_system_manager system_manager.launch.py \
  active_mode:=IDLE runtime_dir:=/absolute/path/to/runtime
```

FastAPI、Starlette 和 Uvicorn 不是基础 ROS 镜像的必需依赖。在目标环境中
安装这些依赖后，使用配置 YAML 启动 Web 进程：

```bash
ros2 run agt_web_console web_console.py \
  --config "$PWD/src/agt_web_console/config/web_console.yaml"
```

如果当前没有 MID360、车辆或 CAN，可以直接启动离线后端。它不要求真实
`agt_system_manager`，只用于检查网页按钮、流程状态和模拟重定位反馈：

```bash
ros2 run agt_web_console web_console.py \
  --config "$PWD/src/agt_web_console/config/web_console.yaml" \
  --backend offline
```

浏览器访问 `http://127.0.0.1:8080/` 打开中文总控台；不要直接访问
`/api/v1/overview` 作为页面地址。总控台顶部按“系统管理器 -> 传感器 -> 建图或导航链
-> 地图与定位 -> 任务执行 -> 重定位”显示流程，每个可由 Web 安全触发的步骤都有对应按钮。
重定位页可以设置本次 Action 的候选模式、最大候选数和总超时，并只发送项目
`/agt/localization/relocalize` Action。

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
