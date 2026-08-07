# rosbag_sensor_trimmer

面向 LiDAR 与 IMU 数据的 rosbag2 统计、话题筛选、时间裁剪、压缩和完整性验证工具。

当前版本提供 rosbag2 裁剪、验证、Qt GUI 和可暂停播放。裁剪直接使用 `rosbag2_cpp` / `rosbag2_storage` 读取和写入序列化消息，不通过 `ros2 bag play` 与重新录制生成输出，因此未知消息类型也可以原样复制。播放使用 ROS 2 Humble 的 `rosbag2_transport::Player`，保留原始消息类型和 QoS 播放能力。

## 设计

- bag 读取：`rosbag2_cpp::Reader`，底层使用 `SequentialReader`；输入是 rosbag2 目录或 `metadata.yaml`。
- bag 写入：`rosbag2_cpp::Writer`，底层使用 `SequentialWriter`；压缩时使用 rosbag2 的 `SequentialCompressionWriter`。
- rosbag2_cpp：已作为核心依赖。
- Qt：使用 Qt5 Widgets 和可选 Qt OpenGL 实现裁剪、时间轴、IMU 曲线、里程计/TF 轨迹和 bag 播放界面；RViz、Foxglove 当前仍未嵌入。
- SQLite3：通过 `rosbag2_storage_default_plugins` 支持。
- MCAP：通过运行时 `rosbag2_storage_mcap` 插件支持；使用 MCAP 前必须安装对应 ROS 插件。
- 自定义消息：裁剪不需要安装消息类型，因为只复制序列化数据；Livox `livox_ros_driver2/msg/CustomMsg` 可被分类和播放检查，3D 预览目前仅解析 `PointCloud2`。

## 已实现功能

- 读取 metadata、storage ID、压缩信息、时间范围、消息数、文件大小和话题类型。
- 轻量扫描索引：只保存消息时间戳、话题名、序号和序列化长度，不把序列化 payload 全部放入内存。
- 通过消息类型识别 PointCloud2、Livox CustomMsg、Imu、LaserScan、TF 和 Odometry。
- 话题白名单、黑名单；黑名单优先。
- 相对 bag 起始记录时间和绝对纳秒时间裁剪。
- Qt GUI 内置 ROS1 Livox bag 转 ROS2 bag 工作台，默认输出项目标准传感器话题。
- 明确使用 rosbag2 记录接收时间，时间规则为 `start_time_ns <= timestamp < end_time_ns`。
- SQLite3 和 MCAP 输出；输出保留原话题名、消息类型、序列化格式、QoS metadata 和原始 serialized data。
- rosbag2 `zstd` file/message 压缩。
- 后台线程可复用的 `TrimWorker` 接口、进度回调和取消标志。
- 裁剪结果重新打开验证：metadata、storage、话题集合、消息数量、时间范围、时间单调性、LiDAR/IMU 区间重叠、IMU 覆盖 LiDAR 起点、频率和最大间隔。
- `trim_report.json` 与 `trim_report.md`。
- 话题级最大间隔和疑似断流检测；时间轴用红色区间标记断流，并可查看检测阈值。
- 播放时低开销监视 `nav_msgs/msg/Odometry` 和 `tf2_msgs/msg/TFMessage`，显示里程计轨迹和 TF frame 关系。
- 播放时可按消息条数单步或批量步进。
- `sensor_msgs/msg/PointCloud2` 三维预览为可选功能，默认关闭；只在用户勾选后订阅并将单帧点数限制为最多 60000 个。

## 依赖安装

目标环境为 Ubuntu 22.04 / ROS 2 Humble：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-rosbag2 \
  ros-humble-rosbag2-cpp \
  ros-humble-rosbag2-transport \
  ros-humble-rosbag2-storage-default-plugins \
  ros-humble-rosbag2-storage-mcap \
  ros-humble-rosbag2-compression-zstd \
  ros-humble-nav-msgs \
  ros-humble-tf2-msgs \
  qtbase5-dev \
  libqt5opengl5-dev \
  libqt5charts5-dev \
  libzstd-dev
```

MCAP 是可选运行时能力；未安装 `rosbag2_storage_mcap` 时仍可构建并使用 SQLite3 输入输出。

## 构建

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install --packages-select rosbag_sensor_trimmer
source install/setup.bash
```

## 使用方式

### Qt GUI 使用

当前版本提供 Qt5 Widgets 图形界面，入口为 `rosbag_sensor_trimmer_gui`。界面支持选择输入 bag、输出目录、时间范围、保留话题、输出 storage、zstd 压缩和覆盖策略；读取完成后可以查看话题统计、消息时间轴和 IMU 运动曲线。IMU 页面显示加速度模长与角速度模长，自动估计车辆开始运动时间，也可以点击曲线手动选择裁剪起点；裁剪、验证和播放都在后台线程执行。另有独立的“ROS1 转换”页，可把 Livox ROS1 分片 bag 直接转换为项目标准 ROS2 话题，再回读到原有 bag 分析页。

启动 GUI：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rosbag_sensor_trimmer rosbag_sensor_trimmer_gui
```

当 bag 含有 `livox_ros_driver2/msg/CustomMsg` 等工作区内自定义消息时，必须在启动 GUI
之前 source 提供消息定义和 typesupport 库的 overlay。只在另一个终端 source 不会影响已经
运行的 GUI。可使用启动脚本显式加载一个或多个 overlay：

```bash
scripts/run_gui.sh --overlay /path/to/navigation_workspace/install
```

也可以在构建本项目之前加载该 overlay，使生成的 `install/setup.bash` 记录完整 underlay：

```bash
source /opt/ros/humble/setup.bash
source /path/to/navigation_workspace/install/setup.bash
colcon build --symlink-install --packages-select rosbag_sensor_trimmer
source install/setup.bash
```

GUI 的“播放支持”列会按 rosbag2 实际使用的 C++ typesupport 检查每种消息类型。所选类型
不可用时，播放默认中止并列出缺失包；用户仍可明确选择跳过，但若因此没有 LiDAR，界面会
说明里程计、PCD、二维地图与 map saver 不会产生结果，避免把下游超时误判为保存故障。

也可以启动时预填路径并自动读取输入 bag：

```bash
ros2 run rosbag_sensor_trimmer rosbag_sensor_trimmer_gui \
  --input /data/original_bag \
  --output /data/trimmed_bag
```

读取 bag 后，在左侧“bag 播放”区域可以：

- 使用话题表的勾选结果作为播放白名单，只播放选中的话题。
- 默认“启动时暂停”，先确认时间轴位置后再点击“继续播放”；取消该选项可以直接播放。
- 点击“暂停播放”随时暂停，使用倍速控件调整 `0.1x` 到 `10.0x` 播放速度。
- 拖动时间滑块后点击“跳转”定位到指定记录时间；“单步”只播放下一条消息，适合检查时间点和消息顺序。
- 将“步进条数”设置为大于 1 后，“单步”会一次发布指定数量的消息，仍保持暂停状态。
- 使用“停止”结束当前播放器；启用“循环播放”可以循环播放当前 bag。
- 话题统计会显示每个话题的最大间隔和疑似断流段数；消息时间轴中的红色区域表示按该话题频率推断出的异常间隔。
- “里程计 / TF”页在播放时显示轨迹和最近的 TF parent/child 关系。
- “3D 点云（可选）”页默认不启动点云订阅。勾选“启用 3D 点云预览”后选择一个 `sensor_msgs/msg/PointCloud2` 话题，播放时可拖动旋转视角、滚轮缩放。

播放器会发布 `/clock`，因此需要查看仿真时间的 ROS 节点设置 `use_sim_time=true`。播放要求所选话题的消息类型在 GUI 启动环境中可发现；对于 `livox_ros_driver2/msg/CustomMsg` 等自定义消息，如果没有 source 对应 overlay，仍可以裁剪，但 rosbag2 不能创建对应发布者。

时间轴、断流检测和 IMU 曲线都使用 rosbag2 记录接收时间。启动点检测是基于初始静止基线的启发式估计，使用前应在曲线上人工确认。Livox `CustomMsg` 可以直接裁剪和统计；3D 预览当前解析 `sensor_msgs/msg/PointCloud2`，要播放或解析 Livox `CustomMsg` 仍需要安装对应的 `livox_ros_driver2` 消息包。

### CLI 使用

只读取并统计 bag：

```bash
ros2 run rosbag_sensor_trimmer rosbag_sensor_trimmer_cli \
  --input /data/original_bag --info
```

相对 bag 起始记录时间裁剪，`--start`/`--end` 单位为秒：

```bash
ros2 run rosbag_sensor_trimmer rosbag_sensor_trimmer_cli \
  --input test_data/bags/example_bag \
  --output test_data/output/example_trimmed \
  --start 5.0 \
  --end 30.0 \
  --topics /livox/lidar /livox/imu /tf_static \
  --output-storage sqlite3 \
  --compression zstd \
  --verify \
  --report test_data/output/example_trimmed_report.json
```

绝对记录时间裁剪：

```bash
ros2 run rosbag_sensor_trimmer rosbag_sensor_trimmer_cli \
  --input /data/original_bag \
  --output /data/trimmed_bag \
  --start-ns 1710000000000000000 \
  --end-ns 1710000010000000000 \
  --output-storage sqlite3 \
  --verify
```

`--start/--end` 与 `--start-ns/--end-ns` 不能混用。未提供 `--topics` 表示保留所有话题；`--exclude-topics` 在白名单之后应用。裁剪时省略 `--output` 会按 `原名称_trimmed_开始秒_结束秒` 生成默认输出目录，永远不会直接覆盖输入目录，已有输出必须显式使用 `--overwrite`。

估算而不写文件：

```bash
ros2 run rosbag_sensor_trimmer rosbag_sensor_trimmer_cli \
  --input /data/original_bag --start 35 --end 185 \
  --topics /livox/lidar /livox/imu --dry-run
```

`--dry-run` 只扫描并估算消息数，不写输出文件。

验证已有 bag：

```bash
ros2 run rosbag_sensor_trimmer rosbag_sensor_trimmer_cli \
  --input /data/trimmed_bag --verify-only \
  --report /tmp/trimmed_report.json
```

帮助命令：

```bash
ros2 run rosbag_sensor_trimmer rosbag_sensor_trimmer_cli --help
```

### ROS 1 Livox 分片 bag 转 ROS 2

GUI 的 “ROS1 转换” 页和 `scripts/convert_ros1_livox_bag_to_ros2.py` 使用同一套转换逻辑。
它们可直接读取 ROS 1 `#ROSBAG V2.0` `.bag`/`.bag.active` 分片，解析
`livox_ros_driver2/CustomMsg` 和 `sensor_msgs/Imu`，写出 ROS 2 `rosbag2`
SQLite3 bag。该工具不依赖 ROS 1，但需要先 source 含
`livox_ros_driver2/msg/CustomMsg` 的 ROS 2 overlay。

默认话题映射为项目标准传感器输入：

```text
/livox/lidar -> /agt/sensors/lidar/custom
/livox/imu   -> /agt/sensors/imu/data
```

同时会把 ROS 1 类型名转换为 ROS 2 metadata 类型名：

```text
livox_ros_driver2/CustomMsg -> livox_ros_driver2/msg/CustomMsg
sensor_msgs/Imu            -> sensor_msgs/msg/Imu
```

先统计不写文件：

```bash
source /opt/ros/humble/setup.bash
source /path/to/navigation_workspace/install/setup.bash
python3 scripts/convert_ros1_livox_bag_to_ros2.py \
  --input "$ROS1_BAG_SPLIT_DIR" \
  --dry-run \
  --report /tmp/ros1_livox_dry_run.json
```

转换一个小样本做回放冒烟：

```bash
python3 scripts/convert_ros1_livox_bag_to_ros2.py \
  --input "$ROS1_BAG_SPLIT_DIR" \
  --output "$OUTPUT_ROSBAG2_DIR" \
  --duration 60 \
  --report "$OUTPUT_ROSBAG2_DIR.report.json"
```

转换完整数据集：

```bash
python3 scripts/convert_ros1_livox_bag_to_ros2.py \
  --input "$ROS1_BAG_SPLIT_DIR" \
  --output "$OUTPUT_ROSBAG2_DIR" \
  --report "$OUTPUT_ROSBAG2_DIR.report.json"
```

默认只输出 LiDAR 和 IMU，不转换左右相机图像。LIO-only FAST-LIVO2 离线回放
应优先选完整关闭、无 `.bag.active` 的分片数据集；`.bag.active` 可被顺序扫描，
但只适合作为补充测试，不能替代完整闭合数据集的基准样本。

## 存储与压缩

SQLite3 和 MCAP 都通过 rosbag2 storage plugin 选择。输入 storage ID 默认从 `metadata.yaml` 读取，输出可由 `--output-storage` 显式指定。MCAP 不是本包内置的文件格式解析器，运行时必须安装 `rosbag2_storage_mcap`。

`--compression zstd --compression-mode file` 使用 rosbag2 file compression；`message` 使用逐消息压缩。压缩不会改变输出中的 ROS 消息类型或 serialized payload 语义。MCAP 原生压缩配置与 rosbag2 compression mode 是两个层次，建议在部署环境实测后选择。

## 时间基准

裁剪、时间轴、断流检测和 IMU 分析默认使用 rosbag2 存储的记录接收时间 `SerializedBagMessage::time_stamp`，不是 `header.stamp`。原因是不同传感器可能使用不同时间源，header 时间戳可能异常或未填充。里程计/TF 监视仍显示消息自身的 frame 信息。

## 报告

验证报告包含：

- metadata 是否存在、输出是否可重新打开。
- 话题名称、类型、序列化格式和每话题消息数量。
- 时间范围半开边界、时间戳单调性和零消息话题。
- 输出 storage ID 是否符合要求。
- LiDAR 与 IMU 的时间区间重叠、IMU 是否覆盖 LiDAR 起点。
- IMU/LiDAR 平均频率、最大消息间隔以及断流警告。

## 测试

单元测试覆盖相对/绝对时间转换、半开边界、时间范围合法性、话题类型分类、白名单/黑名单和默认输出名称：

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select rosbag_sensor_trimmer
colcon test-result --verbose
```

真实 bag 冒烟测试：

```bash
export ROSBAG_SENSOR_TRIMMER_TEST_BAG=/absolute/path/to/small_bag
scripts/smoke_test.sh
```

没有测试 bag 时，冒烟脚本只执行构建和单元测试，并明确跳过真实 bag 集成步骤，不伪造 `ros2 bag info` 或播放结果。

## 架构

```text
Qt GUI
        |
PlaybackTask -> rosbag2_transport::Player -> selected topics + /clock
        |
PlaybackMonitorTask -> Odometry / TF
                   \-> optional PointCloud2 -> Qt OpenGL preview
        |
TrimWorker + IntegrityValidator
        |
BagReader / BagIndex / GapAnalysis / TopicFilter / TimeRange
        |
rosbag2_cpp + rosbag2_transport + rosbag2_storage + compression plugins
```

裁剪核心与播放/预览层保持解耦；即使没有 `livox_ros_driver2`，基础裁剪仍能编译并复制原始消息。播放层会在启动前检查所选消息的 typesupport，避免静默丢失传感器话题。

## 许可证

Apache-2.0，见 [LICENSE](LICENSE)。
