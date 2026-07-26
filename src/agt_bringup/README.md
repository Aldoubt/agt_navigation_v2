# agt_bringup

职责：组合启动、生命周期编排和系统级健康检查入口。

统一入口：

```bash
ros2 launch agt_bringup system.launch.py mode:=mapping map_name:=greenhouse_01
ros2 launch agt_bringup system.launch.py mode:=navigation \
  map:=/absolute/path/map.yaml \
  global_map_pcd:=/absolute/path/map.pcd \
  global_map_processing_record:=/absolute/path/localization_map.processing.yaml \
  auto_relocalize_on_start:=false
```

自动和手动重定位的完整操作、候选 YAML、`/initialpose` 命令和技术差异见
[`docs/workflows/relocalization_usage.md`](../../docs/workflows/relocalization_usage.md)。自动启动默认关闭；
开启 `auto_relocalize_on_start:=true` 只会发送一次有界 Action，不代表无先验全地图搜索。

`mapping` 启动传感器、FAST-LIVO2 PCD 保存、二维投影和 RViz；底盘控制默认关闭。可选
`start_chassis_monitor:=true` 只接收 BUNKER CAN 状态，不启动安全/控制链。实车建图若确需
轮速状态输入，使用该监测选项；不要把监测话题当作速度输出。可用
`start_mapping_gui:=true` 额外启动只监视/编辑、禁止导航任务执行的 mapping Qt profile。
`navigation` 关闭 PCD 保存，启动 LIO 里程计、重定位、Nav2 和 Qt5；真实底盘控制必须显式
设置 `start_chassis:=true`，并经过 `agt_safety`。两种模式均支持 `chassis_backend:=bunker_can|none`
和 `can_interface:=<interface>`，为未来底盘/通讯协议替换保留配置边界。
两个模式均可设置 `record_bag:=true`，输出到 `runtime/rosbag/`。

录包入口兼容原调用，并支持版本化 `bag_profile:=minimal|mapping|localization|navigation|teach_repeat|full_experiment`。
profile 只包含显式 topic 名称，不接受任意 rosbag 参数或 `record -a`。

低速示教路线复扫使用 `teach_mapping_rescan.launch.py`。该入口只用 Session 绑定的 Bootstrap
Map 启动一套 navigation，并组合 `repeat_test.launch.py`；默认底盘关闭、执行关闭、录包开启，
`auto_start` 固定关闭且速度上限为 `0.10 m/s`。完整现场步骤见
[`docs/testing/teach_mapping_mvp_field_test.md`](../../docs/testing/teach_mapping_mvp_field_test.md)。

原导航默认保持语义与覆盖模块关闭。启动完整覆盖作业链前，先 source TASK-08 的外部依赖工作区：

```bash
source /opt/ros/humble/setup.bash
source /path/to/agt_coverage_ws/install/setup.bash
source install/setup.bash

ros2 launch agt_bringup system.launch.py \
  mode:=navigation \
  map:=/absolute/path/greenhouse_01.yaml \
  global_map_pcd:=/absolute/path/localization_map.pcd \
  global_map_processing_record:=/absolute/path/localization_map.processing.yaml \
  semantic_map:=/absolute/path/semantic_map.geojson \
  coverage_params:=/absolute/path/coverage.yaml \
  start_semantic_map_server:=true \
  start_coverage_planning:=true
```

`coverage_params` 必须是 `semantic_map` 同目录的 `coverage.yaml`，因为语义服务器和请求适配器
以二者作为一个原子任务加载。顶层会在启动任何子系统前检查地图、PCD、GeoJSON、coverage 和
platform profile；覆盖规划不能脱离语义服务器启动。

进程按 Nav2、语义服务器、覆盖规划的所有者顺序加入同一 launch。运行时 readiness 由
`map_server active -> localization TRACKING/accepted -> safety localization guard -> semantic LOADED -> keepout mask -> global costmap ->
coverage planner` 链共同决定；进程存在不等于可执行。只有下列检查通过后才允许手动使能安全层：

```bash
ros2 lifecycle get /map_server
ros2 topic echo /agt/localization/status --once
ros2 topic echo /agt/map/semantic_status --once
ros2 topic echo /agt/map/keepout_mask --once --field info
ros2 lifecycle get /planner_server
ros2 action info /agt/coverage/execute
```

`start_coverage_planning:=true` 且 `annotation_mode:=false` 时总控会允许 TASK-14 进入执行门禁，
但仍不会自动调用 `/agt/safety/set_motion_enabled`；上述 readiness、定位和现场检查必须先完成。

标注模式使用项目语义编辑器替代普通 Qt5 操作界面，并强制覆盖 Action 的执行开关为 false：

```bash
ros2 launch agt_bringup system.launch.py \
  mode:=navigation map:=/absolute/path/greenhouse_01.yaml \
  global_map_pcd:=/absolute/path/localization_map.pcd \
  global_map_processing_record:=/absolute/path/localization_map.processing.yaml \
  semantic_map:=/absolute/path/semantic_map.geojson \
  coverage_params:=/absolute/path/coverage.yaml \
  start_semantic_map_server:=true annotation_mode:=true
```

保存标注后调用 `/agt/map/semantic/reload` 或重新启动作业模式。正常 `Ctrl+C` 会关闭同一进程树
中的覆盖与 Nav2 Action Server；安全层和底盘 watchdog 会将残余速度归零。禁止使用 `kill -9`。

建图工作图使用 `/agt/map/mapping_occupancy`，导航 `map_server` 使用
`/agt/map/global_occupancy`，两者不会互相显示旧的 transient-local 地图。

二维地图必须在建图仍运行时保存：

```bash
ros2 launch agt_bringup save_mapping_result.launch.py map_name:=greenhouse_01
```

保存会拒绝覆盖已有的 PGM/YAML；FAST-LIVO2 也会拒绝非空的 PCD 输出目录。重复采集必须使用
新的 `map_name`，确认二维图保存成功后再对建图总控使用 `Ctrl+C`，这样 PGM/YAML 与本次退出
生成的 PCD 始终属于同一份采集结果。

确认二维地图保存成功后，再对建图总控使用 `Ctrl+C`；PCD 将保存到
`runtime/maps/<map_name>/pcd/`，rosbag 也会完成元数据写入。导航前必须同时检查
`localization_map.pcd` 和 `localization_map.processing.yaml`，且处理状态为 `ready`。不要先关闭总控再保存二维地图。

无雷达或 CAN 时可分别使用 `start_sensor:=false`、`start_chassis:=false`。建图无显示器时
使用 `start_rviz:=false`；mapping Qt 默认关闭，按需使用 `start_mapping_gui:=true`。导航无
显示器时使用 `start_gui:=false`。
运行总控后禁止再单独启动 description/chassis launch；真实运动前仍需显式使能安全层。
