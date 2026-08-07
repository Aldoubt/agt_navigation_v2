# FAST-LIVO2 前置 URDF 自身点云滤除

本模块只处理 Livox `CustomMsg` 的机器人自身返回点，不修改
`third_party/fast_livo2_ros2`，也不替代 FAST-LIVO2 后的
`agt_perception/local_obstacle_filter`。

## 数据链

```text
/agt/sensors/lidar/custom (raw CustomMsg)
  -> agt_livox_self_filter
/agt/sensors/lidar/custom_filtered (filtered CustomMsg)
  -> FAST-LIVO2
/agt/mapping/registered_points
  -> agt_perception/local_obstacle_filter
  -> Nav2
```

`/agt/sensors/lidar/custom` 永远保留为原始输入。自滤除节点默认由
`agt_mapping/fast_livo2_mapping.launch.py` 启动，因此真实 MID360 和
`start_sensor:=false` 的历史 bag 回放使用同一处理链。FAST-LIVO2 正常路径只消费
`/agt/sensors/lidar/custom_filtered`。

有两级 A/B 开关：

- `start_lidar_self_filter:=false`：完全绕过自滤除，FAST-LIVO2 直接消费 raw CustomMsg；
- `lidar_self_filter_geometry_source:=profile`：保留自滤除，但回退到旧 profile-box 几何；
- 默认 `lidar_self_filter_geometry_source:=urdf`：使用 URDF collision 主体几何。

## 几何来源与单一职责

默认路径的机器人主体几何来自 `robot_description` 中的 URDF `<collision>`。节点订阅
`/robot_description`，默认以 `base_link` 作为过滤参考坐标系，并按点云时间戳查询各 collision
link 到 `base_link` 的 TF。因此未来存在固定或活动 link 时，过滤判断仍可按 URDF link 的真实
姿态执行。

当前实时实现只接受显式 primitive collision：

- `box`
- `sphere`
- `cylinder`

URDF `mesh` collision 会 fail-closed 拒绝加载，不会静默近似。需要使用 mesh 外形的机器人，应在
URDF 中为 self-filter 提供明确的 primitive collision proxy，再经过 bag/实车验证后启用。

`profiles/platforms/<platform>.yaml` 在 URDF 模式下不再生成第二个 chassis body 参与过滤。
profile 继续提供：

- self-filter enable/padding 策略；
- 与导航几何分离的显式临时补充 box，例如当前尚未实测确认的 BUNKER rear high platform；
- `geometry_source:=profile` A/B 回归时的完整旧几何来源。

因此不会同时维护“URDF chassis box”和“profile chassis box”两个主体真源。BUNKER 的
`base_length/base_width/base_height` 仍由 description 配置注入 Xacro，并由 contract test 与
`profiles/platforms/bunker.yaml` 的已验证物理尺寸保持一致。

## 坐标与 CustomMsg 合同

过滤判断与 FAST-LIVO2 输入坐标是两个不同概念：

```text
原始 Livox point
      |
      | 临时 TF，仅用于 inside-collision 判断
      v
base_link / URDF collision link
      |
      +--> self return ? remove : keep

keep
  -> 从原始 CustomMsg 按原顺序复制原始 CustomPoint
  -> /agt/sensors/lidar/custom_filtered
```

节点不会把通过点永久改写成 `base_link` 坐标。输出继续保留原始 message header/frame，以及
每个 `CustomPoint` 的原始 XYZ、`offset_time`、`line`、`tag`、`reflectivity`；消息级
`timebase`、`lidar_id` 和其他字段也通过原消息复制保留。这样 FAST-LIVO2 的 LiDAR/IMU 外参和
逐点去畸变语义不会因 self-filter 被改变。

## Fail-closed 行为

以下情况默认整帧不进入 FAST-LIVO2：

- 尚未收到有效 `robot_description`；
- URDF 解析失败或没有支持的 collision；
- collision 使用未显式代理的 mesh；
- 点云 frame 到过滤参考 frame 的 TF 不可用；
- URDF collision link 的 TF 不可用。

`fail_open_on_tf_error:=true` 仅用于显式调试/A-B；它会在几何或 TF 前提不可用时透传原始消息，
不应作为实车默认配置。

零点占位 `CustomPoint` 默认按无效点删除，非有限点同样拒绝。

## Debug 与 diagnostics

package-local/debug topic：

- `/agt/sensors/lidar/self_filter/geometry`：`visualization_msgs/msg/MarkerArray`，transient-local；
  URDF primitive 使用其各自 link frame，profile supplemental box 使用 profile frame；
- `/agt/sensors/lidar/self_filter/removed_points`：可选 `sensor_msgs/msg/PointCloud2`，默认关闭；
- `/diagnostics` 中的 `agt_livox_self_filter`：记录 `geometry_source`、geometry readiness、过滤参考
  frame、输入/输出点数、移除比例、URDF primitive 数量、supplemental box 数量、TF/几何失败次数和耗时。

注意 `/agt/sensors/lidar/custom_filtered` 的 output message frame 仍是输入 Livox frame；
`base_link` 只是默认过滤参考 frame。

## 启动前提

URDF 模式要求 `robot_state_publisher` 已发布可用 `robot_description` 和对应 TF。由于
`robot_description` 使用 transient-local 语义，self-filter 后启动也应能取得最后有效描述；在描述
到达前节点保持 fail-closed。

独立运行可显式回退旧 profile 几何：

```bash
ros2 launch agt_sensor_adapters lidar_self_filter.launch.py \
  geometry_source:=profile
```

FAST-LIVO2 mapping A/B：

```bash
# 默认 URDF
ros2 launch agt_mapping fast_livo2_mapping.launch.py \
  lidar_self_filter_geometry_source:=urdf

# 旧 profile box
ros2 launch agt_mapping fast_livo2_mapping.launch.py \
  lidar_self_filter_geometry_source:=profile

# 完全关闭
ros2 launch agt_mapping fast_livo2_mapping.launch.py \
  start_lidar_self_filter:=false
```

## 验收重点

同一原始 bag 至少做 `raw / profile / urdf` 三组对照，检查：

- CustomMsg 输出字段、顺序、message frame 与输入一致；
- 被删点在 RViz 中确实落在机器人 collision/supplemental geometry 内；
- 不误删机器人附近的真实地面、垄体和障碍；
- URDF/profile 过滤比例、CPU 与单帧耗时；
- TF/robot_description 缺失时 fail-closed；
- FAST-LIVO2 轨迹、最终 PCD、自车残影和重定位质量。

新地图和处理结果必须单独保存，不能覆盖旧 baseline。URDF 模式完成 bag 与实车验收前，V2.5
状态仍应描述为 `implemented / pending validation`，不能直接标记为 vehicle-validated DONE。
