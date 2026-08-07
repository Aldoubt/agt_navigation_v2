# FAST-LIVO2 前置 BUNKER 车体点云自滤除

本阶段只处理 Livox `CustomMsg` 的车体自返回点，不修改
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
`start_sensor:=false` 的历史 bag 回放使用相同处理链。FAST-LIVO2 只消费
`/agt/sensors/lidar/custom_filtered`。显式设置
`start_lidar_self_filter:=false` 仅用于 A/B 基线，此时 FAST-LIVO2 回退到原始 topic。

## 几何与安全边界

`profiles/platforms/bunker.yaml` 是唯一几何来源。`geometry.self_filter` 的
`include_chassis_body` 根据物理 `length/width/height` 生成
`base_footprint` 下的底盘盒；显式 box 描述高台等额外结构。`padding` 只扩展
过滤盒，不改变 URDF、物理尺寸或 Nav2 `navigation_footprint`。高台盒当前为
`verified: false`，直到完成实测前必须在 diagnostics 中保持可见。

profile 缺失、非有限值、向量长度错误或 `min >= max` 都会阻止节点启动并报告
具体字段。

## 运行行为

- 节点只在帧切换时或首次遇到帧时查询并缓存 `base_footprint <- input_frame` TF。
- 不查询 `map`、`odom` 或 FAST-LIVO2 当前位姿，不发布 TF。
- 过滤启用且 TF 不可用时默认整帧丢弃；`fail_open_on_tf_error` 只有显式开启才透传。
- 零点占位 `CustomPoint` 默认按无效点删除；其他通过点按原始顺序完整复制，包括 `offset_time`、`line`、`tag` 和
  `reflectivity`；消息级 header、timebase、lidar_id 和 reserved 字段也保留。
- removed cloud 默认关闭；盒子以 transient-local `MarkerArray` 发布，运行统计（含无效点数量）
  发布到 `/diagnostics`。

本任务不实现复杂地图时序滤波。后续若引入时序点云产品，必须继续与原始输入、
FAST-LIVO2 输入和 Nav2 即时局部障碍链分离，并增加有界资源、持久化和 bag 回归合同。

## 验收重点

应分别使用实车驱动和只回放原始 `/agt/sensors/lidar/custom`、IMU、`/tf_static` 的历史
bag，检查原始点计数、过滤点计数、CustomMsg 字段顺序、TF 失败 fail-closed、QoS、
FAST-LIVO2 轨迹以及最终 PCD 处理记录。新地图必须单独保存，不能覆盖旧地图。
