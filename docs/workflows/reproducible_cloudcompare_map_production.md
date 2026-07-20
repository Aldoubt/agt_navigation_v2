# 可重复的 PCD、CloudCompare 与 Nav2 地图生产流程

## 目标与原则

本流程用于重复生产彼此严格同坐标的定位 PCD、导航 PGM/YAML 和后续语义地图。正式链路不依赖四点手工补偿：

```text
原始传感器 bag + 前端输出
  -> 可选后端关键帧/位姿图优化
  -> 优化后的完整 PCD（原始前端坐标）
  -> 一次性 T_raw_to_map（找平 + 可选温室长轴定向）
  -> aligned_full.pcd（定位地图）
  -> 只删除点的 ROI/Z 裁切
  -> nav_source.pcd
  -> CloudCompare Rasterize
  -> observed.png + 完整栅格元数据
  -> 三值 navigation.pgm + navigation.yaml
  -> PCD/PGM 地标验收 + 定位回放
  -> 冻结 manifest
```

硬约束：

- 找平和定向只产生一个可追溯的刚体变换，应用到完整 PCD 一次。
- 若源 PCD 的 `VIEWPOINT` 不是单位位姿，必须先将该位姿并入 `T_raw_to_map` 并烘焙到 XYZ/法向量；冻结输出必须使用单位 `VIEWPOINT`。
- `aligned_full.pcd` 生成后只允许删除点，不再平移、旋转、缩放或交换坐标轴。
- Rasterize 后禁止图片旋转、翻转、缩放、裁边和“适配内容”。
- 四点工具只用于验收或诊断，不作为正式地图生产变换来源。
- PGM/YAML 和定位 PCD 必须由同一份 `aligned_full.pcd` 派生。

## 1. 现场采集与前端输出

使用带版本的地图名启动，开启 bag：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch agt_bringup system.launch.py \
  mode:=mapping map_name:=greenhouse_20260718_v01 \
  record_bag:=true start_mapping_gui:=true start_rviz:=true
```

采集路线应覆盖外墙、四角、入口、每条主通道以及回到起点的闭环；避免只沿温室单方向直线行驶。正常流程是先保存二维参考图，再对总 launch 使用 `Ctrl+C`，等待 FAST-LIVO2 写完：

```text
runtime/maps/<map_id>/pcd/localization_map.pcd
runtime/maps/<map_id>/pcd/localization_map.processing.yaml
runtime/rosbag/mapping_<timestamp>/
```

`localization_map.processing.yaml` 必须为 `state: ready`。当前前端在运行期增量累计体素质心，
不再为退出时的整云降采样长期保留原始点。需要更高密度或后端重优化时，bag 是权威数据源，
应从 bag 重放并输出新的可追溯 PCD，不要回退使用曾经发生整数索引溢出的旧
`all_downsampled_points.pcd`。

bag 至少必须包含且消息数非零：

- `/agt/sensors/lidar/custom`
- `/agt/sensors/imu/data`
- `/tf_static`

当前系统还会尝试记录 `/agt/mapping/odometry`、注册点云、局部雷达点云和 OccupancyGrid。使用 `ros2 bag info <bag>` 检查实际消息数；topic 出现在 metadata 但计数为零不算有效数据。

## 2. 后端优化数据契约

原始雷达/IMU bag 是可重放真值来源；全局 PCD 只是前端结果，不能替代关键帧图。后续位姿图、回环或局部 BA 至少需要保存：

- 稳定 keyframe ID 与传感器时间戳；
- 每个关键帧在 lidar/base 局部坐标下的点云，不能只保存已拼接世界点；
- 前端位姿 `T_odom_sensor`、frame 约定、质量指标或协方差；
- 时间同步和 lidar/IMU/base 外参快照；
- 关键帧顺序邻接、回环候选，以及关键帧与局部地图/体素的可见性关联；
- 优化前后位姿和优化器配置、版本、残差报告。

当前 FAST-LIVO2 基线提供连续 odometry、注册点云和退出时全局 PCD，但**尚未输出完整的关键帧局部云及可见性数据集**。在该导出器实现前，可以依靠原始 bag 重放前端，却不能宣称已具备可重复的后端关键帧图优化输入。

后端优化完成时，应使用“优化后关键帧局部云 + 优化后位姿”重新拼接完整 PCD；不要对旧全局 PCD做非刚性手工拉伸。

## 3. 初始化生产项目

对确定使用的前端或优化后完整 PCD 建立不可覆盖的阶段项目：

```bash
python3 tools/map_tools/init_map_production.py \
  --map-id greenhouse_20260718_v01 \
  --raw-pcd runtime/maps/greenhouse_20260718_v01/pcd/localization_map.pcd \
  --source-bag runtime/rosbag/mapping_<timestamp> \
  --grid-step 0.05
```

`--raw-pcd` 是工具的历史参数名，表示本次地图生产项目的不可覆盖源 PCD；对当前基线它就是
已通过处理记录验收的 `localization_map.pcd`。

输出目录：

```text
runtime/map_projects/<map_id>/
├── 00_source
├── 10_alignment
├── 20_aligned
├── 30_nav_source
├── 40_raster
├── 50_nav2
├── 60_validation
└── map_production.yaml
```

初始化会记录源 PCD 与 bag metadata 哈希。项目存在时工具拒绝覆盖；重做必须使用新版本 map ID。

## 4. CloudCompare 找平与定向

### 4.1 找平

1. 打开完整 PCD，不在原对象上直接删除点。
2. 克隆或分割一小块可靠硬质地面作为 `ground_seed`。不要使用田垄顶、坡道或植被。
3. 用地面 seed 求平面/Level 变换，保存 CloudCompare 给出的全精度 4x4 `T_level`。
4. 撤回到完整原始 PCD，把完全相同的 `T_level` 应用到完整云。
5. 检查地面法向接近 `+Z`，并保留变换前源 PCD。

### 4.2 温室长轴定向（可选但建议一次完成）

如果希望作物行与地图 X 轴平行，在已找平云上选择同一侧长墙或固定梁的两个远距离端点：

```text
yaw_reference = atan2(y2-y1, x2-x1)
T_yaw = Rz(-yaw_reference)
T_raw_to_map = T_yaw * T_level
```

保存两个参考点、`T_yaw` 和最终组合矩阵。不要使用多次鼠标微调。最终从原始完整 PCD 重新应用一次 `T_raw_to_map`，输出：

```text
20_aligned/aligned_full.pcd
10_alignment/T_level.txt
10_alignment/T_yaw.txt
10_alignment/T_raw_to_map.txt
```

矩阵必须是正交、行列式为 `+1` 的刚体变换；`verify_map_production.py` 会拒绝缩放和剪切。

## 5. ROI 与高度裁切

从 `aligned_full.pcd` 复制出导航源云，只做以下操作：

- 删除温室外无关区域；
- 按明确 Z 范围保留墙、立柱、田垄等导航结构；
- 在确认点密度足够后做固定参数降采样或去噪。

裁切不得改变剩余点坐标。保存 ROI OBB/AABB、Z 范围、输入输出哈希：

```text
30_nav_source/roi.yaml
30_nav_source/nav_source.pcd
```

`aligned_full.pcd` 用于三维重定位；`nav_source.pcd` 只用于二维栅格化，两者共享同一个 `map` 坐标。

## 6. CloudCompare Rasterize

只选 `nav_source.pcd`，使用固定参数：

```yaml
grid_step: 0.05
projection_direction: Z
cell_height: Maximum
empty_cells: Leave empty
interpolate_empty_cells: false
```

Rasterize 对话框中必须抄录并保存：

- `min center X/Y`
- `max center X/Y`
- 网格宽高 cells
- grid step
- 投影轴、cell height、empty cell 策略

导出原始观测图到 `40_raster/observed.png`。不得截图，不得使用图像编辑软件裁边、缩放、旋转或翻转。必须满足：

```text
max_center_x = min_center_x + (width_cells  - 1) * resolution
max_center_y = min_center_y + (height_cells - 1) * resolution
```

## 7. 生成三值 PGM 与 Nav2 YAML

先封装保持尺寸不变的 PGM/YAML：

```bash
python3 tools/map_tools/create_cloudcompare_runtime_map.py \
  --source-image runtime/map_projects/<map_id>/40_raster/observed.png \
  --source-pcd runtime/map_projects/<map_id>/20_aligned/aligned_full.pcd \
  --map-id <map_id> --image-format pgm \
  --resolution 0.05 \
  --min-center-x <min_center_x> \
  --min-center-y <min_center_y>
```

提供 `min-center-x/y` 后，封装工具强制使用半栅格公式计算 origin，避免重复手填产生矛盾。

再把观测图转换为障碍/自由/未知三值图，输出直接覆盖运行地图中的 PGM：

```bash
python3 tools/map_tools/prepare_trinary_nav_map.py \
  --input runtime/maps/<map_id>/<map_id>_observed.png \
  --output runtime/maps/<map_id>/<map_id>.pgm \
  --classification point-topology \
  --unknown-value 105 --unknown-margin 4 --closure-size 13
```

Nav2 原点必须严格使用：

```text
origin_x = min_center_x - resolution / 2
origin_y = min_center_y - resolution / 2
origin_yaw = 0
```

如果 CloudCompare 中已经执行长轴定向，不要再通过 YAML yaw 或图片旋转做第二次校正。

## 8. 阶段验收

逐阶段填写 `map_production.yaml` 中的路径和 SHA256，并执行：

```bash
python3 tools/map_tools/verify_map_production.py \
  runtime/map_projects/<map_id>/map_production.yaml \
  --require-stage nav2
```

至少做五个分布在四周和中心的固定地标投影检查。对齐检查器中 PCD 与 PGM 应直接重合，不应再需要求四点补偿：

```bash
python3 tools/map_tools/greenhouse_alignment_viewer_qt5.py \
  --pcd runtime/map_projects/<map_id>/20_aligned/aligned_full.pcd \
  --map runtime/maps/<map_id>/<map_id>.yaml \
  --semantic-map runtime/maps/<map_id>/semantic/semantic_map.geojson
```

最终门槛：

- YAML 元数据公式与图像尺寸检查通过；
- PCD/PGM 至少五点 RMS 不超过 `0.10 m`，且不应用补偿矩阵；
- PGM 只含约定的 occupied/free/unknown 三值；
- 用原始 bag 回放完成 NDT/ICP 定位收敛与恢复测试；
- 所有产物哈希进入新 dataset manifest；
- 通过前不得将 PCD 作为 `global_map_pcd` 启动闭环导航。

## 9. 故障判定

| 现象 | 优先检查 |
|---|---|
| 固定平移 | `min center` 是否误当边界、是否遗漏半栅格 |
| 固定旋转 | PCD 是否在 Rasterize 后又变换、图片是否被旋转 |
| 越远误差越大 | resolution/grid step 不一致或图片被缩放 |
| 上下镜像 | Rasterize 导出方向或图片被翻转 |
| 局部重合局部偏离 | 前端重影/漂移，应回到关键帧与位姿优化，不要做刚体补偿 |
| PCD/PGM 重合但定位偏 | 外参、初始位姿或 `map -> odom` 责任链错误 |
