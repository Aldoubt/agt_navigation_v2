# agt_map_workbench

AGT Navigation V2.5 离线点云编辑、地图坐标系标定与导航地图派生工具

Workbench 是 `agt_offline_assets` 确定性离线合同的可视化客户端

它不发布 ROS topic，不拥有 TF，不控制车辆，不原地修改 READY 地图资产

## 当前数据流

```text
source / cleaned PCD
        ↓
AGT Map Workbench
├── 点云编辑
│   └── polygon × Z → agt_pointcloud_processing_recipe/v1
├── Map Frame Calibration
│   └── 原点 + X墙 + Z柱 → map_frame.yaml
├── Ground-relative Navigation Map
│   └── local ground → relative obstacle / slope / step / unknown
│       → navigation_map.pgm / navigation_map.yaml
└── V25-12F Candidate
    ├── Site Boundary
    ├── Agricultural aisle geometry
    ├── bounded occlusion recovery
    └── navigation_map_12f.* + traversability_evidence.*
```

显示采样不会改变正式处理输入

Contract anchor: `Display sampling never changes the formal processing input`

## 启动

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run agt_map_workbench agt_map_workbench
```

当前完整场景验证地图

```text
runtime/maps/green house full.pcd
```

程序不会硬编码开发机绝对路径

## 显示层

默认深色背景与 15 万显示采样点，可切换

- 深色 / 浅色背景
- 高度 / intensity / 单色
- 1 / 2 / 3 / 4 px 点大小
- 60k / 150k / 300k / 600k 显示采样
- 独立显示 Z window

这些参数只影响 GUI Preview

源 PCD、正式 Recipe 和 Ground-relative Navigation Map 都不会因为显示采样或显示 Z 改变

目前人工检查中使用过 `[-1.505, 1.400] m` 与 `[-1.455, -0.400] m` 等显示窗口观察不同结构，这些都只是观察参数，不是正式地图派生阈值

## 点云编辑

支持

- XY 缩放和平移
- 多边形顶点编号和坐标反馈
- `delete_polygon`
- `crop_polygon`
- Recipe 执行顺序
- 撤销 / 清空
- 导出标准 V25-12B Recipe
- 完整分辨率不可变处理

多边形编辑仍使用机器合同

```text
agt_pointcloud_processing_recipe/v1
delete_polygon
crop_polygon
polygon_xy
z_min
z_max
```

## Map Frame Calibration

坐标系标定与显示 Z 已分离

```text
显示 Z              仅用于看点云
原点吸附 Z          只用于原点最近点搜索
X 墙拟合 Z          只用于墙走廊 PCA
Z 柱拟合 Z          只用于立柱圆柱 PCA
```

界面提供“把当前显示 Z 复制到全部标定选取范围”，复制后仍可分别修改

已经完成的 X/Z 拟合会在状态区显示实际使用的 `z_window_m`，之后修改显示 Z 不会改变历史拟合证据

### 标定流程

```text
选择稳定原点
        ↓
X 墙两端 → XY corridor PCA
        ↓
Z 柱中心 → 3D cylinder PCA
        ↓
X 投影到 Z 正交平面
        ↓
Y = Z × X
X = Y × Z
        ↓
右手正交 map frame
```

坐标轴文字使用固定屏幕像素偏移，标签会跟在轴端附近，不再把像素偏移误当成地图米数

`map_frame.yaml` schema

```text
agt_map_frame_calibration/v1
```

记录 source/target frame、原点、XYZ 轴、刚体变换、X/Z selection、拟合点数、RMS、线性度、X/Z 输入夹角与右手系证据

RTK / ENU / UTM 仍作为独立 `georeference.yaml` 绑定，不强迫温室 `map +X` 指向 East

## Ground-relative Navigation Map

### 为什么不再使用固定绝对 Z

温室地面并不严格平整

```text
absolute Z slice
        ↓
较高但正常的地面也可能进入障碍高度带
        ↓
误 occupied
```

因此 Navigation Map 改为

```text
完整 PCD
    ↓
XY elevation grid
    ↓
每格低分位 ground seed
    ↓
有限距离补洞 + 局部地面平滑
    ↓
local ground z_ground(x,y)
    ↓
h_rel = z - z_ground(x,y)
    ↓
障碍相对高度证据
+ ground support
+ slope
+ step
    ↓
FREE / OCCUPIED / UNKNOWN
```

当前 schema

```text
agt_ground_relative_navigation_map/v1
```

默认保持保守三态

```text
有地面支持且无风险证据 → FREE
有相对障碍 / 超坡度 / 超台阶 → OCCUPIED
证据不足 → UNKNOWN
```

“没有障碍点”不会自动等于 FREE

### 编辑器预览层

导航地图 Tab 可以切换

- 最终 PGM 三态
- 局部地面高度
- 障碍点证据
- 坡度
- 台阶高度
- Ground Confidence / Robust Slope
- 精炼行道
- 结构行道几何包络
- V25-12F Candidate 与 rich traversability masks

这些层作为半透明栅格叠加在当前已加载 PCD 上，便于判断错误来自 ground model、障碍高度、农业结构还是 traversability 合同

当前 Navigation Map 派生使用**已加载 PCD 的坐标系**

Workbench 不会把 `map_frame.yaml` 的旋转偷偷叠到未旋转的源点云上

后续应先 materialize 一个使用新 map frame 的不可变 PCD revision，再加载该 PCD 生成正式 Navigation Map

### 人工 Override

人工修改不是手画像素，而是世界坐标 polygon

```text
FORCE_FREE
FORCE_OCCUPIED
UNKNOWN
NO_GO
```

Override 会随地图分辨率重新投影，避免换 resolution 后丢失人工修正

### 导出

一次 Navigation Map 派生记录包含

```text
navigation_map.pgm
navigation_map.yaml
derivation.yaml
ground_height.npy
slope_deg.npy
step_m.npy
obstacle_count.npy
ground_support_count.npy
```

`navigation_map.yaml` 使用 Nav2 / map_server 常见 trinary 参数

## V25-12F Site Boundary 与 Traversability Candidate

V25-12F 不直接替换当前冻结 Navigation Map，而是先建立独立候选层

### Site Boundary

`Site Boundary` 表示**车辆允许进入区域的内边界**，不是墙体中心线

它由操作者在 Navigation Map 页面点选一次并冻结为

```text
site_boundary.yaml
schema agt_site_boundary/v1
boundary_semantics VEHICLE_PERMITTED_INNER_BOUNDARY
```

边界已经吸收墙体厚度、墙侧不确定性和期望贴墙安全距离，因此规划阶段不再二次猜墙宽

固定安全规则

```text
continuous vehicle footprint strictly inside site_boundary → admissible
footprint touching boundary                           → SITE_BOUNDARY_CONFLICT
footprint crossing boundary                           → SITE_BOUNDARY_CONFLICT
```

该规则独立于 Navigation Grid 栅格值；即使墙外误出现 FREE，Vehicle-Safe Lane、Forward Connector gate 与 R6B primitive search 仍不能让 footprint 穿过 Site Boundary

没有 READY `site_boundary.yaml` 时不能生成 12F Candidate，也不会自动使用地图矩形边界替代

### 结构行道几何包络

V25-12F 把两个过去容易混淆的概念拆开

```text
aisle_geometric_envelope
= 已通过行距 / 结构宽度 / 纵向重叠检查的农业行道几何
= 不要求每一个 cell 都有直接 Ground FREE 证据

aisle_candidate / 精炼行道
= geometric envelope 再经过 Ground / confidence / slope / obstacle gates 后的安全证据
```

因此植被遮挡造成的短 Ground 缺口不会把“这里结构上本来就是行道”这一层信息一起删除

`Vehicle Corridor / required_envelope_mask` 仍然只是围绕行道中心线的期望车辆包络，不作为 observed FREE 真值

### Rich Traversability

内部证据至少保留

```text
OBSERVED_FREE
INFERRED_TRAVERSABLE
HARD_BLOCKED
SENSOR_OBSTACLE
UNKNOWN
```

A-first 策略只恢复满足全部 gate 的**纵向短 UNKNOWN 缺口**

```text
inside aisle_geometric_envelope
inside site_boundary
outside NO_GO
outside row_structural_band
current cell is UNKNOWN, not OCCUPIED
known local aisle direction
both longitudinal ends have OBSERVED_FREE support
gap <= maximum_inferred_gap_m
endpoint height / slope / step continuity acceptable
```

初始

```text
maximum_inferred_gap_m = 0.60 m
```

禁止全局 `UNKNOWN -> FREE`，禁止横向跨垄形态学补洞，当前 OCCUPIED 在 A-first candidate 中仍然保持阻塞

最终只有在 Nav2 compatibility export 时才压成

```text
OBSERVED_FREE        -> FREE
INFERRED_TRAVERSABLE -> FREE
HARD_BLOCKED         -> OCCUPIED
SENSOR_OBSTACLE      -> OCCUPIED
NO_GO                -> OCCUPIED
UNKNOWN              -> UNKNOWN
```

### Candidate 输出

12F Candidate 固定写出

```text
traversability_evidence.yaml
traversability_evidence.npz
navigation_map_12f.yaml
navigation_map_12f.pgm
navigation_map_12f_derivation.yaml
```

当前冻结文件

```text
navigation_map.yaml
navigation_map.pgm
derivation.yaml
```

不会被 Candidate 导出覆盖

Workbench 中的 `生成 12F Candidate` 只更新旁路 candidate state，不会把 `_navigation_result` 偷换成候选地图

## 路径调试 Route Debug 2D

右侧 `路径调试` 是现有 Workbench 内的 **2D、只读** 路径生产证据查看器

它加载一个完整 run directory，而不是让操作者逐个选择 YAML，从冻结资产中联合显示行道结构、Coverage 顺序、ConnectorRequest、Forward/R6 运动、车辆可行性和碰撞来源

```text
运行目录
├── navigation_map.pgm / navigation_map.yaml
├── derivation.yaml
├── aisle_graph.yaml
├── turn_zones.yaml
├── coverage_order.yaml
├── forward / R5.6 / R6A / R6B 可选冻结资产
├── vehicle-safe-lane / occupancy-source 可选冻结资产
└── V25-12F optional candidate
    ├── site_boundary.yaml
    ├── navigation_map_12f.yaml / pgm
    └── traversability_evidence.yaml / npz
        ↓
12E RouteDebugDataset + optional RouteDebug12FBundle
        ↓
agt_route_debug_overlay/v1 + read-only raster layers
        ↓
共享 QGraphicsScene
```

界面预设

```text
Coverage 总览
规划结果
碰撞诊断
12F A/B
```

`12F A/B` 可同时查看 current/candidate Navigation Map、Site Boundary、`INFERRED_TRAVERSABLE`、`HARD_BLOCKED`、`SENSOR_OBSTACLE` 与 `Aisle Geometric Envelope`

12F optional assets 缺失不会让原有 12E Route Debug 数据集失败；缺失只显示为无证据，schema/frame/内容无效则对应 12F layer fail-closed

`ConnectorRequest` 与真实已求解运动轨迹分层显示，Coverage 行道方向不会被误画成倒车；只有冻结 R6B sample 明确包含 `motion_direction=REVERSE` 时才显示 Reverse 段和 cusp

`NO_GO` 保持为独立的语义排除证据，不与物理 `OCCUPIED` 或 Site Boundary 混为一种状态

碰撞诊断可以叠加

```text
RAW_OBSTACLE_DIRECT
GEOMETRY_DIRECT
PADDING_ONLY
UNKNOWN
COLLISION_STATION
```

选中碰撞点后 Inspector 显示对应 footprint、占据来源统计与冻结来源字段，不在 GUI 中重新运行规划器或修改原始 Navigation Map

Route Debug 允许创建或替换的唯一 run-directory 输出是

```text
route_debug_overlay.geojson
```

该文件只是 `DEBUG_RENDER_ONLY` 派生证据，不是新的路径真值，也不能用于 READY promotion

## V25-12F A/B 验收工具

候选文件冻结后，可用同一 MK-mini 配置比较 current/candidate Vehicle-Safe Lane，并用同一组未修改的 R6B 默认参数重放 `connector_015` 与 `connector_017`

```bash
python3 tools/v25_12f_acceptance.py \
  --run-dir runtime/maps/agt_workbench_run \
  --vehicle-profile profiles/platforms/mk_mini.yaml \
  --pretty
```

输出是 `agt_v25_12f_acceptance_report/v1` JSON，只用于离线 A/B 审查，不会把 candidate promotion 为 READY

真实温室视觉验收和 A/B 数据必须由操作者实际完成后再标记 PASS；代码实现本身不能代替现场检查

## 产品边界

```text
master / cleaned PCD
├── pointcloud recipe → immutable processed PCD
├── map_frame.yaml
├── ground-relative navigation derivation
│   ├── evidence layers
│   ├── polygon overrides
│   └── current PGM/YAML
└── V25-12F candidate
    ├── manually frozen Site Boundary
    ├── rich traversability evidence
    └── candidate PGM/YAML
```

Workbench 是 authoring/review 工具，不是第二套运行时地图真源

## 下一步

- 在真实 `runtime/maps/agt_workbench_run` 上人工冻结 Site Boundary
- 生成 12F candidate 并完成 Route Debug current/candidate A/B
- 用相同 MK-mini profile 运行 `tools/v25_12f_acceptance.py`
- 检查 `aisle_003 / 005 / 013` 与 `connector_015 / 017`
- 只有真实 A/B 验收后才决定是否 promotion 12F candidate 为 canonical Navigation Map
- Localization Prior
- Semantic Map
- Route Preview
- RTK georeference control points
