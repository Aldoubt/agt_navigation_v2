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
└── Ground-relative Navigation Map
    └── local ground → relative obstacle / slope / step / unknown
        → navigation_map.pgm / navigation_map.yaml
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

这些层作为半透明栅格叠加在当前已加载 PCD 上，便于判断错误来自 ground model、障碍高度还是 traversability 阈值

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

## 产品边界

```text
master / cleaned PCD
├── pointcloud recipe → immutable processed PCD
├── map_frame.yaml
└── ground-relative navigation derivation
    ├── evidence layers
    ├── polygon overrides
    └── PGM/YAML
```

Workbench 是 authoring/review 工具，不是第二套运行时地图真源

## 下一步

- materialize `map_frame.yaml` 为新的不可变 transformed PCD revision
- 在真实 `green house full.pcd` 上调 Ground-relative 参数并记录 acceptance evidence
- 将 Navigation Map derivation 绑定 Map Version / Site Package lineage
- 增加更强的 DTM / progressive morphology 候选，与当前 elevation-grid MVP 对比
- Localization Prior
- Semantic Map
- Route Preview
- RTK georeference control points
