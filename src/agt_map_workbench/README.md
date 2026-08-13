# agt_map_workbench

AGT Navigation V2.5 离线点云可视化编辑与地图坐标系标定工具

Workbench 是 V25-12B `agt_offline_assets` 点云处理合同的可视化客户端，同时提供独立的 Map Frame Calibration authoring

它不发布 ROS topic，不拥有 TF，不控制车辆，也不会原地修改 READY 地图资产

## V25-12C 当前能力

```text
PCD（ascii / binary / PCL binary_compressed）
        ↓
agt_offline_assets 读取完整点云
        ↓
仅显示层做确定性采样
        ↓
深色 2.5D 点云视图
├── Z 显示窗口
├── 高度 / intensity / 单色
├── 点大小
└── 6万 / 15万 / 30万 / 60万显示采样
        ↓
┌──────────────────────────────┐
│ 点云编辑                     │
│ polygon × Z → Recipe         │
└──────────────────────────────┘
        ↓
V25-12B 完整分辨率不可变处理

同时

┌──────────────────────────────┐
│ Map Frame Calibration        │
│ 原点 + X墙 + Z柱             │
│ → 右手正交 map frame         │
│ → map_frame.yaml             │
└──────────────────────────────┘
```

显示采样不会改变正式处理输入

Contract anchor: `Display sampling never changes the formal processing input`

完整源 PCD 始终直接传给 `process_pointcloud`

## 启动

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run agt_map_workbench agt_map_workbench
```

安装后的可执行文件名为 `agt_map_workbench`，build tree launcher 为 `map_workbench_launcher.py`

launcher 文件名必须与 Python package 名不同，避免 `--symlink-install` 下的 Python import shadowing

## 当前真实地图基准

V25-12C 当前完整场景验证地图为

```text
runtime/maps/green house full.pcd
```

当前开发机对应绝对路径为

```text
/home/yangxuan/agt_navigation_v2/runtime/maps/green house full.pcd
```

程序本身不会硬编码该绝对路径，打开文件时默认进入当前工作区的 `runtime/maps/`

此前使用的

```text
runtime/maps/greenhouse_ground/pcd/greenhouse_aligned_full2.pcd
```

属于已经切割过的地图，仅继续作为 V25-12B 历史 smoke / 快速回归资产

### 当前温室显示经验值

在 `green house full.pcd` 的人工检查中，以下 Z 显示窗口获得了较清楚的平面结构效果

```text
Z min = -1.505 m
Z max =  1.400 m
```

这是 Workbench 观察参数，不是已经冻结的正式 `height_range` 处理参数

## 显示质量

默认使用深色背景与 15 万显示采样点

可选

- 深色 / 浅色背景
- 按 Z 高度着色
- 按 `intensity` 着色
- 单色显示
- 1 / 2 / 3 / 4 px 点大小
- 60k / 150k / 300k / 600k 显示采样上限

这些参数只影响 GUI Preview

源 PCD 点数、正式 Recipe 和 `process_pointcloud` 输入均不改变

### 关于“地图是否已经做过体素降采样”

Workbench 中看起来点少不能作为已经 voxel-downsampled 的证据，因为显示层本身会抽样

如果源 PCD 没有对应的 `processing.yaml` / `recipe.yaml` / lineage 记录，不能仅凭文件外观断言历史上是否执行过体素滤波

后续可以增加点间距 / 最近邻统计作为诊断证据，但它仍只能用于推断，不能替代 asset lineage

## 点云编辑

当前支持

- XY 缩放和平移
- Z 高度可见范围
- 多边形顶点编号与坐标反馈
- `delete_polygon` 三维区域删除
- `crop_polygon` 三维区域保留
- Recipe 执行顺序
- 撤销和清空操作
- 导出标准 V25-12B Recipe YAML
- Qt worker thread 中执行完整分辨率不可变处理
- 处理完成后加载结果进行视觉复查

多边形编辑不会保存成 GUI 私有格式

例如删除一个三维区域会生成

```yaml
- type: delete_polygon
  parameters:
    polygon_xy:
      - [1.0, 2.0]
      - [4.0, 2.0]
      - [4.0, 5.0]
      - [1.0, 5.0]
    z_min: -1.5
    z_max: 4.5
```

可直接由 V25-12B 重放

## Map Frame Calibration

目标是定义一个适合场景使用的 `map` 坐标系，而不是强迫地图 X 轴指向地理正东

推荐温室语义

```text
+X  沿主要温室墙 / 种植行方向
+Z  向上
+Y  由右手定律自动生成
```

### 1. 原点

点击一个稳定、具有物理意义的位置，例如墙角或立柱底部

Workbench 会在当前 Z 窗口中吸附到最近的完整 PCD 3D 点，而不是直接使用鼠标投影坐标

### 2. X 参考墙

沿希望的 `+X` 方向依次点击墙壁两端

Workbench 使用

```text
两点定义 XY 走廊
+ 当前 Z window
+ X corridor half width
        ↓
提取完整 PCD 局部点
        ↓
XY PCA line fit
```

输出点数、RMS 残差、线性度和拟合方向

### 3. Z 参考立柱

点击可靠立柱的 XY 中心

Workbench 使用

```text
XY 圆柱
+ 当前 Z window
+ pillar radius
        ↓
提取完整 PCD 局部点
        ↓
3D PCA principal-line fit
```

默认把候选方向朝向 source map 的正 Z，也可在界面中显式翻转 Z

### 4. 右手正交化

现场墙和柱不会数学上严格 90°，所以不能把两条原始拟合向量直接作为坐标基

Workbench 将 X 投影到垂直于 Z 的平面，再计算

```text
Y = Z × X
X = Y × Z
```

最终输出严格正交、右手的旋转基

### 5. map_frame.yaml

标定文件 schema

```text
agt_map_frame_calibration/v1
```

记录

- source / target frame ID
- source 中的原点
- source 中的 X/Y/Z 单位轴
- `source → map` rotation / translation
- X 墙参考段、走廊宽度、Z window
- Z 柱中心、半径、Z window
- 拟合点数
- RMS 残差
- 线性度
- 输入 X/Z 夹角
- 输出右手系证据
- georeference 状态

## Map Frame 与 RTK Georeference 分离

```text
source/master PCD
        ↓
Map Frame Calibration
        ↓
site-friendly map
X = 温室主方向
Y = 温室横向
Z = Up

之后独立建立

ENU / UTM
    ↕ georeference.yaml
map
```

`map_frame.yaml` 默认将 georeference 标为

```text
UNBOUND
```

后续 RTK 引入时再通过多个 map ↔ ENU/UTM 控制点或可靠绝对航向求地理配准，不要求当前 map 的 X 轴等于 East

## 产品边界

Workbench 编辑的是 processing / calibration draft，而不是 READY Map

```text
source/master PCD
   ├── visual edit → recipe draft → V25-12B immutable processing run
   └── frame authoring → map_frame.yaml

review / quality gate
   ↓
later explicit Map Version materialization
```

source/master PCD 应尽量保留有价值的分析字段

Localization Map 只保留 `x/y/z/intensity` 等字段属于后续派生产品策略，不能通过 Workbench 静默破坏 master asset

## 下一步 MVP 增量

- 将 `map_frame.yaml` 应用为新的不可变 map revision，而不是覆盖 source PCD
- 点间距 / 最近邻统计，用于判断显示稀疏与源数据稀疏
- voxel / SOR / radius / height 数值滤波参数面板
- 原始 / 当前草稿 / 已处理结果切换
- Recipe 导入 / 重放
- Semantic Map overlay
- V25-12D Localization Prior overlay
- georeference control-point authoring
- 2.5D authoring 合同稳定后再评估自由旋转 3D renderer
