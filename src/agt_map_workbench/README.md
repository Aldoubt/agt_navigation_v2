# agt_map_workbench

AGT Navigation V2.5 离线点云可视化编辑工具

Workbench 是 V25-12B `agt_offline_assets` 点云处理合同的可视化客户端

它不发布 ROS topic，不拥有 TF，不控制车辆，也不会原地修改 READY 地图资产

## V25-12C MVP

当前 MVP 使用项目既有 Qt5 / PyQt5 工具链和可扩展的 2.5D 点云视图

```text
PCD（ascii / binary / PCL binary_compressed）
        ↓
agt_offline_assets 读取完整点云
        ↓
仅显示层做确定性采样
        ↓
XY 顶视图 + 高度着色
        ↓
多边形绘制 + 显式 Z 区间
        ↓
crop_polygon / delete_polygon
        ↓
agt_pointcloud_processing_recipe/v1
        ↓
V25-12B 完整分辨率不可变处理
```

显示采样不会改变正式处理输入

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

## 当前工具

- 中文操作界面
- 打开普通 PCD 与 PCL `binary_compressed` PCD
- 面向百万级点云的确定性采样显示
- XY 缩放和平移
- Z 高度可见范围
- `map` 坐标系中的多边形绘制
- `delete_polygon` 三维区域删除
- `crop_polygon` 三维区域保留
- Recipe 操作历史
- 撤销和清空操作
- 导出标准 V25-12B Recipe YAML
- Qt worker thread 中执行完整分辨率不可变处理
- 处理完成后可加载结果进行视觉复查

界面中文只影响人机交互文本

以下机器合同保持原样

```text
agt_pointcloud_processing_recipe/v1
delete_polygon
crop_polygon
polygon_xy
z_min
z_max
processing.yaml
processing_report.json
```

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

## 产品边界

Workbench 编辑的是 processing draft，而不是 READY Map

```text
source/master PCD
   ↓ 可视化编辑
recipe draft
   ↓ V25-12B executor
immutable processing run
   ↓ review / quality gate
later explicit Map Version materialization
```

source/master PCD 应尽量保留有价值的分析字段

Localization Map 只保留 `x/y/z/intensity` 等字段属于后续派生产品策略，不能通过 Workbench 静默破坏 master asset

## 下一步 MVP 增量

- voxel / SOR / radius / height 数值滤波参数面板
- 不生成正式 run 的快速操作预览
- 原始 / 当前草稿 / 已处理结果切换
- selection 列表显隐和编辑
- Recipe 导入 / 重放
- Semantic Map overlay
- V25-12D Localization Prior overlay
- 2.5D authoring 合同稳定后再评估真正自由旋转 3D renderer
