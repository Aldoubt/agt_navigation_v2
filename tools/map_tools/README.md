# map_tools

PCD 清洗、地图格式转换和地图元数据检查工具预留目录。

当前可用工具：

- `apply_pcd_viewpoint.py`
  - 将 PCD 头部的 `VIEWPOINT` 位姿一次性应用到 XYZ 和法向量，并把输出重置为单位 `VIEWPOINT`；
  - 避免 CloudCompare Rasterize 使用实体位姿后，定位 PCD 仍保留局部坐标造成隐性错位。
- `init_map_production.py`
  - 从前端全局 PCD 和可选原始 bag 初始化不可覆盖的分阶段地图生产项目；
  - 固化源文件哈希、坐标契约、CloudCompare 参数槽位和后端优化数据缺口。
- `verify_map_production.py`
  - 按 `source/alignment/crop/rasterize/nav2/validation` 阶段执行 fail-closed 检查；
  - 拒绝带缩放/剪切的矩阵，并验证 Rasterize 尺寸、半栅格原点和 Nav2 YAML。
- `verify_dataset_manifest.py`
  - 按冻结清单逐项验证 PCD、Nav2 地图、语义文件、coverage、profile 和对齐记录的 SHA256；
  - 可用 `--require-navigation-ready` 在闭环启动前执行 fail-closed 就绪检查。
- `greenhouse_alignment_viewer_qt5.py`
  - 只读叠加显示对齐后的 PCD、Nav2 `PNG/YAML` 和语义 `GeoJSON`；
  - 支持在界面中浏览并切换用于对齐检查的 `.pcd` 地图；
  - 支持图层开关、透明度、缩放、点云 Z 高度筛选和鼠标米制坐标读取；
  - 显示各数据边界、点云抽样落图比例、语义坐标落图比例，供整理冻结前人工复核；
  - 可用四组 PCD/栅格同名点求解二维刚体变换，预览后导出 YAML 记录和 4x4 矩阵；
  - 不修改任何源地图，也不发布 ROS 话题或可执行路径。
- `create_cloudcompare_runtime_map.py`
  - 把 CloudCompare 导出的 `png` 封装成 `runtime/maps/<map_id>/` 运行时地图包；
  - 自动生成 `PNG/YAML`、`processing_record.yaml` 和 `semantic/coverage.yaml`；
  - 适合先做 Qt5 二维底图编辑，再进入项目语义标注。
  - 可用 `--image-format pgm` 生成 PGM 格式的 Nav2 地图图像。
- `prepare_trinary_nav_map.py`
  - 把灰度观测图转换为仅含障碍、未知、自由的 Nav2 三值图；
  - `point-topology` 模式将有点云像素标为障碍、外框内部空白标为自由、外框外部标为未知；
  - 输入 PNG 同时包含透明和不透明像素时，优先以 alpha 通道判断有无点云，避免透明背景被灰度值误判；
  - 外框闭合只用于判断内外，不会加粗最终障碍。

## 温室地图对齐检查

在仓库根目录执行：

```bash
python3 tools/map_tools/greenhouse_alignment_viewer_qt5.py
```

默认加载当前温室语义实验使用的三个数据源：

```text
runtime/maps/greenhouse_ground/pcd/greenhouse_aligned_full2.pcd
runtime/maps/greenhouse_ground/greenhouse_ground.yaml
runtime/maps/greenhouse_ground/semantic/semantic_map.geojson
```

也可以显式指定，避免后续整理后误用旧文件：

```bash
python3 tools/map_tools/greenhouse_alignment_viewer_qt5.py \
  --pcd runtime/maps/greenhouse_ground/pcd/greenhouse_aligned_full2.pcd \
  --map runtime/maps/greenhouse_ground/greenhouse_ground.yaml \
  --semantic-map runtime/maps/greenhouse_ground/semantic/semantic_map.geojson
```

PCD 首次加载需要解压约 541 万个点，界面会先显示底图和语义层。右侧“抽样落图”是坐标边界初检，不等于最终对齐结论；仍需通过透明度和 Z 范围检查墙体、作物行、入口位姿及语义线是否重合。该工具不会自动更改原点或分辨率。

右侧“PCD 对齐地图”中的“浏览 PCD...”可以切换到其他点云；“重新加载”用于刷新当前文件。加载期间按钮会暂时禁用，旧点云投影会被清空，避免把上一份点云误认为当前选择结果。

“显示底图外点云”默认开启，画布采用底图与点云抽样边界的联合范围。四点对齐时，选择 PCD 特征点会自动适配完整点云范围，选择对应栅格点会自动恢复到底图范围。若离群点导致联合画布任一方向超过 `6000 px`，工具会安全退回底图范围并提示先过滤离群点，避免异常坐标耗尽内存。

## 四点刚体对齐

当 PCD 裁切完成后与 PGM/PNG 存在固定旋转或平移偏差时：

1. 加载 PCD，并通过 `--map` 指定目标栅格对应的 Nav2 YAML。
2. 点击“开始四点选择”。
3. 按界面提示依次点击 `PCD 点1 -> 栅格同名点1`，完成四组对应点。
4. 工具只求解 `target_map_xy = R * source_pcd_xy + t`，不会缩放、镜像或修改源文件。
5. 查看叠加预览、RMS、最大残差和尺度偏差，再点击“导出矩阵...”。
6. 导出的 YAML 是审计记录；同名 TXT 是可供 CloudCompare `Apply Transformation` 使用的 4x4 矩阵。

选点要求：

- 必须是两张图中能确认属于同一物理位置的墙角、立柱中心或固定结构交点；
- 四点应分布在区域四周，不能全部位于同一条作物行或同一直线上；
- 不要选择植株边缘、稀疏噪点或不同高度投影后位置不稳定的目标；
- 建议 RMS 不超过 `0.10 m`；尺度偏差超过 `0.5%` 时先检查 YAML `resolution`、Rasterize 网格和原点，不要用缩放点云掩盖问题。

示例：

```bash
python3 tools/map_tools/greenhouse_alignment_viewer_qt5.py \
  --pcd "$HOME/ground_segmentation_benchmark/roi_exports/greenhouse_main_manual.pcd" \
  --map runtime/maps/greenhouse_ground/greenhouse_ground.yaml \
  --semantic-map runtime/maps/greenhouse_ground/semantic/semantic_map.geojson
```

导出操作不会生成新的 PCD。先保留原始点云和矩阵记录，经目视及残差确认后，再在 CloudCompare 中把矩阵应用到 PCD 副本；不要覆盖原始地图。
