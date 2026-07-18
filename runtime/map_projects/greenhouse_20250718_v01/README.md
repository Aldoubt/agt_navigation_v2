# greenhouse_20250718_v01

本目录保存 2025-07-18 温室地图重制实验的单次、版本化生产数据。Rasterize 网格元数据
已经补齐，当前坐标基线已冻结，可以进行基础地图清理和语义标注，但尚未完成实车闭环
验收。

## 目录内容

- `00_source/frontend_global.pcd`：本次提交的点云源文件，保持原字节内容。
- `40_raster/observed_color.png`：与 Rasterize `644 x 783` 网格一致的正式观测图。
- `40_raster/superseded/`：保留尺寸不匹配的上一版导出和截图，仅供审计。
- `50_nav2/*.pgm`：按点云像素拓扑转换的黑白灰三值底图。
- `50_nav2/*.yaml`：使用 Rasterize 最小栅格中心计算的 Nav2 元数据。
- `map_production.yaml`：源文件哈希、坐标契约、参数证据及阻塞项。

## 已确认参数

Rasterize 参数为 `Grid step=0.05 m`、`size=644 x 783`、最小栅格中心
`(-2.42542696, -29.70424461)`。截图宽高满足 `(size-1) x step`，新 PNG 像素尺寸也与
网格一致。Nav2 原点按最小栅格中心向外移动半个栅格，得到
`(-2.45042696, -29.72924461, 0)`。

原 PCD 的 `VIEWPOINT` 不是单位位姿。该位姿已一次性应用到 XYZ 和法向量，生成
`20_aligned/frontend_global_map.pcd` 与 `30_nav_source/rasterize_section_map.pcd`，两个
输出的 `VIEWPOINT` 均为单位位姿。转换后的 section PCD 全部 `140,691` 点落入 Rasterize
窗口，其中 `140,689` 点命中 PNG 非空栅格，像素命中率为 `99.999%`。

## Qt5 编辑

运行时标注包位于 `runtime/maps/greenhouse_20250718_v01/`。在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch agt_ui_bridge semantic_editor.launch.py \
  map:="$(pwd)/runtime/maps/greenhouse_20250718_v01/greenhouse_20250718_v01.yaml" \
  platform_profile:="$(pwd)/profiles/platforms/greenhouse_ackermann.yaml"
```

先用地图障碍、地图自由、地图未知和直线工具清理 PGM，再标注作业区、作物行、通行道路、
入口位姿和作业方向。编辑器不得修改 `00_source` PCD 或 `40_raster` 观测图。

## 待验证

本次没有执行找地面，找平矩阵明确记录为单位矩阵。Rasterize 使用的 `140,691` 点 section
PCD 已归档；按用户决定不补独立 ROI 多边形，当前 section 与手动 Rasterize 窗口共同作为
裁切记录。项目目录中的坐标基线不可修改，运行时 PGM 是手工清图工作副本；清图和语义
标注完成后必须创建新版本、更新哈希并完成定位回放，才能标记为实车闭环可执行。
