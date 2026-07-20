# 视觉与点云语义扩展预留

## 目标与边界

当前系统先固定“原始数据采集 → 建图/重定位 → Nav2 → `agt_safety` → 底盘”的可靠闭环，
手工语义地图继续使用 GeoJSON 与 `coverage.yaml`。未来视觉或点云网络输出的是带时间戳的
动态观测，不修改基础 PGM，也不直接成为速度命令。

这份文档只预留模块边界和命名方向，不声明检测器、消息依赖或安全能力已经实现。

## 当前可复用接口

| 边界 | 当前接口 | 后续用途 |
| --- | --- | --- |
| 原始雷达 | `/agt/sensors/lidar/points`、`/agt/sensors/lidar/custom` | 点云语义后端、数据回放 |
| 原始惯导 | `/agt/sensors/imu/data` | 去畸变、时序和姿态约束 |
| 局部障碍 | `/agt/perception/obstacle_cloud` (`PointCloud2`) | Nav2 local costmap 与 Collision Monitor |
| 位姿 | TF `map -> odom -> base_footprint -> sensor` | 将检测结果变换到局部或全局坐标 |
| 静态语义 | `runtime/maps/<map_id>/semantic/*.geojson` 与 `coverage.yaml` | 人工标注的田块、作物行、禁行区和任务 |
| 车辆几何 | `profiles/platforms/<platform>.yaml` | 车体裁剪、碰撞检查与传感器遮挡分析 |
| 操作前端 | 维护版 Qt、项目语义编辑器、项目 Action | 可替换展示层，不改变运动执行边界 |

## 建议的适配层

新增相机时，由 `agt_sensor_adapters` 提供标准 `sensor_msgs/Image` 与 `CameraInfo`；新增雷达
或模型时，也先在适配层统一 topic、时间戳和 frame。算法节点放在 `agt_perception`，避免
Nav2、Qt 或语义地图服务器直接依赖某个相机 SDK、网络模型或厂商消息。

建议在真正选定模型与 ROS 消息依赖后再落地以下保留名：

| 保留名 | 候选类型 | 语义 |
| --- | --- | --- |
| `/agt/perception/semantic_points` | `sensor_msgs/PointCloud2` | 含 `label/confidence/instance_id` 字段的局部语义点 |
| `/agt/perception/detections_3d` | `vision_msgs/Detection3DArray` | 统一三维检测，不暴露模型私有消息 |
| `/agt/perception/tracked_objects` | 待版本化项目接口 | 带稳定 ID、速度、协方差和有效期的跟踪目标 |
| `/agt/perception/semantic_diagnostics` | `diagnostic_msgs/DiagnosticArray` | 模型、频率、延迟、丢帧和 stale 状态 |

保留名尚未实现；在消息字段、QoS、frame、超时和版本策略确定前，不应让下游依赖它们。

## 消费原则

- 人、车辆等动态目标先进入跟踪与时效过滤，再按明确策略转换为有限寿命的障碍观测；检测
  topic 不直接接到底盘或速度 topic。
- Qt/Web 前端只展示检测、置信度和诊断，任务执行仍调用项目 Action。
- 静态对象若经人工确认，可另行写入版本化 GeoJSON；在线模型不得自动回写 PGM 或覆盖源标注。
- 行为层未来可依据语义请求减速、等待或重新规划，但必须经 Nav2 和 `agt_safety`，并为
  无数据、过期数据、TF 失败和模型崩溃定义 fail-safe 行为。

## Bag 与评测预留

语义实验 bag 至少记录原始图像/点云、CameraInfo、IMU、轮速/LIO 里程计、`/tf`、
`/tf_static`、模型输出、诊断、Nav2 路径和安全状态。训练数据与运行 bag 放在 `runtime/`
或外部数据盘，不提交 Git；仓库只提交 topic 清单、标定哈希、模型版本、参数和评测报告。

验收应分别量化检测精度、距离分桶误差、端到端延迟、掉帧、遮挡、夜间/逆光/粉尘退化、
stale 清除时间和资源占用。识别人或数字不等于可用于安全停车；在专门实车验证前只能作为
操作提示或研究输入。

## 分阶段接入

1. 先只录原始视觉/点云并离线推理，不接导航。
2. 发布标准化语义结果和诊断，仅在 Qt/RViz 叠加显示。
3. 将通过时效与几何验证的动态目标投影为局部障碍，保持默认关闭并完成回放测试。
4. 在封闭场地验证减速、绕行、检测丢失和进程退出，再考虑行为层语义策略。
5. 最后才把稳定组件纳入自启动、健康监控、日志轮转和全生命周期维护。

这样可保持前端、传感器和模型可替换，同时不破坏现有地图、Action、安全和底盘合同。
