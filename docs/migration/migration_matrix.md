# 迁移矩阵

更新时间：2026-07-23。

状态按“代码迁移、离线验证、数据回放、实机验收”分级记录。测试数量随模块增长，以提交时
测试报告为准；离线通过不代表定位精度、导航性能或实车安全验收通过。

Web 控制台的同一 `runtime_dir` 只允许一个进程；入口会在创建 ROS bridge
前获取单实例锁，重复启动不会再注册同名 `agt_web_console_ros_bridge` 节点。

| 模块 | Phase | 当前状态 | 已验证范围 | 下一步 |
| --- | --- | --- | --- | --- |
| `agt_interfaces` | 1/8 | TASK-13/14 与 waypoint task 接口完成 | `ExecuteCoverageTask.action` 和 `ExecuteWaypointTask.action` 均通过 ROSIDL 生成；多点任务接口具有序列化回归 | 后续字段变更需兼容性评审，并同步服务端与客户端 |
| `agt_description` | 2/8 | 离线完成 | 固定 TF、MK-mini/BUNKER profile、Xacro 展开、TF 单父节点和可配置 MID360 外参通过；BUNKER 已用静止 bag 地面平面与 IMU 重力估计候选 `0.607 m / +23.2 deg`，仍保留未验证状态 | 实现流式数据提取、时间偏移/激励检查和鲁棒外参优化；用独立 bag 与机械实测验证后再置为 calibrated |
| `agt_bringup` | 1/3/4/5/6/8 | Bunker Qt5 FAST-LIO baseline 与一次性自动重定位启动 Action 已接线 | mapping/navigation 共用总入口；mapping 默认 RViz、可选 `start_mapping_gui` 且 Qt 禁止任务执行；navigation 将同一 map YAML 注入 Nav2 与任务型 Qt；自动客户端默认关闭，显式开启后只发送一次有界 `MODE_AUTO_SEARCH`；大数据节点获得延长正常退出窗口 | 实机验证完整建图关机、候选 YAML 自动重定位、同源 PCD 重定位和项目多点 Action 闭环；确认 Bunker vendor 退出告警不影响停车 |
| `agt_sensor_adapters` | 3 | baseline 完成；前置自滤除已接线，待回放/实机验收 | Livox 驱动保留原始 `/agt/sensors/lidar/custom`；新增 profile 驱动的 CustomMsg 车体/高台盒过滤、字段和顺序保留、TF fail-closed、盒子 MarkerArray 与 diagnostics；输出 `/agt/sensors/lidar/custom_filtered` | 用真实 MID360 和只含原始输入/TF 的历史 bag 验证 QoS、过滤比例、非有限点、TF 失败、频率、CPU 和长时间运行；实测高台边界后再把 `verified` 置为 true |
| `agt_mapping` | 3 | 大地图 PCD 持久化 Release 回放通过；FAST-LIVO2 已切换前置过滤输入 | FAST-LIVO2 与所需 Vikit 均按固定提交 vendor；统一 x86 Eigen/PCL ABI；稀疏 int64 增量体素过滤异常点；backend 消费 `custom_filtered`，原始 CustomMsg 保留用于 bag/replay | 在无旧 overlay 的全新工作区构建并分别回放过滤前后数据；完成 Vikit 独立许可证文本审计、最终 PCD/NDT 地图质量和车辆外参精测 |
| `agt_map_processing` | 5 | 大包混合静态障碍补全、完整车体扫掠和三组可通行性对照已离线回放；全图 OctoMap 投影增加默认 0.2 Hz 有界输入节流、0.10 m 体素压缩和 8,000 点上限，在线 3D 投影可显式关闭；PGM 保存默认保留 `205` unknown round-trip；有界实时化初值仅作为禁用候选保存 | 点云时间同步与 canonical 多边形裁剪；全部 8,072 个里程计位姿形成 72,469 个扫掠栅格；RANSAC/时序/高度对照消费全部 8,073 帧且地面拟合零失败，三图均通过同一长距离规划；实时候选对窗口、活动 cell/tile、内存预算和持久化前淘汰建立边界测试；OctoMap 节流节点仅保护全图投影，不改变 FAST-LIVO2 或局部障碍输入 | 优先人工核验保守 `ground_temporal` 与 v5；在独立 bag 上重新测量节流频率、地图更新延迟、峰值 RSS 和地图差异；完成外参标定后实现分块实时节点、资源诊断和原子落盘，再用独立 bag 调整阈值；实测整车最高点前高度分层仍不可执行 |
| `agt_localization` | 4 | 基础自动重定位、PCD 内容 hash 读取/候选绑定和 ready 记录门禁已接入；完整导航门禁仍未完成 | ICP/NDT core、结构化接口、候选加载/展开、last pose 原子持久化、SHA-256 PCD 身份重算、可选 `pcd_sha256` 记录校验、质量门禁、Action、处理记录 `state: ready`/`map_file` 校验和唯一 `map -> odom` 已编译；监督器单测覆盖确认、取消、超时及 `TRACKING -> DEGRADED -> RECOVERING -> LOST`；低频验证成功不改写 TF；Bunker 实车 4 线程 fitness 约 `0.01`–`0.02`，新 369,970 点大地图同包初验 fitness `0.0401` | 让新地图生产记录写入 `pcd_sha256`；批量验证不同位置/误差初值和短期连续性；完成 Nav2 lifecycle gate、几何质量指标和真实地图验收 |
| `agt_localization_fusion` | 6 | 仅骨架 | package 和领域边界已建立 | 定义融合状态与诊断接口，接入 LIO、轮速和 IMU；后续扩展 RTK/UWB 与失效降级 |
| `agt_perception` | 6/8 | baseline 完成并预留语义边界 | 已实现 base frame 高度/量程/车体裁剪的局部障碍点云并接入 Nav2；文档固定未来相机/点云适配、标准化语义输出、bag 与 fail-safe 边界 | 先做离线语义推理和显示，完成数据集/延迟/stale 评测后再以默认关闭方式接入动态障碍 |
| `agt_navigation` | 6/8 | 项目多点 Action、planner-only 预览和基础定位门禁完成；完整 lifecycle gate 未完成 | `ExecuteWaypointTask` 到 Nav2 `FollowWaypoints` 状态机；Qt 执行只消费 Action且 ROS 派发不占 GUI 线程；新任务入口检查新鲜 accepted `LocalizationStatus` 与 `agt_safety`，运行中状态丢失会取消 child；独立 offline profile 以显式任务点逐段调用 `ComputePathToPose` 并只发布 `/plan`，不启动运动链 | 实车验证多点成功、失败、取消和急停；补 Nav2 lifecycle 与统一运行记录，再实现最小 mission orchestrator |
| `agt_coverage_planning` | 8 | 语义地图、Coverage Server 适配、PathComponents 重建、离线验证和 planner-only 修复链已实现；当前为 `execution-blocked` | 外部锁定依赖已构建；临时 GML 统一为至少三点；语义指纹、SWATH/CONNECTION 分类、full-footprint Validator、keepout 直接检查、仅 CONNECTION 修复、入口 approach 预览和 metrics-only 报告均有回归；真实大棚仍出现 `zero_length_swath`，覆盖率/重叠率保持 null | 定位并修复 Row Coverage 的零长度 SWATH/偶发无结果；补齐版本化任务 manifest、真实场景离线报告和独立样例数据；在语义、碰撞、运动学和安全门槛全部通过前不得执行 |
| `agt_safety` | 6 | baseline 与基础 localization guard 完成 | BUNKER 履带仲裁、手动优先、导航定位失效 fail-closed、限速、输入超时、急停锁存和复位保持禁用的回归通过；手动输入不被定位 guard 抢占 | 架空履带验证方向和急停，再完成低速制动距离、进程退出和通信中断验收 |
| `agt_chassis` | 6 | baseline 完成，控制/只读监测角色已分离 | 官方 `bunker_ros2`、状态桥接、TF 隔离和双层命令 watchdog 已接入；`operation_mode:=monitor` 可只接收 CAN 状态，`control` 仍严格经过安全链；CAN 初始化权限留在主机管理员侧 | CAN 实机验证协议版本、轮速里程计、错误码、方向、断连归零、监测模式无命令输出和长时间通讯稳定性；后续接入替换 backend |
| `agt_ui_bridge` | 6/8 | mapping/navigation/offline 三 profile 能力隔离；维护版 Qt 接入 | navigation 调用项目多点 Action，offline 仅允许路径预览，mapping 禁止任务；导航点两次点击；Task 绑定当前行并随拓扑刷新；离线平移/缩放解除机器人跟随；默认中文、设置页完整双语且可持久切换英文；大地图默认禁用全量 costmap 渲染；MapGeometry 显式校验 P2/P5/PNG | 用昨日大地图验证多段预览、交互帧率和中英文设置页；实车验证两点 yaw 与 navigation Action；后续前端只消费项目 Actions |
| `agt_system_manager` | 1/2/4/6 | P0/P1/P2/P6 离线完成 | `SystemHealth`/`TaskReadiness` ROSIDL、健康合同、频率/新鲜度/TF/lifecycle/条件 evaluator、白名单进程组、模式 Action 和有界重定位策略通过；ROS adapter 以 MID360 原始 CustomMsg 和 IMU 评估传感器健康，传感器 profile 等待健康结果并返回日志；mapping 额外检查 FAST-LIVO2 里程计、注册点云和持久 TRANSIENT_LOCAL 二维栅格，且传感器进程组可跨主链复用；无硬件 smoke publisher 覆盖传感器、底盘、安全、定位、栅格、TF 和 fake lifecycle | 用真实 active-map、Nav2 lifecycle 和硬件执行 graph smoke；完成实机长期健康/进程回收验收 |
| `agt_map_manager` | 7 | P3 离线完成 | manifest source of truth、SQLite scan rebuild、PGM/YAML/PCD processing record/hash 校验、原子 active pointer、pin/archive/trash/retention 通过 | 为已有 legacy runtime maps 编写显式导入工具并在真实地图上审计资产 |
| `agt_experiment_manager` | 7 | P4 离线完成 | 原子 session manifest、events/localization JSONL、health snapshots、异常恢复、显式 bag profiles、Web bag start/stop、固定根目录 bag 列表/回放、mapping/localization 输入回放白名单、summary/report 通过 | 接入完整健康/导航事件和真实 rosbag 长包后生成统一报告 |
| `agt_web_console` | 7/8 | P5/P6/P8 离线完成，FastAPI runtime optional | REST/WebSocket 路由、loopback/token 边界、审计事件、受限日志根、中文总控台、ROS/离线后端切换、白名单 profile 流程和模拟重定位入口完成；ROS bridge 只调用项目 Action/SRV，显示 BUNKER/CAN 状态和有界点云预览；传感器按钮按证据锁定，建图/导航独立控制；建图支持 `start_sensor:=false` 历史 bag 输入，算法等待话题；建图模式强制 raw-input 回放并排除算法输出和旧 TF，持久地图订阅使用 transient-local QoS；预览只在 MAPPING 模式接受点云/栅格并支持拖动、机器人居中；建图完成弹窗确认采集完成并允许最终命名，保留流程为无哈希 ready PCD 补齐 SHA-256 后再登记；建图保留/删除、PGM/YAML+ready PCD 检查和不可变地图登记；导航只接受选定 active READY 版本；离线可选择 bundle 模拟 bag 状态并在模拟建图模式显示一个有界模拟地图，最多保留一个模拟槽位但不读取 ROS 消息或导出资产，ROS 后端提供真实受限回放和 PGM/YAML/PCD 登记；缺少 `agt_system_mode_manager` 时明确提示 Action server 诊断命令；底盘管理员命令和 Qt5 -> 项目 Action -> Nav2 -> 安全链路已明确 | 在目标镜像安装 FastAPI/Starlette/Uvicorn 后做真实 localhost smoke；用真实地图完成一次保留/删除分支和版本登记；用真实 CAN、底盘、安全链和历史 bag 做硬件/回放验收 |
| `agt_evaluation` | 7 | 仅骨架 | package 和指标职责边界已建立 | 实现轨迹、重定位、导航、地图质量和资源占用指标，并生成可复现报告 |

阶段 A-E 记录（2026-07-21）：`agt_interfaces` 已新增并生成 `LocalizationStatus.msg` 与
`Relocalize.action`；`agt_localization` 已接入候选/质量/Action 编排，C++/Python 接口、候选、
质量、配置和 supervisor 测试通过。当前只提供有界基础 supervisor 与低频只读验证，不宣称
PCD 内容 hash 门禁、Nav2 lifecycle、运动一致性证明或实车验收完成。

## 阶段汇总

| 阶段 | 当前结论 | 进入下一验收级别的条件 |
| --- | --- | --- |
| Phase 0：旧系统基线 | 部分完成 | 固定旧仓库 tag/commit、参数快照和可复现报告 |
| Phase 1：仓库与接口 | 已完成 | 后续按实际需求扩充自定义接口，避免提前过度设计 |
| Phase 2：机器人描述 | 已离线完成 | 完成车辆外参和 BUNKER 几何尺寸实测 |
| Phase 3：传感器与建图 | 大包 baseline 可用 | 完成新旧输出量化报告和车辆外参优化/独立 bag 验证 |
| Phase 4：重定位 | 大地图初验通过 | 批量完成不同位置、错误初值、恢复时间和 TF 稳定性验收 |
| Phase 5：地图处理 | baseline 可用 | 完成完整 bag 地图质量对比并固定导航地图参数 |
| Phase 6：Nav2、底盘与安全 | 离线 baseline 完成 | 完成真实地图导航、CAN、硬件急停和低速制动验收 |
| Phase 7：实验与评测 | 部分完成 | 补齐配置/Git 快照、执行指标和统一报告生成 |
| Phase 8：Qt5 与覆盖规划 | 离线链基本实现，但 `execution-blocked` | 修复零长度 SWATH；完成任务 manifest、离线质量报告和独立样例数据；再评审覆盖执行 Action 与实车验收 |
| Phase 9：扩展研究 | 未开始 | 基础导航闭环稳定后再接入 RTK/UWB、语义点云和其他雷达 |
