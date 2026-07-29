# 迁移矩阵

更新时间：2026-07-27。

2026-07-28 统一业务后端增量：定义三层架构、Mission schema、RobotState 读模型及 map/bag
ROS facade。Qt/Web 正式工作流迁移到同一 manager API，旧项目 Action 在兼容期保留。

状态按“代码迁移、离线验证、数据回放、实机验收”分级记录。测试数量随模块增长，以提交时
测试报告为准；离线通过不代表定位精度、导航性能或实车安全验收通过。

Web 控制台的同一 `runtime_dir` 只允许一个进程；入口会在创建 ROS bridge
前获取单实例锁，重复启动不会再注册同名 `agt_web_console_ros_bridge` 节点。

2026-07-27 增量：真实建图统一到 `ManageMappingSession` Action。START 强制 mapping bag，
FINALIZE_CAPTURE 按“在线栅格保存 -> 正常停止 -> PCD/record/bag 校验 -> 离线射线基图 +
ground_temporal -> 质量门禁”收口，COMMIT 才生成
不可变版本。Qt 能力改为 mapping 只监看、candidate 只编辑当前候选、offline/navigation 只读
READY 底图并维护版本任务；Web 不再保留直接 SaveMap/PCD 等待/版本登记策略。

2026-07-28 针对性修复：维护版 Qt candidate 原位保存会省略 Nav2 可选 `mode` 键；
COMMIT 现仅在该键完全缺失时原子补回 `mode: trinary` 并记录修复，显式空值、`scale`、
`raw` 或其他模式继续 fail-closed，避免人工审图后无法登记版本且不放宽栅格语义。

2026-07-27 针对性修复：OctoMap 限频输入改为 steady-time 驱动的单帧 latest-slot，并等待
OccupancyGrid 发布确认后才释放下一帧，保留点云原始 frame/stamp，避免输入及 OctoMap 序列化
两层积压后使用已退出 TF cache 的旧帧；mapping bag 新增记录实际投影输入。FINALIZE 在停止
采集前统计 trinary 栅格，在线图固定为审计预览而不再直接进入 Qt；COMMIT 在编辑后复检，
全 unknown、离线证据不完整或触边候选不再进入可用地图版本。
同时修正 Humble 2.3.1 的 `point_cloud_*` 参数键，关闭其会产生全 unknown 栅格的增量二维
路径，并通过项目节点把内部 volatile raw 栅格中继为原有 transient-local 公共 topic。

| 模块 | Phase | 当前状态 | 已验证范围 | 下一步 |
| --- | --- | --- | --- | --- |
| `agt_interfaces` | 1/8 | TASK-13/14、waypoint/mapping/Mission 与统一状态接口完成 | coverage、waypoint、mapping、Mission Action，以及 RobotState/Mission/Map/Bag/Experiment msg/srv 通过 ROSIDL C++/Python 生成；新增实验枚举列表与扩展生命周期操作，核心状态使用枚举与稳定错误码 | 用真实建图和 Mission launch 验证 Action 超时、取消和进程异常分支；后续字段变更需兼容性评审 |
| `agt_description` | 2/8 | 离线完成 | 固定 TF、MK-mini/BUNKER profile、Xacro 展开、TF 单父节点和可配置 MID360 外参通过；BUNKER 已用静止 bag 地面平面与 IMU 重力估计候选 `0.607 m / +23.2 deg`，仍保留未验证状态 | 实现流式数据提取、时间偏移/激励检查和鲁棒外参优化；用独立 bag 与机械实测验证后再置为 calibrated |
| `agt_bringup` | 1/3/4/5/6/8 | Bunker Qt5 FAST-LIO baseline、一次性自动重定位、定位 freshness gate 和 Teach Mapping 复扫组合已接线 | mapping/navigation 共用总入口；mapping 默认 RViz；navigation 将同一 map YAML 注入 Nav2 与任务型 Qt；定位 5 s 验证周期配套 10 s freshness gate；lifecycle gate 在暂停后 RESUME、启动拒绝后 RESET 重试且 fail-closed；Teach Mapping 只用绑定的 Bootstrap Map 启动单套 navigation，默认断车/禁执行/录包、固定不自动 start、限速 0.10 m/s | 用实车验证完整建图关机、复扫取消/急停/录包、候选 YAML 自动重定位、同源 PCD 重定位和项目多点 Action 闭环；确认 Bunker vendor 退出告警不影响停车 |
| `agt_sensor_adapters` | 3 | baseline 完成；前置自滤除已接线，待回放/实机验收 | Livox 驱动保留原始 `/agt/sensors/lidar/custom`；新增 profile 驱动的 CustomMsg 车体/高台盒过滤、字段和顺序保留、TF fail-closed、盒子 MarkerArray 与 diagnostics；输出 `/agt/sensors/lidar/custom_filtered` | 用真实 MID360 和只含原始输入/TF 的历史 bag 验证 QoS、过滤比例、非有限点、TF 失败、频率、CPU 和长时间运行；实测高台边界后再把 `verified` 置为 true |
| `agt_mapping` | 3 | 大地图 PCD 持久化 Release 回放通过；FAST-LIVO2 已切换前置过滤输入 | FAST-LIVO2 与所需 Vikit 均按固定提交 vendor；统一 x86 Eigen/PCL ABI；稀疏 int64 增量体素过滤异常点；backend 消费 `custom_filtered`，原始 CustomMsg 保留用于 bag/replay | 在无旧 overlay 的全新工作区构建并分别回放过滤前后数据；完成 Vikit 独立许可证文本审计、最终 PCD/NDT 地图质量和车辆外参精测 |
| `agt_map_processing` | 5 | 大包混合静态障碍补全、完整车体扫掠和三组可通行性对照已离线回放；受管会话新增本次 bag 注册点云的确定性二维射线 free/unknown 重建、unknown 画布扩展和 `ground_temporal` 候选报告；全图 OctoMap 仍只作 steady-time latest-slot 在线监看；PGM 保存保留 `205` unknown round-trip；有界实时化初值仅作为禁用候选保存 | 新受管回放消费 3,347/3,347 注册点云与位姿，地面拟合零失败、零裁剪；free 从 538,994 增至 1,508,114，`ground_temporal` occupied 为 171,618，尺寸 2,382×3,034，最小有效边距 148 cells；既有 8,073 帧对照参数与数据集未修改 | 用独立新 bag 重放验证二维射线角分辨率/1 s 采样、峰值 RSS、狭窄障碍后的 unknown 保持和地图差异；完成外参标定后实现分块实时节点、资源诊断和原子落盘；实测整车最高点前高度分层仍不可执行 |
| `agt_localization` | 4 | 基础自动重定位、PCD 内容 hash 读取/候选绑定、点云时间一致性和 ready 记录门禁已接入；完整导航门禁仍未完成 | ICP/NDT core、结构化接口、候选加载/展开、last pose 原子持久化、SHA-256 PCD 身份重算、可选 `pcd_sha256` 记录校验、质量门禁、Action、处理记录 `state: ready`/`map_file` 校验和唯一 `map -> odom` 已编译；动态 `odom -> tracking_frame` 按点云时间查询，tracking validation 使用 odom 传播预测且不改写 TF；fresh duplicate 与 ROS 时间回退基线不执行配准或计数，stale/future/invalid duplicate 继续 fail-closed；tracking 搜索不发布中间权威状态，每个非 skip 结果仅由外层更新 supervisor 并发布一次最终状态；后端 convergence 与质量 accepted 分离；启动确认参数固定只允许 `1`；监督器单测覆盖确认、取消、超时及 `TRACKING -> DEGRADED -> RECOVERING -> LOST`；Bunker 实车 4 线程 fitness 约 `0.01`–`0.02`，新 369,970 点大地图同包初验 fitness `0.0401` | 用 `Benchmark-BAG-260725` 与同源 PCD 做时间一致性、静止/低速/转向、暂停 fresh duplicate 和点云停发 stale-cloud 回放；让新地图生产记录写入 `pcd_sha256`；批量验证不同位置/误差初值和短期连续性；完整多帧 bootstrap 确认留待独立任务；完成 Nav2 lifecycle gate、几何质量指标和真实地图验收 |
| `agt_localization_fusion` | 6 | 仅骨架 | package 和领域边界已建立 | 定义融合状态与诊断接口，接入 LIO、轮速和 IMU；后续扩展 RTK/UWB 与失效降级 |
| `agt_perception` | 6/8 | baseline 完成并预留语义边界 | 已实现 base frame 高度/量程/车体裁剪的局部障碍点云并接入 Nav2；文档固定未来相机/点云适配、标准化语义输出、bag 与 fail-safe 边界 | 先做离线语义推理和显示，完成数据集/延迟/stale 评测后再以默认关闭方式接入动态障碍 |
| `agt_navigation` | 6/8 | 项目多点 Action、版本化离线任务组、planner-only 预览和统一定位 freshness/task gate 完成；完整实车 lifecycle 验收待完成 | schema-v1 任务组提供原子保存/索引/备份、地图版本与 YAML/image/PCD hash 绑定、旧 points/theta 导入及任务点端点栅格校验；相邻点是 Nav2 有序目标而非直线路径，Task Library 用 `ComputePathToPose` 预览实际绕行；预览逐段发布 current/total 状态并以显式正有限超时 fail-closed 清空 `/plan`，后续段从前一段 Nav2 实际终点继续，避免目标容差导致断线或虚假的 lethal-start 失败；navigation/offline Qt 只保留 Task Library，READY 底图编辑/保存和旧拓扑任务入口关闭，新建后直接两点击定姿、保存后退出布点；`ExecuteWaypointTask` 兼容新旧 JSON，只向 Nav2 `FollowWaypoints` 派发并在接收及执行中检查 10 s 定位 freshness、2 s TaskReadiness 和安全状态；offline/mapping profile 禁止执行 | 用正式 READY 地图版本完成 Qt 关闭重载、内容/几何失配和 Action 取消集成验收；实车验证多点成功、失败、missed waypoint 和急停；补 Nav2 lifecycle 与统一运行记录，再实现最小 mission orchestrator |
| `agt_teach_repeat` | 3/4/6/7 | P0 离线数据、预览、门禁执行、重复性评测、Qt 只读路线标注及 Teach Mapping 复扫复用已实现，待实车验收 | rosbag2 直接提取 mapping odometry；原子资产与 map/PCD/record/path/annotation hash；显式 odom-to-map SE(2)；版本化方向/转弯/掉头阈值和 transient-local MarkerArray；canonical full-footprint 验证；定位/safety/TaskReadiness 动态取消；Nav2 SpeedLimit 取 manifest/外部上限较小值并在终态复位；结果/失败记录均有回归 | 用同源真实 bag/map/PCD 做 Qt/RViz 与全局 costmap 验证；先以 0.10 m/s 验证断车、架空履带、取消、急停和完整录包，再开展有人监护的短路线重复试验；独立真值与 Qt 示教执行页仍未实现 |
| `agt_coverage_planning` | 8 | 语义地图、Coverage Server 适配、PathComponents 重建、离线验证和 planner-only 修复链已实现；当前为 `execution-blocked` | 外部锁定依赖已构建；临时 GML 统一为至少三点；语义指纹、SWATH/CONNECTION 分类、full-footprint Validator、keepout 直接检查、仅 CONNECTION 修复、入口 approach 预览和 metrics-only 报告均有回归；真实大棚仍出现 `zero_length_swath`，覆盖率/重叠率保持 null | 定位并修复 Row Coverage 的零长度 SWATH/偶发无结果；补齐版本化任务 manifest、真实场景离线报告和独立样例数据；在语义、碰撞、运动学和安全门槛全部通过前不得执行 |
| `agt_safety` | 6 | baseline 与基础 localization guard、服务响应并发和权威导航门禁诊断完成 | BUNKER 履带仲裁、手动优先、导航定位失效 fail-closed、10 s 定位 freshness、限速、输入超时、急停锁存和复位保持禁用的回归通过；`/agt/safety/status` 发布 `emergency_stop`/`navigation_ready`；SetBool/Trigger 使用独立服务回调组和多线程 executor；手动输入不被定位 guard 抢占 | 架空履带验证方向和急停，再完成低速制动距离、进程退出和通信中断验收 |
| `agt_chassis` | 6 | baseline 完成，控制/只读监测角色已分离 | 官方 `bunker_ros2`、状态桥接、TF 隔离和双层命令 watchdog 已接入；`operation_mode:=monitor` 可只接收 CAN 状态，`control` 仍严格经过安全链；CAN 初始化权限留在主机管理员侧 | CAN 实机验证协议版本、轮速里程计、错误码、方向、断连归零、监测模式无命令输出和长时间通讯稳定性；后续接入替换 backend |
| `agt_ui_bridge` | 6/8 | 五 profile 能力隔离、可替换 Qt 壳层与统一业务客户端已接入 | `control-center-v1`/`legacy` 复用 ROS channel 和 ViewModel；light/dark theme 不改变 capability；RobotState/Mission、模式、受管建图、重定位、地图和 Bag/实验页面只调用项目接口；旧 waypoint/`goal_pose` 默认关闭；READY 底图只读、offline/teach 禁执行、candidate 资产约束和 `/agt/cmd_vel_manual` 默认值由 Profile/Skill 测试覆盖；Qt 完整编译及 navigation headless smoke 通过 | 用真实大地图完成 candidate 保存、COMMIT、Task Library 关闭重载和规划预览；在真实 DDS 图验证 Qt/Web 状态一致、Mission pause/resume/cancel 和 manager 错误反馈；再做实车 navigation/手动控制/急停验收 |
| `agt_system_manager` | 1/2/4/6/7 | 统一建图会话、RobotState 聚合、Teach Mapping 与安全状态来源修复完成 | 新增 2 Hz + 输入变化即时 `/agt/system/robot_state` 和一次性 service，只消费权威 topic；地图身份来自 map manager，过期证据保持 UNKNOWN/blocker；建图 START/FINALIZE/COMMIT 分别通过 experiment/map 服务管理录包、实验和版本，不再直接持有 Bag 进程或 MapRegistry；八节点 smoke 干净退出 | 用真实 MID360 验证自动 mapping profile、建图落盘、候选导入/激活和异常恢复；完成 Qt/Web 共同状态显示和实车验收 |
| `agt_mission_manager` | 7/8 | 第一版有限 Mission 后端完成 | 无 ROS 核心校验 schema/hash/path/FSM/wait/event/audit/restart；ROS Action 只编排项目 waypoint Action，pause/cancel 等待 child 确认，resume 从 feedback 子集继续并复查 map/localization/readiness；25 项纯测试通过 | 用真实 READY task 做 child success/failure/missed/cancel、事件 source/correlation、manager restart 和双客户端一致性集成验收 |
| `agt_map_manager` | 7 | ROS 业务 facade 离线完成 | 保留 registry 为 source of truth；新增 list/manage/active typed API、候选导入、manager-owned asset paths、实验引用与 destructive confirmation 保护；SystemHealth 不再读取 active pointer；14 项包测试通过 | 用已有 legacy runtime 和真实候选图审计导入、激活、保留与依赖删除拒绝 |
| `agt_experiment_manager` | 7 | ROS 业务 facade 离线完成 | list/manage/status 与 experiment list typed API 唯一持有 record/playback 和实验 lifecycle；显式 profile、固定根路径、配置/Git 快照、Bag 元数据摘要、异常进程 ERROR、重启 RUNNING->INTERRUPTED 通过；建图显式 create+start mapping 实验 | 接入完整健康/导航事件，用真实 rosbag 长包验证正常停止、异常中断、空间统计和统一报告 |
| `agt_web_console` | 7/8 | 真实后端已成为纯 ROS 客户端 | entry/service 不再 import 或实例化 MapRegistry/ExperimentManager，不读 map/experiment/process 清单；原 REST 全部经 RosConsoleBridge 调用 manager，WebSocket 以 RobotState/MissionStatus 为主；22 项测试和八节点无运动 smoke 通过；离线 Mission/真实资产操作 fail-closed | 在安装 FastAPI 的目标镜像做 localhost/browser smoke；用真实地图、历史 bag 和 Mission 验证双客户端一致性；实车 CAN/安全/底盘仍需现场验收 |
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
