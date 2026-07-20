# 迁移矩阵

更新时间：2026-07-20。

状态按“代码迁移、离线验证、数据回放、实机验收”分级记录。当前离线回归为
`200 passed`；离线通过不代表定位精度、导航性能或实车安全验收通过。

| 模块 | Phase | 当前状态 | 已验证范围 | 下一步 |
| --- | --- | --- | --- | --- |
| `agt_interfaces` | 1/8 | TASK-13/14 完成 | `ExecuteCoverageTask.action` 已生成并由 `/agt/coverage/execute` 消费；Python/C++ 类型、序列化和安装态导入通过 | 后续字段变更需兼容性评审，并同步服务端与客户端 |
| `agt_description` | 2/8 | 离线完成 | 固定 TF、MK-mini/BUNKER profile、Xacro 展开、TF 单父节点和可配置 MID360 外参通过；BUNKER 已录入 2026-07-19 粗测的雷达中心平移 `[0.2615, 0.0, 0.3000] m` 和向前 `30 deg` pitch；新增温室阿克曼 canonical profile，记录用户提供几何、1.5 m 转弯半径和 Hybrid-A* repair 合同 | 精测 BUNKER `base_link` 高度和 MID360 六自由度外参；实测温室车 `base_link` 相对后轴位置与运动限速，确认各平台几何校验状态 |
| `agt_bringup` | 1/3/4/5/6/8 | Bunker Qt5 FAST-LIO baseline 已接线 | mapping/navigation 共用总入口；mapping 启动独立 Qt5 profile，navigation 保持语义与覆盖默认关闭；单一 TF 和安全速度链由契约测试约束 | 实机完成建图保存、同源 PCD 重定位、Qt 初始位姿与单点导航闭环 |
| `agt_sensor_adapters` | 3 | baseline 完成 | Livox 驱动已迁入，MID360 PointCloud2 到 CustomMsg 转换、统一 topic 和短 bag 回放通过 | 实机验证网络、QoS、频率、时间戳、丢包和长时间运行稳定性 |
| `agt_mapping` | 3 | 大地图 PCD 持久化 Release 回放通过 | FAST-LIVO2 adapter、位姿/twist 外参和 TF 已接线；新增稀疏 int64 增量体素地图，过滤非有限/越界坐标并生成 ready 处理记录；x86 固定通用 ABI 以修复 Eigen/PCL 对齐导致的关机崩溃；完整 Bunker 大包将 56,263,430 输入点压至 369,970 点，关机落盘 0.134 s，峰值内存约 1.0 GiB、0 swap 并干净退出 | 使用最终 PCD 验证 NDT 收敛率和地图质量；完成车辆外参精测 |
| `agt_map_processing` | 5 | baseline 可用 | OctoMap 动态射线原点、二维 OccupancyGrid 以及 PGM/YAML 保存已通过短回放 | 完整 bag 调整高度阈值并对比旧 `/projected_map`；后续增加 PCD 离线转换和几何地面分割后端 |
| `agt_localization` | 4 | 实车与最终大地图离线重定位初验通过 | ICP/NDT core、局部点云输入、base/lidar 初值修正和唯一 `map -> odom` 发布逻辑已编译；修复 `ndt_num_threads=0` 越界并完成边界回归；Bunker 实车 4 线程 fitness 约 `0.01`–`0.02`，新 369,970 点大地图配合同包首段局部点云从原点初值成功，fitness `0.0401` | 批量验证不同位置/误差初值的收敛率、误差、恢复时间和 TF 稳定性；当前离线样本达到 100 次迭代上限，需继续调参并验证错误初值拒绝 |
| `agt_localization_fusion` | 6 | 仅骨架 | package 和领域边界已建立 | 定义融合状态与诊断接口，接入 LIO、轮速和 IMU；后续扩展 RTK/UWB 与失效降级 |
| `agt_perception` | 6/8 | baseline 完成 | 已实现 base frame 高度/量程/车体裁剪的局部障碍点云，并接入 Nav2 costmap 和 Collision Monitor；裁剪边界已由契约测试约束到 BUNKER profile | 使用典型场景点云评估地面/障碍精度、误检漏检和频率，再增加可靠地面分割 |
| `agt_navigation` | 6/8 | TASK-07 离线完成 | 原 1 m 闭环继续通过；FilterInfo、global `Static -> Keepout -> Inflation`、跨禁行墙规划失败及 toggle 后恢复规划已验证 | 使用真实语义地图与重定位验证边界误差、切换时延、规划成功率和窄通道通过性 |
| `agt_coverage_planning` | 8 | TASK-00~15 完成，TASK-16 部分；新增 planner-only 修复实验 | 外部锁定依赖已构建；Coverage Server configure 前消费 canonical 几何；临时 GML 统一为至少三点，并由已验证请求几何补正锁定 PathComponents 漏失端点；离线预览可自动调用 Hybrid-A* 修复无效 CONNECTION，并单独规划入口 approach，均复用底图、keepout、完整 footprint 和转弯半径验证 | 真实大棚已确认 planner/keepout ACTIVE；继续定位 Row Coverage 偶发无结果，并根据 repair report 调整地头/禁行语义，任何 invalid SWATH 仍禁止修复和执行 |
| `agt_safety` | 6 | baseline 完成 | BUNKER 履带仲裁、手动优先、限速、输入超时、急停锁存和复位保持禁用的合成消息回归通过 | 架空履带验证方向和急停，再完成低速制动距离、进程退出和通信中断验收 |
| `agt_chassis` | 6 | baseline 完成 | 官方 `bunker_ros2`、状态桥接、TF 隔离和双层命令 watchdog 已接入并离线构建 | CAN 实机验证协议版本、轮速里程计、错误码、方向、断连归零和长时间通讯稳定性 |
| `agt_ui_bridge` | 6/8 | Bunker 双 profile baseline；TASK-15 总控接入完成；保留 CloudCompare 修图与同窗路线预览 | 上游 Qt5 源码不修改；mapping/navigation 模板分别绑定工作图/导航图和 `odom`/`map`，运行配置隔离；手动速度统一进入 `/agt/cmd_vel_manual` | 实机验证 Qt 地图刷新、位姿、初始位姿、单点目标和手动控制；语义编辑器后续独立验收 |
| `agt_experiment_manager` | 7 | 仅骨架 | package、profile 和 runtime 目录边界已建立 | 实现配置合并、Git/参数快照、产物命名、失败恢复和一键复现实验 |
| `agt_evaluation` | 7 | 仅骨架 | package 和指标职责边界已建立 | 实现轨迹、重定位、导航、地图质量和资源占用指标，并生成可复现报告 |

## 阶段汇总

| 阶段 | 当前结论 | 进入下一验收级别的条件 |
| --- | --- | --- |
| Phase 0：旧系统基线 | 部分完成 | 固定旧仓库 tag/commit、参数快照和可复现报告 |
| Phase 1：仓库与接口 | 已完成 | 后续按实际需求扩充自定义接口，避免提前过度设计 |
| Phase 2：机器人描述 | 已离线完成 | 完成车辆外参和 BUNKER 几何尺寸实测 |
| Phase 3：传感器与建图 | baseline 完成 | 完成完整 bag 新旧输出对比和车辆外参验证 |
| Phase 4：重定位 | 代码已落地 | 使用同源全局 PCD 完成回放精度与失败恢复验收 |
| Phase 5：地图处理 | baseline 可用 | 完成完整 bag 地图质量对比并固定导航地图参数 |
| Phase 6：Nav2、底盘与安全 | 离线 baseline 完成 | 完成真实地图导航、CAN、硬件急停和低速制动验收 |
| Phase 7：实验与评测 | 部分完成 | 补齐配置/Git 快照、执行指标和统一报告生成 |
| Phase 8：Qt5 与覆盖规划 | TASK-00~15 完成，TASK-16 部分 | 修复零长度 SWATH 后启用面积指标，再完成可复现执行报告 |
| Phase 9：扩展研究 | 未开始 | 基础导航闭环稳定后再接入 RTK/UWB、语义点云和其他雷达 |
