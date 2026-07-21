# Automatic Relocalization Research

审计日期：2026-07-21
适用发行版：ROS 2 Humble Hawksbill
仓库基线：`c71dce3f4f196de9b611a37169a6b98d22c88b84`
状态：仅完成资料调研与架构参考，未引入新的第三方算法代码

## 1. 研究边界

本次调研服务于 `agt_localization` 的通用自动重定位设计。结论只用于确定接口、状态、编排
和验证方法，不代表本项目已经完成实车精度或安全认证。温室语义只能作为离线候选生成工具的
输入，不能进入通用重定位核心。

资料优先级为：ROS 2/Nav2/PCL 官方文档和源码、算法作者论文及官方仓库、项目本地固定源码
和许可证文本。没有确认许可证的代码只记录设计，不复制实现。

## 2. 资料与设计结论

| 资料名称 | 来源与版本/分支 | 许可证 | 准备借鉴的设计 | 明确不复制的部分 | Humble 兼容性 |
| --- | --- | --- | --- | --- | --- |
| ROS 2 managed/lifecycle nodes | [ROS 2 Humble lifecycle README](https://docs.ros.org/en/humble/p/lifecycle/__README.html)，Humble `lifecycle` 文档；[lifecycle_msgs Humble](https://docs.ros.org/en/ros2_packages/humble/api/lifecycle_msgs/index.html) | ROS 2 核心包按包声明，生命周期相关包为 Apache-2.0 | 使用 `unconfigured -> inactive -> active` 的标准状态和 `ChangeState`/`GetState` 服务；只在 active 状态允许正式工作 | 不把普通 `rclcpp::Node` 伪装成 lifecycle node；不自定义替代 `lifecycle_msgs` 状态机 | 直接兼容 Humble。定位节点是否改成 lifecycle node 要在单独阶段验证，不能与 Nav2 节点重复发布 TF |
| Nav2 Lifecycle Manager | [Nav2 Lifecycle Manager 文档](https://docs.nav2.org/configuration/packages/configuring-lifecycle.html)，[Humble 分支源码](https://github.com/ros-navigation/navigation2/tree/humble/nav2_lifecycle_manager)，`nav2_lifecycle_manager` Humble `package.xml` | `nav2_lifecycle_manager` 包声明 Apache-2.0 | 按 `node_names` 顺序配置/激活，利用 bond 检测节点失联；通过 `/<manager>/manage_nodes` 调用 `STARTUP/PAUSE/RESUME/RESET/SHUTDOWN` | 不通过 shell、kill 或重启进程控制 Nav2；不把 lifecycle manager 的 active 视为定位有效 | Humble 直接兼容。计划使用标准 `nav2_msgs/srv/ManageLifecycleNodes`，并保留 Action gate 与 safety gate |
| Nav2 初始定位与延迟可用实践 | [Nav2 Getting Started](https://docs.nav2.org/getting_started/index.html)，[AMCL 配置文档](https://docs.nav2.org/configuration/packages/configuring-amcl.html)，[Commander API](https://docs.nav2.org/commander_api/index.html) | Nav2 包按包声明；导航文档对应开源 Nav2 | 初始位姿是定位输入，定位就绪后 TF 树才完整；使用显式 `waitUntilNav2Active`/lifecycle startup 语义，并把初始位姿、定位状态和导航 Action 分开 | 不直接采用 AMCL；不把 `setInitialPose` 或“节点 active”当作点云配准可信度；不把 Commander 作为本项目运行时依赖 | 文档体现的接口在 Humble 有对应概念，但当前仓库使用自有 PCD 定位，必须通过本项目状态门禁实现 |
| Nav2 生命周期服务定义 | [Humble `ManageLifecycleNodes.srv`](https://github.com/ros-navigation/navigation2/blob/humble/nav2_msgs/srv/ManageLifecycleNodes.srv)，[Humble lifecycle manager implementation](https://github.com/ros-navigation/navigation2/blob/humble/nav2_lifecycle_manager/src/lifecycle_manager.cpp) | Apache-2.0 的 Nav2 包 | 固定命令枚举：`STARTUP=0`、`PAUSE=1`、`RESUME=2`、`RESET=3`、`SHUTDOWN=4`；调用 manager 服务，而不是逐个 hack 节点 | 不编造不存在的“定位有效后自动激活”服务；不在 supervisor 中复制 Nav2 lifecycle manager | 直接兼容 Humble。失败时先取消项目 Action，再按策略调用 manager；若暂停不适合当前栈，保留节点存活并关闭执行门禁 |
| PCL registration API | [PCL registration module](https://pointclouds.org/documentation/group__registration.html)，[PCL Registration API](https://pointclouds.org/documentation/tutorials/registration_api.html)，[Registration class](https://pointclouds.org/documentation/classpcl_1_1_registration.html) | PCL 许可证以实际发行包为准，常见 PCL 源码许可为 BSD-3-Clause | 继续使用需要初值的 ICP/NDT/GICP；通过 KD-tree 对应、最大距离和 correspondence rejection 构造有界质量检查 | 不从文档复制实现；不把 PCL `hasConverged` 或单独 `getFitnessScore` 直接当作全局定位接受条件；不对巨大地图做无界全对全距离计算 | PCL API 在 Humble 系统包中可用，但具体版本和编译选项必须以构建环境核对 |
| PCL fitness 与对应关系 | [PCL `getFitnessScore`](https://pointclouds.org/documentation/classpcl_1_1_registration.html)，[PCL correspondence rejection](https://pointclouds.org/documentation/correspondence__rejection_8h_source.html) | 同 PCL 实际包 | 将 fitness 明确定义为有最大对应距离约束的平均平方距离；inlier/overlap 使用同一有界 KD-tree 与阈值计算，并记录有效点数 | 不声称 fitness 是概率或唯一性证明；不把不同采样率、不同裁剪区域的分数直接跨实验比较 | 逻辑可在 Humble PCL 上实现，阈值需要用同源 PCD 和合成数据标定 |
| NDT-OMP/GICP 现有后端 | [项目本地 `ndt_omp_ros2`](../third_party/ndt_omp_ros2/)，[上游 ndt_omp](https://github.com/koide3/ndt_omp)；本地 `LICENSE` 和测试 | 本地 LICENSE 为 BSD-2-Clause；包 metadata 记录为 BSD | 复用当前 NDT-OMP 线程边界、`DIRECT7` 默认搜索和 GICP/NDT 后端边界；候选级顺序执行，避免候选并发叠加内部 OpenMP | 不在本轮升级或重写 NDT-OMP；不使用未检查线程数创建工作缓冲；不同时启用候选并发和每候选多线程 | 当前本地源码已接入 Humble 构建；运行时保持 `ndt_num_threads=4`，参数入口拒绝非正数，核心继续防御性钳制 |
| 项目现有 `relocalization_core` | 项目本地 [README](../third_party/relocalization_core/README.md)、[types](../third_party/relocalization_core/include/relocalization_core/types.hpp)、[LICENSE](../third_party/relocalization_core/LICENSE) | Apache-2.0 | 保留 `Relocalizer`、`RelocalizationRequest/Result`、地图加载、预处理和 ICP/NDT 后端；在 ROS 适配层外增加候选、质量和 supervisor 边界 | 不修改已有配准算法来伪造自动搜索；不把 ROS topic、TF、Action 或生命周期塞回纯 C++ 核心；不删除已有 `/initialpose` 行为 | 当前已用于 ROS 2 Humble；新增调用必须先通过现有 core 单测，再逐阶段扩展 |
| Scan Context 原始论文 | [IROS 2018 DOI](https://doi.org/10.1109/IROS.2018.8593953)，作者参考仓库 [SignalImageCV/scancontext](https://github.com/SignalImageCV/scancontext) | 原始参考仓库根目录审计时未找到可确认的宽松代码许可证；论文/算法引用不等于代码许可 | 只借鉴“全局描述子 -> 候选地点检索 -> 精配准”的两级架构；未来 CandidateProvider 可接入关键帧数据库 | 不复制 MATLAB/C++ 代码，不把未确认许可证代码 vendoring；不在当前阶段实现 Scan Context 数据库 | 作为架构参考与 ROS 2 无直接依赖；后续若实现必须重新确认许可证、数据格式和 Humble 构建方式 |
| Scan Context++ | 作者维护仓库 [gisbi-kim/scancontext_tro](https://github.com/gisbi-kim/scancontext_tro)，T-RO 2021/2022 代码线 | 仓库 README 明确为 CC BY-NC-SA 4.0 | 只记录其旋转/横向变化鲁棒性和候选检索思路，作为未来独立任务输入 | 不复制、修改、编译或分发其代码；非商业共享许可与本项目 Apache 自有代码及潜在产品目标不作为默认兼容依赖 | 与 Humble 没有现成 ROS 2 包兼容性；本轮明确排除 |
| Autoware NDT scan matcher | [Autoware Universe NDT scan matcher docs](https://autowarefoundation.github.io/autoware.universe_planning/pr-5694/localization/ndt_scan_matcher/)，[Autoware Core docs](https://autowarefoundation.github.io/autoware_core/pr-602/localization/autoware_ndt_scan_matcher/)，[Autoware repository](https://github.com/autowarefoundation/autoware.universe) | Autoware Universe 相关包为 Apache-2.0，最终以实际 package/file 声明为准 | 借鉴初值服务、`transform_probability`/likelihood、初值到结果距离、执行时间、点数和 diagnostics；借鉴把定位计算与质量监控分开的做法 | 不迁移 Autoware 节点、参数、消息或 Monte Carlo 实现；不把 Autoware API 当作本项目接口；不复制其源码 | Autoware 版本与 Humble 组合需单独确认；本项目只借鉴公开架构，当前不增加 Autoware 依赖 |
| MOLA relocalization | [MOLA 官方仓库](https://github.com/MOLAorg/mola)，相关 `mola_relocalization` 模块 | MOLA 采用 Open Core；核心部分与 `mola_relocalization` 许可需按模块核对，公开资料显示核心包含 GPLv3，MRPT/mp2p_icp 等组件另有 BSD-3-Clause | 借鉴“候选/地图检索 -> 粗配准 -> 精配准 -> 质量判定”的分层思想和可观测性 | 不引入 MOLA、MRPT、mp2p_icp 或其模块源码；不把 GPLv3/Open Core 组件默认为产品可用；不复制实现 | 当前没有经本项目确认的 Humble 主链依赖；列为未来架构参考和许可证阻塞项 |
| robot_localization | [官方 ROS 2 repository](https://github.com/cra-ros-pkg/robot_localization/tree/ros2)，[navsat_transform_node 文档](https://docs.ros.org/en/jade/api/robot_localization/html/navsat_transform_node.html) | `robot_localization` package metadata 为 Apache License 2.0；具体发行版仍需核对 | 未来融合接口使用 `nav_msgs/Odometry`、`sensor_msgs/Imu`、`sensor_msgs/NavSatFix` 和带协方差的绝对位姿；明确 GPS/IMU/odometry 的时间、frame、ENU 与协方差要求 | 不在本轮启动 EKF/UKF、`navsat_transform_node` 或 GNSS/UWB 驱动；不把粗位置未经验证直接写入 `map -> odom` | 官方 ros2 分支当前可能随发行版演进；仅作为接口设计参考，不作为本轮 Humble 运行依赖 |
| RViz PointCloud2/MarkerArray/TF | [RViz Humble default plugins](https://docs.ros.org/en/ros2_packages/humble/api/rviz_default_plugins/generated/index.html)，[PointCloud2 display](https://docs.ros.org/en/humble/p/rviz_default_plugins/generated/file_include_rviz_default_plugins_displays_pointcloud_point_cloud2_display.hpp.html)，[MarkerArray display](https://docs.ros.org/en/ros2_packages/humble/api/rviz_default_plugins/generated/classrviz__default__plugins_1_1displays_1_1MarkerArrayDisplay.html) | `rviz_default_plugins` Humble package metadata 为 BSD | 使用标准 `sensor_msgs/PointCloud2`、`visualization_msgs/MarkerArray`、`Marker` 和 TF；全局地图 transient-local、扫描 debug topic 低频/可选 | 不创建 PCL/VTK GUI，不把渲染代码放入 Qt 或 localization core，不让 RViz 计算质量或发布控制 | 直接兼容 Humble RViz2；具体 QoS 要与发布端一致 |

## 3. 关键技术结论

### 3.1 生命周期不是定位有效性

Humble lifecycle 节点的 active 表示节点完成配置并允许其 active publisher/工作逻辑运行。Nav2
Lifecycle Manager 还会按顺序管理节点，并用 bond 监测节点存活。这两件事都不能证明点云已经与
正确的全局 PCD 对齐。

因此本项目采用两层门禁：

```text
LocalizationStatus.pose_valid == true
  -> project Action gate 允许新的导航任务
  -> agt_safety localization guard 允许导航输入继续通过

Nav2 lifecycle manager
  -> 只负责 lifecycle 节点 startup/pause/resume/reset/shutdown
```

定位丢失时的顺序必须是：取消项目 Action 及其 Nav2 child，通知安全层，关闭新的任务入口，
再按验证过的策略调用 `PAUSE` 或保持 Nav2 节点存活但禁止执行。不能通过进程 kill 伪造暂停。

### 3.2 配准结果要分成三层

当前 core 提供 `has_converged` 和 `fitness_score`，它们只说明后端完成了数值迭代和一个距离
指标。第一版自动重定位应分成：

```text
REGISTRATION_CONVERGED
  后端收敛，保留所有原始质量数据

LOCALIZATION_ACCEPTED
  fitness、inlier、overlap、修正量、地图边界、候选分数差、短期运动一致性和连续稳定性通过

LOCALIZATION_REJECTED / AMBIGUOUS_RESULT
  收敛但无法证明是唯一正确地点，必须禁止更新可执行 TF
```

`overlap_ratio` 和 `inlier_ratio` 不是 PCL 统一标准字段，必须在本项目文档中定义：在有限的
对应距离和采样预算下，用最终变换后的 scan 点对 map KD-tree 查询，分别记录有效对应点占比和
阈值内点占比。候选唯一性至少需要第一、第二候选在同一配置和同一质量函数下的分数 margin；
重复平行行场景中 margin 不足应标记 `AMBIGUOUS_RESULT`，不能用 heading 猜测正确行。

### 3.3 计算资源必须分层限流

第一版候选测试固定顺序执行。粗配准只保留有界 `top_k` 候选，再对少量候选执行精配准。
候选级并发和每候选内部 NDT-OMP 线程不能同时无界增加。ndt_num_threads 在 ROS 参数描述符
中限制为正整数，核心 `sanitizeNdtNumThreads()` 继续防御性钳制，默认保留当前经过验证的 `4`。

### 3.4 可视化应使用标准消息

RViz 已能显示 PointCloud2、MarkerArray 和 TF。定位节点只发布计算结果和可选调试数据；RViz
配置负责显示开关、颜色和视角。全局地图加载后使用 transient-local、depth 1 的独立 publisher，
扫描和对齐结果只在 debug 开启时限频发布，避免每帧复制整张大地图。

## 4. 建议的最小候选架构

```text
/initialpose --------------------┐
Relocalize goal explicit seed ----┤
last_valid_pose.yaml -------------┤
configured candidates YAML --------┤ -> CandidateProvider chain
/agt/localization/coarse_pose ----┘             |
                                               v
                                   bounded deterministic SE(2) expansion
                                               |
                         coarse NDT -> top K -> fine NDT/ICP
                                               |
                         QualityValidator -> Supervisor state machine
                                               |
                    LocalizationStatus / global_pose / map -> odom
```

温室工具只生成通用 `localization_seeds.yaml`，运行时不链接 Qt、Shapely、GeoJSON 编辑器、
Fields2Cover 或温室对象类型。

## 5. 许可证决策

- 当前本地 `relocalization_core` 为 Apache-2.0，`ndt_omp_ros2` 为 BSD-2-Clause，可继续使用本地快照，保留许可证和版权文件。
- ROS 2、Nav2、robot_localization 的具体 package 许可证以安装版本 `package.xml` 和源码为准，不能用总项目许可证替代。
- Scan Context 原始参考代码未确认宽松许可证，Scan Context++ 明确为 CC BY-NC-SA 4.0，均不作为本轮代码依赖。
- Autoware 只作为 Apache-2.0 项目的架构参考，不复制源码或消息。
- MOLA/Open Core 的模块级许可证未满足本项目的默认依赖条件，不引入。
- `third_party/ros_qt5_gui_app` 保持不改；其 GPL-2.0 归属和 fork provenance 仍按仓库现有规则管理。

## 6. 资料限制与后续复核

官方 Nav2 文档站点当前展示的是最新页面；本次凡涉及实际 API 的地方优先核对 Humble 分支源码和
Humble package metadata。PCL、RViz 和 robot_localization 的系统安装版本、编译选项、CPU ABI
和传递依赖在实际构建时必须重新记录。

本文件不构成许可证法律意见、不构成算法精度承诺，也不授权将任何参考仓库代码复制进项目。
