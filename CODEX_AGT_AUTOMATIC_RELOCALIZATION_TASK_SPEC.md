# Automatic Relocalization Task Specification

审计日期：2026-07-21
仓库基线：`c71dce3f4f196de9b611a37169a6b98d22c88b84`
分支：`main`
适用发行版：ROS 2 Humble Hawksbill
状态：分阶段实现中；A-E 基础链路已实现，F-I 及完整验收仍未完成

## 0. 任务原则

本任务是在现有 V2 BUNKER + MID360 + FAST-LIVO2 + Nav2 baseline 上增量建立通用自动重定位
能力，不是一次性重写定位系统。实现顺序必须是接口、候选、配准编排、质量、状态机、门禁、
测试、可视化，且每一阶段完成后才能进入下一阶段。

通用定位能力不得依赖温室语义、RTK、UWB、某个传感器驱动或 Qt。温室先验只允许通过离线工具
生成通用候选 YAML。`agt_localization_fusion` 本阶段只完善边界文档，不实现 EKF、UKF、因子图
或 `robot_localization` 主链。

## 1. 当前仓库已经具备的能力

### 1.1 系统与数据链

- MID360、IMU 和 FAST-LIVO2 纯 LIO 已接入项目 topic；FAST-LIVO2 adapter 将连续位姿转换为项目 `odom -> base_footprint` 语义，并可转发注册点云。
- 建图模式可保存二维 PGM/YAML 和增量稀疏体素定位 PCD；PCD 只有旁路 `localization_map.processing.yaml` 为 `state: ready` 时才是正式定位输入。
- 当前导航系统已有 map server、planner、controller、BT navigator、waypoint follower、collision monitor 和 Nav2 lifecycle manager。
- `agt_safety` 已有手动优先、导航输入超时、急停锁存、速度限制、加减速限制、明确 motion enable 和 BUNKER watchdog 边界。
- `agt_ui_bridge`/维护版 Qt 已支持地图显示、`/initialpose` 入口和 waypoint Action 前端；本任务不修改 `third_party/ros_qt5_gui_app`。

### 1.2 当前重定位 baseline

`agt_localization` 目前由 ROS 2 C++ 节点 `relocalization_node` 编排：

- 启动时加载 `global_map_pcd`，调用现有 `relocalization_core::Relocalizer`；core 提供 ICP/NDT、体素预处理、CropBox、fitness 阈值和 NDT-OMP。
- 订阅 `/agt/mapping/registered_points`，缓存最新单帧点云；`/initialpose` 和 `/agt/localization/relocalize` 统一进入同一候选、配准和质量路径。
- CandidateProvider 支持 configured、last pose、外部 coarse pose 和 Action 初值，执行有界、确定性的 SE(2) 展开、排序、去重和裁剪。
- 质量门禁区分 backend convergence 与 localization acceptance；只有接受结果才允许基准模式更新唯一 `map -> odom`，并发布结构化 status/global pose。
- LocalizationSupervisor 已提供有界状态计数和 `TRACKING -> DEGRADED -> RECOVERING -> LOST` 迁移；接受后按周期执行只读 tracking 验证，验证不改写 `map -> odom`。
- 旧的 `std_msgs/String` status 继续作为人类调试兼容接口；`LocalizationStatus`/`Relocalize` 由 ROSIDL 生成并有 C++/Python 类型测试。
- ROS 参数入口已对 `ndt_num_threads` 使用正整数范围描述符；core 和 NDT-OMP 也会对绕过 ROS 入口的非正值做防御性钳制，当前验证值为 `4`。

### 1.3 当前明确缺失项

- 已实现 PCD `localization_map.processing.yaml` 的 `state: ready`/`map_file` 运行时门禁；定位读取端已自动计算并绑定 PCD 内容 hash，新地图生产记录写入 `pcd_sha256` 仍待补齐。
- overlap/inlier、地图边界、运动预测一致性和候选 margin 仍未接入真实质量计算；当前 QualityValidator 只覆盖基础 fitness、点数、创新量和可选几何字段。
- 尚未实现自动重试冷却、完整的启动前置条件状态机和可复现的恢复策略；当前 supervisor 只提供有界状态迁移和低频验证。
- waypoint Action 与 `agt_safety` 已接入基础 localization guard；Nav2 lifecycle gate、状态 grace period 和完整导航编排仍未完成。
- 当前点云 topic/frame 的时间、点数、单帧/累计语义尚无专门 bag 契约测试。
- 已有一个默认关闭、单次有界的启动自动重定位 Action 客户端；尚无温室候选生成工具、离线合成点云 smoke test、launch smoke 报告和 RViz 定位调试配置。
- `agt_localization_fusion` 只有 package 骨架，未来输入和 TF ownership 尚未形成定位专用合同。

## 2. TF 发布责任

### 2.1 本任务基准模式

```text
FAST-LIVO2 adapter / continuous odometry:
    odom -> base_footprint

agt_localization:
    map -> odom，唯一发布者

agt_description / robot_state_publisher:
    base_footprint -> base_link -> sensor frames
```

当前 `fast_livo2_mapping.launch.py` 已关闭 FAST-LIVO2 backend 自身 TF；adapter 的
`publish_tf` 默认保持连续 odometry 的唯一责任。自动重定位不得新增第二个 `map -> odom` 发布者，
也不得让 debug topic 或 RViz 发布 TF。

### 2.2 未来融合模式

未来启用 `agt_localization_fusion` 时，必须在同一启动模式中选择唯一 owner：

```text
agt_localization:
    publish_tf=false
    publish_global_pose_measurement=true

agt_localization_fusion:
    唯一发布连续状态和所需 TF
```

本任务只预留参数和文档，不同时启动两个 owner，不实现融合。

## 3. Package 职责边界

| Package | 本任务职责 | 输入 | 输出 | TF 责任 | 非目标 |
| --- | --- | --- | --- | --- | --- |
| `agt_interfaces` | 生成稳定定位 msg/action | 状态、候选请求、质量结果的 ROS 字段 | `LocalizationStatus`、`Relocalize` 生成类型 | 无 | 不暴露所有内部候选结构，不安装未生成的接口文本 |
| `agt_localization` | 地图校验/加载、候选、点云预处理、core 粗到精配准、质量、supervisor、Action、状态和 debug | ready PCD、当前点云、TF、`/initialpose`、coarse pose、候选 YAML | status、global pose、Action、aligned/debug clouds、候选 markers、可选 `map -> odom` | 基准模式唯一发布 `map -> odom` | 不做连续多传感器融合、Nav2 控制、速度、安全使能、Qt 算法 |
| `agt_localization_fusion` | 完善未来输入/输出/TF ownership 文档和接口草案 | LIO、轮速、IMU、RTK、UWB、global pose measurement 的未来输入 | 未来融合状态和诊断的边界说明 | 未来模式唯一 owner，当前不发布 | 不实现 EKF/UKF/因子图/驱动 |
| `agt_bringup` | 启动定位、定位门禁、Nav2 lifecycle 调用和失效编排 | LocalizationStatus、Action 状态、Nav2 manager 服务 | Nav2 startup/pause/resume/reset 请求、系统诊断 | 不发布 TF | 不实现 ICP/NDT、候选或质量算法 |
| `agt_safety` | 消费结构化定位状态，增加独立 localization guard | `LocalizationStatus`、现有速度/急停/手动输入 | 停止/禁止导航输入的安全状态 | 无 | 不执行重定位、不替代硬件急停和 watchdog |
| `agt_mapping` | 只提供当前点云/里程计语义确认与必要的 topic 文档/测试 | FAST-LIVO2 output | 既有 odometry、registered points | 继续只负责 `odom -> base_footprint` | 不累计无限点云，不改 validated map/PCD 算法 |
| `agt_ui_bridge` | 启动 RViz 调试界面，显示标准 topic，触发 Action/service | status、PointCloud2、MarkerArray、TF | RViz 配置和可选薄控制面板 | 无 | 不实现候选搜索、ICP/NDT、TF 计算 |
| `third_party/ros_qt5_gui_app` | 本阶段不修改 | 既有标准接口 | 无 | 无 | 除非后续证明标准接口不足并完成 fork/provenance/GPL 流程 |

## 4. 计划新增或修改的文件

以下是计划，不代表本次已经创建。具体文件可在实现阶段按真实编译依赖合并，但不得扩大为大规模重构。

### 4.1 阶段 A：结构化接口

计划新增：

```text
src/agt_interfaces/msg/LocalizationStatus.msg
src/agt_interfaces/action/Relocalize.action
src/agt_interfaces/test/test_localization_status_serialization.py
src/agt_interfaces/test/test_relocalize_serialization.py
docs/interfaces/automatic_relocalization.md
```

计划修改：

```text
src/agt_interfaces/CMakeLists.txt
src/agt_interfaces/package.xml
src/agt_interfaces/README.md
docs/interfaces/core_topics.md
docs/migration/migration_matrix.md
```

### 4.2 阶段 B-C：候选与配准编排

优先在现有 `agt_localization` 内增加少量清晰组件：

```text
src/agt_localization/include/agt_localization/candidate_provider.hpp
src/agt_localization/include/agt_localization/quality_validator.hpp
src/agt_localization/include/agt_localization/localization_supervisor.hpp
src/agt_localization/src/candidate_provider.cpp
src/agt_localization/src/quality_validator.cpp
src/agt_localization/src/localization_supervisor.cpp
src/agt_localization/src/relocalization_node.cpp
src/agt_localization/config/relocalization.yaml
src/agt_localization/config/candidates.example.yaml
src/agt_localization/test/test_candidate_provider.py
src/agt_localization/test/test_quality_validator.py
src/agt_localization/test/test_localization_supervisor.py
```

如果 C++/Python 边界需要调整，必须说明原因；第一版不引入 pluginlib。

### 4.3 阶段 D-F：质量、supervisor、门禁

计划修改：

```text
src/agt_localization/src/relocalization_node.cpp
src/agt_localization/config/relocalization.yaml
src/agt_localization/launch/relocalization.launch.py
src/agt_bringup/launch/navigation_system.launch.py
src/agt_navigation/launch/navigation.launch.py
src/agt_navigation/config/nav2_bunker.yaml
src/agt_safety/scripts/tracked_safety_controller.py
src/agt_safety/config/bunker_safety.yaml
src/agt_safety/README.md
src/agt_bringup/README.md
```

可能新增一个 `agt_bringup` supervisor 节点，但只有在 launch 编排无法由现有节点/标准 service
完成时才创建；该节点不得包含点云配准算法。

### 4.4 阶段 G-H：温室离线工具与 smoke test

计划新增：

```text
tools/localization/generate_semantic_relocalization_seeds.py
tools/localization/run_relocalization_smoke_test.sh
tools/localization/testdata/README.md
tools/localization/testdata/*.pcd  # 仅在体积可控时；优先测试时生成
docs/workflows/relocalization_offline_smoke.md
docs/architecture/localization_and_fusion.md
```

候选生成工具依赖应保持离线/工具侧，不增加 `agt_localization` 运行时对 Qt、Shapely、GeoJSON、
Fields2Cover 的依赖。

### 4.5 阶段 I：RViz 调试

计划新增：

```text
src/agt_ui_bridge/rviz/localization_alignment.rviz
src/agt_ui_bridge/launch/localization_alignment_view.launch.py
docs/workflows/relocalization_offline_visualization.md
```

计划修改：

```text
src/agt_ui_bridge/README.md
src/agt_ui_bridge/CMakeLists.txt
```

不修改 `third_party/ros_qt5_gui_app`。

### 4.6 文档同步清单

实现阶段每个产生架构/接口变化的阶段必须同步检查：

```text
README.md
AGENTS.md
docs/architecture/overview.md
docs/architecture/localization_and_fusion.md
docs/interfaces/core_topics.md
docs/interfaces/automatic_relocalization.md
docs/workflows/relocalization_offline_smoke.md
docs/workflows/relocalization_offline_visualization.md
docs/migration/migration_matrix.md
src/agt_localization/README.md
src/agt_localization_fusion/README.md
src/agt_bringup/README.md
src/agt_safety/README.md
src/agt_ui_bridge/README.md
```

根目录现有总体方案和用户未提交修改必须保留；只有自动重定位方案与现有 V2 说明对齐后，才在
实现阶段以最小差异同步内容。

## 5. 分阶段输入、输出和验收方法

### 阶段 0：审计与计划（当前）

输入：附件任务、`AGENTS.md`、当前 V2 文档、指定 package、Git 状态、官方资料。
输出：本规格、`docs/research/automatic_relocalization_research.md`。
验收：已记录 HEAD/分支/工作区未提交修改；已读取当前重定位 core、TF、launch、Nav2、safety、interfaces 和 UI 边界；未覆盖用户改动。
状态：完成；A-E 基础链路和 F 阶段 Action/safety 基础门禁已实现，后续门禁与验收继续分阶段进行。

### 阶段 A：结构化接口

输入：现有 `/initialpose`、字符串 status、core result、标准 PoseWithCovariance。
输出：生成的 `LocalizationStatus`、`Relocalize`，稳定状态/错误码、兼容旧 status topic。
验收：

- `rosidl_generate_interfaces` 生成 Python/C++ 类型；不得仅安装 `.msg`/`.action` 文本。
- 序列化测试覆盖 Goal/Feedback/Result 和字段默认值；C++ 生成类型编译测试通过。
- 新系统逻辑只解析数值 `state/error_code/pose_valid`，不解析字符串。
- docs/interfaces、core topics、migration matrix 同步。

### 阶段 B：通用 CandidateProvider

输入：manual initial pose、Action 显式初值、runtime last pose、配置 YAML、`/agt/localization/coarse_pose`。
输出：带来源、ID、map/PCD hash、协方差和 SE(2) 搜索范围的内部候选列表。
验收：

- last pose 原子写入，记录 map_id、PCD hash、时间、frame 和质量；hash 不一致拒绝。
- configured YAML 与温室语义完全解耦；source ID 和 priority 保留。
- 外部 coarse pose 只做 frame/时间/协方差校验和候选范围转换，不直接发布 TF。
- 候选展开顺序确定、无 NaN/Inf、去重、数量上限有效；裁剪顺序固定为 source priority、距离/协方差、candidate ID。

### 阶段 C：粗到精配准

输入：`state: ready` 的 PCD、经 TF 转到 tracking frame 的最小点云、候选列表、当前 core。
输出：粗 NDT top-K、细 NDT 或现有 ICP/NDT backend 结果及每候选运行数据。
验收：

- 保留现有 `relocalization_core` 主接口和 `/initialpose` 行为，不复制第二套配准逻辑。
- 候选顺序执行；不同时使用候选级并发和每候选 NDT-OMP 多线程。
- `ndt_num_threads` ROS 参数拒绝 `0`/负数，core/NDT-OMP 仍防御性钳制，默认值保持 `4`。
- 点云 topic 语义通过 bag/launch 测试确认是单帧、局部累计或运动补偿点云；若是累计点云，必须有时间窗口、点数、内存和过期限制。
- ready metadata、map load failure、TF missing、点数不足、非有限初值均 fail-closed。

### 阶段 D：QualityValidator

输入：每候选 `RelocalizationResult`、初值/最终位姿、扫描和有限 map KD-tree、短期 LIO 预测。
输出：`REGISTRATION_CONVERGED`、`LOCALIZATION_ACCEPTED` 或明确 rejection/ambiguity，包含质量字段。
验收：

- 质量参数全部在 YAML，记录单位、计算窗口、最大对应距离和点数上限。
- fitness、有效对应点比例、inlier ratio、overlap ratio、平移/yaw correction、候选分数 margin、地图边界和短期运动一致性均可测试。
- 结果必须连续满足配置次数才接受；重复平行行的近同分候选必须标记 ambiguous 或 wrong-row suspected。
- 收敛但质量不可信时不更新 `map -> odom`。

### 阶段 E：Localization Supervisor

输入：地图状态、点云就绪、TF、CandidateProvider、QualityValidator、Action cancel/deadline。
输出：`UNINITIALIZED/SEARCHING/VERIFYING/TRACKING/DEGRADED/RECOVERING/LOST/ERROR`，自动重试、稳定 pose 和 diagnostics。
验收：

- 合法状态迁移和非法迁移有单测；重试次数、冷却、deadline 和 cancellation 可复现。
- 启动顺序为 map ready -> TF -> scan -> last/configured/external candidates -> search -> consecutive verify -> TRACKING。
- TRACKING 默认低频验证且不持续修改 `map -> odom`；只有显式 refine 或 RECOVERING 允许更新。
- 连续失败按 `TRACKING -> DEGRADED -> RECOVERING -> LOST`，恢复前取消自动导航并通知安全层。
- LOST 保持 fail-closed，允许人工 `/initialpose` 或外部 coarse pose 再次触发。

### 阶段 F：Nav2 lifecycle 与 safety gate

输入：结构化 `LocalizationStatus`、项目导航 Action 状态、Nav2 lifecycle manager API、现有 safety inputs。
输出：定位门禁、导航可用性、标准 lifecycle startup/pause/resume/reset 编排和安全诊断。
验收：

- 未 `pose_valid` 时项目导航 Action 拒绝；不依赖 Qt 或距离轮询。
- 定位稳定后按标准 `/<manager>/manage_nodes` 服务和 `nav2_msgs/srv/ManageLifecycleNodes` 激活/恢复，不使用 kill/shell hack。
- LOST/过期/DEGRADED 超过 grace period 时先取消 active child，再关闭新任务入口，最后按验证策略 PAUSE 或保持节点存活但 safety gate 阻止运动。
- `agt_safety` 保留手动优先、急停、输入超时、watchdog；定位 guard 是独立条件，状态 stale fail-closed。
- 不产生第二套 Nav2、第二个 lifecycle manager owner 或第二个 TF publisher。

### 阶段 G：温室候选优化

输入：Nav2 map YAML、GeoJSON、coverage.yaml、canonical platform profile。
输出：通用 `localization_seeds.yaml`、可视化数据、JSON 过滤报告。
验收：

- entry/access lane/safe staging/row endpoints/derived aisle/charging point 只作为来源对象生成候选。
- 排除 field 外、exclusion/keepout、无效 heading 的候选；支持正反向和配置间距。
- 输出来源 object ID、过滤原因和 map hash；运行时只读取通用 YAML。
- 工具不被 `agt_localization` 编译或运行时导入；不修改 GeoJSON、coverage 或 PGM。

### 阶段 H：离线 smoke test

输入：小型可版本控制的合成点云/地图、配置快照、ROS 2 Humble 环境。
输出：单元/合成配准/launch 报告目录，至少含 `summary.json`、`test_commands.txt`、`colcon_test_result.txt`、`configuration_snapshot.yaml`、`git_snapshot.txt`。
验收：

- 覆盖候选解析/排序/裁剪、协方差、hash、原子保存、NaN/Inf、质量阈值、状态迁移、超时取消、TF 计算、非零外参、线程边界。
- 合成数据覆盖唯一结构、重复平行行、错误候选、同分候选、点数不足、异常点、无地图、metadata not ready。
- launch smoke 不启动真实底盘、不发布速度；验证 Action、结构化 status、唯一 `map -> odom`、定位门禁开闭。
- 脚本检查 ROS 2 Humble 环境和依赖版本，但不写死用户名、工作区、地图或设备路径；runtime 报告不提交 Git。

### 阶段 I：RViz 调试界面

输入：global map、initial scan、aligned scan、candidate/best markers、status、TF。
输出：RViz 配置和标准 topic 启动入口。
验收：

- 全局地图 frame `map`，transient-local、depth 1、加载后低频/一次发布。
- scan initial/aligned 使用 SensorDataQoS 或明确匹配的 QoS，仅 debug 开启时限频且限制最大点数。
- RViz 同窗显示全局地图、初值扫描、最终扫描、候选箭头、最好候选、TF、crop/status text。
- 不引入 Qt/PCLVisualizer，Qt 如增加只调用 Action、订阅 status，不复制算法。

## 6. 质量与状态接口草案

### 6.1 `LocalizationStatus.msg`

第一版至少需要：Header、状态枚举、`pose_valid`、稳定 error code、backend、candidate source、
map id/hash、`PoseWithCovarianceStamped global_pose`、converged/accepted/ambiguous 语义、fitness、
overlap、inlier、ambiguity、translation/yaw innovation、runtime、tested/total candidates、
success/failure counters 和人类可读 message。最终字段在阶段 A 根据 ROSIDL 生成和测试便利性定稿，
任何字段变化必须更新接口文档、序列化测试和 migration matrix。

### 6.2 `Relocalize.action`

Goal 只暴露自动搜索、单初值/局部/外部 coarse 选项、last/configured candidate 开关、最大候选数、
debug 开关和 timeout/deadline 等稳定控制字段。Feedback 只包含当前状态、候选计数、最好分数/来源和
运行时间。Result 包含成功、稳定 error code、最终 pose、最终 status 和失败原因。内部候选结构不
定义成 ROS message。

## 7. 可能的许可证和依赖风险

- 本地 `ndt_omp_ros2` 为 BSD-2-Clause、`relocalization_core` 为 Apache-2.0，新增代码必须保留现有许可证文件和版权信息。
- Scan Context 原始代码许可证未确认；Scan Context++ 为 CC BY-NC-SA 4.0；不复制、不 vendoring、不将其加入运行依赖。
- Autoware/MOLA 只作架构参考；MOLA Open Core/GPLv3 模块不默认接入，Autoware API/依赖不默认接入。
- ROS 2/Nav2/PCL/RViz/robot_localization 的实际包版本、许可证和传递依赖必须在构建/发布快照中核对，不能只依赖网络页面。
- 本轮不添加 `robot_localization`、RTK/UWB 驱动、因子图库、Scan Context 数据库或新的 GUI 运行时依赖。
- 任何接口生成依赖必须通过 `rosidl_generate_interfaces` 和 package manifest；不提交未生成文本来冒充实现。

## 8. 回滚方式

每个阶段独立提交，提交前保存最小构建和测试结果。回滚时只回退该阶段提交，不使用 destructive
Git 命令覆盖用户工作区。运行时 `publish_tf`、自动搜索、Nav2 autostart gate、safety localization
guard 和 debug output 都应有明确默认值，使旧 `/initialpose` baseline 可在关闭新能力时继续运行。

如果某阶段的接口已经进入下游，先保留兼容字段/旧字符串 topic，再回滚实现；不删除已发布接口或
改变 TF owner 而不更新 migration matrix。runtime 报告和 generated build/install/log 目录不作为回滚内容。

## 9. 明确暂不实现的内容

- 完整 Scan Context/Scan Context++ 数据库和 place recognition 实现。
- 完整 MOLA 集成、Autoware NDT 模块迁移、robot_localization EKF/UKF 配置。
- RTK、UWB、轮速、独立 IMU 驱动及因子图融合。
- 在线语义点云定位、动态地图更新和端到端深度定位。
- 自定义 PCL/VTK 三维 GUI 或对 vendor Qt GUI 的修改。
- 无界全地图搜索、无界扫描累计、候选级无界并发。
- 自动导航期间未经门禁的恢复运动、速度直发、底盘命令或安全使能。
- 把温室 GeoJSON、Fields2Cover 对象、地图 PGM 或语义状态直接链接到通用定位 core。

## 10. 完整任务清单和完成状态

| ID | 任务 | 状态 | 进入条件/备注 |
| --- | --- | --- | --- |
| R0 | 仓库、约束、Git 状态、指定模块审计 | 已完成 | 用户未提交修改已识别并保留 |
| R1 | 官方资料、论文、参考实现和许可证调研 | 已完成 | 结果见 `docs/research/automatic_relocalization_research.md` |
| R2 | 创建本任务规格并与 V2 边界对齐 | 已确认 | 用户已确认规格，开始分阶段实现 |
| A1 | LocalizationStatus msg | 已完成 | ROSIDL 生成、C++/Python 类型测试通过 |
| A2 | Relocalize action | 已完成 | ROSIDL 生成、C++/Python 类型测试通过 |
| A3 | `/initialpose` 与新内部请求统一 | 已完成 | `/initialpose` 与 Action 共用同一候选/配准/质量路径 |
| B1 | CandidateProvider interface | 已完成 | 普通 C++ 组件，不上 pluginlib |
| B2 | Manual/last/configured/external providers | 已完成基础版 | runtime 原子写入、PCD SHA-256 身份重算与候选/last-pose 绑定、processing record 摘要校验和外部 coarse frame/时间/协方差校验已覆盖；新地图生产记录写入 `pcd_sha256` 仍待完成 |
| B3 | 确定性 SE(2) 展开/裁剪 | 已完成基础版 | 数量、NaN/Inf、去重、优先级和稳定排序有测试 |
| C1 | 粗 NDT + top-K | 已完成基础版 | 复用 core，候选顺序执行；当前没有独立粗/细 top-K 后端 |
| C2 | 细 NDT/ICP + debug result | 已完成基础版 | 复用现有 backend，`ndt_num_threads=4` 保持基线，Action/debug 结果已接入 |
| C3 | 当前点云语义确认 | 未开始 | 明确单帧/累计/运动补偿及时间边界 |
| D1 | QualityValidator | 已完成基础版 | 收敛与接受分离，fitness/点数/创新量/可选几何字段有单测 |
| D2 | overlap/inlier/margin/innovation | 部分完成 | innovation、候选 ambiguity 和 YAML 边界已接入；真实 overlap/inlier/margin/运动一致性待完成 |
| E1 | LocalizationSupervisor | 已完成基础版 | 状态计数、确认、取消、超时和有界失败阈值有 C++ 单测；自动重试冷却待完成 |
| E2 | TRACKING/DEGRADED/LOST | 已完成基础版 | 低频只读验证与 `TRACKING -> DEGRADED -> RECOVERING -> LOST` 已接入，验证不抖动 `map -> odom` |
| F1 | Nav2 lifecycle gate | 基础 gate 已接入 | 只调用标准 manager service；定位 gate 会在 `TRACKING/accepted` 后启动 Nav2，导航启动自动重定位客户端默认关闭且只发送一次有界 Action；完整 pause/cancel 编排仍待完成 |
| F2 | agt_safety localization guard | 已完成基础版 | waypoint Action 与安全层消费结构化状态，不破坏急停/手动/watchdog |
| G1 | 温室 seeds 离线生成器 | 未开始 | 输出通用 YAML，不进入 core |
| H1 | 单元与合成点云测试 | 部分完成 | 接口、候选、质量、supervisor、map record 和 guard 单测已覆盖；合成点云 smoke 仍待完成 |
| H2 | launch smoke 与报告脚本 | 未开始 | 无底盘、无速度、runtime 报告 |
| I1 | RViz 点云/候选调试配置 | 未开始 | 标准 PointCloud2/MarkerArray/TF |
| D3 | 文档同步与迁移矩阵 | 已完成本阶段同步 | 接口、PCD record/hash、supervisor、Action/safety guard 变更已同步；后续阶段继续更新 |

## 11. 计划确认后的工作纪律

开始实现前应确认：

1. `LocalizationStatus`/`Relocalize` 字段是否接受本规格的最小范围。
2. `agt_localization` 继续承担基准模式 `map -> odom` 唯一发布责任。
3. Nav2 门禁采用 Action gate + safety guard，生命周期服务只做标准编排。
4. Scan Context、MOLA、Autoware、robot_localization 均保持参考/预留，不进入本轮运行依赖。
5. 阶段 A 完成并通过接口测试后，才进入阶段 B；任一阶段已有测试失败时停止叠加功能。

计划确认后，每个阶段汇报修改文件、接口/架构变化、实际构建命令、实际测试命令、失败及修复，
并更新本文件状态和迁移矩阵。未真实执行的命令不得标记为通过。
