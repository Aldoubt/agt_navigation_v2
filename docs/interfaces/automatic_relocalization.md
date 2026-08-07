# Automatic Relocalization Interface

当前阶段：结构化接口、候选加载/展开、外部粗位姿校验、顺序配准编排、基础质量门禁和基础
supervisor 已实现。低频 TRACKING 验证不改写 `map -> odom`，并使用点云时间的 odom 传播预测
作为当前配准初值；PCD processing record 的 `state: ready`/`map_file` 门禁和 waypoint/safety
基础定位门禁已接入，Nav2 lifecycle、PCD 内容 hash 的读取、候选身份绑定和 processing-record
校验已接入。旧记录的 hash 写回和长期运动质量评估仍按后续阶段实现。

运维控制器的 `/agt/localization/set_mode` 只选择有界 `MANUAL_ONLY`、`AUTO_ON_START` 或
`AUTO_RECOVERY` policy；它复用下方唯一 `/agt/localization/relocalize` Action，不创建第二套
候选或 TF 链。失败不会无限重试，详情见 [`localization_modes.md`](localization_modes.md)。

导航可通过 `agt_bringup` 显式启用一次性自动 Action 客户端。该客户端只等待 Action Server、发送
一个有界 `MODE_AUTO_SEARCH` 请求并报告结果，不发布速度、不使能安全层、不进行无限重试。默认
关闭时，启动导航不会自动发起重定位，仍由 `/initialpose` 或显式 Action 请求触发。

## Topics

| 接口 | 类型 | 方向 | 语义 |
| --- | --- | --- | --- |
| `/agt/localization/status` | `agt_interfaces/msg/LocalizationStatus` | localization -> consumers | 机器可解析状态。`pose_valid` 只有在质量门禁通过后为 true；`has_converged` 不等于可信定位。`status_stale` 由消费者按时间策略判断，定位节点在正常发布时为 false。QoS 初版为普通可靠 topic，后续 gate 以 header stamp 和 timeout 判定新鲜度。 |
| `/agt/localization/global_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | localization -> fusion | 通过质量验证的 `map` frame 全局位姿测量。基准模式可选发布；未来融合模式必须关闭基准节点 TF 输出，再由唯一 fusion owner 消费该测量并维护 authoritative `map -> odom`。 |
| `/agt/localization/coarse_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | external provider -> localization | 外部粗位姿输入。只用于 frame、时间和协方差校验及候选搜索范围，不直接写 TF。 |
| `/agt/localization/candidate_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | localization -> RViz/debug | 当前正在测试的候选位姿。只用于可视化，不是外部输入，不直接写 TF。 |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | RViz/Qt -> localization | 兼容入口，必须转换到同一内部重定位请求，不维护第二套配准逻辑。 |

## Action

`/agt/localization/relocalize` 使用 `agt_interfaces/action/Relocalize`。

Goal 支持：

- `MODE_AUTO_SEARCH`：按配置组合候选来源；
- `MODE_SINGLE_INITIAL_POSE`：只使用明确的 `initial_pose`；
- `MODE_LOCAL_CANDIDATES`：使用 last/configured/external 候选的有界集合；
- `MODE_EXTERNAL_COARSE_POSE`：只接受有效外部粗位姿派生的候选；
- last pose/configured/external 开关、最大候选数、debug 开关和超时时间。

Feedback 只报告状态、候选总数/已测试数、当前最好 fitness/来源和 elapsed time。Result 报告
成功标志、稳定错误码、最终 pose、完整最终 `LocalizationStatus` 和失败原因。内部候选的来源
优先级、搜索半径、展开索引和配准中间点云不定义为 ROS message。

## State and error contract

状态枚举为：

```text
UNINITIALIZED -> SEARCHING -> VERIFYING -> TRACKING
                              |             |
                              v             v
                           ERROR/LOST <- DEGRADED -> RECOVERING
```

基础搜索路径已实现 `SEARCHING -> VERIFYING -> TRACKING/LOST`；`LocalizationSupervisor`
现已实现并测试 `TRACKING -> DEGRADED -> RECOVERING -> LOST` 的有界迁移、取消和超时。
低频验证成功时只恢复结构化状态，不重写 `map -> odom`。稳定错误码包括 map not ready、scan
too small、backend/fitness failure、invalid request/guess、timeout/canceled、ambiguous result、
stale status、TF unavailable、map hash mismatch、no candidates、stale scan 和 invalid scan
timestamp。

点云只保留最新一帧。`max_cloud_age_s`、`max_cloud_future_tolerance_s` 和
`require_nonzero_cloud_stamp` 在 ROS 参数入口校验；点云过期、来自未来或时间戳无效时，节点
拒绝本次配准并保留具体 stamp/age 原因。动态 `odom -> tracking_frame` 只在点云时间查询，
不会用最新 TF 静默替代。

`has_converged=true` 只代表后端数值迭代收敛；`localization_accepted=true` 和 `pose_valid=true`
还需要 fitness、inlier、overlap、初值修正量、候选分数差、地图范围、短期运动一致性和连续验证
通过。重复平行行的同分候选必须保持 `ambiguous_result=true` 并 fail-closed。

## PCD identity

定位节点每次检查 ready processing record 时都重新计算 PCD 文件的
`sha256:<64位小写十六进制>` 内容身份，并将它用于 `LocalizationStatus.map_hash`、候选和
last-pose 的地图绑定。新 processing record 应增加 `pcd_sha256`；如果存在该字段，它必须与
实际 PCD 完全一致。缺少该字段的历史记录不会被静默赋予记录级验证，启动诊断会保留这一事实。

## TF ownership

localization subsystem 是 authoritative `map -> odom` 的唯一 authority，但“authority”与
“具体 node 名”需要区分：任一运行时 profile 中只能有一个 selected TF publisher。

当前基准模式中，`agt_localization` package 内的 `agt_relocalization` node 使用
`publish_tf=true`，因此它是唯一 `map -> odom` runtime publisher；FAST-LIVO2 adapter 只发布
`odom -> base_footprint`；`agt_description`/robot_state_publisher 负责机器人描述和传感器静态
关系。RViz、Action 客户端和 debug publisher 不发布 TF。

未来融合模式如果由 `agt_localization_fusion` 维护连续全局状态，则必须令基准
`agt_relocalization` 路径 `publish_tf=false`，再由 fusion node 成为唯一 selected TF publisher。
NDT/ICP backend、GTSAM/iSAM2、GNSS factor、loop/place-recognition 等只能向该 localization
subsystem 提供测量、factor、candidate transform 或 correction evidence，不能再增加第二个
`map -> odom` publisher。本阶段绝不同时启动两个 owner。

## Generation and compatibility

接口必须由 `rosidl_generate_interfaces` 生成，安装目录中的 `.msg`/`.action` 文本不是实现。字段
变化必须同时更新 C++/Python serialization tests、该文档、`docs/interfaces/core_topics.md`、
`AGENTS.md` 和 `docs/roadmap/v2_5.md`。旧字符串 status topic 继续作为人类调试
兼容接口，但系统逻辑不得解析字符串。
