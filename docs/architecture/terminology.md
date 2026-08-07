# 术语与命名约定

本文档统一当前阶段建图、定位、感知、地图处理和运行能力相关的中英文术语。
英文名称用于代码、ROS topic、Action、参数和接口标识；中文名称用于操作界面、报告和
面向操作者的说明。除非接口文档另有明确约定，新增文档应优先使用这里的中文译法。

## 使用范围

- 术语表统一概念名称，不替代 topic、消息、Action 或参数的接口定义。
- 代码标识保持稳定的英文命名，不因为中文译法变化而重命名已有接口。
- `map`、`odom`、`base_footprint` 等坐标系名称保持 ROS 标准英文写法。
- 算法尚未进入运行闭环时，仍可记录其术语，但必须在模块文档中标明实验性或离线性质。

## 建图与感知

| English | 推荐中文 | 核心职责 | 边界说明 |
| --- | --- | --- | --- |
| LIO Frontend | 连续里程计前端 | 高频输出连续局部位姿 | 负责实时状态估计，不等同于全局回环或地图优化 |
| Registered Cloud | 配准点云 | 已经变换到 `odom` 的点云 | 点云必须带有效时间戳和明确坐标系；不等同于最终持久化 PCD |
| Terrain Classification | 地形与障碍分类 | 判断地面、缓坡、障碍 | 上位概念，可综合高度、法向、时序和几何证据 |
| Ground Segmentation | 地面分割 | 对点云逐点进行 ground/non-ground 分类 | 是地形与障碍分类的一种基础处理，不等同于地面平面拟合 |
| Local Ground Plane | 局部地面平面 | 输出局部地面的数学模型 | 可用于高度残差、坡度和姿态约束；不能单独代表所有地面点 |

`Terrain Classification`、`Ground Segmentation` 和 `Local Ground Plane` 必须保持层次
区分：分类描述结果，分割描述逐点标签，平面描述拟合模型。

## 地图表示与更新

| English | 推荐中文 | 核心职责 | 边界说明 |
| --- | --- | --- | --- |
| Rolling Local Map | 滚动局部地图 | 只维护机器人周围一定区域 | 服务实时匹配或局部规划，不等同于全局持久化地图 |
| Log-Odds | 对数概率占据 | 累积障碍和自由空间观测 | 是占据更新的内部数值表示，不直接作为 ROS OccupancyGrid 值 |
| Raycast | 射线清空 | 根据激光传播路径清除自由空间 | 只清除从传感器原点到有效观测点之间经过的空间 |
| Occupancy Timeout | 占据超时衰减 | 清理长期未再次确认的动态障碍 | 只能处理有明确时序证据的占据，不得覆盖静态地图真值 |
| ESDF | 欧氏有符号距离场 | 查询距离障碍物的有符号距离 | 是距离查询数据结构，不等同于占据栅格或代价地图 |
| Traversability | 可通行性 | 描述能否通行、方向限制和通行代价 | 必须结合 canonical vehicle footprint、地形和障碍证据 |

当前静态导航地图的权威资产、Keepout mask 和运行时 costmap 仍遵循各自接口合同；这些
术语不能被用来暗示把动态证据写回基础 PGM，或把离线评估结果直接变成可执行输出。

## 回环与后端优化

| English | 推荐中文 | 核心职责 | 边界说明 |
| --- | --- | --- | --- |
| Keyframe | 关键帧 | 后端优化的离散节点 | 保存关键观测和位姿，不等同于每一帧实时点云 |
| Place Recognition | 地点识别 | 判断是否曾经到过相似位置 | 产生检索结果或候选，不直接确认回环 |
| Loop Candidate | 回环候选 | 表示可能对应历史位置的候选 | 必须经过几何验证后才能形成回环约束 |
| Geometric Verification | 几何验证 | 使用 ICP/GICP 等方法验证回环 | 输出匹配质量和相对变换，不等同于全局优化 |
| Pose Graph | 位姿图 | 表示节点以及相对或绝对约束 | 是优化问题的结构，不是优化算法本身 |
| PGO | 位姿图优化 | 对整个位姿图进行全局联合校正 | 优化后结果必须与原始观测、地图资产和版本记录绑定 |
| Ground Factor | 地面因子 | 限制 Z、roll 和 pitch 漂移 | 是后端约束的一类，不能替代完整的 LiDAR/IMU 观测 |
| RTK Factor | RTK 因子 | 提供绝对位置约束 | 只有在质量、时间和坐标系经过验证后才能进入优化 |

`Place Recognition` 负责“找可能的位置”，`Geometric Verification` 负责“验证是否匹配”，
`PGO` 负责“使用约束联合校正”，三者不能互换使用。

## 运行时能力

| English | 推荐中文 | 核心职责 | 边界说明 |
| --- | --- | --- | --- |
| Capability | 能力接口 | 面向 BT 和业务编排提供稳定任务接口 | 能力接口只暴露受控的 Action/Service/状态合同，不拥有前端状态或安全绕行权限 |

能力接口属于机器人后端合同。Qt、Web、CLI 和未来 Mission manager 都是客户端，不应在
前端重复实现系统模式、任务状态、地图身份、Bag 进程、安全状态或底盘命令逻辑。

## 相关合同

- 架构分层见 [`overview.md`](overview.md) 和 [`three_layer_system_architecture.md`](three_layer_system_architecture.md)。
- 核心 ROS topic 与消息边界见 [`../interfaces/core_topics.md`](../interfaces/core_topics.md)。
- 车辆几何和 canonical footprint 见 `profiles/platforms/<platform>.yaml` 及项目架构约束。
- 可通行性离线产品和实时化候选的运行限制见 [`../experiments/bunker_traversability_comparison_20260720.md`](../experiments/bunker_traversability_comparison_20260720.md)。
