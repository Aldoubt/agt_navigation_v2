# Paper I RQ / Contribution / Experimental Matrix Freeze

## Status

**PAPER CLAIM FREEZE v1 / EXPERIMENT EXECUTION READY**

本文件用于冻结 Paper I 的研究问题、贡献边界、实验矩阵、数据记录要求与停止规则。

从本文件开始：

- A 组地图消融实验可以正式开始；
- B 组建图来源证据实验可与 A 组并行；
- V3 runtime / RPP 属于后续 E3，仅在 A、B 主要证据冻结后启动；
- 不再向 Paper I 增加全局搜索、覆盖规划、Fields2Cover、Ackermann 全覆盖优化等研究内容。

---

# 1. Paper I 一句话问题定义

给定一份**全局一致的温室三维点云**，研究如何在植被遮挡、地面观测缺失和结构重复的条件下，利用地面相对几何证据与农业行列结构先验，**保守地恢复车辆真实可通行区域**，并形成可审计、可复现、可供后续机器人导航直接使用的结构化语义地图。

本文**不提出新的 SLAM 前端或后端**，也**不研究全局覆盖路径规划算法**。

---

# 2. Paper I 核心研究边界

## 2.1 输入

Paper I 的方法输入是已经完成全局配准的三维点云，例如：

- FAST-LIVO2 LIO-only 输出的全局点云；
- Kilo-Map 输出的全局点云；
- 手持三维扫描仪 + FAST-LIO 输出的全局点云。

所有输入进入同一套 Paper I 离线地图处理 pipeline。

## 2.2 核心输出

Paper I 输出至少包括：

- ground-relative terrain evidence；
- obstacle / unknown evidence；
- row structural representation；
- aisle structural representation；
- Site Boundary；
- conservative traversability classes；
- accepted navigation map；
- semantic / structural map assets；
- vehicle-feasible structural evidence；
- 完整 asset lineage / manifest / hash。

## 2.3 Traversability 语义

正式语义保持：

- `OBSERVED_FREE`
- `INFERRED_TRAVERSABLE`
- `HARD_BLOCKED`
- `SENSOR_OBSTACLE`
- `UNKNOWN`

核心安全原则：

> **UNKNOWN 不允许被全局、无条件转换为 FREE。**

结构先验只能在有明确空间边界、地形约束、农业结构证据和恢复距离约束时，将局部缺测区域提升为 `INFERRED_TRAVERSABLE`。

---

# 3. Research Questions

## RQ1 — Structure-aware traversability recovery

**在植被遮挡与地面观测不完整的温室环境中，农业行列结构先验能否在控制 False-Free 风险的同时，减少普通二维栅格化造成的 False-Blocked 和 UNKNOWN 区域？**

RQ1 是 Paper I 的主研究问题。

### RQ1 对应证据

- A 组消融实验；
- 人工参考标注；
- traversable precision / recall / IoU；
- False-Free Rate；
- False-Blocked Rate；
- UNKNOWN Ratio；
- 连通性与可达行道指标；
- 典型植被遮挡失败案例。

---

## RQ2 — Mapping-source robustness

**当全局地图由不同但合理的三维建图来源生成时，本文的农业结构恢复和可通行语义输出是否保持稳定？**

RQ2 不研究“哪一种 LIO 算法绝对最好”，而研究：

> Paper I 方法是否依赖某一份恰好非常干净的点云。

### RQ2 对应证据

- B 组 `lio_benchmark_tools` 建图来源证据；
- 相同输入数据条件下可比较的 LIO 轨迹/地图指标；
- 各冻结 map source 通过同一 Paper I pipeline 后的结构一致性指标。

---

## RQ3 — Downstream executability

**由本文结构化地图产生并冻结的路线资产，是否能够被独立 runtime 系统直接执行，并在真实温室中保持稳定路径跟踪？**

RQ3 只用于验证“地图是可执行资产”，不是控制器贡献。

### RQ3 对应证据

后续由 `agt_navigation_runtime` V3 完成：

```text
READY Site / Map Asset
  -> READY Route Asset
  -> Route Runtime
  -> Nav2 FollowPath
  -> fixed RPP configuration
  -> real robot execution
```

A、B 主要证据冻结前，不启动 E3 正式实验。

---

# 4. Working Hypotheses

以下假设用于指导实验设计，不允许在数据产生前写成论文结论。

## H1

相对于常规 occupancy / height projection，ground-relative 表达可以减少地面高度变化对障碍判定的影响，但仍会受到植被遮挡造成的 ground missing / UNKNOWN 问题。

## H2

引入 row / aisle 结构先验和 bounded occlusion recovery 后，可以显著降低 False-Blocked 和 UNKNOWN，同时不会显著恶化 False-Free Rate。

## H3

对于能够生成全局一致温室点云的不同建图来源，row orientation、row spacing、aisle topology 和 traversable structure 应保持较高一致性；差异主要出现在局部点密度、边缘和遮挡区域。

## H4

固定语义地图和固定 Route Asset 在独立 V3 runtime 中应具有可执行性；如果失败，失败应可归因到 localization / controller / safety / route feasibility，而不是由地图资产身份不一致导致。

---

# 5. Contribution Freeze

Paper I 暂定只保留 **3 个方法贡献**。

最终投稿文字可根据相关工作综述调整表述强度，但不允许再扩展贡献数量。

## C1 — Ground-relative conservative traversability representation

提出面向温室地形的 ground-relative 表达方式，将绝对高度栅格转化为局部地面参考下的：

- ground height；
- ground validity / confidence；
- slope / robust slope；
- step；
- obstacle evidence；
- unknown evidence。

目标不是简单增加 FREE 区域，而是建立**可解释的地形与障碍证据层**。

## C2 — Agricultural structure-aware bounded traversability recovery

利用温室具有重复 crop-row / aisle 布局这一农业结构先验，通过：

- row support；
- row structural band；
- row centerline；
- aisle candidate；
- aisle geometric envelope；
- aisle centerline / graph；
- Site Boundary；

约束局部 vegetation-induced occlusion 下的可通行空间恢复。

核心区别：

> 恢复是 **bounded inference**，不是 morphology-based global free-space completion。

## C3 — Auditable agricultural semantic-map asset representation

将原始观测、推断可通行区域、硬边界、传感器障碍和未知区域显式区分，并通过不可变 map-resource bundle、accepted map、manifest 和 SHA256 lineage 形成可复现的地图资产。

这一贡献服务于真实机器人导航中的：

- 可解释性；
- 可审计性；
- map / semantic consistency；
- downstream reproducibility。

## 不作为 Contribution 的内容

以下内容只作为实验支撑或系统验证：

- FAST-LIVO2 / Kilo-Map / FAST-LIO 本身；
- `lio_benchmark_tools`；
- Nav2；
- RPP；
- BUNKER / Ackermann runtime；
- BT / Mission；
- CSV / Route Asset 格式。

---

# 6. Experimental Matrix Overview

| Experiment | Research question | Role | Repository | Start status |
|---|---|---|---|---|
| E1 Representation Ablation | RQ1 | **Primary experiment** | `agt_navigation_v2` | **START NOW** |
| E2 Mapping-source Robustness | RQ2 | Secondary robustness experiment | `lio_benchmark_tools` + `agt_navigation_v2` | **START IN PARALLEL** |
| E3 Runtime Route Validation | RQ3 | Downstream real-robot validation | `agt_navigation_runtime` | DEFER until E1/E2 freeze |

---

# 7. E1 — Representation Ablation

## 7.1 Goal

回答 RQ1：结构先验是否真正改善温室可通行地图，而不是仅产生更平滑或视觉效果更好的地图。

## 7.2 Canonical input

首轮正式实验固定使用：

```text
greenhouse_01_map_assets_v01
```

必须记录：

- `map_resource_manifest.yaml` SHA256；
- `pointcloud/processed.pcd` SHA256；
- frame id；
- resolution；
- origin；
- width / height；
- source git commit；
- vehicle footprint / padding（若用于可达性指标）。

`v01` 不允许覆盖修改。任何地图算法参数变化导致正式输出变化时，必须形成新的 experiment run，不得修改已有结果。

---

## 7.3 Ablation variants

### A0 — Conventional projection baseline

目的：提供最简单、可解释的传统基线。

允许使用：

- fixed Z / absolute-height projection；或
- 当前仓库已经存在并可确定性复现的 conventional raw occupancy projection。

不允许使用：

- ground-relative correction；
- row structure；
- aisle inference；
- Site Boundary assisted recovery。

A0 必须固定配置和代码 commit。

### A1 — Ground-relative terrain only

加入：

- ground height；
- ground valid / confidence；
- slope / step；
- ground-relative obstacle classification。

不加入 row / aisle structural inference。

### A2 — Ground-relative + Row structure

A1 +：

- row support；
- row regularization；
- row structural band；
- row centerline。

仍不允许使用 aisle geometric envelope 完成 occlusion recovery。

### A3 — Ground-relative + Row + Aisle structure

A2 +：

- aisle candidate；
- aisle geometric envelope；
- aisle centerline / graph。

该阶段可输出结构存在性，但**不执行最终 bounded UNKNOWN recovery**。

### A4 — Full Paper I method

A3 +：

- Site Boundary hard gate；
- bounded occlusion recovery；
- final conservative traversability classes；
- accepted-map policy / auditable override policy（若正式输出需要）。

A4 即 Paper I 完整方法。

---

# 8. E1 Reference / Ground Truth Protocol

## 8.1 Reference 的目标

Reference 不是重做一张“理想地图”，而是回答：

> 对给定车辆 footprint，在当前温室静态几何条件下，这个栅格位置是否可被合理视为车辆可通行区域？

## 8.2 Reference classes

人工参考至少分：

- `REF_TRAVERSABLE`
- `REF_BLOCKED`
- `REF_UNCERTAIN`
- `REF_OUTSIDE_SITE`

`REF_UNCERTAIN` 不参与主要 precision / recall 分母，单独统计比例。

## 8.3 Reference evidence hierarchy

优先级从高到低：

1. 现场实测 / 车辆实际通过证据；
2. 高质量手持扫描地图 + 人工 3D 检查；
3. 多视角点云与地面连续性；
4. 温室固定设施和明确 crop-row geometry；
5. 单一栅格或单一局部图像。

禁止仅根据 A4 输出反向标注 Ground Truth。

## 8.4 Reference freeze

Reference 一旦用于正式 E1：

- 保存独立 manifest；
- 记录 annotator / date / source assets；
- hash freeze；
- 不得在看到 A0-A4 quantitative result 后修改标签以改善结果。

如发现参考标注错误，必须创建 `reference_v02` 并重新运行全部 A0-A4。

---

# 9. E1 Primary Metrics

主要指标全部基于统一 grid / reference mask。

## 9.1 Traversable Precision

```text
TP / (TP + FP)
```

其中 FP 表示算法判为可通行但 reference 为 blocked。

## 9.2 Traversable Recall

```text
TP / (TP + FN)
```

FN 表示 reference 可通行但算法没有恢复为可通行。

## 9.3 Traversable IoU

```text
TP / (TP + FP + FN)
```

## 9.4 False-Free Rate — Safety-critical primary metric

必须单独报告。

定义建议：

```text
False-Free Rate = FP / (# reference blocked cells)
```

可同时报告 FP cell count / area。

论文讨论必须首先确认：

> A4 的 recall / connectivity 改善是否以不可接受的 False-Free 增加为代价。

## 9.5 False-Blocked Rate

```text
FN / (# reference traversable cells)
```

这是 vegetation occlusion recovery 的直接效果指标。

## 9.6 UNKNOWN Ratio

统计 Site Boundary 内仍处于 UNKNOWN 的面积比例。

UNKNOWN 降低本身不是成功；必须与 False-Free 同时解释。

---

# 10. E1 Structural / Navigation-relevance Metrics

除像素级指标外，增加结构性指标。

至少包括：

- largest connected traversable component area；
- connected traversable ratio；
- reachable aisle count；
- disconnected aisle count；
- aisle centerline coverage ratio；
- minimum / distribution of corridor clearance where available。

这些指标用于回答：

> 恢复后的地图是否改善农业机器人真正关心的“行道连通性”，而不只是像素分类数字。

---

# 11. E1 First-round Data Layout

建议正式实验目录：

```text
paper1/experiments/e1_ablation/
├── experiment_manifest.yaml
├── reference/
│   ├── reference_v01.*
│   └── reference_manifest.yaml
├── A0_conventional/
│   ├── config.yaml
│   ├── manifest.yaml
│   ├── map.*
│   └── metrics.yaml
├── A1_ground_relative/
├── A2_row_structure/
├── A3_aisle_structure/
├── A4_full_method/
├── comparison/
│   ├── metrics.csv
│   ├── metrics.md
│   └── failure_cases.yaml
└── figures/
```

大体积 PCD / NPY 是否放 Git 由 `.gitignore` 控制，但每个外部 asset 必须记录 SHA256 和路径/Artifact ID。

---

# 12. E1 Required Figures

第一轮实验至少预留以下图位：

### Fig. A — Overall pipeline

```text
PCD -> Ground -> Row -> Aisle -> Bounded Recovery -> Semantic Traversability
```

### Fig. B — Same-region ablation crop

同一局部区域并排：

```text
A0 | A1 | A2 | A3 | A4 | Reference
```

优先选 vegetation occlusion 明显的行道。

### Fig. C — Traversability error visualization

显示：

- TP；
- FP / false-free；
- FN / false-blocked；
- UNKNOWN。

### Fig. D — Connectivity / aisle topology

展示 A0 与 A4 对 aisle connectedness 的差异。

### Fig. E — Failure cases

必须主动保存 A4 仍失败的区域，不允许只展示成功样例。

---

# 13. E1 Acceptance Gate

E1 第一轮完成必须满足：

- [ ] `greenhouse_01_map_assets_v01` 身份冻结；
- [ ] reference_v01 冻结；
- [ ] A0-A4 使用同一 grid；
- [ ] A0-A4 每个 variant 有独立 config + manifest + hash；
- [ ] primary metrics 全部可自动生成；
- [ ] False-Free / False-Blocked / UNKNOWN 都被报告；
- [ ] 至少一个真实 vegetation-occlusion region 有可视化；
- [ ] 至少一个失败案例进入报告；
- [ ] comparison.csv 可由脚本重新生成；
- [ ] 不手工修改任何算法输出以改善数字。

E1 达到该 gate 后即可开始撰写 Results 4.3 的第一版。

---

# 14. E2 — Mapping-source Robustness Matrix

E2 与 E1 并行，但分为 **B1 和 B2 两层**。

## B1 — Same-input LIO benchmark

只比较共享同一数据集、传感器输入、时间范围和 reference 定义的方法。

例如如果 FAST-LIVO2 LIO-only 与 Kilo-Map 确实使用同一 ROS bag，则可以比较：

- APE；
- RPE；
- final drift / loop consistency；
- runtime；
- CPU / memory（若已有稳定采样）；
- map physical bounds / completeness。

### Important

手持扫描仪若不是同一条机器人轨迹、同一数据采集过程，则**不与机器人 LIO 强行比较 APE/RPE 排名**。

## B2 — Cross-map-source downstream robustness

将冻结的：

- FAST-LIVO2 LIO-only map；
- Kilo-Map map；
- handheld FAST-LIO map；

分别经过**同一 Paper I semantic-map pipeline、同一参数政策和同一 reference region**。

比较：

- row dominant direction deviation；
- row spacing deviation；
- row topology agreement；
- aisle centerline deviation；
- aisle topology agreement；
- traversable-area agreement；
- structured map connectedness；
- pipeline failure / missing-structure cases。

B2 才是 RQ2 的主要论文证据。

---

# 15. E2 Interpretation Rule

Paper I 不需要证明三张地图像素完全一致。

期望论证是：

```text
Map source changes
    -> local density / edge / occlusion differences
    -> Paper I structure extraction may vary locally
    -> dominant row / aisle topology and traversable structure remain stable
```

如果结构输出显著不一致，也必须如实报告，并将其作为方法对 map-source quality 的适用边界。

---

# 16. E3 — Deferred Runtime Validation

E3 本文件只冻结目标，不启动正式数据采集。

启动条件：

- E1 主要结果已冻结；
- E2 evidence snapshot 已冻结；
- semantic map frozen；
- accepted navigation map frozen；
- fixed Route Asset frozen。

随后由 `agt_navigation_runtime` V3：

```text
READY deployment artifacts
  -> RouteRuntime
  -> Nav2 FollowPath
  -> fixed RPP profile
  -> Safety
  -> Chassis
```

E3 不做 controller comparison。

---

# 17. Paper Results Mapping

未来论文 Results 建议直接映射：

## 4.3 Representation Ablation

回答 RQ1，对应 E1。

## 4.4 Robustness Across Mapping Sources

回答 RQ2，对应 E2 B1 + B2，其中 B2 为主。

## 4.5 Real-Robot Route Execution

回答 RQ3，对应 E3。

---

# 18. Paper I Stop Rules

从本文件冻结后，以下内容不得进入 Paper I 主贡献分支：

- planner matrix；
- A* / Theta* / Hybrid-A* / RRT* 对比；
- maximum coverage；
- Fields2Cover；
- headland / turn optimizer；
- Ackermann coverage ordering；
- VLA / LLM；
- GNSS fusion；
- 新 LIO 算法开发。

这些内容进入后续独立工作。

---

# 19. Immediate Execution Order

现在直接执行：

```text
A0. Freeze site + reference protocol
A1. Produce reference_v01
A2. Freeze A0-A4 configs
A3. Run E1 A0-A4
A4. Generate comparison metrics / figures

同时：

B0. Freeze lio_benchmark local development state
B1. Identify valid FAST-LIVO2 / Kilo-Map / handheld runs
B2. Freeze map/config/data identities
B3. Produce B1 same-input benchmark summary
B4. Export three map sources for common Paper I pipeline
B5. Produce B2 structural consistency results
```

V3 暂停，直到 A、B 两条线达到本文件定义的 freeze gate。
