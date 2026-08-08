# Reproducible Map Derivation and Alignment Contract

本合同定义从一个受管 Dataset/Bag 派生正式地图资产的最小可复现流程。它不规定必须使用某一种 SLAM/优化算法，但规定所有地图产品必须共享同一 canonical map frame、来源 lineage 和质量证据。

## 1. Canonical lineage

```text
Dataset / Bag
    + Calibration Set
    + Platform Profile
    + Derivation Recipe
            |
            v
Raw trajectory / raw cloud
            |
            v
Offline trajectory optimization (optional but recorded)
            |
            v
Map Alignment -> canonical site map frame
            |
            +--> localization prior PCD
            +--> global navigation OccupancyGrid
            +--> semantic base geometry
            +--> previews / reports
```

同一 `map_version_id` 内的 PCD、PGM/YAML、Semantic Map 和 Route 必须从同一个 canonical transform chain 派生。禁止分别手工平移/旋转这些资产后仍声明它们属于同一 READY version。

## 2. Derivation recipe

每次派生必须保存 `derivation/recipe.yaml`，最低字段：

```yaml
schema_version: 1
recipe_id: map_recipe_fastlivo2_ground_v1
source_dataset_id: ds_greenhouse_a_20260810_am
source_dataset_sha256: sha256:<64 lowercase hex>
calibration_id: cal_bunker_mid360_20260810
calibration_sha256: sha256:<64 lowercase hex>
platform_profile: bunker
platform_profile_sha256: sha256:<64 lowercase hex>
repository_commit: <git sha>
random_seed: 0
mapping:
  backend: fast_livo2
  config_sha256: sha256:<64 lowercase hex>
trajectory_optimization:
  backend: none
alignment:
  mode: SITE_CONTROL_POINTS
cleaning:
  pipeline:
    - voxel_downsample
    - crop_workspace
    - statistical_outlier_removal
    - ground_classification
products:
  localization_prior: true
  navigation_occupancy: true
  semantic_base: true
```

所有影响输出的阈值必须在 recipe 或其 hash 绑定的配置文件中。正式 recipe 不得依赖未记录的用户 shell history。

## 3. Site frame 与多时期地图

同一实际场景应有稳定 `site_id` 和 site frame 定义：

```text
site_id: greenhouse_a
frame_id: map
```

不同日期/生长时期：

```text
greenhouse_a / epoch_2026_08
greenhouse_a / epoch_2026_09
greenhouse_a / epoch_2026_10
```

分别产生独立 `map_version_id`，但都必须对齐到同一 site frame。这样 Semantic Map、控制点、长期结构层和跨时期定位实验才有可比较意义。

### EVALUATION site

有可靠 RTK/GNSS 时允许使用固定 ENU 基准建立 site frame。必须记录 origin、datum/投影说明以及从 estimator frame 到 evaluation frame 的 transform。RTK truth 本身不自动成为 operational map。

### OPERATIONAL facility site

无 GNSS 场景必须使用稳定 site reference，例如固定立柱、墙角、温室结构或人工测量控制点。site frame 定义应保存在 `alignment/site_frame.yaml`，不能由每次 FAST-LIVO2 启动位置隐式决定。

## 4. Alignment artifact

每个正式 map version 必须保存：

```text
alignment/
  site_frame.yaml
  alignment.yaml
  alignment_report.json
```

`alignment.yaml` 最低字段：

```yaml
schema_version: 1
site_id: greenhouse_a
epoch_id: 2026-08-10-am
map_frame: map
source_frame: mapping_session
method: SITE_CONTROL_POINTS
reference_map_binding: null
transform:
  translation: [0.0, 0.0, 0.0]
  quaternion_xyzw: [0.0, 0.0, 0.0, 1.0]
control_points:
  - id: post_a
    source_xyz: [1.0, 2.0, 0.0]
    reference_xyz: [4.0, 6.0, 0.0]
```

同一场景不同时期可把一个已接受的 map version 作为 `reference_map_binding`，但配准应优先使用稳定结构/控制点，不应让季节性叶片和软枝主导全云 ICP。

## 5. Map cleaning artifact

任何会改变正式 PCD/栅格的清理步骤都必须可回放。建议：

```text
pointcloud/
  raw_map.pcd
  cleaned_map.pcd
  localization_map.pcd
  localization_map.processing.yaml
processing/
  cleaning.yaml
  cleaning_report.json
```

清理可以包含 voxel、crop、SOR、地面分类、稳定结构筛选等。人工删除区域必须保存为 polygon/box patch 文件或可重放操作列表。源 `raw_map.pcd` 不被原地覆盖。

## 6. 地面分割与地图产品

地面/障碍分类的结果可以同时派生：

- `localization_map.pcd`：用于全局初始化/重定位的先验
- `navigation/map.pgm + map.yaml`：用于全局二维几何与离线路线规划
- `reports/ground_report.json`：记录地面模型、残差、点数比例和配置 hash

Global Navigation Map 与 Localization Prior 必须保持不同产品语义，即使它们来自同一 cleaned cloud。

## 7. Map Quality Gate

READY 不是“文件存在”的同义词。派生流程至少应产生 `reports/map_quality_report.json`，包含：

```yaml
status: PASS
checks:
  asset_hashes: PASS
  shared_frame_identity: PASS
  alignment: PASS
  occupancy_extent: PASS
  localization_prior: PASS
  semantic_binding: PASS
metrics:
  control_point_rmse_m: 0.0
  control_point_max_error_m: 0.0
  localization_query_success_rate: null
```

阈值由 recipe/policy 配置，不在本文写死统一数值。任何 ERROR 级检查失败都不能进入 READY。

## 8. 输出目录

推荐扩展现有 Map Registry 布局：

```text
runtime/maps/<map_id>/versions/<map_version_id>/
  manifest.yaml
  source/
    dataset_binding.yaml
  derivation/
    recipe.yaml
  alignment/
    site_frame.yaml
    alignment.yaml
    alignment_report.json
  pointcloud/
    raw_map.pcd
    cleaned_map.pcd
    localization_map.pcd
    localization_map.processing.yaml
  navigation/
    map.pgm
    map.yaml
  semantic/
    semantic_map.geojson
    coverage.yaml
    validation_report.json
  routes/
  preview/
  reports/
    map_quality_report.json
```

`manifest.yaml` 仍是 portable truth；SQLite 仍只是可重建索引。

## 9. 复现定义

“可复现到另一个场景”意味着相同 pipeline/recipe schema 可运行，但必须换新的 `site_id`、Dataset binding 和 alignment/site-frame 参数。

“可复现到同一场景不同时间”意味着保留相同 `site_id` 和 site-frame definition，使用新的 `epoch_id`/dataset/map version，并输出跨时期 alignment report；不得直接覆盖旧 READY map。
