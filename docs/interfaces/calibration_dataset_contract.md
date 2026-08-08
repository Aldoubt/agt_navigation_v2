# Calibration and Dataset Provenance Contract

本合同定义离线建图、定位精度评估和路线派生所使用的 Calibration Set 与 Dataset/Bag lineage。目标是保证“同一 bag 可重复派生”“同一场景不同时间可比较”“不同场景可复用同一处理流程”，而不是把一次手工处理结果当作不可追溯的地图真值。

## 1. 单一真源

### Capture rig 与执行车辆

`capture_rig` 是产生 bag 的传感器安装关系，不等于之后执行路线的车辆。
为保持 schema-v1 兼容，旧 `platform` 字段在离线 Dataset 中解释为 capture rig；
新 binding 应显式记录：

```yaml
capture_rig:
  profile_id: handheld_mid360_rig
  profile_sha256: sha256:<64 lowercase hex>
execution_vehicle:              # 地图生产数据可省略
  profile_id: mk_mini
  profile_sha256: sha256:<64 lowercase hex>
```

LiDAR↔IMU 等传感器 intrinsic/extrinsic、sensor-rig→vehicle-base 外参和 GNSS
杆臂是三个不同层级。后两者必须按车型独立维护；GNSS 杆臂只属于 GNSS
evaluation 车辆，不得进入无 GNSS 的手持温室 bag lineage。Route Asset 独立绑定
最终执行车辆。

- `profiles/platforms/<platform>.yaml` 继续是车辆物理尺寸、navigation footprint、运动学限制和平台策略的唯一真源
- Calibration Set 只保存传感器内外参、时间同步/杆臂和与测量模型相关的标定结果，不复制 footprint、wheelbase、minimum turning radius 等车辆几何真值
- Dataset/Bag 是不可变输入证据；派生处理不得修改原 bag
- 同一个正式派生任务必须显式绑定 `dataset_sha256`、`calibration_sha256`、`platform_profile_sha256` 和 `recipe_sha256`

## 2. Calibration Set

建议运行目录：

```text
runtime/calibrations/<calibration_id>/
  manifest.yaml
  lidar_imu.yaml
  lidar_base.yaml
  gnss_base.yaml          # 可选
  camera_base.yaml        # 可选
  timing.yaml             # 可选
  reports/
```

`manifest.yaml` 最低字段：

```yaml
schema_version: 1
calibration_id: cal_bunker_mid360_20260810
platform_id: bunker
created_at: 2026-08-10T08:00:00Z
frame_convention: ros_rep_103
assets:
  lidar_imu:
    path: lidar_imu.yaml
    sha256: sha256:<64 lowercase hex>
  lidar_base:
    path: lidar_base.yaml
    sha256: sha256:<64 lowercase hex>
  gnss_base:
    path: gnss_base.yaml
    sha256: sha256:<64 lowercase hex>
content_sha256: sha256:<64 lowercase hex>
```

所有刚体外参必须显式写 `parent_frame`、`child_frame`、平移单位 m、旋转表示法以及来源。禁止只保存一个无方向说明的 4x4 数组。重新标定后必须生成新的 `calibration_id`，不能覆盖已被 Experiment/Map 引用的 calibration。

正式 Map derivation 会把所绑定的 Calibration Set artifact 复制到 map version 的 `source/calibration.yaml` 并再次校验 hash，从而使派生 bundle 不依赖某个开发机上的隐式外参文件。

## 3. Dataset / Bag binding

一个用于正式地图派生或精度实验的 bag 必须有不可变 binding：

```yaml
schema_version: 1
dataset_id: ds_greenhouse_a_20260810_am
site_id: greenhouse_a
epoch_id: 2026-08-10-am
purpose: OPERATIONAL
bag:
  path: <managed-bag-relative-path>
  sha256: sha256:<64 lowercase hex>
  storage_id: sqlite3
platform:
  profile_id: bunker
  profile_sha256: sha256:<64 lowercase hex>
calibration:
  calibration_id: cal_bunker_mid360_20260810
  calibration_sha256: sha256:<64 lowercase hex>
topics:
  lidar: /agt/sensors/lidar/custom
  imu: /agt/sensors/imu/data
  wheel: /agt/chassis/odometry
  gnss: /agt/sensors/gnss/fix
```

`purpose` 至少区分：

- `EVALUATION`：用于 RTK/GNSS 真值对比、算法 A/B 和指标报告
- `OPERATIONAL`：用于设施场所地图、语义、路线和后续任务执行资产

同一 site 不同时期使用相同 `site_id`、不同 `epoch_id` 和不同 dataset/map version。不同 site 复用同一 recipe 时，不能复用对方的 site frame 或 control-point 数值。

### 3.1 `bag.sha256` 的正式算法

`bag.sha256` 不是只对 `metadata.yaml` 做 hash。V25-09A 使用 deterministic bundle hash：

- 若 `bag.path` 是单个文件，则直接对该文件 bytes 做 SHA256
- 若 `bag.path` 是 rosbag2 目录，则必须包含 `metadata.yaml`
- 递归枚举目录内所有 regular files
- 使用相对 bag root 的 POSIX 路径排序
- 每个文件记录依次加入 `relative_path + NUL + file_size + NUL + file_content_sha256 + newline`
- 对整个记录流再做 SHA256，输出 `sha256:<64 lowercase hex>`
- 正式 Dataset bundle 禁止 symlink，避免 hash 依赖目录外不可控文件

因此任一 `.db3`、`.mcap`、`metadata.yaml` 或同 bundle 中正式文件变化都会改变 `bag.sha256`。

工具命令：

```bash
ros2 run agt_offline_assets agt_offline_assets.py hash-path /path/to/rosbag2_directory
```

`init-map` 会重新计算 bundle hash；bag 不存在、metadata 缺失或 hash 不匹配时 fail-closed，不建立正式 PROCESSING map workspace。

## 4. RTK 评价隔离

当 GNSS/RTK 被声明为 evaluation truth 时，该数据不得同时进入被评价 estimator 的状态更新。典型 A/B：

```text
A: LiDAR + IMU -> FAST-LIVO2
B: LiDAR + IMU + wheel -> wheel-aided odometry
Truth: RTK/GNSS -> evaluator only
```

如果某实验确实把 GNSS 融入 estimator，则必须改用独立真值来源或把该实验明确标记为 fusion performance，而不能继续把同一 GNSS 输出称为独立 truth。

## 5. 时间与 frame 约束

- 正式离线重放统一使用 bag timestamp / `/clock`，不把 wall-clock 重新写入消息
- 所有用于地图派生的静态 TF/外参必须来自绑定 Calibration Set 或 canonical robot description，不允许实验脚本临时覆写但不记录
- `odom -> base_footprint`、`map -> odom` 的运行时 ownership 规则保持 V25-08 contract，不因离线处理改变
- GNSS evaluation 可建立 ENU reference frame，但必须保存 ENU origin/转换参数并与 operational `map` frame 区分

## 6. 可复现性

一次正式派生必须能仅凭以下信息重新运行：

```text
Dataset binding + verified Bag bundle
+ Calibration Set
+ Platform profile
+ Derivation recipe
+ repository commit / dependency snapshot
```

任何手工点云清理、控制点选择、地图裁剪或路线微调如果改变最终资产，都必须保存为显式 patch/parameter artifact；“在 GUI 里拖过但没有记录”不能生成 READY 资产。
