# V25-12E R5.5 Forward Navigation Preview Gate — 2026-08-14

## Scope

真实温室冻结资产

```text
coverage_order.yaml
turn_zones.yaml
navigation_map.pgm/yaml
profiles/platforms/mk_mini.yaml
```

车辆 preview 约束

```text
platform                  mk_mini
kinematics                ackermann
minimum turning radius    1.5 m
canonical preview body    0.84 x 0.60 m
preview footprint padding 0.05 m
sample step               0.05 m
```

正式 `base_footprint` 参考点仍未实车冻结，因此本记录只属于 `PREVIEW_ONLY_NOT_R8_VEHICLE_READY`

## Operator real-data result

```text
connector requests        17
PREVIEW_FOOTPRINT_FREE     0
review required           17
```

所有 connector 的所有 analytic Dubins family 中，没有 candidate 满足当前严格 preview gate

```text
OCCUPIED fraction = 0
UNKNOWN fraction  = 0
grid coverage      = 1
```

这比上一轮共享 Turn Zone 矩形 FREE/OCCUPIED 统计更有意义，因为 R5.5 是逐 connector / 逐轨迹 sweep，而不是评价整个棚头矩形

## Important diagnostic correction

R5.5 的 `preview_free_count=0` 是可靠结论，因为所有 candidates 都实际被检查

但当所有 candidates 都失败时，当前 representative rejected candidate 的排序优先级会先最小化 OCCUPIED fraction，再考虑 UNKNOWN / Turn-Zone deficit

因此它可能选择一个远离局部棚头、绕入 UNKNOWN 的大回环作为“代表失败候选”

典型例子

```text
connector_002

R5 zone-fit local candidate
RSR
max Turn-Zone deficit ≈ 0.517 m

R5.5 representative rejected candidate
RLR
length                 12.801 m
zone_max                3.414 m
footprint UNKNOWN       0.563
```

所以不能直接使用 R5.5 输出中的单个 `path_type / length / zone_max` 来决定该 connector 是否进入 reverse fallback

## Strong known-collision examples from representative output

部分 connector 的 representative candidate 已经在完整 grid coverage、极低 UNKNOWN 下出现明显 OCCUPIED，说明至少这些 local-forward 几何值得重点怀疑

```text
connector_003
center OCCUPIED          0.333
footprint OCCUPIED       0.414
UNKNOWN                  0
zone_max                 0.468 m

connector_013
center OCCUPIED          0.321
footprint OCCUPIED       0.300
UNKNOWN                  ≈ 0
zone_max                 0.666 m

connector_016
center OCCUPIED          0.457
footprint OCCUPIED       0.502
UNKNOWN                  ≈ 0
zone_max                 0.846 m

connector_017
center OCCUPIED          0.250
footprint OCCUPIED       0.261
UNKNOWN                  0
zone_max                 0.189 m
```

这些仍不是“所有 Dubins family 都被已知障碍封死”的充分证明，因为 representative candidate 不是完整 candidate audit

## R5.6 correction

新增

```text
agt_forward_connector_candidate_audit/v1
```

对每个 connector 保留全部 Dubins candidates，并按照

```text
max required Turn-Zone extension
→ path length
→ path type
```

排序

定义 local-headland candidate set

```text
candidate.max_zone_extension
<=
minimum_zone_extension + local_zone_extension_slack
```

首版默认

```text
local_zone_extension_slack = 0.25 m
```

它不是绝对“棚头最多允许 0.25 m”，而是只允许比该 connector 最局部的 Dubins 解额外多绕 0.25 m，用于避免大回环 UNKNOWN candidate 干扰 local-headland diagnosis

Connector-level classifications

```text
FORWARD_PREVIEW_FREE
LOCAL_FORWARD_OCCUPANCY_BLOCKED
LOCAL_FORWARD_MAP_EVIDENCE_INSUFFICIENT
LOCAL_FORWARD_MIXED_EVIDENCE
LOCAL_FORWARD_POLICY_REVIEW
```

## R6 decision rule

不要因为 `R5.5 preview_free_count=0` 就把 17 条全部交给 Reeds-Shepp

优先依据 R5.6

```text
LOCAL_FORWARD_OCCUPANCY_BLOCKED
→ local forward candidates 有充分地图证据且全部撞 OCCUPIED
→ 可以进入 R6 reverse fallback 候选集

LOCAL_FORWARD_MAP_EVIDENCE_INSUFFICIENT
→ 先解决 UNKNOWN / map evidence
→ 不允许用 reverse planner 掩盖地图缺口

LOCAL_FORWARD_MIXED_EVIDENCE
→ 同时存在已知 collision 与未知替代路线
→ operator / map review 后再决定

FORWARD_PREVIEW_FREE
→ 保留 forward connector
→ 再处理最小 Turn Zone authoring + 后续 R8 gate
```

本记录不宣称任何 connector 已经正式 R6-ready，等待 R5.6 真实候选审计
