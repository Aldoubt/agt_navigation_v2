# Paper I Workbench — Selectable Map Resource Bundle Handoff

Date: 2026-08-22  
Branch: `fix/paper1-m154b-v25-map-authority-recovery`  
PR: `#17 fix(paper1): recover V25 map authority`  
Status: `CODE_IMPLEMENTED / CORE_CONTRACT_VERIFIED / TARGET_WORKBENCH_VERIFICATION_REQUIRED`

## Goal

Add one Workbench-level **Save Map Resource Bundle As...** workflow so a reviewed greenhouse map session can persist only the resources needed for later reuse, without repeatedly copying the original large PCD or scattering exports across multiple directories.

This is an export/organization feature only. It does not change map derivation, V25-12F policy, planner behavior, or Paper map authority.

## User-facing entry points

Paper I Workbench now exposes:

- Navigation Map tab: `另存为地图资源包…`
- Menu: `离线资产 -> 另存为地图资源包…`

The dialog lets the operator choose one target parent directory and one immutable bundle name.

## Selectable groups

The current implementation exposes these options:

1. `处理后的点云 processed.pcd`
2. `正式 Navigation Map（Generated + Accepted + Derivation）`
3. `地形 / Ground / Slope / Step / Obstacle 派生层`
4. `农业结构层（Row / Aisle / Aisle Graph）`
5. `V25-12F Traversability / Candidate 资源`
6. `Vehicle-Feasible Segments`
7. `处理 Recipe processing_recipe.yaml`
8. `Map Frame 标定 map_frame.yaml`
9. `Site Boundary`
10. `复制原始输入 PCD 到资源包`

Unavailable resources are disabled and labeled `未生成` instead of being fabricated.

## Defaults

Recommended/default selection:

```text
ON   processed.pcd              if available
ON   formal Navigation revision if available
ON   terrain layers             if available
ON   structure layers           if available
OFF  V25-12F candidate
OFF  vehicle-feasible segments
ON   recipe                     if available
ON   map_frame                  if available
ON   site_boundary              if available
OFF  copy original PCD
```

`全选` explicitly enables every available resource including original-PCD copy. `推荐选择` always resets original-PCD copy to OFF.

## PCD identity behavior

When a raw PCD is opened and Workbench full-resolution processing is executed:

```text
raw input PCD
  -> remembered as original source
processed output PCD
  -> remembered as processed_pointcloud
```

If the operator directly opens a file named `processed.pcd` in a fresh Workbench session:

- it is treated as the processed PCD;
- original PCD identity is unknown;
- the `copy original PCD` checkbox is disabled rather than guessed.

When original-PCD copy is OFF, the bundle does **not** duplicate the file. `map_resource_manifest.yaml` still records the original absolute path, SHA256, size, and `copied: false` when the original source is known.

## Immutable output

The bundle is written through a hidden staging directory and atomically renamed only after all selected writers and SHA256 calculations succeed.

Existing bundle directories are never overwritten. A writer failure removes the staging directory and leaves no final partial bundle.

Example:

```text
greenhouse_01_map_assets_v01/
├── map_resource_manifest.yaml
├── pointcloud/
│   └── processed.pcd
├── navigation/
│   ├── generated/
│   │   ├── navigation_map.pgm
│   │   └── navigation_map.yaml
│   ├── accepted/
│   │   ├── navigation_map.pgm
│   │   └── navigation_map.yaml
│   └── derivation.yaml
├── authoring/
│   ├── processing_recipe.yaml
│   ├── map_frame.yaml
│   └── site_boundary.yaml
└── layers/
    ├── terrain/
    ├── obstacle/
    ├── structure/
    └── traversability/    # only when selected
```

The formal Navigation revision remains atomic: Generated + Accepted + Derivation are exported together so the resulting `navigation/` directory can retain the V25 revision contract. This bundle does not itself assert that planner-independent QA or human acceptance has passed.

## 12F export rule

12F is independently selectable and defaults OFF.

When selected, its group writes a local `layers/traversability/site_boundary.yaml` before invoking the existing V25-12F candidate writer. Its derivation records `WORKBENCH_CURRENT_NAVIGATION` rather than assuming the formal Navigation revision was also selected.

This keeps the 12F resource group self-contained without turning the candidate map into canonical map authority.

## Manifest

`map_resource_manifest.yaml` uses:

```text
agt_map_resource_bundle/v1
```

It records:

- selected groups;
- availability state;
- current PCD identity;
- original PCD identity and whether it was copied;
- every actually written file;
- relative path;
- SHA256;
- size in bytes.

## Verification evidence available in this implementation session

A local temporary reconstruction of the pure resource-bundle core was executed with pytest:

```text
4 passed in 0.07s
```

Those tests cover:

- recommended selection leaves original-PCD copy OFF;
- only selected groups are written;
- original PCD is referenced but not copied by default;
- explicit original-PCD copy works;
- existing bundle names are rejected;
- writer failure leaves no final or hidden partial bundle.

The current execution environment does not contain PyQt5 and cannot access GitHub to clone the repository, so the full Workbench test suite has not been executed here.

## Required target-machine gate

Run on the Paper workspace before treating this feature as verified:

```bash
cd ~/agt_navigation_v2_paper1

git fetch origin
git switch fix/paper1-m154b-v25-map-authority-recovery
git pull --ff-only origin fix/paper1-m154b-v25-map-authority-recovery

source /opt/ros/humble/setup.bash

QT_QPA_PLATFORM=offscreen \
PYTHONPATH=src/agt_map_workbench:src/agt_offline_assets:src/agt_ui_bridge:src/agt_coverage_planning \
python3 -m pytest -q \
  src/agt_map_workbench/test/test_navigation_override_metadata.py \
  src/agt_map_workbench/test/test_resource_bundle.py \
  src/agt_map_workbench/test/test_paper1_resource_bundle_integration.py

python3 -m compileall -q src/agt_map_workbench/agt_map_workbench
```

Then launch the Paper I Workbench and perform one real manual export:

```bash
source install/setup.bash
ROS_LOG_DIR="$PWD/runtime/log/paper1_map_workbench" \
ros2 run agt_map_workbench agt_map_workbench_paper1
```

Manual acceptance checklist:

```text
[ ] Save Map Resource Bundle As... button is visible
[ ] unavailable resources are disabled
[ ] original PCD is unchecked by default
[ ] recommended selection restores original PCD to unchecked
[ ] selected folder + bundle name create one new immutable directory
[ ] unselected groups do not create directories/files
[ ] map_resource_manifest.yaml exists
[ ] formal navigation/ contains generated + accepted + derivation together
[ ] 12F-only selection succeeds when 12F is available
[ ] reusing the same bundle name is rejected
```

## Map-authority invariant remains unchanged

```text
V25 Map Workbench accepted revision
= only formal Paper Navigation Map authority

resource bundle
= persistence / reuse container

V25-12F candidate
= evidence / candidate only
```

Do not resume Paper M1.5-5 planner tuning merely because a bundle exports successfully. The selected accepted map still needs the existing map-only QA + human acceptance gate before formal benchmark use.
