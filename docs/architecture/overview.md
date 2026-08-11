# Architecture Documentation

The authoritative target system architecture is [`system_architecture.md`](system_architecture.md)

V25-12 freezes the new Site Workflow and BT-composable capability direction before implementation continues

## Current architecture baseline

- V25-12 frozen requirements: [`V25_12_SITE_WORKFLOW_REQUIREMENTS.md`](../v2.5/V25_12_SITE_WORKFLOW_REQUIREMENTS.md)
- System architecture and main Mermaid diagram: [`system_architecture.md`](system_architecture.md)
- Offline Site Package production and benchmark workflow: [`offline_asset_pipeline.md`](offline_asset_pipeline.md)
- Mission and capability composition: [`navigation_task_orchestration.md`](navigation_task_orchestration.md)
- BehaviorTree execution and replaceable-backend boundary: [`behavior_tree_execution.md`](behavior_tree_execution.md)
- Runtime data flow: [`runtime_dataflow.md`](runtime_dataflow.md)

## Supporting architecture documents

- Sensor preprocessing: [`livox_custom_self_filter.md`](livox_custom_self_filter.md)
- Business control layer: [`business_control_layer.md`](business_control_layer.md)
- Cross-module terminology: [`terminology.md`](terminology.md)
- Future semantic perception reservation: [`future_semantic_perception_interfaces.md`](future_semantic_perception_interfaces.md)

## Architecture reading order

For new development, read in this order

```text
V25_12_SITE_WORKFLOW_REQUIREMENTS
        ↓
system_architecture
        ↓
offline_asset_pipeline
        ↓
behavior_tree_execution
        ↓
navigation_task_orchestration
        ↓
runtime_dataflow / topic contracts
```

The central rule is that scene-specific differences belong in versioned Site Packages, algorithm-specific differences belong behind project adapters/policies, and business-task differences belong in Mission/BehaviorTree orchestration
