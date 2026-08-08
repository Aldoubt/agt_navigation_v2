# Route Asset canonical example

本目录是 `route_asset_contract.md` 的最小可版本化样例，用于后续 parser/validator、Qt/RViz preview 和 Route backend 测试 fixture。

文件：

```text
route.yaml                 资产 identity/binding
policy.yaml                语义派生/优化规则
route.csv                  高密度 map-frame route
feasibility_report.json    footprint/运动学验收证据
```

示例中的 SHA256 使用占位值，不是实际文件 hash，因此不能直接作为 READY runtime asset。真实工具生成时必须用实际内容 hash 替换。
