# 测试数据

不要把大型 rosbag 文件提交到仓库。将一个小型 rosbag2 复制到：

```text
test_data/bags/example_bag/
```

也可以通过环境变量指定真实测试 bag：

```bash
export ROSBAG_SENSOR_TRIMMER_TEST_BAG=/absolute/path/to/bag
scripts/smoke_test.sh
```

冒烟脚本会先构建和运行单元测试；如果没有测试 bag，会明确跳过真实 bag 裁剪，不伪造集成测试结果。
