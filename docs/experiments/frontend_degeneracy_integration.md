# Cartographer 前端退化观测联调记录

## 范围与基线

- 日期：2026-10-31
- 工作区：`/home/ubuntu2204/cartographer_ros2_ws`
- 分支：`main`
- 基线 HEAD：`b1a37b625af2aaf5339f23da3f037dcfa3873895`
- 本阶段只做观测式集成、运行验证和数据导出，不改变 ScanMatcher 权重、PoseGraph 或退化数学定义。
- 运行数据来自 `data/converted_test_bag/` 的临时 ROS 2 bag。它只有 `/scan`，没有 TF、odom、IMU 和轨迹真值，因此结果是链路冒烟证据，不是正式走廊实验结论。

## 现有实现

`LocalTrajectoryBuilder2D::AddAccumulatedRangeData()` 在
`src/cartographer/cartographer/mapping/internal/2d/local_trajectory_builder_2d.cc:231`
取得 `filtered_gravity_aligned_point_cloud`。该点云来自
`AdaptiveVoxelFilter(gravity_aligned_range_data.returns, ...)`，随后同一个对象被传给
`ScanMatch()`（约第 269 行）和 `InsertIntoSubmap()`。观测代码只把每个点的 x、y
转换成 `std::vector<Eigen::Vector2d>`，没有引入 ROS topic、消息或 TF，也没有修改
ScanMatch 的输入、返回值和后续插图流程。

核心公共接口位于
`src/degeneracy_detector_core/include/degeneracy_detector_core/analysis.h`：

- `compute_covariance(points, CovarianceResult&)`
- `analyze_covariance(covariance, AnalysisConfig, DegeneracyResult&)`
- `analyze_points(points, AnalysisConfig, CovarianceResult&, DegeneracyResult&)`

`analysis.cpp` 使用样本协方差 `1/(N-1)`，Eigen `SelfAdjointEigenSolver` 返回升序
特征值，索引 0/1 分别作为 `lambda_min`/`lambda_max`。退化分数为
`lambda_max / lambda_min`，默认阈值为 100，判定比较为 `score > threshold`；最大
特征值对应的 `eigen_vectors.col(1)` 作为 `direction`。点数少于 2、非有限点、无效
参数、最小特征值小于 `1e-8`、非有限分解结果或除法结果都会返回失败。

## 运行验证

短时运行使用 scan-only 配置和转换 bag，bag 播放速率为 0.5 倍，运行约 50 秒；运行
日志保存于 `/tmp/frontend_degeneracy_cartographer_20261031.log`。实际观察到：

- Cartographer 节点启动并创建 trajectory，`/scan` publisher 与 Cartographer subscriber 匹配。
- 传感器时间速率约 11--12 Hz；日志中持续出现前端退化观测。
- 没有出现 scan matching failure、TF lookup/extrapolation 错误或异常退出。
- 23 条观测日志被解析为有效 CSV 行；0 条 `unavailable` 日志，0 条畸形观测行。
- 退化分数最小值 `2.31794`，中位数 `4.18723`，最大值 `56.7857`；23 条日志全部为 `degenerate=false`（默认阈值 100）。
- 有一帧滤波后只有 1 个点，核心库按点数保护拒绝该次指标；Cartographer 随后仍继续处理扫描。

日志使用 `LOG_EVERY_N(INFO, 20)` 和对应的 warning。该宏只限制日志输出，不限制
`analyze_points()` 的调用；源码控制流显示每次非空累计数据在 `ScanMatch()` 前都会尝试
计算一次。当前没有独立计数器，因此不能从 23 条限频日志精确推导总调用数、有效计算总数
或跳过总数。能够严格报告的数字是“23 条有效日志样本、0 条无效日志样本”；不能把 23
称为全部扫描数或全部算法调用数。

CSV 由 `tools/degeneracy_analysis/parse_frontend_log.py` 从现有日志解析生成：

`docs/experiments/frontend_degeneracy_metrics_20261031.csv`

字段为 `timestamp`、`point_count`、`lambda_max`、`lambda_min`、`degeneracy_score`、
`is_degenerate`、`direction_x`、`direction_y`。脚本拒绝覆盖已有输出，并丢弃点数不足或
非有限字段。该 CSV 是限频日志抽样，不是完整逐帧数据集。

## 数学和语义审查

点云是重力对齐参考系中的二维投影，并已经过自适应体素滤波；其坐标与 ScanMatch
实际收到的点云一致。滤波改变点数和采样分布，但不改变参考坐标系。协方差描述点云
空间分布的各向异性，最大特征向量是最大方差主轴，向量符号 `v` 与 `-v` 等价。

该 PCA 分数不等价于 Ceres ScanMatcher 的 Hessian 条件数、位姿协方差或某个位姿方向
必然弱约束。日志名称中的 “scan-match degeneracy” 应理解为“用于 scan-match 输入
点云的几何观测”，不能直接作为优化可观测性结论。分数在单帧几何、体素采样和视场变化
下都会变化；当前临时 bag 的分数没有越过默认阈值，也不足以证明长走廊退化已被完整捕获。

## 构建与测试

已执行的定向验证：

```text
colcon build --packages-select degeneracy_detector_core cartographer --symlink-install
colcon test --packages-select degeneracy_detector_core degeneracy_detector cartographer
colcon test-result --verbose
python3 -m py_compile tools/degeneracy_analysis/parse_frontend_log.py
```

- `degeneracy_detector_core` 构建成功；核心 GTest 10 项通过。
- `cartographer` 构建成功；CeresScanMatcher 4 项、RealTimeCorrelativeScanMatcher 4 项、PoseGraph2D 5 项通过。
- detector GTest 18 项通过。
- `colcon test-result` 汇总为 111 项、0 失败。
- 本次 detector 单独构建命令遇到已有 `build/degeneracy_detector/ament_cmake_python/degeneracy_detector/degeneracy_detector` 目录，ament 无法在那里创建符号链接；没有清理或删除 build 目录。该环境问题应在后续隔离构建目录或人工处理后再复核，不能记为 detector 构建成功。

## 后续自适应 Scan Matching 设计

当前阶段不应把 PCA 最大特征向量直接用作弱约束方向，也不应直接据此调整 Ceres
残差权重。建议下一阶段：

1. 在 `LocalTrajectoryBuilder2D` 保留每次观测，定义明确的
   `DegeneracyObservation`（时间、点数、特征值、分数、方向、有效标志）。
2. 以显式参数扩展 `CeresScanMatcher2D::Match()`，默认关闭自适应策略；正常环境下
   保持原始残差和权重路径，支持 baseline/improved 对照。
3. 在 Ceres residual Jacobian 或局部 Hessian 上验证实际弱可观测方向，再决定是否
   使用 PCA 结果作为辅助特征，而不是把主方差轴直接等同于弱方向。
4. 对判定使用时间平滑或迟滞，避免单帧分数抖动导致权重跳变，并记录原始观测和最终
   权重用于复现实验。
5. 后端 PoseGraph2D 暂不接入；若将来筛选回环约束，必须单独验证局部退化不能简单
   等价为回环错误。

## 限制和下一步

- 当前日志没有精确的总计算/跳过计数；若论文实验需要逐帧统计，应增加不影响算法的
  诊断计数器或结构化记录接口。
- 没有真实轨迹真值，未验证绝对定位、回环质量或完整往返地图精度。
- 原始 `test.bag` 和转换数据库未改变，且不应提交到 Git。
- 下一步优先建立完整数据记录接口和 Ceres 局部可观测性基线，再实现默认关闭的
  自适应匹配分支。
