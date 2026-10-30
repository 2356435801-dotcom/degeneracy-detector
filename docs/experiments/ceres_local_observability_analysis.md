# Cartographer Ceres 2D 局部可观测性基线

## 范围和基线

- 日期：2026-10-31
- 工作区分支：`main`
- 开始时本地和 `origin/main` 均为 `83c903007908352f15c3f636847a1836440ea36f`。
- 工作区原有 Cartographer、detector、core 和 `ros1_original/` 未提交修改已保留。本阶段在已有前端 PCA 观测集成之上增加 opt-in 的 Ceres 诊断；没有改变 ScanMatcher 残差、权重、初值或 Solver 配置，也没有修改 PoseGraph 约束策略。

## 源码结构和优化问题

调用路径是 `LocalTrajectoryBuilder2D::AddAccumulatedRangeData()` → `ScanMatch()` →
`CeresScanMatcher2D::Match()`。前端把优化前经过 AdaptiveVoxelFilter 的点云交给 Ceres，优化变量是单个长度为 3 的参数块：

```text
ceres_pose_estimate = [x, y, yaw]
```

当前代码没有为该参数块注册 manifold 或 local parameterization，因此 ambient 与 local Jacobian 都是 3 列；yaw 是以弧度表示的标量，不是独立四元数参数。

`CeresScanMatcher2D::Match()` 实际添加：

| 残差块 | 输出维度 | 作用和缩放 |
| --- | ---: | --- |
| ProbabilityGrid occupied-space 或 TSDF match（二选一） | 每个点一个残差 | 点云与当前子图网格的匹配残差；缩放为 `occupied_space_weight / sqrt(point_count)`，具体网格插值/权重还在 cost function 内 |
| Translation delta | 2 | 优化平移相对 `target_translation` 的先验，使用 `translation_weight` |
| Rotation delta | 1 | 优化 yaw 相对初始 Ceres yaw 的先验，使用 `rotation_weight` |

所有残差块传给 Ceres 的 loss function 均为 `nullptr`。占用空间残差本身已应用配置缩放；`Problem::Evaluate(..., apply_loss_function=true)` 返回经过 loss/parameterization 处理的 Jacobian，而本实现无 robust loss、无 parameterization，故返回列对应实际三个参数，残差缩放已经体现在 Jacobian 中。

## Jacobian 与 Hessian 定义

求解结束后，仅当 Lua 开关 `enable_local_observability_diagnostics=true` 且前端提供诊断输出结构时，代码对同一个 `ceres::Problem` 调用 Ceres 2.0 `Problem::Evaluate`。Ceres 文档说明 Evaluate 使用当前参数内存中的值，不在 solve 期间调用，也不改写参数块；额外计算只在求解之后发生。

为把米和弧度放到可解释的共同尺度，定义扫描长度尺度：

```text
L = sqrt(mean(x_i^2 + y_i^2))  [m]
q = [dx, dy, L * dyaw]
J_q = J_[x,y,yaw] * diag(1, 1, 1/L)
H_q = J_q^T J_q
```

`L` 是当前匹配点云相对 tracking origin 的 RMS 半径。该缩放把一弧度 yaw 扰动换算成在典型扫描半径处的等效平移米数。记录的最弱方向是 `q` 坐标下归一化的特征向量；第三分量对应缩放后的 yaw 坐标，物理角度增量需再除以 `L`。不同扫描半径下的结果使用了不同 `L`，跨数据集比较时应同时保留该尺度。

每次诊断输出两个 Hessian：

1. `measurement` Hessian 只由 ProbabilityGrid/TSDF 占用空间测量残差构成，作为 scan-to-submap 局部测量可观测性基线。
2. `objective` Hessian 包含占用空间、平移先验和旋转先验，代表当前完整局部目标的线性化信息；它可能比纯测量 Hessian 更好条件化，不能用它替代测量退化分析。

矩阵显式对称化为 `(H + H^T)/2` 后使用 Eigen `SelfAdjointEigenSolver`。特征值升序排列。显著负特征值（小于 `-1e-12 * max(1, |lambda_max|)`）视为计算失败；小于容差的舍入级负值钳为零。条件数使用 `lambda_max/lambda_min`；若 `lambda_min <= 1e-12 * max(1, lambda_max)`，记录为 `inf`，表示数值秩不足，而不是报告一个巨大但有限的条件数。即使秩不足，只要 Jacobian 和分解有效，仍保留最小特征值和对应弱方向。

这些 Hessian 是给定最终匹配位姿、当前子图、当前残差缩放下的局部 Gauss-Newton 信息近似。它们不自动等于真实位姿协方差或定位精度；协方差还需要噪声模型、残差概率解释和可逆性假设。

## 开关和输出

- Cartographer 默认 Lua 配置新增 `enable_local_observability_diagnostics = false`。未配置该字段的现有 Lua 仍按 false 解析。
- 新增 `scan_only_2d_observability.lua`，只在专用短时实验中将开关设为 true；原 `scan_only_2d.lua` 保持不变。
- Ceres 公共 Match 的诊断输出指针默认是 `nullptr`。LocalTrajectoryBuilder 仅在配置开启时传入指针；PoseGraph/ConstraintBuilder 仍不传诊断指针。
- 开启后每次前端匹配在求解结束后执行两次 Evaluate、构造 3x3 Hessian 并做特征分解；关闭时不执行这部分操作。
- 输出日志以同一条记录包含 Cartographer 时间戳、PCA 点数/特征值/分数/方向、测量 Hessian 与完整目标 Hessian 特征值/条件数、最弱方向、Ceres solver 时间、诊断耗时和有效标志。PCA 与 Hessian 来自同一个 `AddAccumulatedRangeData()` 调用，不做近似时间戳拼接。

## 一致性测试

新增 `DiagnosticsDoNotChangePoseEstimate`：使用相同初值、点云、网格、残差权重和 solver options，分别运行诊断默认关闭和显式开启的匹配器，逐分量使用 `EXPECT_DOUBLE_EQ` 比较最终 x/y/yaw；同时检查两类 Hessian 的 3 参数维度、有限特征值/方向和有效诊断结果。该测试通过，证明在当前测试输入下，后求解 Evaluate 不改变匹配结果。该证明覆盖 Ceres Match 输出，不是对所有硬件/线程配置的形式化证明。

## 短时 bag 结果

测试输入为临时 ROS 2 转换 bag `data/converted_test_bag/`，配置为 `scan_only_2d_observability.lua`，命令：

```text
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run cartographer_ros cartographer_node -configuration_directory install/cartographer_ros/share/cartographer_ros/configuration_files -configuration_basename scan_only_2d_observability.lua --ros-args -p use_sim_time:=true
ros2 bag play data/converted_test_bag --clock -r 0.5
```

约 32 秒回放后正常停止节点。运行日志保存在 `/tmp/ceres_local_observability_20261031.log`，配对 CSV 为：

`docs/experiments/ceres_local_observability_metrics_20261031.csv`

解析结果：185 个有效同时间戳 PCA/Hessian 样本、1 个初始化阶段无 active submap 的无效诊断、0 条畸形记录。185 个 PCA 分数最小值 `1.08933`、中位数 `3.36401`、最大值 `23.2589`；该 bag 默认 PCA 阈值 100 下均为 `false`。测量 Hessian 条件数均为有限值。

- Pearson(`pca_score`, `log10(measurement_condition_number)`) = `0.107396`，该短片段中线性关联较弱。
- 方向夹角未计算：PCA 主轴在当前点云坐标系，而 Hessian 平移分量在子图坐标系，且弱向量混合 x/y/yaw；没有姿态旋转变换和投影可比性检查前不能直接比较两个 2D 向量。
- Ceres solver `summary.total_time_in_seconds` 中位数 `0.323057 ms`，P95 `0.533819 ms`，最大 `0.861883 ms`。
- 两次额外 Evaluate、矩阵构造和特征分解的 `diagnostic_ms` 中位数 `0.059551 ms`，P95 `0.089183 ms`，最大 `0.201789 ms`。中位诊断耗时约为 solver 中位耗时的 18.4%；日志 I/O 和节点整体吞吐影响未隔离测量，因此该比值是计算部分估算，不是完整端到端开销。
- 弱向量常包含显著 yaw 分量；此处的弱方向只表示该加权局部线性化测量模型的最小信息方向。

该 bag 只有 `/scan`，不含轨迹真值，且有效覆盖只是短时片段。以上只是工具链和基线能力验证，不能据此宣称 PCA 与位姿退化具有稳定相关性，也不能作为论文结论。

## 构建与测试

```text
colcon build --packages-select cartographer --symlink-install --event-handlers console_direct+
colcon build --packages-select cartographer_ros --symlink-install --event-handlers console_direct+
colcon test --packages-select degeneracy_detector_core degeneracy_detector --event-handlers console_direct+
ctest --test-dir build/cartographer --output-on-failure -R 'ceres_scan_matcher_2d_test|real_time_correlative_scan_matcher_2d_test|pose_graph_2d_test'
colcon test-result --verbose
```

Cartographer 与 Cartographer ROS 目标构建成功；核心库 10 项和 detector 18 项测试通过；CeresScanMatcher2D 目标（含新增一致性用例）、RealTimeCorrelativeScanMatcher2D 和 PoseGraph2D 三个测试目标通过；test-result 汇总 111 项、0 失败。Python 解析器语法检查通过，诊断 CSV 解析为 185/1/0（有效/无效/畸形）。

## 限制与后续工作

- 本地 Hessian 会依赖子图插值梯度、网格状态、残差权重和最终匹配位姿；测量 Hessian 不代表环境几何的纯理论可观测性。
- 目前只在 ProbabilityGrid scan-only bag 上进行了运行采样；TSDF 分支还没有单独诊断测试或运行验证。
- 计时仅覆盖 Ceres solver summary 和求解后诊断区间，未对完整节点 CPU/内存/ROS 输出负载做基准。
- 下一步应使用具有多种场景和真值的数据，重复采集并报告置信区间；需要比较方向前先将 PCA 轴通过匹配 yaw 旋转到同一子图坐标系，并处理 Hessian 弱向量的平移/旋转混合。
- 自适应策略仍未实现。应先固定基线记录格式和诊断开关，再评估 measurement Hessian 的弱方向与前端误差/轨迹真值的关系。
- 本报告中的配对运行数据是在工作区已有但未提交的 PCA 前端观测代码和核心库链接配置上采集。为保护初始化前的修改，这些旧改动不属于本阶段提交；单独检出本阶段提交不会包含 PCA 计算和与之配对的运行日志生产路径。
