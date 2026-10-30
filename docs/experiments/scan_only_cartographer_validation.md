# Cartographer ROS2 Scan-only 运行链路验证

## 结论

基于 `data/converted_test_bag/` 的 scan-only 验证通过了配置加载、节点启动、`/scan` 订阅、同帧 TF、SensorBridge 到前端、ScanMatch 以及子图和占据栅格发布。实验没有修改 Cartographer 算法、退化检测算法或原有 Lua 配置。

“前端通过”表示本次运行有真实扫描进入 LocalTrajectoryBuilder2D 并产生匹配/插图证据；“SLAM 建图通过”表示本次确实发布了非空 OccupancyGrid 和多个子图，不代表已经完成精度评估或证明走廊往返轨迹正确。

## 环境和基线

- 工作区：`/home/ubuntu2204/cartographer_ros2_ws`
- ROS：Ubuntu 22.04 / ROS 2 Humble
- 分支：`main`
- 验证开始时本地和 `origin/main`：`a405dee7f315daa3e37565449803c207cf4875cb`
- 工作区已有 Cartographer、`degeneracy_detector`、`degeneracy_detector_core` 和 `ros1_original` 的未提交修改；本任务没有暂存或覆盖它们。
- 输入 bag：`/home/ubuntu2204/cartographer_ros2_ws/data/converted_test_bag/`
- 本次运行前后 `test.bag` SHA256：`a95acbf282ce36695a943b6b86b4a8ce235f1d4bc76156c85bbcd8a96307fb54`
- 转换数据库 SHA256：`e6f27c750892f5a62fb476b3eb66e4e4205b72951cb7df4160a172e102eb07cb`

## 配置和启动文件

新增：

- `src/cartographer_ros/cartographer_ros/configuration_files/scan_only_2d.lua`
- `src/cartographer_ros/cartographer_ros/launch/scan_only_2d.launch.py`

`scan_only_2d.lua` 根据当前分支的 `map_builder.lua`、`trajectory_builder.lua` 和 `trajectory_builder_2d.lua` 字段组织，关键值为：

```text
tracking_frame = "laser"
published_frame = "laser"
provide_odom_frame = false
use_odometry = false
use_nav_sat = false
use_landmarks = false
num_laser_scans = 1
num_multi_echo_laser_scans = 0
num_point_clouds = 0
num_subdivisions_per_laser_scan = 1
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
```

由于当前 `backpack_2d.lua` 配置的是 MultiEcho，而输入是 `sensor_msgs/msg/LaserScan`，没有复用它。`scan_only_2d.launch.py` 启动 `cartographer_node` 和 `cartographer_occupancy_grid_node`，默认 `use_sim_time=true`，并通过 occupancy-grid 可执行文件的 gflags 参数设置 `resolution=0.05`、`publish_period_sec=1.0`。`/scan` 显式映射到 `/scan`；bag 播放保持在独立终端。

实际命令：

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu2204/cartographer_ros2_ws/install/setup.bash

colcon build --packages-select cartographer_ros --symlink-install \
  --event-handlers console_direct+

ros2 launch cartographer_ros scan_only_2d.launch.py
ros2 bag play /home/ubuntu2204/cartographer_ros2_ws/data/converted_test_bag \
  --topics /scan --rate 0.5 --clock
```

构建结果：`cartographer_ros` 定向构建成功，所有相关 target（包括 `cartographer_node` 和 `cartographer_occupancy_grid_node`）成功完成；新 Lua 和 launch 文件通过 symlink 安装到 `install/cartographer_ros/share/cartographer_ros/`。Python launch 文件 `py_compile` 检查通过。

## 验证证据

### 配置加载和节点启动

Cartographer 日志实际报告找到了：

```text
.../install/cartographer_ros/share/cartographer_ros/configuration_files/scan_only_2d.lua
.../install/cartographer/share/cartographer/configuration_files/map_builder.lua
.../install/cartographer/share/cartographer/configuration_files/trajectory_builder.lua
.../install/cartographer/share/cartographer/configuration_files/trajectory_builder_2d.lua
```

随后报告 `Added trajectory with ID '0'.`。启动后节点为 `/cartographer_node` 和 `/cartographer_occupancy_grid_node`。

### 同帧 TF

在没有发布 `/tf` 或 `/tf_static` 的独立检查中执行：

```bash
ros2 run tf2_ros tf2_echo laser laser
```

实际结果为零平移、单位四元数。Cartographer 运行期间还实际发布了：

```text
frame_id: map
child_frame_id: laser
```

因此 `tracking_frame="laser"` 与 LaserScan 的 `frame_id="laser"` 在本实验中不需要外部传感器 TF；Cartographer 自己发布的 `map -> laser` 也被观测到。

### `/scan`、SensorBridge 和前端

`ros2 topic info /scan --verbose` 观测到：

- bag 播放器 1 个 publisher，类型 `sensor_msgs/msg/LaserScan`，RELIABLE/VOLATILE；
- `cartographer_node` 1 个 BEST_EFFORT/VOLATILE subscriber；
- 实际回放期间 `ros2 topic hz /scan` 约 5.99 Hz（第一次 0.5 倍速记录；Cartographer 内部报告约 11--12 Hz的传感器时间速率，符合 0.5 倍实时播放）。

Cartographer 日志出现：

```text
All sensor data for trajectory 0 is available starting at ...
scan rate: 11.99 Hz ... (pulsed at 50.00% real time)
```

`LocalTrajectoryBuilder2D::AddAccumulatedRangeData()` 的观测日志实际出现，例如：

```text
2D scan-match degeneracy observation: ... points=209
lambda_max=4.19066 lambda_min=0.93084 score=4.50202
degenerate=false direction=(0.919608,-0.392838)
```

这证明退化核心分析在当前已集成的调用点执行，且点云随后进入原有 `ScanMatch()` 路径。`LOG_EVERY_N(INFO, 20)` 只限制日志输出频率；从持续的 scan rate、子图插入和多次观测可见，未将计算改为每 20 帧一次。

第二次回放期间出现一次：

```text
2D scan-match degeneracy observation unavailable: ... points=1
```

这是分析库对点数不足的保护日志；后续扫描继续处理，没有导致轨迹终止。该警告与输入中极少有效点的扫描相符。

### ScanMatch、子图和地图

运行期间日志实际出现：

```text
Inserted submap (0, 0).
Inserted submap (0, 1).
Inserted submap (0, 2).
```

`/submap_list` 回显包含 trajectory 0 的 submap 0 和 1，版本分别为 152 和 62；另一时刻已出现 submap 2。`/trajectory_node_list` 回显包含 trajectory 0 的连续轨迹点。

`/scan_matched_points2` 的实际类型为 `sensor_msgs/msg/PointCloud2`，回显宽度为 575，说明匹配结果点云有真实发布。`/map` 的实际类型为 `nav_msgs/msg/OccupancyGrid`，发布 QoS 为 TRANSIENT_LOCAL；自定义订阅器收到：

```text
frame=map resolution=0.0500000007 width=816 height=746 cells=608736
min=-1 max=87
```

数据中既有 `-1` 未知单元，也有 0 和正的占用概率值，说明地图不是空消息。第一轮回放还观测到 877x868 的地图，尺寸随子图增长是预期行为。

### 错误、警告和停止

- 没有观察到 `Could not find a connection between frames`、`Transform unavailable`、时间外推错误、`Scan matching failed` 或 extrapolator 初始化失败。
- 首帧日志有 `Extrapolator is still initializing.`，随后正常进入 scan matching，这是初始化阶段的预期日志。
- 有一次退化分析点数不足警告，见上文；不影响后续处理。
- 两次实验均通过 Ctrl-C 正常停止，Cartographer 报告关闭 `/scan` subscriber、完成 trajectory、运行 final optimization，两个进程 cleanly finish。launch 日志中的 warning 仅为用户中断。
- 本次是两次受控的部分 bag 回放（约几十秒到一分钟），用于验证运行链路和输出；没有等待 5 分 44 秒 bag 全部播放结束。因此本报告不声称完成全程走廊往返、全程退化统计或最终地图质量评估。

## 验收结论与限制

| 项目 | 结论 |
| --- | --- |
| Lua 配置加载 | 通过 |
| ROS2 launch 和两个节点启动 | 通过 |
| `/scan` 订阅 | 通过 |
| 同帧 TF 查询 | 通过 |
| SensorBridge/前端接收 | 通过，有 scan rate 和前端日志 |
| LocalTrajectoryBuilder2D | 通过，有退化观测和子图插入 |
| ScanMatcher | 通过，有持续退化观测、匹配点云和轨迹输出 |
| Submap | 通过，至少插入 submap 0、1、2 |
| OccupancyGrid | 通过，收到非空、含已观测单元的地图 |

本阶段证明了 scan-only 数据链路和建图输出可以运行。它没有提供外部真值轨迹，没有证明走廊往返的绝对位姿精度，也没有把几何退化分数直接等同为位姿可观测性。下一阶段可在此配置和 bag 基线上采集每帧指标，建立退化区间与轨迹/地图误差的实验对照；前端匹配权重和 PoseGraph 仍保持未修改。
