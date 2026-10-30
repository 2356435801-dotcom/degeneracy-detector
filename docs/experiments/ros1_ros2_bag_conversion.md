# ROS1 到 ROS2 Bag 转换与逐字段验证

## 结论

转换已完成。ROS1 `test.bag` 被转换为独立的 ROS2 SQLite3 bag，ROS2 能识别、播放并在 `/scan` 收到 `sensor_msgs/msg/LaserScan`。4100 条消息逐帧对应，所有 LaserScan 字段和特殊浮点值均通过比较。原始 bag 只读，SHA256 在验证前后保持不变。

本任务没有启动 Cartographer，也没有修改 Lua 配置、Cartographer 算法或退化检测算法。

## 输入、环境与脚本

| 项目 | 实际值 |
| --- | --- |
| 原始 bag | `/home/ubuntu2204/cartographer_ros2_ws/test.bag` |
| 原始格式 | ROS1 `#ROSBAG V2.0` |
| 原始大小 | 54,162,931 bytes |
| 原始 SHA256 | `a95acbf282ce36695a943b6b86b4a8ce235f1d4bc76156c85bbcd8a96307fb54` |
| Python | 3.10.12 |
| 虚拟环境 | `/home/ubuntu2204/.venvs/rosbags-convert` |
| rosbags | 0.11.9 |
| 转换脚本 | `tools/bag_conversion/convert_ros1_to_ros2.py` |
| 输出 bag | `data/converted_test_bag/` |

目标转换命令（输出目录已存在时脚本会拒绝覆盖）：

```bash
~/.venvs/rosbags-convert/bin/python \
  tools/bag_conversion/convert_ros1_to_ros2.py \
  test.bag data/converted_test_bag
```

脚本使用 `rosbags` 的 ROS1 Noetic typestore、ROS2 Humble typestore、ROS1 Reader 和 SQLite3 Writer（version 8）。输出目录已存在时脚本拒绝覆盖。转换脚本另行输出到 `/tmp/cartographer_ros2_bag_conversion_verify_20261030`，返回 `converted_messages=4100`，证明脚本可重复执行而没有覆盖项目内输出。

工作区目标输出目录在本次验证开始前已经存在；本次没有覆盖或重写它，而是对其进行完整读回比较。下方临时目录是本次从原始 bag 新生成的副本。

本次实际重跑命令为：

```bash
~/.venvs/rosbags-convert/bin/python \
  tools/bag_conversion/convert_ros1_to_ros2.py \
  test.bag /tmp/cartographer_ros2_bag_conversion_verify_20261030
```

## ROS1 原始 bag 统计

- 连接数：1；话题：`/scan`；类型：`sensor_msgs/msg/LaserScan`；消息数：4100。
- 所有 `frame_id` 均为 `laser`。
- `ranges`/`intensities` 长度范围：30 到 2001；长度组合有 240 种；两数组长度逐帧一致。
- `angle_increment` 有 240 个不同值。
- `ranges`：5,162,605 个有限值，1,545,818 个 `+Inf`，0 个 `-Inf`，0 个 `NaN`。
- `intensities`：6,708,423 个有限值，0 个 `Inf`/`-Inf`/`NaN`。
- header 时间戳首值 `1782376742464036297` ns，末值 `1782377087038855142` ns，跨度 `344574818845` ns；无重复、无倒退。
- bag 记录时间首值 `1782376742548128087` ns，末值 `1782377087121693895` ns，跨度 `344573565808` ns；无重复、无倒退。两类时间保持原有差异，没有混用或改写。

## ROS2 bag 基础验证

`source /opt/ros/humble/setup.bash && ros2 bag info data/converted_test_bag` 的实际结果：

- Storage ID：`sqlite3`
- 消息数：4100
- 话题：`/scan`
- 类型：`sensor_msgs/msg/LaserScan`
- 序列化：`cdr`
- 记录时长：`344.573565808 s`
- 起始记录时间：`1782376742548128087 ns`

## 逐字段验证

使用 ROS1 Reader 和 ROS2 Reader 按顺序读取全部 4100 帧，比较 header、标量字段和两个可变数组。结果如下：

| 字段 | 结果 |
| --- | --- |
| `header.stamp.sec/nanosec` | 4100/4100 一致 |
| `header.frame_id` | 4100/4100 一致，均为 `laser` |
| `angle_min`、`angle_max`、`angle_increment` | 逐帧 float32 数值精确一致，最大误差 0 |
| `time_increment`、`scan_time` | 逐帧 float32 数值精确一致，最大误差 0 |
| `range_min`、`range_max` | 逐帧 float32 数值精确一致，最大误差 0 |
| `ranges` | 长度和每个 float32 数值一致，最大误差 0 |
| `intensities` | 长度和每个 float32 数值一致，最大误差 0 |
| ROS1 bag 记录时间 | 4100/4100 一致 |
| `Inf`/`-Inf`/`NaN` 分类 | ranges 与 intensities 均逐元素一致 |

转换前后统计完全相同：长度范围 30--2001、240 种长度组合、240 种角度增量、`ranges` 的 1,545,818 个 `+Inf`，以及所有有限值/NaN 计数均一致。未发现数值差异、缺失帧、乱序帧或时间戳倒退。

## ROS2 播放验证

播放命令：

```bash
source /opt/ros/humble/setup.bash
ros2 bag play data/converted_test_bag --topics /scan --rate 1
```

另一个终端执行 `ros2 topic list -t`，看到 `/scan [sensor_msgs/msg/LaserScan]`；随后 `ros2 topic echo /scan --once` 成功收到消息。回显首帧包含 `frame_id: laser`、有效 header 时间戳、原始角度参数、`+Inf` ranges 和完整 intensities 数组。高速率首次 echo 在 DDS 图发现前结束，降低到 rate 1 后验证成功；这属于订阅时序，不是 bag 数据错误。

## 数据用途与限制

- 该 ROS2 bag 已满足后续 scan-only Cartographer 输入的消息格式和字段完整性要求。
- 原始 bag 没有 `/tf`、`/odom` 或 `/imu`；本次没有伪造这些数据。因此 Cartographer 是否能在具体 tracking/published frame 配置下运行，仍需单独验证。
- 只有 `/scan` 和几何连续性不能提供外部轨迹，不能单凭本转换报告证明走廊往返或 SLAM 精度。
- 原始 bag SHA256 在转换前后均为 `a95acbf282ce36695a943b6b86b4a8ce235f1d4bc76156c85bbcd8a96307fb54`。原始文件和转换产物均通过本地 `.git/info/exclude` 防止误提交；大 bag、数据库和虚拟环境不属于 Git 提交。
