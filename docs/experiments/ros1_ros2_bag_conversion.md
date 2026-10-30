# ROS1 到 ROS2 Bag 转换记录

## 状态

本次转换未完成。原始 ROS1 bag 已确认可读，但当前 WSL 环境缺少可用的 Python `pip/venv` 引导组件，且从 PyPA 下载 `get-pip.py` 两次均因网络超时得到截断文件。因此没有安装 `rosbags`、没有写入 ROS2 bag、没有执行转换后 `ros2 bag info` 或播放验证。

## 输入文件

- 路径：`/home/ubuntu2204/cartographer_ros2_ws/test.bag`
- 格式：ROS1 `#ROSBAG V2.0`
- 大小：`54,162,931` 字节
- SHA256：`a95acbf282ce36695a943b6b86b4a8ce235f1d4bc76156c85bbcd8a96307fb54`
- 原始审查：4100 条 `/scan`，`sensor_msgs/LaserScan`，`frame_id=laser`

转换前后已重新计算原始文件 SHA256，仍与上值一致；原始文件没有被修改。

## 环境准备

- 系统 Python：3.10.12
- 目标虚拟环境：`/home/ubuntu2204/.venvs/rosbags-convert`
- `python3 -m venv` 初次失败，因为 `ensurepip` 不可用。
- 使用 `python3 -m venv --without-pip` 创建了隔离环境，但环境内没有 pip。
- `virtualenv`、`uv`、系统 `pip`、ROS1 `rosbag`、Python `rosbag` 和 `rosbags` 均不可用。
- PyPA `get-pip.py` 下载尝试：30 秒超时后收到约 245,760 字节；第二次 180 秒/重试后仍超时，收到约 1,579,008 字节。两次均未执行截断脚本。
- 没有使用 sudo、系统 Python 安装、`/opt/ros/humble` 修改或 ROS1 环境修改。

## 输出目录与 Git 保护

目标输出目录为 `/home/ubuntu2204/cartographer_ros2_ws/data/converted_test_bag/`，本次未创建 bag 产物。原始 `test.bag` 和该输出目录已加入本地 `.git/info/exclude`，没有修改公共 `.gitignore`，也没有提交数据文件或虚拟环境。

## 未执行项目

以下项目必须在 `rosbags` 可用后执行，目前不能报告为通过：

- ROS1 Reader/ROS2 SQLite3 Writer API 检查
- `/scan` 转换脚本
- ROS2 `metadata.yaml` 和 SQLite3 生成
- 转换前后 4100 条消息数量对比
- header stamp 首末值、跨度、重复/倒退检查
- `frame_id` 全量对比
- `angle_min`、`angle_max`、`angle_increment`、`time_increment`、`scan_time`、range 限制、ranges/intensities 数组逐元素对比
- `Inf`/`NaN` 统计对比
- `ros2 bag info` storage ID 和 topic 验证
- `ros2 bag play` 与 `/scan` 订阅验证

## 下一步

优先安装发行版提供的 `python3.10-venv` 或在用户目录提供可用的隔离 Python 引导方式，随后在该 venv 中安装并锁定 `rosbags` 版本。安装完成前不应运行 Cartographer 离线节点，也不应把原始 ROS1 bag 当作 ROS2 bag 播放。
