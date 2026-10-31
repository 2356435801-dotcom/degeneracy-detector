# Public 2D LiDAR SLAM dataset download inventory

审计日期：2026-10-31 14:55（Asia/Shanghai）
工作区：`/home/ubuntu2204/cartographer_ros2_ws`
原始数据目录：`数据集/`
外部清单：`/home/ubuntu2204/slam_public_datasets/manifests/dataset_file_inventory_20261031.csv`

本报告只记录本地已经存在的文件和只读解析结果。未下载新的数据集、未移动或覆盖原始文件、未使用 `test.bag`、未启动 Cartographer 正式实验。CSV 清单包含 221 个非 `Zone.Identifier` 文件及其大小、扩展名、识别格式和 SHA-256；Windows 下载附带的 `:Zone.Identifier` 文件未计入数据文件。

## 1. 工作区和 Git 保护状态

- 分支：`main`；HEAD 和 `origin/main` 均为 `47db2b7`（检查时）。
- 暂存区为空。工作区已有此前的 Cartographer、退化检测器、核心库和实验文档修改；本次未触碰这些文件。
- `数据集/` 是未跟踪目录。本报告没有把原始数据加入 Git，也没有使用 `git add .`。
- 未执行 `reset`、`clean`、`rebase`、force push、构建、数据转换或 Cartographer 运行。

## 2. 总体文件盘点

| 数据集 | 非 Zone.Identifier 文件数 | 总字节数 | 本地实际内容 |
|---|---:|---:|---|
| ESTS_ROS_Dataset | 48 | 2,638,270,853 | 1 个独立 SQLite3 db3、7 个 ROS 2 metadata、CSV/地图/视频；7 个 metadata 目录没有对应 db3 |
| mbarah | 31 | 430,847,879 | 3 个 ROS 2 SQLite3 bag、CSV、metadata、轨迹脚本 |
| mit stata | 130 | 2,242,403,737 | 82 个 ROS1 `.bag.info`、24 个 `.gt.laser.poses`、24 个 `.gt.laser.scans`；没有 `.bag` |
| predict 6G | 12 | 1,766,090 | README、launch、图片、一个 CSV 和 bag 下载说明；没有 bag |

完整逐文件清单见外部 CSV。下面的目录摘要保留了影响数据判定的文件类型和关键文件。

### ESTS_ROS_Dataset

`Complet/SLAM_ESTS/` 下有 `ADMINISTRATION/RDC/odom_scan_camera`、`ADMINISTRATION/ETAGE1/ODOM_SCAN_CAM`、`BLOC_1/RDC/ODM_SCAN_CAM`、`BLOC_1/ETAGE1/ODOM_SCAN_CAM`、`BLOC_1/ETAGE1/Nav`、`DEPARTEMENT/RDC/ODOM_SCAN_CAM`、`DEPARTEMENT/ETAGE1/ODOM_SCAN_CAM` 七个 `metadata.yaml` 目录，另含 `.pgm/.png/.yaml` 地图、CSV 导出和少量 `.mp4/.py`。实际可读的 db3 是顶层 `ODM_SCAN_CAM_0-002.db3`（约 2.60 GB）。其 SQLite 话题与 `BLOC_1/RDC/ODM_SCAN_CAM` metadata 的计数相符，但缺少顶层 sidecar metadata，因此不把两者的身份关系当作独立证明。只有 `ADMINISTRATION/RDC/odom_scan_camera/csv_output/_tb2_scan.csv` 被发现为 LaserScan CSV 导出。

### mbarah

`indoor-outdoor-multisensor-robot-dataset/` 包含 indoor basic、indoor full、outdoor 三个 `rosbag/` 目录，各有 db3 和 `metadata.yaml`；同时有对应 CSV、README、轨迹脚本和图片。三个 db3 均为 ROS 2 SQLite3。

### MIT Stata

`rosbag_info_files/.../rosbag_info_files/` 有 82 个 ROS1 bag 信息文本，`ground_truth_poses/.../ground_truth/laser_scans/` 有 24 个 `.gt.laser.scans`，同层有 24 个 `.gt.laser.poses`。实际原始 bag 没有下载到当前目录，故不能用 `rosbag info` 读取消息或确认消息内 frame_id。

### PREDICT 6G

`slam-datasets-main/` 只有 `README.md`、`SLAMdataset.launch`、`demo_backpack_2d.launch`、`bags/bags` 下载说明、`mmwaveglasscorridor.csv`、图片和空占位目录。`bags/bags` 提到 1.bag 至 5.bag 以及 6.mmwavebag，但这些文件本地不存在。

## 3. 可直接解析的 ROS 2 bag 和 LaserScan 审计

逐条读取了四个本地 db3 中的 LaserScan CDR（ESTS 顶层 db3 通过 SQLite 只读读取；其余通过 ROS 2 消息类型反序列化）。有效点定义为 finite 且满足 `range_min <= range <= range_max`。NaN、Inf 和越界值分别统计；每帧角度字段存在设备级微小变化，因此 `field_variants` 是观测到的完整字段组合数量，而不是消息类型变化。

| 数据文件 | ROS2 话题/类型 | scan 数 | frame_id | 时间范围/时长 | 有效点均值（范围） | NaN / Inf / 越界 | 时间间隔均值 |
|---|---|---:|---|---:|---:|---:|---:|
| ESTS `ODM_SCAN_CAM_0-002.db3` | `/tb2/scan` `sensor_msgs/msg/LaserScan` | 3,539 | `rplidar_link` | 448.839 s | 505.6 (348–594)/720 | 0 / 717,879 / 40,983 | 0.1269 s |
| mbarah indoor basic | `/scan` `sensor_msgs/msg/LaserScan` | 397 | `laser` | 46.931 s | 777.8 (662–844)，约 1,111–1,113 beams | 0 / 132,760 / 0 | 0.1185 s |
| mbarah indoor full | `/scan` `sensor_msgs/msg/LaserScan` | 262 | `laser` | 47.939 s | 764.8 (653–828)，约 1,111–1,114 beams | 0 / 90,961 / 0 | 0.1837 s；19 个大间隔 |
| mbarah outdoor | `/scan` `sensor_msgs/msg/LaserScan` | 34,404 | `laser` | 3,237.019 s | 671.1 (0–1,071)，约 1,111–1,113 beams | 0 / 15,172,752 / 0 | 0.0941 s；140 个大间隔 |
| ESTS CSV 导出（仅一份） | CSV `_tb2_scan.csv`，应为 `/tb2/scan` 导出 | 807 | 未导出 | 386.557 s | 33.4 (2–51)/52 | 0 / 14,466 / 513 | 0.4796 s |

ROS 2 metadata 中还确认了 ESTS 七个 bag 的声明信息（实际 db3 缺失）：总消息分别为 9,943、10,926、25,983、47,872、5,364、28,992、17,963；scan 数分别为 807、1,539、3,539、6,573、479、3,889、2,472；声明时长约 238.356–941.186 s。除顶层独立 db3 外，这些目录不能在当前文件集上重新读取消息内容，因此没有为它们杜撰逐帧字段统计。

已实际确认的 LaserScan 常量/范围：

- ESTS：角度约 `[-3.12414, 3.14159]`，增量约 `0.00871451`，`range_min=0.15`、`range_max=12.0`，720 beams；`time_increment` 和 `scan_time` 随帧轻微变化。
- mbarah：角度约 `[-2.094395, 2.094395]`，增量约 `0.00377`，`range_min=0`、`range_max=200`，约 1,110–1,114 beams；basic/full 的 `time_increment`/`scan_time` 有零值和小幅变化。
- 以上数据存在 Inf，而没有观测到 NaN。预处理仍应按 `isfinite`、量程上下限和最小点数过滤，不能把 Inf 当作远距离有效回波。

### 其他传感器和时间/坐标证据

- ESTS 顶层 db3：`/tb2/odom` `nav_msgs/msg/Odometry` 8,973 条，RGB `/tb2/oakd/rgb/preview/image_raw` 13,471 条；没有 IMU、TF、TF_static 话题。metadata 声明的各 bag 同样主要是 scan、odom、RGB image。
- mbarah indoor basic：只有 `/scan` 397 和 `/odom` 956。
- mbarah indoor full：有 `/scan`、`/odom`、`/odom_combined`、`/imu/data`、`/imu/data_raw`、`/tf`、`/tf_static`、`/joint_states` 等；`tf_static` CSV 中实测 `base_footprint -> laser` 平移约 `(0.497, 0, 0.228)`，LaserScan frame 为 `laser`。
- mbarah outdoor：只有 `/scan` 和 `/odom`，没有 TF/IMU；其 LaserScan frame 为 `laser`，odom 的 frame/child 为 `odom_combined`/`base_footprint`。
- ESTS scan 的消息 header 时间戳与 bag 存储时间均连续到约 0.1 s 量级，但没有 TF，不能据此假定 scan 到 odom 的空间变换存在。
- mbarah 三个 bag 的 scan `header.stamp` 与 SQLite 存储时间存在明显基准差异：indoor basic 约 `-1.562e6 s`、indoor full 约 `-1.562e6 s`、outdoor 约 `-3.329e7 s`（header 起止区间仍各自连续）。这是回放/TF 同步前必须解决或明确处理的完整性风险，不能把 metadata 的时间范围直接当作 LaserScan header 时间范围。

## 4. MIT Stata 的 ROS1 元数据和参考轨迹

82 个 `.bag.info` 是 ROS1 bag 的 `rosbag info` 文本。抽样并批量解析确认：绝大多数包含 `/base_scan`、`/tilt_scan`（约 20 Hz）、`/base_odometry/odom`、`/robot_pose_ekf/odom_combined`、`/torso_lift_imu/data`、`/tf`；不同 bag 的相机话题有所不同。示例 `2012-05-03-13-01-04.bag.info` 声明 1,894 s、910,965 条消息、`/base_scan` 37,759 条，原始 bag 约 33.1 GB；这些只是 info 文件中的声明，原始 bag 不在本地。

24 个 `.gt.laser.poses` 每行是四列逗号分隔数据（微秒时间戳、x、y、yaw）；24 个 `.gt.laser.scans` 使用自定义文本格式，首行含时间戳、起始角、角增量和 0，后续行含时间戳、角度参数、束数及扫描值。它们与 MIT 官方提供的激光参考轨迹/扫描相符，但本目录没有对应原始 bag，且官方轨迹是由 floorplan、激光匹配和 pose graph/iSAM 生成的参考轨迹，不应直接称为独立外部真值。使用前必须确认 pose 与 scan 的时间单位/序列配对，并写专用解析器。

## 5. PREDICT 6G 的实际状态

本地没有任何 `.bag` 或 `.mmwavebag`，只能确认 README/下载清单声称包含 ROS1 LaserScan、odom、poseupdates、commands、trajectory、clock、TF。6 个条目中 4.bag 的描述最接近长玻璃走廊退化场景，6.mmwavebag 还含 mmWave，但当前不能确认真实 topic、消息数、frame_id、时间戳或真值来源。因此 PREDICT 6G 暂不算已获取实验数据，不能执行 `rosbag info` 或 Cartographer 回放。

## 6. Ground Truth 分类与完整性结论

| 数据集 | 证据类别 | 结论 |
|---|---|---|
| ESTS | 无独立 GT 证据；地图和 odom 可作为实验辅助 | 地图文件应视为 SLAM/实验产物，不能当外部真值；odom 的来源尚待数据集文档确认 |
| mbarah | 轮式 odometry；README/`trajectory.py` 由 odometry 生成轨迹 | 没有独立 GT 证据；full 的 IMU/TF 可用于复现实验，但不是 GT |
| MIT Stata | `.gt.laser.poses/.scans` 参考轨迹 | 有官方参考轨迹文件，但其生成流程包含激光匹配和 pose graph，标注为参考轨迹而非独立外部 GT |
| PREDICT 6G | 下载说明声称 trajectory/poseupdates | 本地没有数据，来源和精度待验证 |

完整性方面，所有本地文件均已列入外部 CSV 并计算 SHA-256；ROS 2 db3 可读取的文件通过 SQLite/ROS2 反序列化完成抽样到全量 scan 审计。ESTS 7 个 metadata 目录缺少引用的 db3，MIT 和 PREDICT 缺少原始 bag，这些是当前数据包不完整的明确证据。

## 7. 对 Cartographer 2D 的可用性

- **优先：mbarah indoor full/basic。** basic 有 scan+odom 但无 TF；若将 tracking frame 设为 `laser` 并关闭 IMU/odometry 使用，可以先做 scan-only 几何验证，但正式使用 odom 需要对应 TF。full 有 IMU、动态/静态 TF 和 odom，最适合先验证标准 Cartographer 传感器链路；仍需检查 bag 中 TF 在每个 scan 时刻可查询。
- **第二优先：mbarah outdoor。** scan 数量大、时长约 54 分钟、带 odom，但无 TF/IMU；适合独立几何指标和 odom 辅助分析，直接 Cartographer 回放前需补齐或明确 tracking frame 与 TF 语义，不能伪造 TF。
- **第三优先：ESTS 顶层 db3。** scan+odom+RGB 且有长走廊/楼层地图上下文，但 frame 为 `rplidar_link`、没有 TF；可先做 LaserScan/退化指标审计，Cartographer 回放前需确认 tracking frame 与传感器 frame 是否可直接相同。其余七个 metadata 目录必须先取得对应 db3。
- **暂缓：MIT Stata。** 数据量大且原始 bag 不在本地；先选择一个 `.bag.info` 与对应参考轨迹，核对原始下载、时间单位和转换工具，再做 ROS1→ROS2 转换评估。
- **暂缓：PREDICT 6G。** 仅有下载说明，不计入已获取数据；不得用 README 代替 bag 审计。

## 8. 预处理要求

1. 对所有 LaserScan 统一执行 finite/量程过滤，记录每帧有效点数、Inf/NaN/越界比例和时间间隔异常；保留原始时间戳。
2. ROS 2 bag 回放前逐个检查 metadata、QoS、scan frame 与 TF 覆盖区间；不以文件名推断话题名。
3. 对没有 TF 的数据，先使用 `tracking_frame` 与 scan frame 相同的只读链路验证，或明确提供真实标定/TF；不得伪造动态 TF、IMU 或 odom。
4. MIT 需要 ROS1 bag 本体、ROS1 `rosbag info`/转换工具和 `.gt.laser.*` 解析器；PREDICT 需要先下载并校验 bag 本体。
5. 退化检测输入应保留单帧和 Cartographer matcher 坐标系两种语义，不能把跨帧 odom 轨迹直接当作单帧点云。

## 9. 仓库外迁移方案（本阶段未执行）

目标根目录：`/home/ubuntu2204/slam_public_datasets/raw/`。建议映射：

| 当前目录 | 目标目录 |
|---|---|
| `数据集/ESTS_ROS_Dataset/` | `raw/ests_ros/` |
| `数据集/mbarah/` | `raw/mbarah/` |
| `数据集/mit stata/` | `raw/mit_stata/` |
| `数据集/predict 6G/` | `raw/predict6g/` |

执行迁移前应再次核对源文件数量/总字节数、目标目录同名文件和本 CSV 中的 SHA-256；先复制并逐文件校验，待用户确认后再考虑删除源目录。当前没有移动、复制、覆盖或删除任何数据。外部目录已有其他历史 manifest/目录，不能直接覆盖；目标子目录需由下一阶段先做冲突清单。

## 10. 推荐实验优先级

1. 用 mbarah indoor full 做 ROS 2 bag、TF、LaserScan 和退化指标的短时链路验证。
2. 用 mbarah indoor basic 做无 IMU/TF 条件下的 scan-only 对照，明确其限制。
3. 用 ESTS 顶层 db3 做走廊/楼层几何退化统计；取得缺失 db3 后再扩展序列。
4. 用 mbarah outdoor 做长时指标统计与时间中断鲁棒性分析。
5. 下载并校验 MIT 的一个小 bag，再把参考轨迹作为独立评估输入；最后再处理 PREDICT 6G 的实际 bag。

以上优先级是数据可获得性和可验证性的排序，不是对 SLAM 精度的预判。
