# ranger_nav

Ranger 底盘 + Livox MID360 双方案 3D 建图与 Nav2 导航集成包。

## 建图方案（两套并行）

| | 方式 B：纯里程计 | 方式 A：回环建图 |
|--|------------------|------------------|
| Launch | `mapping.launch.py` | `mapping_sam.launch.py` |
| LIO | `fast_lio` | `spark_fast_lio` |
| 回环 | 无 | `kiss_matcher_ros` |
| 底层保存 | `/map_save` → 临时 PCD | `/km_sam/save_dir` → 回环优化 PCD |
| 统一归档 | `~/maps/<时间戳>/cloud.pcd` + `map.yaml/map.pgm` | 同左 |

## TF 树

**方式 B（fast_lio）：**

```
map --(AMCL)--> odom --(静态)--> camera_init --(FAST-LIO)--> body --(URDF)--> base_footprint --> base_link
```

**方式 A（spark + SAM）：**

```
map --(SAM)--> base_sam
odom --(spark)--> body --(URDF)--> base_footprint --> base_link
```

雷达安装位置与底盘尺寸统一在 `urdf/ranger_mini.urdf.xacro` 中配置
（`lidar_x/y/z` 属性，当前为底盘中心前方 0.20 m、离地 0.30 m），
由 `robot_state_publisher` 发布 `body -> base_footprint -> base_link` TF，
**按实际测量值修改 xacro 即可，无需改 launch**。

Nav2 的长方形底盘碰撞模型在 `config/nav2_params.yaml` 的
`footprint` 参数中配置（local/global costmap 各一处，需保持一致）。

RViz 中添加 **RobotModel** 显示项（Description Topic 选
`/robot_description`）即可看到底盘与雷达模型。

## 使用流程

### 1. 建图

**方式 A：spark_fast_lio + KISS-Matcher-SAM（长走廊/大场景，需回环）**

```bash
ros2 launch ranger_nav mapping_sam.launch.py

# 另开终端，键盘遥控建图（尽量走回起点形成回环）
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 推荐通过 krt_human_robot 语音/行为树说“保存地图”。
# 保存后统一生成 ~/maps/<时间戳>/cloud.pcd、map.yaml、map.pgm。
```

底层 launch：`mapping_spark.launch.py`（spark + livox + 底盘）。SAM 订阅 `/odometry` + `/cloud_registered`（世界系）。
回环参数：`config/kiss_matcher_sam.yaml`；LIO 参数：`config/spark_fast_lio_mid360.yaml`。

**RViz 无点云 / Global Status: Error 排查：**

1. 确认 spark 在跑：`ros2 topic hz /odometry`、`ros2 topic hz /cloud_registered` 应有数据。
   - 若 `/livox/lidar` 是 CustomMsg 但 spark 只订阅 PointCloud2，需重新编译 spark_fast_lio
     （须链接 `livox_ros_driver2`）。
   - 若 spark 节点已退出，检查终端是否有 `Invalid visualization frame`——
     `common.visualization_frame` 必须是 `imu`/`lidar`/`base`，不能填 TF 名 `body`。
2. 确认 SAM 已初始化：终端应出现 `The first node comes. Initialization complete.`；
   `ros2 topic hz /km_sam/curr_scan` 应有输出。
3. RViz Fixed Frame 保持 `map`；若仍黑屏，选中 **Current scan** 点 **Focus Camera** 重置视角。
4. **Global map** 需遥控移动约 1 m（`keyframe_threshold`）后才会累积显示。

**方式 B：纯 FAST-LIO 里程计（小场景、快速验证）**

```bash
ros2 launch ranger_nav mapping.launch.py

# 另开终端，键盘遥控建图
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 推荐通过 krt_human_robot 语音/行为树说“保存地图”。
# 保存后统一生成 ~/maps/<时间戳>/cloud.pcd、map.yaml、map.pgm。
```

也可以直接 Ctrl+C 退出，FAST-LIO 会把累计点云保存到
`src/FAST_LIO_ROS2/PCD/scans.pcd`。

### 2. 地图保存结果

通过 `krt_human_robot` 执行“保存地图”后，两种 backend 都会归档成同一结构：

```text
~/maps/
  20260624_091530/
    cloud.pcd
    map.pgm
    map.yaml
    metadata.yaml
  map.pgm
  map.yaml
```

时间戳目录保留历史地图，不互相覆盖；根目录 `map.yaml/map.pgm`
始终更新为最新地图，供默认导航启动使用。
`--lidar-height` 必须与 urdf 中的 `lidar_z` 一致。

### 3. 2D AMCL 导航

```bash
ros2 launch ranger_nav navigation.launch.py map:=$HOME/maps/map.yaml
```

`map` 是 2D occupancy map yaml。该模式由 AMCL 发布 `map -> odom`。

在 RViz 中：

1. 用 **2D Pose Estimate** 设置机器人初始位姿（必须，AMCL 需要初值）；
2. 用 **Nav2 Goal** 发送导航目标点。

### 4. 3D Localization 导航

```bash
ros2 launch ranger_nav navigation_3dloc.launch.py \
  map:=$HOME/maps/map.yaml \
  pcd_map_path:=$HOME/maps/scans.pcd
```

`map` 是 2D occupancy map yaml；`pcd_map_path` 是 3D PCD map。
默认 3D 定位使用固定的 `$HOME/maps/scans.pcd`；时间戳目录里的
`cloud.pcd` 只是保存地图时的归档副本。
该模式由 `pcl_localization_ros2` 发布 `map -> odom`，不要同时启动
AMCL `navigation.launch.py`。

默认语音“开始导航”通过 `krt_human_robot` 启动 3D Localization 模式。

```bash
ros2 run ranger_nav nav_tf_diagnostics
```

## 地图去噪调参

地图或代价地图上出现杂乱孤立点时，按「现象 → 参数」对照调整。
改 `pcd2pgm` 参数只需重新生成地图；改 `nav2_params.yaml` / launch
需重启导航（无需重新编译，launch 修改后需重新 `colcon build`）。

### 静态地图杂点（pgm 上的孤立黑点）

方式 A（spark + SAM）的 PCD 密度由三处共同决定：

1. `config/spark_fast_lio_mid360.yaml` 的 `publish.dense_publish_en` 控制
   `/cloud_registered` 是否发布全量去畸变点云；建图保存推荐保持 `true`。
2. `config/kiss_matcher_sam.yaml` 的 `save_voxel_resolution` 控制回环优化地图
   保存体素；当前按 5 cm 栅格地图设为 `0.05`。
3. 外部 `pcd2pgm` 的 `thre_radius` / `thres_point_count` 控制 PCL 半径离群点滤波。

自动保存地图时，`krt_human_robot` 会启动外部 `pcd2pgm_node` 发布 `/map`，
再用 Nav2 `map_saver_cli` 生成标准 `map.pgm` / `map.yaml`。手动转换可用：

```bash
ros2 run pcd2pgm pcd2pgm_node --ros-args --params-file pcd2pgm.yaml
ros2 run nav2_map_server map_saver_cli -t map -f ~/maps/map --fmt pgm --mode trinary
```

`pcd2pgm.yaml` 的关键参数由 `krt_human_robot` 配置生成：
`pcd2pgm_resolution`、`pcd2pgm_z_min`、`pcd2pgm_z_max`、
`pcd2pgm_lidar_height`、`pcd2pgm_ror_radius`、`pcd2pgm_ror_min_pts`。

注意：建图时定位漂移产生的"重影墙"不是噪点，滤波救不了，
需要控制建图环境（避开行人、降低速度）重新建图。

### 运行时代价地图杂点（RViz 中实时出现的噪障碍）

参数在 `config/nav2_params.yaml`（local/global costmap 各一份，保持一致）：

| 参数 | 当前值 | 作用与调整方向 |
|------|--------|----------------|
| `denoise_layer.minimal_group_size` | 2 | 剔除小于 N 格的孤立障碍组（Nav2 官方椒盐噪点过滤层）。噪点仍多 → 调大；细小真实障碍被滤掉 → 调小或 `enabled: False` |
| `scan.obstacle_max_range` | 2.5 | 只在该距离内标记障碍，远处点云稀疏噪点多 → 调小 |
| `scan.raytrace_max_range` | 3.0 | 射线清除范围，可擦除移动物体残影，保持略大于 `obstacle_max_range` |

### 地面毛刺（地面不平被扫成障碍）

`launch/navigation.launch.py` 顶部常量：

| 常量 | 当前值 | 作用与调整方向 |
|------|--------|----------------|
| `SCAN_MIN_HEIGHT` | 0.15 | 点云转激光的切片下限（相对地面）。地面毛刺多 → 调大（0.20）；低矮障碍漏检 → 调小 |
| `SCAN_MAX_HEIGHT` | 1.2 | 切片上限，高于机器人通过高度的部分无需保留 |

## 依赖

- 方式 B：`fast_lio`、`livox_ros_driver2`、`agx_bringup`
- 方式 A：另需 `spark_fast_lio`、`kiss_matcher_ros`
- 导航：`ros-humble-nav2-bringup`、`ros-humble-pointcloud-to-laserscan`
