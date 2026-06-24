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

### 3. 导航

```bash
ros2 launch ranger_nav navigation.launch.py map:=$HOME/maps/map.yaml
```

在 RViz 中：

1. 用 **2D Pose Estimate** 设置机器人初始位姿（必须，AMCL 需要初值）；
2. 用 **Nav2 Goal** 发送导航目标点。

## 地图去噪调参

地图或代价地图上出现杂乱孤立点时，按「现象 → 参数」对照调整。
改 `pcd2pgm` 参数只需重新生成地图；改 `nav2_params.yaml` / launch
需重启导航（无需重新编译，launch 修改后需重新 `colcon build`）。

### 静态地图杂点（pgm 上的孤立黑点）

参数都在 `pcd2pgm` 命令行，推荐起步命令：

```bash
ros2 run ranger_nav pcd2pgm --pcd ~/maps/scans.pcd --out ~/maps/map \
    --lidar-height 0.30 --occ-thresh 3 --min-blob 4
```

| 参数 | 默认 | 作用与调整方向 |
|------|------|----------------|
| `--occ-thresh` | 2 | 栅格内点数达到该值才视为占据。孤立点多 → 调大（3~5）；细小真实障碍丢失 → 调小 |
| `--min-blob` | 3 | 剔除面积小于 N 格的孤立占据块（4 连通域分析）。噪团偏大 → 调大（4~8）；0 关闭 |
| `--ror-radius` / `--ror-min-pts` | 0（关）/ 5 | 3D 半径离群点滤波：半径 R 内邻居少于 K 的点被丢弃。成片稀疏虚假障碍（玻璃反射、动态物体轨迹）→ 试 `--ror-radius 0.3 --ror-min-pts 5` |
| `--z-min` | 0.15 | 障碍切片下限（相对地面）。地面被误判为障碍 → 调大（0.2）；低矮障碍漏检 → 调小 |

每步处理会打印剔除的点数/格数，按输出逐项调参。
注意：建图时定位漂移产生的"重影墙"不是噪点，滤波救不了，
需要控制建图环境（避开行人、降低速度）重新建图。

### 运行时代价地图杂点（RViz 中实时出现的噪障碍）

参数在 `config/nav2_params.yaml`（local/global costmap 各一份，保持一致）：

| 参数 | 当前值 | 作用与调整方向 |
|------|--------|----------------|
| `denoise_layer.minimal_group_size` | 2 | 剔除小于 N 格的孤立障碍组（Nav2 官方椒盐噪点过滤层）。噪点仍多 → 调大；细小真实障碍被滤掉 → 调小或 `enabled: False` |
| `voxel_layer.mark_threshold` | 1 | 体素列内需超过该数量的体素命中才标记占据。噪点多 → 2；灵敏度不足 → 0 |
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
