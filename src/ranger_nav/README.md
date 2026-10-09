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

**MID360 机身 / 双臂自身点云过滤：**

当前 Livox CustomMsg 非特征提取链路在 SPARK 预处理阶段剔除距离雷达原点
不超过 `preprocess.blind` 的点（当前为 0.5 m，按 XYZ 三维距离计算），
在里程计估计和 SAM 累积地图之前生效。双臂固定下垂时，先检查自身回波
是否落在该范围内；不要直接屏蔽整个后半圈或扩大盲区，以免丢失真实环境点。
`preprocess.blind_for_human_pilots` 仅用于其他雷达的特定处理分支，
不作用于当前 MID360 CustomMsg 链路。

旧版 SPARK 的距离判断存在 `&&` / `||` 优先级问题，X 或 Y 变化的近点
可能绕过距离过滤；更新后需重新构建 `spark_fast_lio` 并重启建图进程。
已有地图中的自身残影不会自动清除，需要重新建图或用原始雷达 / IMU bag
重新生成地图。机械臂姿态改变后应重新验证自身回波范围。

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

`pcl_localization_ros2` 必须先获得初始位姿才会开始配准并发布 `map -> odom`。
命令行可通过 `set_initial_pose:=true initial_pose_x:=... initial_pose_y:=...`
`initial_pose_yaw:=...` 传入，或在 `set_initial_pose:=false` 时发布 `/initialpose`。
Web 控制台使用地图点位提供该初始位姿。

默认语音“开始导航”通过 `krt_human_robot` 启动 3D Localization 模式。

此模式默认加载 `rviz/navigation_3dloc.rviz`，以 Orbit 斜视角显示
三维雷达地图（`/initial_map`，按高度着色）、实时点云
（`/cloud_registered_body`，白色）、机器人以及全局/局部导航路径。
无需 RGB 相机或相机与雷达标定。二维地图和代价地图默认关闭，
可在 Displays 中勾选 `2D Map`、`Global Costmap`、`Local Costmap`。
Views 面板中可选择保存的 `Navigation Top Down` 俯视视角，
方便使用 **2D Pose Estimate** 和 **Nav2 Goal**；切换 Type 为 Orbit 可恢复旋转观察。

通过 `rviz:=false` 关闭 RViz，或通过 `rviz_config:=/absolute/path/custom.rviz`
覆盖显示配置。新增配置后先执行 `colcon build --packages-select ranger_nav --symlink-install`
并加载 `install/setup.bash`。已有导航运行时，可单独打开新视图，无需重启导航：

```bash
rviz2 -d "$(ros2 pkg prefix ranger_nav)/share/ranger_nav/rviz/navigation_3dloc.rviz"
```

三维显示排查：

- 静态地图为空：检查 `ros2 topic info -v /initial_map`；RViz 订阅应为
  Reliable、Transient Local，以接收定位节点已经发布的地图。
- 实时点云或机器人为空：先完成初始定位，确认 `map -> odom -> camera_init -> body`
  TF 连通；Fixed Frame 保持 `map`。用 `ros2 topic hz /cloud_registered_body`
  检查数据流，实时点云订阅使用 Best Effort、Volatile。
- 地图不在视野内：选择地图上的点后使用 **Focus Camera**，或调整 Views 的
  Focal Point 和 Distance。路径在规划发生后才会出现。

```bash
ros2 run ranger_nav nav_tf_diagnostics
```

## 可选三维局部避障（完整机器人，室内平地）

`navigation_3dloc.launch.py` 新增 `use_voxel_obstacles`，默认 `false`。
开启后局部 costmap 使用 VoxelLayer，碰撞监控增加三维点云源；
全局规划仍使用原二维地图和 `/scan`。导航几何按机械臂下垂、
长 0.552 m、宽 0.55 m、高 1.50 m，双 costmap footprint 为
`x=±0.30, y=±0.295 m`（另有 Nav2 自身的 footprint_padding）。
关闭开关时完整恢复原配置。

本次构建使用工作区内的持久目录，避免 symlink-install 指向可被清理的 `/tmp`：

```bash
source /opt/ros/humble/setup.bash
colcon build --build-base build/voxel_nav --packages-select ranger_nav --symlink-install
source install/setup.bash
```

```bash
ros2 launch ranger_nav navigation_3dloc.launch.py \
  map:=$HOME/maps/map.yaml pcd_map_path:=$HOME/maps/scans.pcd \
  use_voxel_obstacles:=true
```

上述命令会启动真实导航及硬件节点，不要与已有导航重复运行。
未做静止验收前不要发送行驶目标。回退时退出该次 launch，
使用 `use_voxel_obstacles:=false` 重启。

参数集中在 `config/nav2_voxel_overrides.yaml`，运行时与原 Nav2 参数合并到
临时文件，正常退出删除；原文件不被覆盖。无需 OctoMap 或 RGB 相机。

数据流：`/cloud_registered_body` → `navigation_obstacle_cloud` →
`/navigation/obstacle_points`（标记、碰撞监控）及
`/navigation/clearing_points`（仅清除）。节点用原始时间戳的 TF 在
`base_footprint` 下过滤：障碍高度为离地 0.08～1.60 m，自身外廓内的回波剔除；
地面、高处有效回波保留给清除源。输出保留原 frame、stamp 和字段。
无效点不生成射线；TF 缺失、点云超过 0.5 秒或时间戳无效时丢弃帧并限频报警。
有效空点云仍作为新观测发布，但不会凭空清除未观测区域。

局部体素为 16 层、每层 0.20 m，odom 下 Z 范围 [-1.0, 2.2)，
水平分辨率保持 0.05 m，标记/清除距离分别为 2.5/3.0 m。
高度筛选与体素原点是不同坐标概念；现场须确认雷达原点、离地 1.60 m 范围
都在体素网格中，不能把 RViz 的 map 零平面直接当作地面。
VoxelLayer 标记阈值为 0，不使用局部 denoise 删除孤立栅格。
碰撞监控保留 `/scan` 并增加点云。Humble 1.1.20 的 Collision Monitor
会忽略超时来源，而不会自动停车，因此三维模式增加 `navigation_obstacle_guard`：
`/cmd_vel` → Collision Monitor → `/navigation/collision_cmd_vel` →
速度门控 → `/cmd_vel_safe`。点云、scan 或速度超过 0.5 秒未更新时，
门控通过 20 Hz 单调时钟定时器输出零速度；即使 ROS 时钟停止也继续检查。
观测时间戳与对应 TF 必须有效，数据恢复后仅放行新收到的速度命令。
这不替代底盘控制器自身的通信看门狗：门控进程被强制杀死时无法保证发送零速度。

`navigation_lidar_origin` 从当前 FAST-LIO 参数的 `mapping.extrinsic_T/R`
发布名义雷达到 IMU 变换。FAST-LIO 当前开启在线外参估计，估计值没有发布，
此静态原点不会跟随在线估计；启用时会提醒验证射线清除。
现有 URDF 的雷达/IMU 高度近似也没有自动改动，现场应对照实测地面检查。
预处理不会补足 MID360 的盲区（当前 FAST-LIO `preprocess.blind=0.5`），
不承诺检测低于 8 cm 的障碍，也不覆盖机械臂展开、坡道和楼梯。

### 检查与验收

```bash
ros2 topic hz /navigation/obstacle_points
ros2 topic info -v /navigation/clearing_points
ros2 run tf2_ros tf2_echo odom navigation_lidar_origin
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 param get /local_costmap/local_costmap plugins
```

RViz 可勾选 `Navigation Obstacle Points`、`Navigation Clearing Points` 和
`Local Costmap`，检查地面不标记、自身回波不标记、桌沿及高处物体进入代价地图。
原始体素消息在 `/local_costmap/voxel_grid`（`nav2_msgs/msg/VoxelGrid`）。
附加 `voxel_visualization:=true` 启动 Nav2 自带转换节点后，可勾选
`Local Obstacle Voxels`，显示 `/local_costmap/voxel_marked_cloud`。
方块用于表示已占据体素中心，显示边长为 5 cm，不代表实际 20 cm 的垂直层厚度。
移动物体离开后必须有后续有效射线穿过原位置，才会清除；没有回波不能保证消失。
静止检查通过后，再由现场人员进行低速绕行与停车验收。

自动测试使用仓库虚拟环境：

```bash
PYTHONPATH=src/ranger_nav:$PYTHONPATH \
  src/voice_assistant/.venv/bin/python -m pytest -q src/ranger_nav/test
```

真实 Nav2 节点隔离测试（不启动硬件，域 91、仅本机通信，速度输出为测试专用话题）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
env ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ROS_LOG_DIR=/tmp/krt-voxel-test-logs KRT_VOXEL_RUNTIME_TEST=1 \
  PYTHONPATH=src/ranger_nav:$PYTHONPATH \
  src/voice_assistant/.venv/bin/python -m pytest -q -s \
  src/ranger_nav/test/test_voxel_runtime.py
```

测试覆盖非零地面高度、高处障碍、射线清除、桌沿减速停车、TF 缺失和点云超时。
这些检查不代替实机传感器、外参和制动距离验收。

## 巡航控制

`waypoint_manager cruise` 提供独立的 Trigger 服务：
`<control-prefix>/pause`、`resume`、`cancel`、`status`，默认前缀为
`/ranger_nav/cruise`；Web 为每次巡航分配唯一前缀，避免控制其他进程。
服务回执表示请求已接收，`status` 返回 `running`、`pausing`、`paused`
或 `canceling`；`control pause` 命令会等待 `paused` 后才成功返回。

```bash
ros2 run ranger_nav waypoint_manager cruise --repeat 2 入口 走廊
# 另一个终端，使用相同的 ROS 环境和 control-prefix
ros2 run ranger_nav waypoint_manager control pause
ros2 run ranger_nav waypoint_manager control resume
ros2 run ranger_nav waypoint_manager control cancel
```

暂停会取消并等待当前导航目标终止，保留当前轮次，恢复时重发未完成的目标。
正在执行的点位 Routine 会完成后再暂停，不会在恢复时重放；取消则取消当前
导航或 Routine 并退出。控制服务失联、取消超时会报错，不以杀客户端代替停车确认。
进程退出后不保留恢复进度。

无硬件隔离验证：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
env ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ROS_LOG_DIR=/tmp/krt-cruise-test-logs KRT_CRUISE_RUNTIME_TEST=1 \
  PYTHONPATH=src/ranger_nav:$PYTHONPATH \
  src/voice_assistant/.venv/bin/python -m pytest -q src/ranger_nav/test/test_cruise_runtime.py
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
