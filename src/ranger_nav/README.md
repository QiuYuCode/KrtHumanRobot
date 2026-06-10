# ranger_nav

Ranger 底盘 + Livox MID360 + FAST-LIO 的 3D 建图与 Nav2 导航集成包。

## TF 树

```
map --(AMCL)--> odom --(静态)--> camera_init --(FAST-LIO)--> body --(URDF)--> base_footprint --> base_link
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

```bash
ros2 launch ranger_nav mapping.launch.py

# 另开终端，键盘遥控建图
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 建图完成后保存 PCD（保存到 ~/maps/scans.pcd）
ros2 service call /map_save std_srvs/srv/Trigger
```

也可以直接 Ctrl+C 退出，FAST-LIO 会把累计点云保存到
`src/FAST_LIO_ROS2/PCD/scans.pcd`。

### 2. PCD 转 2D 栅格地图

```bash
ros2 run ranger_nav pcd2pgm --pcd ~/maps/scans.pcd --out ~/maps/map \
    --lidar-height 0.30 --z-min 0.15 --z-max 1.2 --resolution 0.05
```

生成 `~/maps/map.pgm` 和 `~/maps/map.yaml`。
`--lidar-height` 必须与 launch 中的 `LIDAR_Z` 一致。

### 3. 导航

```bash
ros2 launch ranger_nav navigation.launch.py map:=$HOME/maps/map.yaml
```

在 RViz 中：

1. 用 **2D Pose Estimate** 设置机器人初始位姿（必须，AMCL 需要初值）；
2. 用 **Nav2 Goal** 发送导航目标点。

## 依赖

- 已编译：`fast_lio`、`livox_ros_driver2`、`agx_bringup`
- 系统包：`ros-humble-nav2-bringup`、`ros-humble-pointcloud-to-laserscan`
