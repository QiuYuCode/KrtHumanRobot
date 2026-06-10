# ranger_nav

Ranger 底盘 + Livox MID360 + FAST-LIO 的 3D 建图与 Nav2 导航集成包。

## TF 树

```
map --(AMCL)--> odom --(静态)--> camera_init --(FAST-LIO)--> body --(静态)--> base_footprint
```

雷达安装位置在 `launch/mapping.launch.py` 与 `launch/navigation.launch.py`
顶部的 `LIDAR_X/Y/Z` 常量中配置（默认雷达位于底盘中心正上方 0.40 m），
**两个文件务必保持一致，并按实际测量值修改**。

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
    --lidar-height 0.40 --z-min 0.15 --z-max 1.2 --resolution 0.05
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
