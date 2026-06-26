# 人形机器人组件环境安装说明

## 一、MID 360 雷达

### 参考资料

1. [览沃 ros2 驱动程序](https://github.com/Livox-SDK/livox_ros_driver2)
2. [FAST_LIO](https://github.com/hku-mars/FAST_LIO/tree/ROS2?tab=readme-ov-file)

## 二、双目相机 realsens

**需要将代码切换到 4.56.4 分支版本进行编译安装，否则会报错**
原因是通过 jsetson 安装的 sdk 版本低于 realsense-ros 最新分支要求的 2.57
系统默认的 sdk 版本是 2.56.4 

```bash
git submodule add -b 4.56.4 https://github.com/realsenseai/realsense-ros.git
```

### 参考资料

1. [RealSense ros 最新分支代码](https://github.com/realsenseai/realsense-ros)
2. [RealSense ros 4.56.4分支代码](https://github.com/realsenseai/realsense-ros/tree/r/4.56.4)
3. [Jetson 安装 realsense sdk](https://github.com/realsenseai/librealsense/blob/master/doc/installation_jetson.md)

## 三、灵巧智能 DexHand021s

### 参考资料

1. [zlg 转换器驱动下载地址](https://manual.zlg.cn/web/#/146)
2. [DexHand 官方文档](https://dexrobot.feishu.cn/docx/ATs0dq9TAolpKpxXaZvcY8t7nZd)

## 四、CAN 设备绑定规则（udev + bitrate 初始化）

该工作通过组合两部分实现“固定 CAN 接口命名 + 自动设置 CAN bitrate”：

1. `/etc/udev/rules.d/99-can-names.rules`：udev 在内核新增网络接口时匹配设备并重命名
2. `/usr/local/sbin/can-link-up.sh`：在接口出现后执行 `ip link` 完成 down/up 与 bitrate 配置

### 1）udev 规则做什么

`99-can-names.rules` 针对 `ACTION=="add"` 且 `SUBSYSTEM=="net"`、`KERNEL=="can*"` 的网卡事件生效。
规则进一步用 `ENV{ID_SERIAL_SHORT}` 区分不同 CAN 适配器，并将其重命名为固定接口名（例如 `can_chassis` / `can_right` / `can_left`）。

同时，udev 会在匹配成功时触发：

`RUN+="/usr/local/sbin/can-link-up.sh <接口名> <bitrate>"`

从而在接口创建时就完成链路初始化。

### 2）`can-link-up.sh` 做什么

脚本接收两个参数：
- 第 1 个参数：要初始化的接口名（例如 `can_chassis`）
- 第 2 个参数：CAN bitrate（例如 `500000`）

脚本内容如下：

```bash
#!/usr/bin/env bash
set -euo pipefail
IFACE="${1:?missing interface}"
BITRATE="${2:?missing bitrate}"
IP_BIN="$(command -v ip)"
# udev时序下接口可能刚创建，短重试提高成功率
for _ in {1..15}; do
  if "$IP_BIN" link show "$IFACE" >/dev/null 2>&1; then
    "$IP_BIN" link set "$IFACE" down 2>/dev/null || true
    "$IP_BIN" link set "$IFACE" up type can bitrate "$BITRATE"
    exit 0
  fi
  sleep 0.2
done
exit 1
```

脚本内部会最多重试 15 次（每次间隔 `0.2s`）等待接口真正可见（`ip link show <iface>` 成功）。
一旦接口出现，它会按顺序执行：

1. `ip link set <iface> down`
2. `ip link set <iface> up type can bitrate <bitrate>`

这样可避免 udev 触发时接口尚未完全建立导致的失败。

### 3）整体效果

插入/重启 CAN 适配器后，系统会自动把不同物理设备绑定为稳定的 CAN 接口名，并自动应用对应的 bitrate，减少手动配置和接口名漂移问题。

## 五、左右手 USB 摄像头

`hand_camera_driver` 发布左右手中心 USB 摄像头图像，默认依赖稳定设备别名：

- 左手：`/dev/camera_left`
- 右手：`/dev/camera_right`

### 编译

```bash
cd /home/nvidia/WorkSpace/KrtHumanRobot
colcon build --packages-select hand_camera_driver --symlink-install
source install/setup.bash
```

### 启动

```bash
ros2 launch hand_camera_driver hand_cameras.launch.py
```

需要临时覆盖设备或分辨率时：

```bash
ros2 launch hand_camera_driver hand_cameras.launch.py \
  left_device:=/dev/video0 right_device:=/dev/video2 width:=640 height:=480 fps:=30.0
```

### ROS 话题

| 节点 | 话题 | 消息类型 | 说明 |
|------|------|----------|------|
| left_hand_camera | `/left_gripper/image_raw` | `sensor_msgs/Image` | 左手 USB 摄像头图像 |
| right_hand_camera | `/right_gripper/image_raw` | `sensor_msgs/Image` | 右手 USB 摄像头图像 |

## 六、巡航点打点与任务配置

`ranger_nav` 的 `waypoint_manager mark` 会把机器人当前位姿保存成巡航点，默认写入：

```bash
~/maps/waypoints.yaml
```

打点前需已启动导航/定位，并确保 TF 中有 `map -> base_footprint`。

### 基础打点

```bash
source install/setup.bash
ros2 run ranger_nav waypoint_manager mark 前台
```

点位名必须唯一。未指定任务时默认 `task: wait`。

### 带任务打点

等待 3 秒：

```bash
ros2 run ranger_nav waypoint_manager mark 前台等待 \
  --task wait --args '{"wait_ms":3000}'
```

到点拍照保存：

```bash
ros2 run ranger_nav waypoint_manager mark 前台拍照 --task photo
```

到点拍照、视觉理解并播报：

```bash
ros2 run ranger_nav waypoint_manager mark 前台讲解 \
  --task describe \
  --args '{"camera_id":"head","question":"请描述这个巡航点看到的内容"}'
```

`describe` 的 `camera_id` 可选：`head`、`left_palm`、`right_palm`。

### 常用管理命令

```bash
ros2 run ranger_nav waypoint_manager list
ros2 run ranger_nav waypoint_manager remove 前台
ros2 run ranger_nav waypoint_manager clear
ros2 run ranger_nav waypoint_manager cruise --loop
```

同一点想“先等待再拍照/讲解”时，保存两个同位姿点即可：第一个 `wait`，第二个 `photo` 或 `describe`，并保持 YAML 顺序。

## 七、3D Localization Mode with QiuYuCode/lidar_localization_ros2

新增 3D localization 导航模式，保留原 AMCL `navigation.launch.py` 不变。

- `map`：2D occupancy map yaml，供 Nav2 `map_server` 使用。
- `pcd_map_path`：3D PCD map，供 `pcl_localization_ros2` NDT/GICP 配准使用。
- AMCL 模式和 3D localization 模式不能同时启动；两者都会发布 `map -> odom`。

依赖导入：

```bash
./scripts/import_3dloc_deps.sh
```

脚本要求 `QiuYuCode/lidar_localization_ros2` 已存在远程分支
`krt-nav2-map-odom`；不存在会直接停止，不回退到 `humble`。

构建：

```bash
cd /home/nvidia/WorkSpace/KrtHumanRobot
colcon build --symlink-install --packages-up-to ranger_nav pcl_localization_ros2
source install/setup.bash
ros2 pkg prefix ranger_nav
ros2 pkg prefix pcl_localization_ros2
```

启动：

```bash
ros2 launch ranger_nav navigation_3dloc.launch.py \
  map:=$HOME/maps/map.yaml \
  pcd_map_path:=$HOME/maps/scans.pcd
```

如果要通过命令行给初始位姿，而不是在 RViz 里点 **2D Pose Estimate**：

```bash
ros2 launch ranger_nav navigation_3dloc.launch.py \
  map:=$HOME/maps/map.yaml \
  pcd_map_path:=$HOME/maps/cloud.pcd \
  set_initial_pose:=true \
  initial_pose_x:=0.0 \
  initial_pose_y:=0.0 \
  initial_pose_yaw:=0.0
```

单独查看 3D localizer wrapper 参数：

```bash
ros2 launch ranger_nav lidar_localization_ros2_krt.launch.py --show-args
```

运行时诊断：

```bash
ros2 run ranger_nav nav_tf_diagnostics
```

验证 3D 定位是否生效：

```bash
ros2 node list | rg 'amcl|pcl_localization'
ros2 topic echo /alignment_status --once
ros2 topic hz /pcl_pose
ros2 topic hz /cloud_registered_body
ros2 run tf2_ros tf2_echo map odom
ros2 run ranger_nav nav_tf_diagnostics
```

期望结果：

- 有 `/pcl_localization`，没有 `/amcl`。
- `/alignment_status` 为 `data: true`。
- `/pcl_pose` 和 `/cloud_registered_body` 有稳定频率，实测约 10 Hz。
- `map -> odom` 持续输出。
- `nav_tf_diagnostics` 全部 `OK`。

RViz 显示：

- 默认看到的 `Map` 是 2D occupancy map，这是 Nav2 规划用地图。
- 要看 3D PCD 地图，添加 `PointCloud2`，Topic 选 `/initial_map`。
- 要看实时点云，添加 `PointCloud2`，Topic 选 `/cloud_registered_body`。
- 要看路线，添加 `Path`，Topic 可选 `/plan`、`/plan_smoothed`、`/local_plan`。
- Fixed Frame 使用 `map`。

注意事项：

- 不要同时启动原 AMCL `navigation.launch.py` 和 3D localization `navigation_3dloc.launch.py`，两者都会发布 `map -> odom`。
- 当前路线仍是 Nav2 的 2D 路线；3D PCD 地图只用于定位配准，不会生成真正的 3D 路线。
- `/alignment_status` 为 `false` 时，优先检查初始位姿是否偏太远、`pcd_map_path` 是否匹配当前环境、`/cloud_registered_body` 是否有数据。
- 如果 `ros2 launch --show-args` 在受限环境报 `~/.ros/log` 只读，可临时加 `ROS_LOG_DIR=/tmp/ros-log-check`。

### 已遇到的编译问题

PCL 查找 VTK/MPI 时可能报错：

```text
Imported target "MPI::MPI_C" includes non-existent path
.../src/ndt_omp_ros2/-I/usr/lib/aarch64-linux-gnu/openmpi/include
```

原因是 CMake/MPI 探测把 OpenMPI 输出的 `-I/usr/lib/.../openmpi/include`
当成普通路径拼进了源码目录。

清 CMake 缓存并显式传 MPI 路径可通过编译：

```bash
colcon build --packages-select ndt_omp_ros2 pcl_localization_ros2 \
  --symlink-install \
  --cmake-clean-cache \
  --cmake-args \
  -DMPI_C_INCLUDE_DIRS=/usr/lib/aarch64-linux-gnu/openmpi/include \
  -DMPI_C_LIBRARIES=/usr/lib/aarch64-linux-gnu/libmpi.so \
  -DMPI_CXX_INCLUDE_DIRS=/usr/lib/aarch64-linux-gnu/openmpi/include \
  -DMPI_CXX_LIBRARIES=/usr/lib/aarch64-linux-gnu/libmpi_cxx.so
```

另外，`ndt_omp_ros2` 原始 CMake 默认编译 `apps/align.cpp` 示例，会额外拉入
PCL visualizer/VTK/MPI。当前仅安装并导出 `ndt_omp` 库，定位节点不需要该示例程序。
