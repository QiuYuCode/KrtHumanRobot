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
