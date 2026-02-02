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

## 测试程序

`device_test` 功能包用于测试外部设备（摄像头、麦克风、扬声器）。

### 编译

```bash
cd /home/nvidia/WorkSpace/KrtHumanRobot
colcon build --packages-select device_test --symlink-install
source install/setup.bash
```

### 设备列表

列出系统所有可用的外部设备：

```bash
ros2 run device_test device_list
```

### USB 摄像头测试

测试普通 USB 摄像头（排除 RealSense），显示预览窗口并发布图像到 ROS 话题：

```bash
# 默认使用 /dev/video0
ros2 run device_test usb_camera_test

# 指定设备ID和分辨率
ros2 run device_test usb_camera_test --ros-args -p device_id:=0 -p width:=1280 -p height:=720

# 使用 launch 文件
ros2 launch device_test usb_camera_test_launch.py device_id:=0
```

参数说明：
- `device_id`: 摄像头设备ID（/dev/videoX 中的 X，默认 0）
- `width`/`height`: 分辨率（默认 640x480）
- `fps`: 帧率（默认 30）
- `show_preview`: 是否显示预览窗口（默认 true）

### Intel RealSense 测试

测试 Intel RealSense 摄像头的 RGB、深度和红外流：

```bash
# 默认测试 RGB 和 深度流
ros2 run device_test realsense_test

# 启用红外流
ros2 run device_test realsense_test --ros-args -p enable_ir:=true

# 使用 launch 文件
ros2 launch device_test realsense_test_launch.py enable_rgb:=true enable_depth:=true enable_ir:=false
```

设备映射 (Intel RealSense D435 via V4L2):
| 设备 | 流类型 | 格式 |
|------|--------|------|
| `/dev/video2` | Depth | Z16 (16-bit) |
| `/dev/video4` | Infrared | GREY |
| `/dev/video6` | RGB | YUYV |

参数说明：
- `rgb_device_id`: RGB 流设备 ID（默认 6）
- `depth_device_id`: 深度流设备 ID（默认 2）
- `ir_device_id`: 红外流设备 ID（默认 4）
- `enable_rgb`: 启用 RGB 流（默认 true）
- `enable_depth`: 启用深度流（默认 true）
- `enable_ir`: 启用红外流（默认 false）
- `width`/`height`: 分辨率（默认 640x480）
- `show_preview`: 是否显示预览窗口（默认 true）

> **注意**: 
> - 此测试工具通过 V4L2 直接访问 RealSense，功能有限（深度流 Z16 格式需特殊处理）
> - 如需完整的 RealSense 功能（点云、IMU、深度对齐等），建议使用 realsense-ros 包：
> ```bash
> ros2 launch realsense2_camera rs_launch.py
> ```

### 麦克风测试

测试麦克风录音功能，按 Enter 开始录音，录音完成后自动播放。
默认使用 **Unitek Y-247A** (C-Media USB Audio Device)。

```bash
# 默认设备 (Unitek Y-247A)
ros2 run device_test mic_test

# 指定录音时长
ros2 run device_test mic_test --ros-args -p duration:=3

# 使用其他设备
ros2 run device_test mic_test --ros-args -p device:=alsa_input.usb-rockchip_rk3xxx_5c48d945b00a6e83-00.analog-stereo
```

参数说明：
- `device`: PulseAudio 设备名（默认 `alsa_input.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.mono-fallback`）
- `sample_rate`: 采样率 Hz（默认 44100）
- `channels`: 音频通道数（默认 1）
- `duration`: 录音时长秒（默认 5）

### 扬声器测试

测试扬声器播放功能，按 Enter 播放测试音频。
默认使用 **Unitek Y-247A** (C-Media USB Audio Device)。

```bash
# 默认设备 (Unitek Y-247A)
ros2 run device_test speaker_test

# 指定频率和音量
ros2 run device_test speaker_test --ros-args -p frequency:=1000.0 -p volume:=0.3

# 使用其他设备
ros2 run device_test speaker_test --ros-args -p device:=alsa_output.platform-3510000.hda.hdmi-stereo
```

参数说明：
- `device`: 输出设备名（默认 `alsa_output.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.analog-stereo`）
- `frequency`: 测试音频频率 Hz（默认 440）
- `test_duration`: 播放时长秒（默认 2.0）
- `volume`: 音量 0.0-1.0（默认 0.5）

测试选项：
- 按 Enter 或输入 `0`: 播放单音 (440Hz)
- 输入 `1`: 播放扫频 (200-2000Hz)
- 输入 `2`: 左右声道测试

### 音频设备列表

| 设备类型 | PulseAudio 名称 |
|----------|-----------------|
| Y-247A 麦克风 | `alsa_input.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.mono-fallback` |
| Y-247A 扬声器 | `alsa_output.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.analog-stereo` |
| rk3xxx 麦克风 | `alsa_input.usb-rockchip_rk3xxx_5c48d945b00a6e83-00.analog-stereo` |
| HDMI 输出 | `alsa_output.platform-3510000.hda.hdmi-stereo` |

### ROS 话题

| 节点 | 话题 | 消息类型 | 说明 |
|------|------|----------|------|
| usb_camera_test | `/usb_camera/image_raw` | `sensor_msgs/Image` | USB摄像头图像 |
| realsense_test | `/realsense/color/image_raw` | `sensor_msgs/Image` | RealSense RGB图像 |
| realsense_test | `/realsense/depth/image_raw` | `sensor_msgs/Image` | RealSense 深度图像 |
| realsense_test | `/realsense/infra/image_raw` | `sensor_msgs/Image` | RealSense 红外图像 |
| mic_test | `/mic/status` | `std_msgs/String` | 录音状态 |
| speaker_test | `/speaker/status` | `std_msgs/String` | 播放状态 |
| device_list | `/devices/list` | `std_msgs/String` | 设备列表 JSON |
