# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**KrtHumanRobot** is a ROS 2 workspace for a humanoid robot platform integrating multiple sensor systems and device testing capabilities. The project is designed for Jetson embedded platforms and includes custom device testing tools alongside third-party sensor drivers.

**Primary Language:** Mixed (C++ and Python)
**Build System:** Colcon (ROS 2)
**Python Version:** 3.10+

## Build Commands

### Full Workspace Build
```bash
cd /home/create/DataDisk/WorkSpace/CrtWorkSpace/KrtHumanRobot
colcon build --symlink-install
source install/setup.bash
```

### Build Specific Packages
```bash
# Device testing package (Python)
colcon build --packages-select device_test --symlink-install

# LiDAR driver (C++)
colcon build --packages-select livox_ros_driver2 --symlink-install

# RealSense camera (C++ - multiple packages)
colcon build --packages-select realsense2_camera realsense2_camera_msgs realsense2_description --symlink-install
```

**Note:** Use `--symlink-install` for faster Python development iteration.

### Testing
```bash
# Run tests for device_test package
colcon test --packages-select device_test
colcon test-result --verbose

# Tests include: flake8, pep257, copyright checks
```

## Architecture

### Package Structure

The workspace contains three main packages:

1. **device_test** (Custom Python Package)
   - Location: `src/device_test/`
   - Purpose: External device testing and validation
   - Type: ament_python
   - ~1,405 lines of Python code

2. **livox_ros_driver2** (Git Submodule)
   - Location: `src/livox_ros_driver2/`
   - Purpose: Livox MID-360 LiDAR driver
   - Type: ament_cmake
   - Repository: https://github.com/Livox-SDK/livox_ros_driver2.git

3. **realsense-ros** (Git Submodule)
   - Location: `src/realsense-ros/`
   - Purpose: Intel RealSense camera driver
   - Type: ament_cmake (3 sub-packages)
   - Repository: https://github.com/realsenseai/realsense-ros.git
   - **Critical:** Must use branch 4.56.4 (not latest)

### device_test Package Architecture

The device_test package provides ROS 2 nodes for testing external hardware:

**ROS 2 Nodes:**
- `usb_camera_test_node` - Tests standard USB cameras (excludes RealSense)
- `realsense_test_node` - Tests RealSense D435 via V4L2 (RGB, Depth, IR)
- `mic_test_node` - Tests microphone input with PulseAudio
- `speaker_test_node` - Tests speaker output with tone generation
- `device_list` - Enumerates all available devices

**Published Topics:**
- `/usb_camera/image_raw` (sensor_msgs/Image)
- `/realsense/color/image_raw` (sensor_msgs/Image)
- `/realsense/depth/image_raw` (sensor_msgs/Image)
- `/realsense/infra/image_raw` (sensor_msgs/Image)
- `/mic/status` (std_msgs/String)
- `/speaker/status` (std_msgs/String)
- `/devices/list` (std_msgs/String)

**Launch Files:**
- `usb_camera_test_launch.py`
- `realsense_test_launch.py`
- `mic_test_launch.py`
- `speaker_test_launch.py`
- `multi_camera_test_launch.py`
- `all_devices_test_launch.py`

### Hardware Integration

**Supported Sensors:**

1. **Livox MID-360 LiDAR**
   - 3D point cloud scanning
   - FAST_LIO compatible for SLAM
   - Configuration: `MID360_config.json`

2. **Intel RealSense D435**
   - RGB, Depth, Infrared streams
   - V4L2 device mapping:
     - `/dev/video2` - Depth (Z16 format)
     - `/dev/video4` - Infrared (GREY)
     - `/dev/video6` - RGB (YUYV)

3. **USB Cameras**
   - Generic V4L2 compatible webcams
   - Typically `/dev/video0`

4. **Audio Devices**
   - Default: Unitek Y-247A (C-Media USB Audio Device)
   - Microphone: `alsa_input.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.mono-fallback`
   - Speaker: `alsa_output.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.analog-stereo`

5. **DexHand021s** (Dexterous Hand)
   - Requires ZLG converter driver

## Common Development Tasks

### Running Device Tests

```bash
# List all available devices
ros2 run device_test device_list

# Test USB camera (default /dev/video0)
ros2 run device_test usb_camera_test
ros2 run device_test usb_camera_test --ros-args -p device_id:=0 -p width:=1280 -p height:=720

# Test RealSense (RGB + Depth)
ros2 run device_test realsense_test
ros2 run device_test realsense_test --ros-args -p enable_ir:=true

# Test microphone (5 second recording)
ros2 run device_test mic_test
ros2 run device_test mic_test --ros-args -p duration:=3

# Test speaker (440Hz tone)
ros2 run device_test speaker_test
ros2 run device_test speaker_test --ros-args -p frequency:=1000.0 -p volume:=0.3
```

### Launching Sensors

```bash
# RealSense camera (full functionality with point clouds, IMU, etc.)
ros2 launch realsense2_camera rs_launch.py

# Livox MID-360 LiDAR
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# Livox with RViz visualization
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

## Critical Notes

### RealSense Version Constraint
**IMPORTANT:** The realsense-ros package MUST use branch 4.56.4, not the latest version.

**Reason:** Jetson SDK includes librealsense2 version 2.56.4, which is incompatible with newer realsense-ros branches (require 2.57+).

```bash
# Correct submodule setup
git submodule add -b 4.56.4 https://github.com/realsenseai/realsense-ros.git
```

### Git Submodules
Two submodules must be initialized before building:

```bash
git submodule update --init --recursive
```

### Device Permissions
- User must be in `video` group for camera access
- PulseAudio required for audio device testing
- V4L2 device IDs may vary between systems

### Platform-Specific Considerations
- Optimized for Jetson embedded platform
- Some documentation paths reference `/home/nvidia/WorkSpace/KrtHumanRobot`
- Current working directory: `/home/create/DataDisk/WorkSpace/CrtWorkSpace/KrtHumanRobot`

### Build Order Dependencies
- `realsense2_camera_msgs` must be built before `realsense2_camera`
- Use `colcon build --symlink-install` for Python packages to avoid rebuilding on code changes

### RealSense Testing Limitations
The `realsense_test` node uses V4L2 direct access with limited functionality (depth stream Z16 format requires special handling). For full RealSense features (point clouds, IMU, depth alignment), use the official realsense-ros driver:

```bash
ros2 launch realsense2_camera rs_launch.py
```

## File Locations

**Key Configuration:**
- Main README: `README.md` (Chinese documentation)
- Python config: `pyproject.toml`
- Git submodules: `.gitmodules`

**Package Directories:**
- `src/device_test/` - Custom device testing package
- `src/livox_ros_driver2/` - LiDAR driver submodule
- `src/realsense-ros/` - RealSense driver submodule

**Build Artifacts (gitignored):**
- `build/` - Colcon build directory
- `install/` - Installation directory
- `log/` - Build and runtime logs

## References

**LiDAR:**
- [Livox ROS2 Driver](https://github.com/Livox-SDK/livox_ros_driver2)
- [FAST_LIO ROS2](https://github.com/hku-mars/FAST_LIO/tree/ROS2)

**RealSense:**
- [RealSense ROS Latest](https://github.com/realsenseai/realsense-ros)
- [RealSense ROS 4.56.4](https://github.com/realsenseai/realsense-ros/tree/r/4.56.4)
- [Jetson Installation Guide](https://github.com/realsenseai/librealsense/blob/master/doc/installation_jetson.md)

**DexHand:**
- [ZLG Converter Driver](https://manual.zlg.cn/web/#/146)
- [DexHand Documentation](https://dexrobot.feishu.cn/docx/ATs0dq9TAolpKpxXaZvcY8t7nZd)
