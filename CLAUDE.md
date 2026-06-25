# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**KrtHumanRobot** is a ROS 2 workspace for a humanoid robot platform integrating multiple sensor systems. The project is designed for Jetson embedded platforms and includes hand USB camera nodes alongside third-party sensor drivers.

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
# Hand USB cameras (Python)
colcon build --packages-select hand_camera_driver --symlink-install

# LiDAR driver (C++)
colcon build --packages-select livox_ros_driver2 --symlink-install

# RealSense camera (C++ - multiple packages)
colcon build --packages-select realsense2_camera realsense2_camera_msgs realsense2_description --symlink-install
```

**Note:** Use `--symlink-install` for faster Python development iteration.

### Testing
```bash
# Run tests for hand camera package
colcon test --packages-select hand_camera_driver
colcon test-result --verbose

# Tests include: flake8, pep257, copyright checks
```

## Architecture

### Package Structure

The workspace contains these main sensor packages:

1. **hand_camera_driver** (Custom Python Package)
   - Location: `src/hand_camera_driver/`
   - Purpose: Left/right hand USB camera image publishers
   - Type: ament_python

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

### hand_camera_driver Package Architecture

The hand_camera_driver package publishes the two hand USB camera streams:

**ROS 2 Nodes:**
- `left_hand_camera` - Publishes `/dev/camera_left`
- `right_hand_camera` - Publishes `/dev/camera_right`

**Published Topics:**
- `/left_gripper/image_raw` (sensor_msgs/Image)
- `/right_gripper/image_raw` (sensor_msgs/Image)

**Launch Files:**
- `hand_cameras.launch.py`

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
   - Left hand: `/dev/camera_left`
   - Right hand: `/dev/camera_right`

4. **Audio Devices**
   - Default: Unitek Y-247A (C-Media USB Audio Device)
   - Microphone: `alsa_input.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.mono-fallback`
   - Speaker: `alsa_output.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.analog-stereo`

5. **DexHand021s** (Dexterous Hand)
   - Requires ZLG converter driver

## Common Development Tasks

### Launching Hand Cameras

```bash
ros2 launch hand_camera_driver hand_cameras.launch.py
ros2 topic hz /left_gripper/image_raw
ros2 topic hz /right_gripper/image_raw
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
- Hand cameras should be bound to stable udev aliases:
  `/dev/camera_left` and `/dev/camera_right`

### Platform-Specific Considerations
- Optimized for Jetson embedded platform
- Some documentation paths reference `/home/nvidia/WorkSpace/KrtHumanRobot`
- Current working directory: `/home/create/DataDisk/WorkSpace/CrtWorkSpace/KrtHumanRobot`

### Build Order Dependencies
- `realsense2_camera_msgs` must be built before `realsense2_camera`
- Use `colcon build --symlink-install` for Python packages to avoid rebuilding on code changes

## File Locations

**Key Configuration:**
- Main README: `README.md` (Chinese documentation)
- Python config: `pyproject.toml`
- Git submodules: `.gitmodules`

**Package Directories:**
- `src/hand_camera_driver/` - Left/right hand USB camera driver
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
