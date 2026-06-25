# AGENTS.md

This file provides guidance to agentic coding agents working with code in this repository.

## Project Overview

**KrtHumanRobot** is a ROS 2 workspace for humanoid robot platform integration with multiple sensor systems. The project uses mixed C++ and Python with Colcon build system.

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

**Important:** Use `--symlink-install` for faster Python development iteration.

### Lint and Test Commands

#### Python (hand_camera_driver package)
```bash
# Run linting
colcon test --packages-select hand_camera_driver --event-handlers console_direct+

# Run specific lint tests
colcon test --packages-select hand_camera_driver --ctest-args -R test_flake8
colcon test --packages-select hand_camera_driver --ctest-args -R test_pep257
colcon test --packages-select hand_camera_driver --ctest-args -R test_copyright

# View test results
colcon test-result --verbose
```

#### C++ (realsense2_camera, livox_ros_driver2)
```bash
# Build with warnings (enabled in CMakeLists.txt)
colcon build --packages-select realsense2_camera --symlink-install

# Run any available tests
colcon test --packages-select realsense2_camera
colcon test --packages-select livox_ros_driver2
```

## Code Style Guidelines

### Python (hand_camera_driver package)

#### Imports and Formatting
- Follow PEP 8 style (enforced by flake8)
- Use standard ROS 2 imports: `rclpy`, `sensor_msgs`, `std_msgs`, `cv_bridge`
- Import order: standard library, third-party, ROS 2, local modules
- Maximum line length: 88 characters (flake8 default)

#### Naming Conventions
- Classes: `PascalCase` (e.g., `USBCameraTestNode`)
- Functions/variables: `snake_case` (e.g., `device_id`, `init_camera()`)
- Constants: `UPPER_SNAKE_CASE`
- Private members: prefix with underscore (e.g., `_init_camera()`)

#### ROS 2 Patterns
- Inherit from `rclpy.node.Node` for all ROS nodes
- Declare parameters in `__init__()` using `self.declare_parameter()`
- Use `self.get_parameter().value` to retrieve parameter values
- Create publishers with `self.create_publisher()` and QoS profile of 10
- Use `CvBridge` for OpenCV-ROS image conversion
- Include proper error handling for device initialization

#### Documentation
- Use docstrings for all classes and public methods
- Chinese comments acceptable for device-specific descriptions
- Include parameter descriptions in docstrings

#### Error Handling
- Use try-except blocks for device operations (camera, audio)
- Log errors with `self.get_logger().error()`
- Handle device disconnection gracefully
- Check return values from OpenCV operations

### C++ (realsense2_camera, livox_ros_driver2)

#### Standards and Formatting
- C++14 standard (configured in CMakeLists.txt)
- Follow Google C++ style guide
- Use clang-format if available
- Compiler warnings: `-Wall -Wextra -Wpedantic`

#### Naming Conventions
- Classes: `PascalCase` (e.g., `BaseRealSenseNode`)
- Functions/variables: `snake_case` (e.g., `init_camera()`)
- Member variables: trailing underscore (e.g., `device_id_`)
- Constants: `kPascalCase` or `UPPER_SNAKE_CASE`
- Namespaces: `lowercase`

#### ROS 2 Patterns
- Use `rclcpp::Node` as base class for nodes
- Smart pointers for ROS objects: `rclcpp::Publisher::SharedPtr`
- Use `std::mutex` for thread safety
- Prefer `RCLCPP_INFO`, `RCLCPP_ERROR` macros for logging
- Include proper header guards

#### Memory Management
- Use RAII principles
- Smart pointers over raw pointers
- Proper cleanup in destructors
- Avoid memory leaks in long-running processes

## Testing Guidelines

### Running Single Tests
```bash
# Python specific test
colcon test --packages-select hand_camera_driver --ctest-args -R test_flake8

# C++ specific test (if available)
colcon test --packages-select realsense2_camera --ctest-args -R specific_test_name
```

### Test Structure
- Python tests use `pytest` framework
- Test files named `test_*.py` in `test/` directory
- C++ tests use gtest framework
- All tests must pass before merge

## Commit Message Guidelines

- Use Conventional Commits: `<type>[optional scope]: <中文描述>`.
- Commit messages must be written in Chinese.
- Use `!` and a `BREAKING CHANGE:` footer when removing packages, public APIs, topics, launch files, or executables.
- Examples:
  - `feat(camera): 新增左右手摄像头驱动`
  - `fix(nav): 修复代价地图参数`
  - `feat(camera)!: 移除旧设备测试包`

## Critical Development Notes

### RealSense Version Constraint
**CRITICAL:** The realsense-ros submodule MUST use branch 4.56.4, not latest.
Reason: Jetson SDK includes librealsense2 version 2.56.4, incompatible with newer realsense-ros branches.

### Git Submodules
Initialize before building:
```bash
git submodule update --init --recursive
```

### Device Permissions
- User must be in `video` group for camera access
- PulseAudio required for audio device testing
- V4L2 device IDs may vary between systems

### Build Order
- `realsense2_camera_msgs` before `realsense2_camera`
- Use `colcon build --symlink-install` for Python packages

## Package-Specific Guidelines

### hand_camera_driver (Python)
- Location: `src/hand_camera_driver/`
- Type: `ament_python`
- Purpose: Left/right hand USB camera image publishers
- Key files: `usb_camera_node.py`, `hand_cameras.launch.py`

### realsense2_camera (C++)
- Location: `src/realsense-ros/realsense2_camera/`
- Type: `ament_cmake`
- Purpose: Intel RealSense camera driver
- Branch constraint: 4.56.4

### livox_ros_driver2 (C++)
- Location: `src/livox_ros_driver2/`
- Type: `ament_cmake`
- Purpose: Livox MID-360 LiDAR driver
- Config: `MID360_config.json`

## Common Debugging Commands

```bash
# Check device permissions
ls -l /dev/video*
ls -l /dev/camera_left /dev/camera_right
groups $USER

# Test individual nodes
ros2 launch hand_camera_driver hand_cameras.launch.py

# Check topic data
ros2 topic echo /left_gripper/image_raw --once --field header.frame_id
ros2 topic hz /left_gripper/image_raw
ros2 topic hz /right_gripper/image_raw

# View node graph
ros2 run rqt_graph rqt_graph
```
