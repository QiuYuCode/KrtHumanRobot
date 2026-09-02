# AGENTS.md

本文件约束在本仓库中工作的编码代理。以仓库现状和就近文档为准；修改子包前，先阅读该子包的 `README.md`。

## 项目概览

KrtHumanRobot 是面向人形机器人平台的 ROS 2 Humble 工作区，使用 Python 3.10+、C++ 和 Colcon，覆盖传感器、底盘导航、机械臂、灵巧手、语音及双机部署。

- 工作区：`/home/create/WorkSpace/KrtHumanRobot`
- ROS 发行版：Humble（Ubuntu 22.04）
- Python 包：`ament_python`，使用 `rclpy`
- C++ 包：`ament_cmake`，使用 C++14 和 `rclcpp`
- Python 开发构建始终使用 `--symlink-install`

## 开始工作

```bash
cd /home/create/WorkSpace/KrtHumanRobot
git submodule update --init --recursive
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

优先构建受影响的包及其依赖，避免无必要的全工作区构建：

```bash
colcon build --packages-select <package_name> --symlink-install
colcon build --packages-up-to <package_name> --symlink-install
```

常用示例：

```bash
colcon build --packages-select hand_camera_driver --symlink-install
colcon build --packages-select hands_control_interfaces hands_control --symlink-install
colcon build --packages-select realsense2_camera_msgs realsense2_description realsense2_camera --symlink-install
colcon build --packages-up-to ranger_nav pcl_localization_ros2 --symlink-install
```

## 验证

修改后运行覆盖改动的最小验证；合并前相关测试必须通过。

```bash
colcon test --packages-select <package_name> --event-handlers console_direct+
colcon test-result --verbose
```

单个 Python 测试可直接运行：

```bash
src/voice_assistant/.venv/bin/python -m pytest -q src/<package_name>/test/test_<name>.py
```

仓库的 Python 测试环境位于 `src/voice_assistant/.venv`，由 uv 创建且已就绪；
不要依赖系统全局的 `pytest`，测试时优先使用该虚拟环境。

仅运行 ament lint：

```bash
colcon test --packages-select hand_camera_driver --ctest-args -R 'test_(flake8|pep257|copyright)'
```

涉及 launch、DDS、设备或多节点交互时，除单元测试外还应运行对应 launch/CLI 冒烟验证。硬件不可用时明确说明未执行的验证，不要伪造结果。

## 代码规范

### 通用

- 先复用仓库已有模式，避免新增无必要的抽象、依赖或配置层。
- 只修改任务相关文件；不要覆盖用户的未提交改动。
- 不直接编辑 `build/`、`install/` 或 `log/` 中的生成文件。
- 修改 ROS 接口、话题、参数、frame 或 launch 参数时，同步更新调用方、配置和文档。
- 传感器流优先采用 `sensor_data` QoS；其他通信根据可靠性和时延需求显式选择，避免随意固定 depth。
- 硬件访问必须检查返回值、记录可诊断错误并安全处理断连。

### Python

- 遵循 PEP 8；导入顺序为标准库、第三方、ROS 2、本地模块。
- 类使用 `PascalCase`，函数和变量使用 `snake_case`，常量使用 `UPPER_SNAKE_CASE`。
- ROS 节点继承 `rclpy.node.Node`；参数在构造阶段声明并校验。
- 使用 `get_logger()` 记录日志，不使用 `print()` 代替节点日志。
- 设备和外部服务操作保留必要的异常处理；节点退出时释放相机、音频、CAN 等资源。
- 测试位于包内 `test/test_*.py`，使用 pytest/ament pytest。

### C++

- 使用 C++14、RAII 和智能指针，遵循现有包的格式。
- 编译警告保持 `-Wall -Wextra -Wpedantic`。
- 类使用 `PascalCase`，函数和变量使用 `snake_case`，成员变量使用尾下划线。
- 使用 `RCLCPP_*` 日志宏；共享状态按 callback group/executor 模型正确加锁。
- 头文件使用 include guard；避免裸 `new`/`delete` 和长期运行节点中的资源泄漏。

### Launch 与配置

- Launch 文件命名为 `*.launch.py`，参数应可从 YAML 或 launch argument 覆盖。
- 不在代码中硬编码机器专属路径、设备号或网络地址；复用现有配置和环境变量。
- 双机部署统一使用 `ROS_DOMAIN_ID=42`、`ROS_LOCALHOST_ONLY=0`、`rmw_cyclonedds_cpp`，并加载 `deploy/cyclonedds/` 中对应设备配置。

## 主要包

| 领域 | 主要包/目录 |
|---|---|
| 系统编排与 Web 控制 | `src/krt_human_robot/` |
| 任务与动作组 | `src/krt_task/`、`src/agx_action_group_runner/` 及对应 interfaces |
| 手部相机 | `src/hand_camera_driver/` |
| 灵巧手控制 | `src/hands_control/`、`src/hands_control_interfaces/` |
| 底盘与导航 | `src/ranger_ros2/`、`src/ranger_nav/` |
| LiDAR/定位/建图 | `src/livox_ros_driver2/`、`src/FAST_LIO_ROS2/`、`src/spark-fast-lio/`、`src/lidar_localization_ros2/`、`src/ndt_omp_ros2/` |
| RealSense | `src/realsense-ros/` |
| 机械臂与 MoveIt | `src/agx_arm_ros/` |
| 语音 | `src/voice_stack/`、`src/voice_assistant/`、`src/voice_test_tools/` |
| 部署 | `deploy/`、`scripts/udev/`、`docs/dual_device_deployment.md` |

## 关键约束

### RealSense 版本

`src/realsense-ros` 必须保持在 `r/4.56.4` 系列，不得升级到最新分支。Jetson 上的 librealsense2 为 2.56.4，新分支可能不兼容。先构建 `realsense2_camera_msgs`，再构建相机包。

### Git 子模块

多个驱动和算法目录是带项目定制分支的子模块。除非任务明确要求，不要改变子模块 URL、分支或提交。新增 3D 定位依赖时使用：

```bash
./scripts/import_3dloc_deps.sh
```

### 硬件与设备

- 相机需要 `video` 组权限；设备 ID 会变化，优先使用 `/dev/camera_left`、`/dev/camera_right` 等 udev 稳定别名。
- CAN 接口和 bitrate 由 `scripts/udev/` 中规则管理。
- 音频功能依赖主机音频服务和实际设备。
- 不在没有明确授权时修改 `/etc`、systemd、udev 或真实硬件状态。

## 常用诊断

```bash
ros2 doctor --report
ros2 node list
ros2 topic list -t
ros2 topic info -v <topic>
ros2 topic hz <topic>
ros2 launch hand_camera_driver hand_cameras.launch.py
ls -l /dev/video* /dev/camera_left /dev/camera_right
```

## 提交规范

- 使用 Conventional Commits：`<type>[optional scope]: <中文描述>`。
- 提交信息必须使用中文，例如 `fix(nav): 修复定位参数加载`。
- 删除包、公共 API、话题、launch 文件或可执行程序时使用 `!`，并添加 `BREAKING CHANGE:` footer。
- 提交前检查 `git diff`，确保不包含生成产物、日志、密钥或无关改动。
