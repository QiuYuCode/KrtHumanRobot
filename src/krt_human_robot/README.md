# krt_human_robot

整机核心入口包。这里拥有 `krtHumanRobot` 节点、行为树、黑板、意图识别、LLM 规划和动作分发。

`voice_assistant` 只作为语音子系统存在，核心包通过 `/voice/...` ROS 接口调用它，不 import 语音包内部 Python 实现。

## 包边界

- `krt_human_robot/behaviors/core`：黑板、意图、规划、对话循环。
- `krt_human_robot/behaviors/voice.py`：语音行为树适配，只调用 `/voice/...` topic/service/action。
- `krt_human_robot/adapters`：相机、雷达、底盘、导航、机械臂等 ROS 接口适配入口预留。
- `config/krt_human_robot.yaml`：整机行为树、意图、LLM、动作开关和功能包接口名。

依赖方向固定为：`krt_human_robot` 调用功能包 ROS 接口；功能包不反向依赖核心包。

## 启动

```bash
cd /home/nvidia/WorkSpace/KrtHumanRobot
source install/setup.bash

# 默认入口：先拉起相机栈和 voice_stack，再启动核心行为树
ros2 launch krt_human_robot robot.launch.py

# 只测核心，不启动语音栈
ros2 launch krt_human_robot robot.launch.py enable_voice_stack:=false

# 不启动相机栈
ros2 launch krt_human_robot robot.launch.py enable_camera_stack:=false
```

常用参数：

- `config_file`：核心配置，默认 `config/krt_human_robot.yaml`。
- `voice_config_file`：语音栈配置，默认 `voice_assistant/config/voice_assistant.yaml`。
- `enable_voice_stack`：是否一起启动语音栈。
- `enable_camera_stack`：是否一起启动 RealSense 和左右手 USB 相机。
- `core_start_delay_s`：相机/语音栈启动后延迟多少秒启动核心。

## 验证

```bash
colcon build --packages-select hand_camera_driver krt_human_robot --symlink-install

ROS_LOG_DIR=/tmp/ros_log ros2 launch krt_human_robot robot.launch.py --show-args
ROS_LOG_DIR=/tmp/ros_log ros2 launch krt_human_robot robot.launch.py enable_voice_stack:=false
```

相机话题检查：

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
ros2 topic hz /left_gripper/image_raw
ros2 topic hz /right_gripper/image_raw
```
