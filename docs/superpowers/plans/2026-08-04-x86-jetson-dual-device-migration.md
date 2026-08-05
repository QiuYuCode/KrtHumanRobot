# x86 + Jetson 双机迁移 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将当前仍完整运行在现有 Jetson Orin 上的 KrtHumanRobot，迁移为 x86 控制/导航/语音/Web 主机与 Jetson AGX Orin 相机/离线模型主机组成的双机系统。

**Architecture:** x86 与 Jetson 组成一台机器人的同一 ROS 2 Domain，并使用 Cyclone DDS 固定对端发现。机器人硬件闭环全部留在 x86；Jetson 发布按需压缩图像并提供 Ollama HTTP 服务，云端 LLM/VLM 失败时由 x86 调用该服务回退。

**Tech Stack:** Ubuntu 22.04、ROS 2 Humble、Cyclone DDS、Python 3.10、rclpy、RealSense ROS `r/4.56.4`、Ollama 0.18.2、systemd。

## Current State

- 迁移尚未开始；当前分支仍在现有 Jetson Orin 上开发和运行。
- 当前机器是迁移来源和软件参考环境，不是文档所述的目标 AGX Orin 设备。
- 当前本地 LLM/VLM 地址均为 `http://localhost:11434`；当前相机消费者订阅原始 `sensor_msgs/Image`。
- 本文所有 IP、节点分工和 systemd 服务描述均为目标状态，实施前不得据此判断迁移已经完成。

## Global Constraints

- 目标 x86：`10.168.1.100`；目标 Jetson AGX Orin：`10.168.1.101`；MID360：`10.168.1.102`；网关：`10.168.1.1`。
- 两端统一使用 Ubuntu 22.04、ROS 2 Humble、`ROS_DOMAIN_ID=42`、`ROS_LOCALHOST_ONLY=0` 和 `rmw_cyclonedds_cpp`。
- Jetson 使用 JetPack 6 / L4T R36.4.x；RealSense ROS 固定 `r/4.56.4`，librealsense 固定 2.56.4。
- x86 承担双臂、双手、底盘、雷达、导航、语音、任务、动作组、核心行为树和 Web 控制台；Jetson 只承担相机和 Ollama。
- Web 控制台默认开启并监听 `0.0.0.0:8443`。
- 百兆链路只传当前被订阅的压缩彩色画面，默认 640×480、15 FPS、JPEG 质量 70；D435 深度不跨机。
- 云端 LLM/VLM 主用；Jetson 的 `qwen2.5:0.5b` 与 `qwen3.5:0.8b` 仅作为离线回退。
- 保留现有原始图像话题，不删除或重命名现有 ROS 公共接口。
- 当前 LAN 按可信网络处理；本次不加入 SROS2、VLAN、防火墙、Ollama 鉴权或强制 TLS。
- 后续每台机器人使用独立 Domain；跨机器人协作通过单独网关选择性桥接，不在本次迁移中实现。

---

### Task 1: 按需压缩相机接口

**Files:**
- Modify: `src/hand_camera_driver/hand_camera_driver/usb_camera_node.py`
- Modify: `src/hand_camera_driver/launch/hand_cameras.launch.py`
- Modify: `src/hand_camera_driver/package.xml`
- Test: `src/hand_camera_driver/test/test_usb_camera_node.py`

**Interfaces:**
- Preserve: `/left_gripper/image_raw`、`/right_gripper/image_raw`，类型 `sensor_msgs/Image`。
- Produce: `/left_gripper/image_raw/compressed`、`/right_gripper/image_raw/compressed`，类型 `sensor_msgs/CompressedImage`。
- Produce parameters: `compressed_topic`、`jpeg_quality`，默认分别为 `<raw_topic>/compressed` 和 `70`。

- [ ] 增加失败测试：验证 JPEG 消息保留时间戳/frame ID、可被 OpenCV 解码，并且压缩话题没有订阅者时不执行编码。
- [ ] 运行 `colcon test --packages-select hand_camera_driver --event-handlers console_direct+`，确认新增测试先失败。
- [ ] 在现有 USB 相机节点中增加第二个 `CompressedImage` publisher，仅当其 subscription count 大于零时调用 `cv2.imencode()`。
- [ ] 将左右相机 launch 默认规格改为 640×480@15 FPS，并透传 JPEG 质量参数。
- [ ] 重新运行 `hand_camera_driver` 测试并执行 `colcon test-result --verbose`，确认通过。
- [ ] 提交：`feat(camera): 新增按需压缩图像话题`。

### Task 2: x86 压缩图像消费

**Files:**
- Modify: `src/krt_human_robot/krt_human_robot/behaviors/core/actions/camera_source.py`
- Modify: `src/krt_human_robot/krt_human_robot/config.py`
- Modify: `src/krt_human_robot/config/krt_human_robot.yaml`
- Modify: `src/krt_task/krt_task/routine_runner.py`
- Modify: `src/krt_task/launch/routine_runner.launch.py`
- Test: `src/krt_human_robot/test/test_camera_source.py`
- Test: `src/krt_task/test/test_routine_runner.py`

**Interfaces:**
- Produce config: `camera_ros_transport`, choices `raw|compressed`，目标部署默认 `compressed`。
- Consume: base raw topic from each `cameras.<id>.ros_topic`; compressed mode subscribes to `<base>/compressed` as `sensor_msgs/CompressedImage`。
- Produce routine parameter: `image_transport`, choices `raw|compressed`，目标部署默认 `compressed`。

- [ ] 增加失败测试：压缩消息解码、无效 JPEG、首帧超时、关闭后销毁 subscription，以及 routine 拍照保存压缩消息。
- [ ] 运行 `UV_CACHE_DIR=/tmp/uv-cache uv run --no-sync --project src/voice_assistant python -m pytest src/krt_human_robot/test/test_camera_source.py src/krt_task/test/test_routine_runner.py -q`，确认新增用例失败。
- [ ] 最小扩展 `RosCameraSource`，根据 `camera_ros_transport` 选择 `Image + CvBridge` 或 `CompressedImage + cv2.imdecode`。
- [ ] 对 `routine_runner` 使用相同 transport 语义，保持现有 raw 配置兼容。
- [ ] 将目标配置中的三路相机都设为 compressed、640×480@15 FPS；D435 深度 topic 保持不变且不被 x86 launch 订阅。
- [ ] 重跑聚焦测试并确认通过。
- [ ] 提交：`feat(vision): 支持按需订阅压缩相机话题`。

### Task 3: 双机 ROS 启动入口

**Files:**
- Create: `src/krt_human_robot/launch/x86_bringup.launch.py`
- Create: `src/hand_camera_driver/launch/jetson_cameras.launch.py`
- Modify: `src/krt_human_robot/setup.py`
- Modify: `src/hand_camera_driver/setup.py`
- Modify: `src/livox_ros_driver2/config/MID360_config.json`
- Test: `src/krt_human_robot/test/test_dual_device_launch.py`

**Interfaces:**
- Produce launch: `ros2 launch krt_human_robot x86_bringup.launch.py`。
- Produce launch: `ros2 launch hand_camera_driver jetson_cameras.launch.py`。
- x86 launch arguments: `map`、`navigation_mode`、`web_host`、`web_port`、`web_certfile`、`web_keyfile`。

- [ ] 增加 launch 结构测试，确认 x86 入口关闭 camera stack、开启 arm/Web，并包含导航；Jetson 入口只包含 D435 与两路 USB 相机。
- [ ] 运行聚焦测试并确认因 launch 文件缺失而失败。
- [ ] x86 入口复用现有 `robot.launch.py`，传入 `enable_camera_stack=false`、`enable_arm_control_stack=true`、`enable_web_console=true`，并包含选定导航 launch，默认 `rviz=false`。
- [ ] Jetson 入口包含 RealSense `rs_launch.py` 和 `hand_cameras.launch.py`；D435 彩色设置为 640×480@15 FPS，深度仅本地发布。
- [ ] 确认 `MID360_config.json` 的四个 `host_net_info` 地址保持 `10.168.1.100`，雷达地址保持 `10.168.1.102`。
- [ ] 运行 launch 测试以及两个入口的 `--show-args`，确认参数和默认值正确。
- [ ] 提交：`feat(bringup): 新增 x86 与 Jetson 双机启动入口`。

### Task 4: Cyclone DDS 固定对端

**Files:**
- Create: `deploy/cyclonedds/x86.xml`
- Create: `deploy/cyclonedds/jetson.xml`
- Create: `deploy/env/x86.env.example`
- Create: `deploy/env/jetson.env.example`
- Create: `docs/dual_device_deployment.md`

**Interfaces:**
- x86 binds `10.168.1.100` and peers with `10.168.1.101`。
- Jetson binds `10.168.1.101` and peers with `10.168.1.100`。
- Both export `ROS_DOMAIN_ID=42`、`ROS_LOCALHOST_ONLY=0`、`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and device-specific `CYCLONEDDS_URI`。

- [ ] 编写两份最小 Cyclone DDS 配置，使用固定接口地址和单一对端，不引入 Discovery Server 或 DDS Router。
- [ ] 编写环境文件示例，包含工作区、地图、DDS 和 Web 参数；密钥只记录变量名，不写入值。
- [ ] 在部署文档中记录两端安装、同一 Git commit、rosdep、分包构建、设备权限和 NTP 校时步骤。
- [ ] 使用 XML 解析器验证两份配置格式，并在两台目标机上双向运行 talker/listener。
- [ ] 提交：`docs(deploy): 新增双机 DDS 配置与部署说明`。

### Task 5: Jetson Ollama 离线回退

**Files:**
- Modify: `src/krt_human_robot/config/krt_human_robot.yaml`
- Create: `deploy/systemd/ollama-krt.conf`
- Modify: `docs/dual_device_deployment.md`
- Test: `src/krt_human_robot/test/test_llm_model_config.py`

**Interfaces:**
- Serve: `http://10.168.1.101:11434` with `OLLAMA_HOST=0.0.0.0:11434`。
- Configure: `local_llm_base_url=http://10.168.1.101:11434` and `local_vlm_base_url=http://10.168.1.101:11434`。
- Models: `qwen2.5:0.5b` for text and `qwen3.5:0.8b` for text+image。

- [ ] 增加配置测试，确认云端 provider 保持主用、两个 fallback 开关为 true，且本地 URL 指向 Jetson 而非 localhost。
- [ ] 运行 `src/krt_human_robot/test/test_llm_model_config.py` 并确认新增断言先失败。
- [ ] 更新配置并增加 Ollama systemd drop-in；模型只在部署阶段 `ollama pull`，不得在开机服务中下载。
- [ ] 文档记录 `/api/tags` 健康检查、文本请求、图片请求、`ollama ps` 和 `tegrastats` GPU 验证。
- [ ] 强制云端 LLM/VLM 请求失败，确认分别回退到 Jetson；停止 Ollama 后机器人控制和 Web 仍可运行。
- [ ] 提交：`feat(ai): 将离线 LLM 与 VLM 回退迁移到 Jetson`。

### Task 6: systemd 开机启动

**Files:**
- Create: `deploy/systemd/krt-x86.service`
- Create: `deploy/systemd/krt-jetson.service`
- Modify: `docs/dual_device_deployment.md`

**Interfaces:**
- x86 user service runs `x86_bringup.launch.py` from `%h/WorkSpace/KrtHumanRobot`。
- Jetson user service runs `jetson_cameras.launch.py` from `%h/WorkSpace/KrtHumanRobot`。
- Ollama remains an independent system service so camera startup does not depend on model readiness。

- [ ] 编写两个 user unit，source `/opt/ros/humble/setup.bash` 和工作区 `install/setup.bash`，加载 `%h/.config/krt/*.env`。
- [ ] 设置 `Restart=on-failure`、`RestartSec=5`、`TimeoutStopSec=20` 和 SIGINT 关闭；不要把 API key 写进 unit。
- [ ] 文档记录安装到 `~/.config/systemd/user/`、`systemctl --user enable --now` 和 `loginctl enable-linger <user>`。
- [ ] 分别重启目标设备，确认 `krt-x86.service`、`krt-jetson.service` 和 `ollama.service` 自动 active，日志可由 journalctl 查询。
- [ ] 提交：`feat(deploy): 新增双机 systemd 自动启动服务`。

### Task 7: 端到端验收

**Files:**
- Modify only if verification exposes a migration regression.

- [ ] 两端确认 Ubuntu 22.04、ROS 2 Humble、相同 Git commit、相同自定义 interface 定义和正确架构构建产物。
- [ ] 验证 `/livox/lidar`、`/livox/imu`、`/scan`、TF、Nav2、双臂、双手、底盘、语音、任务和动作组。
- [ ] 从 LAN 打开 `http://10.168.1.100:8443`，验证 Web 控制台默认可访问及控制接口正常。
- [ ] 逐一选择头部、左掌、右掌相机，确认压缩话题至少 12 FPS、首帧不超过 3 秒，未选中两路不产生跨机图像流量。
- [ ] 使用 `ros2 topic bw` 和网卡统计确认相机与 VLM 请求并发时总链路占用低于 60 Mbps，控制和 Web 操作无明显延迟。
- [ ] 拔掉 Jetson 网络，确认 x86 控制、导航、语音和 Web 继续运行；恢复网络后无需重启 x86 即可重新获取图像和调用离线模型。
- [ ] 使用另一 Domain 验证默认不可见，记录后续机器人从 43 开始分配独立 Domain。
- [ ] 运行 `git diff --check`、相关 pytest、`colcon test` 和 `colcon test-result --verbose`，审查最终差异后再合并迁移分支。

## Deferred Work

- 全项目 `/robot_id` namespace 和 TF frame 前缀改造。
- DDS Router/Zenoh 跨机器人协作网关。
- SROS2、Ollama 鉴权、VLAN、防火墙、Web 强制 TLS/认证。
- 更大模型、vLLM/SGLang、模型并发优化和性能基准；只有当前 Ollama 模型实测不足时再增加。
