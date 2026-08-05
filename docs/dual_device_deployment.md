# x86 + Jetson 双机部署

目标地址：x86 `10.168.1.100`，Jetson `10.168.1.101`，MID360
`10.168.1.102`。两台主机均使用 Ubuntu 22.04、ROS 2 Humble、Domain 42
和 Cyclone DDS。

## 1. 基线检查

在两端分别确认版本与代码一致：

```bash
cat /etc/os-release
uname -m
git -C /home/create/WorkSpace/KrtHumanRobot rev-parse HEAD
git -C /home/create/WorkSpace/KrtHumanRobot submodule status
timedatectl status
```

x86 应为 `x86_64`，Jetson 应为 `aarch64`。两端必须使用同一个主仓库 commit；
各自本机构建，不能复制另一架构的 `build/` 或 `install/`。

## 2. udev 和设备权限

x86 负责 CAN、底盘、机械臂、夹爪和音频；Jetson 负责 D435、左手和右手
相机。详细规则见 `scripts/udev/README.md`。

Jetson 安装项目相机规则：

```bash
cd /home/create/WorkSpace/KrtHumanRobot
sudo install -o root -g root -m 0644 \
  scripts/udev/99-camera-alias.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=video4linux --action=add
ls -l /dev/camera_left /dev/camera_right \
  /dev/camera_head_rgb /dev/camera_head_depth /dev/camera_head_ir
```

用户必须属于 `video`、`dialout`、`audio` 和 `plugdev` 中与本机硬件对应的组。
组变更后重新登录。

## 3. ROS 与依赖安装

两端安装 Cyclone DDS：

```bash
sudo apt update
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
```

初始化子模块并安装依赖：

```bash
cd /home/create/WorkSpace/KrtHumanRobot
git submodule update --init --recursive
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Jetson 的 `src/realsense-ros` 必须位于 `r/4.56.4`，librealsense 必须为
2.56.4。安装后检查：

```bash
git -C src/realsense-ros branch --show-current
rs-enumerate-devices
```

分机构建：

```bash
# x86
colcon build --symlink-install

# Jetson 相机侧至少构建以下包
colcon build --packages-select realsense2_camera_msgs realsense2_camera \
  realsense2_description hand_camera_driver --symlink-install
```

## 4. Cyclone DDS

从示例创建本机私有环境文件，不要提交密钥：

```bash
mkdir -p ~/.config/krt
cp deploy/env/x86.env.example ~/.config/krt/x86.env
# Jetson 使用：cp deploy/env/jetson.env.example ~/.config/krt/jetson.env
chmod 600 ~/.config/krt/*.env
```

交互终端测试时导出对应文件中的变量，然后验证双向发现：

```bash
set -a
source ~/.config/krt/x86.env   # Jetson 改为 jetson.env
set +a
source /opt/ros/humble/setup.bash
source /home/create/WorkSpace/KrtHumanRobot/install/setup.bash
```

x86 运行 `ros2 run demo_nodes_cpp talker`，Jetson 运行
`ros2 run demo_nodes_py listener`；再交换角色验证。`ros2 doctor --report` 的
RMW 必须显示 Cyclone DDS。

## 5. 手动启动

```bash
# x86
ros2 launch krt_human_robot x86_bringup.launch.py \
  map:=/home/create/maps/map.yaml \
  pcd_map_path:=/home/create/maps/scans.pcd \
  navigation_mode:=3dloc

# Jetson
ros2 launch hand_camera_driver jetson_cameras.launch.py
```

确认 Jetson 发布原始及压缩彩色话题；x86 不订阅 D435 深度话题。分别检查：

```bash
ros2 topic hz /camera/camera/color/image_raw/compressed
ros2 topic hz /left_gripper/image_raw/compressed
ros2 topic hz /right_gripper/image_raw/compressed
```

## 6. 当前迁移检查项

- Jetson 必须初始化 RealSense 子模块并安装 librealsense 2.56.4 工具。
- 两端必须安装 `ros-humble-rmw-cyclonedds-cpp`。
- 完整构建后运行两个 launch 的 `--show-args`。
- systemd、Ollama 回退和断网恢复按后续任务部署并验收。

## 7. Jetson Ollama 回退

Jetson 使用 Ollama 0.21.0，目标模型为 `qwen2.5:0.5b` 和
`qwen3.5:0.8b`。安装版本化 drop-in 后重载服务：

```bash
sudo install -d -m 0755 /etc/systemd/system/ollama.service.d
sudo install -o root -g root -m 0644 deploy/systemd/ollama-krt.conf \
  /etc/systemd/system/ollama.service.d/krt.conf
sudo systemctl daemon-reload
sudo systemctl restart ollama
```

模型仅在部署阶段下载：

```bash
ollama pull qwen2.5:0.5b
ollama pull qwen3.5:0.8b
curl --fail http://127.0.0.1:11434/api/tags
```

x86 验证文本请求：

```bash
curl --fail http://10.168.1.101:11434/api/chat \
  -H 'Content-Type: application/json' \
  -d '{"model":"qwen2.5:0.5b","stream":false,"messages":[{"role":"user","content":"你好"}]}'
```

验证图片请求时，把 JPEG 转成 base64 后放入 `images` 数组：

```bash
curl --fail http://10.168.1.101:11434/api/chat \
  -H 'Content-Type: application/json' \
  -d '{"model":"qwen3.5:0.8b","stream":false,"messages":[{"role":"user","content":"描述图片","images":["<BASE64_JPEG>"]}]}'
```

请求期间在 Jetson 运行 `ollama ps` 和 `tegrastats`，确认模型已加载并使用
GPU。云端请求失败时应回退到 Jetson；停止 Ollama 后，x86 控制、导航、语音
和 Web 仍须继续运行。
