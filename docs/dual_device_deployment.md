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

两端安装 Cyclone DDS；Jetson 构建 RealSense ROS 还需要诊断依赖：

```bash
sudo apt update
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
# 仅 Jetson
sudo apt install -y ros-humble-diagnostic-updater
```

x86 的 KISS-Matcher 与核心行为包还需要：

```bash
sudo apt install -y ros-humble-gtsam python3-loguru
```

Ubuntu 22.04 / ROS Humble 构建应优先使用系统 CMake 3.22。若用户环境中安装了
较新的 CMake，先执行 `export PATH=/usr/bin:/bin:$PATH`；中文 locale 会使
KISS-Matcher 内置旧版 TBB 无法解析汇编器版本，构建时设置
`LC_ALL=C LANG=C`。

初始化子模块并安装依赖：

```bash
cd /home/create/WorkSpace/KrtHumanRobot
git submodule update --init --recursive
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Jetson 的 `src/realsense-ros` 必须位于 `r/4.56.4`，librealsense 必须为
2.56.4。JetPack 6 设备使用从官方 `v2.56.4` 源码构建、启用 RSUSB backend
的 librealsense，并安装同版本源码中的 udev 规则。安装后检查：

```bash
git -C src/realsense-ros branch --show-current
rs-enumerate-devices --version
rs-enumerate-devices -s
ldconfig -p | grep realsense
```

不要在该 Jetson 上安装 `ros-humble-compressed-image-transport`。ROS Humble
二进制包链接系统 OpenCV 4.5，而当前 JetPack RealSense 进程链接 NVIDIA
OpenCV 4.8；在同一进程加载两套 ABI 会导致异常内存分配。D435 彩色 JPEG
由 `hand_camera_driver/compressed_image_relay` 独立进程按需生成到专用话题
`/camera/camera/color/image_jpeg`。不要使用 `/image_raw/compressed` 后缀，避免
RealSense 进程自动加载 `compressed_image_transport`。

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
RMW 必须显示 Cyclone DDS。配置同时列出 `localhost` 和固定远端地址，使同机
多个 ROS 进程在禁用 multicast 时仍能发现彼此。

## 5. 手动启动

```bash
# x86
ros2 launch krt_human_robot x86_bringup.launch.py \
  enable_navigation:=true \
  map:=/home/create/maps/map.yaml \
  pcd_map_path:=/home/create/maps/scans.pcd \
  navigation_mode:=3dloc

# Jetson
ros2 launch hand_camera_driver jetson_cameras.launch.py
```

确认 Jetson 发布原始及压缩彩色话题；x86 不订阅 D435 深度话题。压缩节点
没有订阅者时跳过解码与 JPEG 编码。分别检查：

```bash
ros2 topic hz /camera/camera/color/image_jpeg
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

## 8. systemd 用户服务

在对应设备安装 unit 和环境文件：

```bash
mkdir -p ~/.config/systemd/user ~/.config/krt

# x86
install -m 0644 deploy/systemd/krt-x86.service ~/.config/systemd/user/
install -m 0644 deploy/systemd/krt-quest-teleop.service ~/.config/systemd/user/
test -f ~/.config/krt/x86.env || \
  install -m 0600 deploy/env/x86.env.example ~/.config/krt/x86.env

# Jetson 使用以下两条代替上面的 x86 安装
install -m 0644 deploy/systemd/krt-jetson.service ~/.config/systemd/user/
test -f ~/.config/krt/jetson.env || \
  install -m 0600 deploy/env/jetson.env.example ~/.config/krt/jetson.env

systemctl --user daemon-reload
```

检查私有环境文件中的路径和密钥后启用。x86 的
`KRT_DDS_BIND_ADDRESS` 必须与 `deploy/cyclonedds/x86.xml` 的
`NetworkInterface` 地址一致；两个服务默认最多等待 90 秒设备就绪，
可通过 `KRT_STARTUP_TIMEOUT_S` 调整。

`krt-x86.service` 保持 `PYTHONNOUSERSITE=1`，以隔离 Quest 环境的 Python
依赖。DexHand SDK 不应依赖用户级 editable 安装；在 `~/.config/krt/x86.env`
设置 `KRT_DEXHAND_SDK_PYTHONPATH` 指向 `dexhand_sdk_python` 源码目录。若不设置，
默认使用 `KRT_WORKSPACE` 同级的 `dexhand_sdk_python`。

Web 启动导航时，`krt-x86.service` 会自动选择当前用户活动的本地 X11 会话
并启动 RViz；不需要填写机器相关的 display。若自动选择失败，可在
`~/.config/krt/x86.env` 手动设置 `KRT_RVIZ_DISPLAY` 和
`KRT_RVIZ_XAUTHORITY`，然后执行
`systemctl --user daemon-reload && systemctl --user restart krt-x86`。
这只适用于本机已登录的 X11 桌面；RViz 不会嵌入远程浏览器。

`network-online.target` 对用户级服务不保证网卡 IP 或 USB 设备已经可用。因此
`krt-x86.service` 会先确认 DDS 绑定 IP 和 NetworkManager 的 `CONNECTED_GLOBAL`
状态；如设置 `KRT_CAN_REQUIRED_CHANNELS`，还会检查每条总线是否为 `ERROR-ACTIVE`
且已收到报文。默认留空，避免单侧机械臂故障阻塞语音等无关组件。`krt-jetson.service`
会先确认两路手部相机别名和 D435 可由
`rs-enumerate-devices -s` 枚举。前置检查超时会退出非零，并由 systemd 每 5 秒
自动重试，无需人工重启服务。

更新 unit 或脚本后先重载用户服务：

```bash
systemctl --user daemon-reload
```

随后启用：

```bash
# x86
systemctl --user enable --now krt-x86.service

# Jetson
systemctl --user enable --now krt-jetson.service
```

允许用户退出登录后继续运行需要管理员执行一次：

```bash
sudo loginctl enable-linger create
```

状态与日志：

```bash
systemctl --user status krt-x86.service
systemctl --user status krt-jetson.service
journalctl --user -u krt-x86.service -f
journalctl --user -u krt-jetson.service -f
sudo systemctl status ollama.service
```

两个 ROS unit 使用 `Restart=on-failure`、五秒重启间隔、SIGINT 停止和二十秒
停止超时。Ollama 是独立 system service；相机服务不等待模型加载。

### Quest 双臂遥操作

`krt-quest-teleop.service` 是按需服务：Web 控制台的“开启 Quest 遥操作”会先确保
Web 已管理的 `/left`、`/right` 机械臂驱动存在，然后启动该服务；它只启动 Quest
位姿、增量位姿和 IK 节点，绝不重复启动 CAN 驱动。关闭遥操作会停止该服务。

普通 x86 机械臂驱动使用系统 SciPy/NumPy，但会从
`~/.local/lib/python3.10/arm_sdk` 读取单独安装的 `pyAgxArm` SDK；不要将整个
`~/.local/lib/python3.10/site-packages` 加回 `PYTHONPATH`，否则会重新引入 Quest
环境的 NumPy ABI 冲突。

在 `~/.config/krt/x86.env` 中设置 `KRT_QUEST_VT_BIN` 为 Quest 专用 conda 环境的
`bin` 目录。该环境必须安装包含 CasADi 支持的 Pinocchio 以及 ADB Python 客户端：

```bash
conda run -n vt python -m pip install --ignore-installed --no-user pure-python-adb casadi
conda run -n vt python -c 'from pinocchio import casadi; from ppadb.client import Client'
```

首次使用前仍须在头显授权 USB 调试并安装 APK；服务会通过 ADB 自动检查/安装 APK，
无需手工提前打开。更新 Quest unit 后执行：

```bash
systemctl --user daemon-reload
systemctl --user enable krt-quest-teleop.service
systemctl --user start krt-quest-teleop.service
journalctl --user -u krt-quest-teleop.service -f
```

Quest 运行中，Web 会拒绝启动动作组回放和 Routine。若要录制 Quest 动作，在 Web 的
“示教录制”选择“左臂”“右臂”或“双臂同步”；录制将直接采集机械臂反馈关节，不进入
Nero 手拖示教模式。双臂样本逐帧配对并在回放时同时发送给 `/left` 和 `/right`。

Quest 服务默认在本机 X11 桌面打开三个 RViz 窗口：手柄坐标、左臂模型、右臂模型。
可在 `x86.env` 中设置 `KRT_QUEST_RVIZ=false` 关闭；未找到可用桌面时服务会自动以
无界面模式继续运行。
