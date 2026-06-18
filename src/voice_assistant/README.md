# voice_assistant

基于 **py_trees_ros** 驱动的 ROS 2 语音对话助手，自 smart-voice-robot 迁入。

## 功能

- 唤醒词 (KWS) → ASR → 意图识别 / LLM 规划 → 动作执行 → TTS
- 支持拍照、录像、场景描述 (VLM)、机械臂、灵巧手、导航（ROS 接口待接）
- 行为树生命周期由 `py_trees_ros.trees.BehaviourTree` 托管（`setup/tick_tock/shutdown`）
- 自定义节点仍基于 `py_trees` 行为节点基类（`py_trees_ros` 官方设计依赖）

## 依赖分层

| 层级 | 工具 | 内容 |
|------|------|------|
| ROS 2 | `apt` | `rclpy`, `py_trees`, `py_trees_ros`, `cv_bridge` |
| 语音栈 | `uv`（`.venv`） | `sherpa-onnx`, `langchain`, `loguru`, `sounddevice` 等 |
| 打包 | `colcon` | launch、config、Python 包安装到 `install/` |

**不要**用 `pip install` 往系统 Python 装语音依赖，**不要**在 `pyproject.toml` 里写 `rclpy` / `py-trees`。

## 一次性环境准备

```bash
# ROS 2
sudo apt install ros-humble-py-trees ros-humble-py-trees-ros \
  ros-humble-py-trees-ros-interfaces python3-opencv

# colcon 构建
cd /home/nvidia/WorkSpace/KrtHumanRobot
colcon build --packages-select voice_assistant --symlink-install
source install/setup.bash

# uv 虚拟环境（继承 apt 包，隔离语音 pip 依赖）
bash src/voice_assistant/scripts/setup_uv.sh

# 可选：仍使用 SDK 直连臂/手
cd src/voice_assistant && uv sync --extra arm --extra hand
```

`setup_uv.sh` 会创建 `src/voice_assistant/.venv`（`--system-site-packages`），使 venv 内可 `import rclpy` 同时语音包装在 venv 中。

## 模型文件

模型**不入 Git**。见 [voice_assistant/model/README.md](voice_assistant/model/README.md)。

```bash
# 复用已下载模型
ln -sf /path/to/smart-voice-robot/model/voice_models \
  src/voice_assistant/voice_assistant/model/voice_models
```

## 运行

| Launch | 作用 | 唤醒后是否有语音回复 |
|--------|------|---------------------|
| `voice_stack.launch.py` | 仅底层：采集/KWS/VAD/ASR/TTS/播放 | **否**（无行为树） |
| `voice_full.launch.py` | voice_stack + bt_manager | **是**（推荐） |
| `voice_assistant.launch.py` | 仅行为树（需另起 voice_stack） | 是 |

**唤醒词**（见 `model/voice_models/.../keywords.txt`）：你好小特、小特小特、小特同学、小科小科、小科同学。

```bash
cd /home/nvidia/WorkSpace/KrtHumanRobot
source install/setup.bash

# 仅验 KWS/采集（终端会打印 KWS 命中，但不会播报）
ros2 launch voice_assistant voice_stack.launch.py

# 完整唤醒闭环（推荐）
ros2 launch voice_assistant voice_full.launch.py

# 仅行为树（voice_stack 需另起）
ros2 launch voice_assistant voice_assistant.launch.py

# 或直接脚本
bash src/voice_assistant/scripts/run_voice_node.sh
```

> **不要**用 `ros2 run voice_xxx ...` 或 colcon 入口直接跑语音节点，它们走系统 Python，**缺少 uv 依赖**（`sherpa_onnx`、`sounddevice` 等）。`voice_stack.launch.py` 与 `voice_assistant.launch.py` 内部均通过 `uv run` 启动。

启动前确认 uv 依赖就绪：

```bash
cd src/voice_assistant
uv run python -c "import sounddevice, sherpa_onnx; print('deps OK')"
```

环境变量（launch 已自动设置，手动运行时可用）：

- `VOICE_ASSISTANT_CONFIG` — YAML 配置路径
- `VOICE_TICK_INTERVAL_MS` — 行为树 tick 周期
- `VOICE_ENABLE_MONITOR` — 是否启用树监控发布（`true/false`）
- `VOICE_SNAPSHOT_PERIOD_S` — 快照发布周期（秒）
- `VOICE_SNAPSHOT_BLACKBOARD_DATA` — 快照是否附加 blackboard 数据
- `VOICE_SNAPSHOT_BLACKBOARD_ACTIVITY` — 快照是否附加 blackboard 活动流

说明：旧 Web 监控面板已移除，统一使用 `py_trees_ros_tree_watcher` 观测树状态。

> `ros2 run voice_assistant voice_node` 走系统 Python，**缺少 uv 依赖，不推荐**。请用 launch 或 `run_voice_node.sh` / `run_voice_stack_node.sh`。

### voice_stack 栈级验收

```bash
ros2 launch voice_assistant voice_stack.launch.py
# 另开终端：
ros2 topic hz /voice/audio/raw              # 应约 10Hz，否则检查麦克风/input_device_hint
ros2 topic echo /voice/kws/events --once    # 对麦说「你好小特」，应看到 KWS 命中日志与消息
ros2 topic list | grep voice
ros2 service list | grep voice
```

通过标准：7 个进程无 `ModuleNotFoundError`；启动日志含 `[Audio] 输入设备:`；topic 含 `/voice/audio/raw`、`/voice/kws/events`、`/voice/vad/events`；service 含 `/voice/asr/recognize`、`/voice/tts/synthesize`。

### 完整唤醒验收

```bash
ros2 launch voice_assistant voice_full.launch.py
```

对麦克风说「你好小特」：终端出现 `唤醒成功!`，并 TTS 播报「我在，请说。」

## 配置

默认：`config/voice_assistant.yaml`（安装后 `share/voice_assistant/config/`）。

模型路径相对 Python 包目录 `voice_assistant/`，例如 `model/voice_models/...`。

## 与其他 ROS 包联调

```bash
ros2 launch realsense2_camera rs_launch.py          # 相机
ros2 launch agx_action_group_runner run_action_group_runner.launch.py
ros2 launch hands_control hand_control_launch.py
ros2 launch voice_assistant voice_assistant.launch.py
```

`voice_assistant.yaml` 中设置 `camera_backend: ros` 并配置 `cameras.*.ros_topic`。

## 调试

```bash
source install/setup.bash
cd src/voice_assistant
uv run python -c "import rclpy, py_trees_ros, sherpa_onnx, loguru; print('deps OK')"
uv run python -c "from voice_assistant.config import default_config; print(default_config.asr_model_dir)"
```

```bash
ros2 run py_trees_ros_tree_watcher tree_watcher
```
