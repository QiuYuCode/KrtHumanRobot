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

说明：旧 Web 监控面板已移除，统一使用 `py-trees-tree-watcher`
观测树状态。`VOICE_ENABLE_MONITOR=true` 只开启 snapshot 发布，不会把
行为树逐 tick 追加到主节点日志。

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

### 讯飞云语音验收

默认配置使用本地 ASR、讯飞云 TTS：`asr_backend: local`、
`tts_backend: iflytek_cloud`、`cloud_tts_fallback_to_local: true`。
密钥不要写入 YAML。
`voice_stack.launch.py` 会自动读取 `src/voice_assistant/.env`；
也可以用 `VOICE_ASSISTANT_DOTENV=/path/to/.env` 指定其他密钥文件。

默认云 TTS 验证：

```bash
ros2 launch voice_assistant voice_full.launch.py
```

确认 `.env` 中已有 `XFYUN_TTS_APPID`、`XFYUN_TTS_API_KEY`、
`XFYUN_TTS_API_SECRET`。启动提示音和对话回复应由讯飞云 TTS 合成，
日志出现 `TTS 已提交播放: backend=iflytek_cloud ...`。

断网/密钥错误回退验证：

```bash
VOICE_ASSISTANT_DOTENV=/tmp/not_exists.env \
ros2 launch voice_assistant voice_full.launch.py
```

调用 `/voice/tts/synthesize` 或唤醒后播报时，日志应出现“讯飞云 TTS 失败，回退本地 TTS”，并最终正常播放。若只设置通用变量，也兼容 `XFYUN_APPID`、`XFYUN_API_KEY`、`XFYUN_API_SECRET`。

云 ASR/TTS 联合验证时，将临时配置里的 `asr_backend` 切到 `iflytek_cloud`，并同时设置 `XFYUN_IAT_*` 与 `XFYUN_TTS_*`。完成多轮“唤醒 -> 识别 -> 回复 -> 播放”，检查节点不退出且没有重复 playback server。

### 小米 MiMo LLM/VLM 验收

默认配置使用小米 MiMo OpenAI-compatible 接口处理闲聊和视觉理解：
`llm_provider: openai`、`llm_model: mimo-v2.5`、
`llm_base_url: https://token-plan-cn.xiaomimimo.com/v1`；VLM 同样默认
`vlm_provider: openai`、`vlm_model: mimo-v2.5`。密钥只从环境读取，
`.env` 中配置 `LLM_API_KEY`；视觉可单独配置 `VLM_API_KEY`，未配置时复用
`LLM_API_KEY`。

关键词动作仍优先。`use_llm_planner: false` 时行为树路径为
`RecognizeIntent -> ActionSelector -> LLMDialog fallback`，所以“拍照”、
“左手张开”、“介绍一下你自己”等命中关键词的指令仍走本地动作；只有没有命中关键词的闲聊问题才进入 LLM 对话。

```bash
ros2 launch voice_assistant voice_full.launch.py
```

唤醒后说一个不匹配关键词的问题，日志应显示
`provider=openai, model=mimo-v2.5`，并由云端生成简短中文回复。说“看一下前面有什么”，应使用 `mimo-v2.5` 完成视觉描述。

云端失败回退验证：临时使用错误密钥或断网，再发起闲聊或视觉请求。日志应先记录云端 LLM/VLM 失败，再尝试本地 Ollama：
对话回退 `local_llm_model: qwen2.5:0.5b`，视觉回退
`local_vlm_model: qwen3.5:0.8b`。若本地 Ollama 不可用，闲聊返回明确失败话术；若本地 VLM 不可用或不支持图片输入，回复“视觉分析暂时不可用”。

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
py-trees-tree-watcher
```

`py-trees-tree-watcher` 需要在单独终端直接运行，它会原地刷新树快照；不要通过
`ros2 launch` 聚合输出查看，否则 launch 日志仍会按行追加。
