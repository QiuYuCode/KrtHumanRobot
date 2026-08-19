# voice_assistant

纯语音子系统包。负责采集、KWS、VAD、ASR、TTS、播放、媒体播报和音量服务。

行为树、意图识别、LLM 规划、动作分发已经迁到 `krt_human_robot`。默认整机入口是：

```bash
ros2 launch krt_human_robot robot.launch.py
```

## 包边界

`voice_assistant` 不 import `krt_human_robot`，也不保存相机、导航、机械臂、灵巧手业务动作代码。

对外接口固定在 `/voice/...`：

- `/voice/audio/raw`
- `/voice/kws/events`
- `/voice/vad/events`
- `/voice/asr/recognize`
- `/voice/asr/stream`
- `/voice/tts/synthesize`
- `/voice/playback/play`
- `/voice/playback/stop`
- `/voice/media/play`
- `/voice/volume/get`、`/voice/volume/set`、`/voice/volume/mute`

## 配置

`config/voice_assistant.yaml` 只保留语音参数：

- 音频输入/输出设备及 PulseAudio 输出端口
- KWS 模型和关键词
- VAD 参数
- ASR 后端和模型
- TTS 后端、模型、云端字段

整机意图、LLM、动作和功能包接口配置在 `krt_human_robot/config/krt_human_robot.yaml`。

敏感字段通过环境变量注入：

- `XFYUN_IAT_APPID` / `XFYUN_IAT_API_KEY` / `XFYUN_IAT_API_SECRET`
- `XFYUN_TTS_APPID` / `XFYUN_TTS_API_KEY` / `XFYUN_TTS_API_SECRET`
- `LLM_API_KEY`，用于 MiMo TTS

## 环境准备

```bash
cd /home/nvidia/WorkSpace/KrtHumanRobot
colcon build --packages-select voice_assistant --symlink-install
source install/setup.bash

bash src/voice_assistant/scripts/setup_uv.sh
```

不要把 `sherpa_onnx`、`sounddevice` 等语音依赖装进系统 Python；语音节点通过 `uv` 虚拟环境启动。

## 模型文件

模型不入 Git。见 `voice_assistant/model/README.md`。

```bash
ln -sf /path/to/voice_models \
  src/voice_assistant/voice_assistant/model/voice_models
```

## 启动

```bash
cd /home/nvidia/WorkSpace/KrtHumanRobot
source install/setup.bash

# 只启动语音栈
ros2 launch voice_assistant voice_assistant.launch.py

# 兼容入口：转发到 krt_human_robot robot.launch.py
ros2 launch voice_assistant voice_full.launch.py
```

`voice_assistant.launch.py` 只启动语音栈，不启动行为树。

`voice_full.launch.py` 是兼容入口，默认启动 `krt_human_robot robot.launch.py`。

## 单元测试 launch

```bash
ros2 launch voice_assistant kws_test.launch.py
ros2 launch voice_assistant vad_test.launch.py
ros2 launch voice_assistant asr_test.launch.py
ros2 launch voice_assistant tts_test.launch.py
ros2 launch voice_assistant media_test.launch.py
ros2 launch voice_assistant volume_test.launch.py
```

这些 launch 都接受 `config_file:=/absolute/path/voice_assistant.yaml`。

## 验收

```bash
ROS_LOG_DIR=/tmp/ros_log ros2 launch voice_assistant voice_assistant.launch.py --show-args
ROS_LOG_DIR=/tmp/ros_log ros2 launch voice_assistant voice_assistant.launch.py
```

如果当前机器没有麦克风或音频权限，可以先关闭采集节点验证其余服务：

```bash
ROS_LOG_DIR=/tmp/ros_log ros2 launch voice_assistant voice_assistant.launch.py enable_capture:=false
```
