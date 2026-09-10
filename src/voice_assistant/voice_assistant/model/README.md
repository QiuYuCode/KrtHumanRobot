# 语音模型目录

本目录下的 `voice_models/` **不纳入 Git**（见 `.gitignore`）。首次使用前请下载模型并解压到：

```
voice_assistant/voice_assistant/model/voice_models/
```

## 必需模型（与 `config/voice_assistant.yaml` 默认路径一致）

| 用途 | 目录/文件 | 下载 |
|------|-----------|------|
| ASR | `sherpa-onnx-streaming-zipformer-bilingual-zh-en-2023-02-20/` | [sherpa-onnx ASR 模型](https://github.com/k2-fsa/sherpa-onnx/releases) |
| KWS | `sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20/` | [KWS zh-en 3M](https://github.com/k2-fsa/sherpa-onnx/releases/download/kws-models/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20.tar.bz2) |
| TTS | `vits-melo-tts-zh_en/` | [vits-melo-tts-zh_en](https://github.com/k2-fsa/sherpa-onnx/releases/download/tts-models/vits-melo-tts-zh_en.tar.bz2) |
| VAD | `silero_vad.onnx` | 放入 `voice_models/` 根目录 |

## 快速安装脚本

离线备用 TTS 使用 sherpa-onnx 的 MeloTTS 中英文模型，说话人 ID 为 `0`。
默认仍优先使用讯飞在线合成，失败后按现有策略回退本地。
加载时优先使用 `model.onnx`，避免同目录的可选量化模型影响标准模型加载。
已有旧模型可以保留；模型目录仍可通过 `tts_model_dir` 覆盖。

```bash
# 从源码目录
bash src/voice_assistant/scripts/download_voice_models.sh

# 或指定输出目录
VOICE_MODELS_DIR=/path/to/voice_models bash scripts/download_voice_models.sh
```

或从旧仓库复用已下载模型（**推荐开发机**）：

```bash
ln -sf /path/to/smart-voice-robot/model/voice_models \
  src/voice_assistant/voice_assistant/model/voice_models
```

## 唤醒词

解压 KWS 模型后，用 `text2token` 生成 `keywords.txt`（勿手写）。参见 [sherpa-onnx KWS 文档](https://k2-fsa.github.io/sherpa/onnx/kws/pretrained_models/index.html)。
