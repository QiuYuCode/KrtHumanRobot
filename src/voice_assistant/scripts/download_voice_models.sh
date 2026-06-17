#!/usr/bin/env bash
# 下载 voice_assistant 默认 sherpa-onnx 模型到 model/voice_models/
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_DIR="${VOICE_MODELS_DIR:-$SCRIPT_DIR/../voice_assistant/model/voice_models}"
mkdir -p "$MODEL_DIR"
cd "$MODEL_DIR"

download_and_extract() {
  local url="$1"
  local name="$2"
  if [[ -d "$name" ]]; then
    echo "[skip] $name 已存在"
    return
  fi
  echo "[download] $name"
  wget -q --show-progress -O "/tmp/${name}.tar.bz2" "$url"
  tar xf "/tmp/${name}.tar.bz2"
  rm -f "/tmp/${name}.tar.bz2"
}

download_and_extract \
  "https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/sherpa-onnx-streaming-zipformer-bilingual-zh-en-2023-02-20.tar.bz2" \
  "sherpa-onnx-streaming-zipformer-bilingual-zh-en-2023-02-20"

download_and_extract \
  "https://github.com/k2-fsa/sherpa-onnx/releases/download/kws-models/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20.tar.bz2" \
  "sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20"

download_and_extract \
  "https://github.com/k2-fsa/sherpa-onnx/releases/download/tts-models/vits-zh-hf-fanchen-C.tar.bz2" \
  "vits-zh-hf-fanchen-C"

if [[ ! -f silero_vad.onnx ]]; then
  echo "[warn] 请手动下载 silero_vad.onnx 到 $MODEL_DIR"
  echo "       可从 smart-voice-robot 复制或参考 sherpa-onnx 文档"
fi

echo "完成。模型目录: $MODEL_DIR"
