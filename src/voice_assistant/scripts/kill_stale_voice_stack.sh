#!/usr/bin/env bash
# 启动前清理残留的 voice_stack / voice_assistant 进程，避免重复 TTS/播放。
set -eo pipefail

patterns=(
  "voice_assistant\\.voice_node"
  "voice_tts\\.tts_node"
  "voice_playback\\.playback_node"
  "voice_kws\\.kws_node"
  "voice_asr\\.asr_node"
  "voice_audio_capture\\.capture_node"
  "voice_audio_process\\.process_node"
  "voice_volume\\.volume_node"
)

killed=0
for pattern in "${patterns[@]}"; do
  if pgrep -f "$pattern" >/dev/null 2>&1; then
    pkill -f "$pattern" 2>/dev/null || true
    killed=1
  fi
done

if [[ "$killed" -eq 1 ]]; then
  for _ in {1..30}; do
    remaining=0
    for pattern in "${patterns[@]}"; do
      if pgrep -f "$pattern" >/dev/null 2>&1; then
        remaining=1
        break
      fi
    done
    if [[ "$remaining" -eq 0 ]]; then
      echo "[voice] 已清理残留语音进程"
      exit 0
    fi
    sleep 0.1
  done

  for pattern in "${patterns[@]}"; do
    pkill -9 -f "$pattern" 2>/dev/null || true
  done
  echo "[voice] 已强制清理未退出的残留语音进程"
fi
