#!/usr/bin/env bash
# 在 uv 虚拟环境中启动 voice_stack 节点（先 source ROS + colcon）
# 用法: run_voice_stack_node.sh <python_module> [--ros-args ...]
set -eo pipefail

if [[ $# -lt 1 ]]; then
  echo "用法: $0 <python_module> [--ros-args ...]"
  echo "示例: $0 voice_kws.kws_node --ros-args -r __node:=voice_kws"
  exit 1
fi

MODULE="$1"
shift

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

_find_workspace_root() {
  local dir="$1"
  while [[ "$dir" != "/" ]]; do
    if [[ -f "$dir/install/setup.bash" ]]; then
      echo "$dir"
      return 0
    fi
    dir="$(dirname "$dir")"
  done
  return 1
}

WS_DIR="$(_find_workspace_root "$SCRIPT_DIR")" || {
  echo "错误: 无法定位工作区根目录（需包含 install/setup.bash）"
  exit 1
}

UV_PKG_DIR="${VOICE_ASSISTANT_PKG_DIR:-$WS_DIR/src/voice_assistant}"
export UV_CACHE_DIR="${UV_CACHE_DIR:-/tmp/uv-cache}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
else
  echo "错误: 未找到 /opt/ros/humble/setup.bash"
  exit 1
fi

# shellcheck source=/dev/null
source "$WS_DIR/install/setup.bash"

if [[ ! -d "$UV_PKG_DIR/.venv" ]]; then
  echo "错误: 未找到 $UV_PKG_DIR/.venv"
  echo "请先运行: bash $UV_PKG_DIR/scripts/setup_uv.sh"
  exit 1
fi

cd "$UV_PKG_DIR"

if [[ "${VOICE_STACK_SKIP_DEP_CHECK:-0}" != "1" ]]; then
  case "$MODULE" in
    voice_audio_capture.*|voice_playback.playback_node)
      REQUIRED_IMPORTS="import sounddevice"
      REQUIRED_NAMES="sounddevice"
      ;;
    voice_kws.*|voice_audio_process.*|voice_asr.*|voice_tts.*)
      REQUIRED_IMPORTS="import sherpa_onnx"
      REQUIRED_NAMES="sherpa_onnx"
      ;;
    *)
      REQUIRED_IMPORTS=""
      REQUIRED_NAMES=""
      ;;
  esac
  if [[ -n "$REQUIRED_IMPORTS" ]]; then
    uv run --no-sync python -c "$REQUIRED_IMPORTS" 2>/dev/null || {
      echo "错误: uv 环境中缺少或无法加载 $REQUIRED_NAMES"
      echo "请先运行: bash $UV_PKG_DIR/scripts/setup_uv.sh"
      exit 1
    }
  fi
fi

exec uv run --no-sync python -m "$MODULE" "$@"
