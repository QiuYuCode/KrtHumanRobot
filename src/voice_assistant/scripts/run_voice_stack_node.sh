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
  uv run python -c "import sounddevice, sherpa_onnx" 2>/dev/null || {
    echo "错误: uv 环境中缺少 sounddevice 或 sherpa_onnx"
    echo "请先运行: bash $UV_PKG_DIR/scripts/setup_uv.sh"
    exit 1
  }
fi

exec uv run python -m "$MODULE" "$@"
