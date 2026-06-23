#!/usr/bin/env bash
# Run the core node inside the voice_assistant uv environment.
set -eo pipefail

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
exec uv run python -m krt_human_robot.robot_node "$@"
