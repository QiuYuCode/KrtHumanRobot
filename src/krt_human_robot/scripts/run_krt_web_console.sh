#!/usr/bin/env bash
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
  echo "错误: 无法定位工作区根目录"
  exit 1
}
UV_PKG_DIR="${VOICE_ASSISTANT_PKG_DIR:-$WS_DIR/src/voice_assistant}"
export UV_CACHE_DIR="${UV_CACHE_DIR:-/tmp/uv-cache}"

source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"
cd "$UV_PKG_DIR"
exec uv run --no-sync gunicorn "$@"
