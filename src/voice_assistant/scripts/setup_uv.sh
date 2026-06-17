#!/usr/bin/env bash
# 创建 voice_assistant 专用 uv 虚拟环境（继承 ROS apt 包，隔离语音 pip 依赖）
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PKG_DIR"

if ! command -v uv >/dev/null 2>&1; then
  echo "错误: 未找到 uv，请先安装: curl -LsSf https://astral.sh/uv/install.sh | sh"
  exit 1
fi

echo "[setup_uv] 包目录: $PKG_DIR"
echo "[setup_uv] 创建 .venv（--system-site-packages 以使用 apt 的 rclpy / py_trees_ros）"

uv venv --python python3.10 --system-site-packages .venv
uv sync

echo ""
echo "完成。运行前请确保已 colcon build voice_assistant。"
echo "  cd ../../ && colcon build --packages-select voice_assistant --symlink-install"
echo "  ./src/voice_assistant/scripts/run_voice_node.sh"
echo ""
echo "可选：仍使用 SDK 直连臂/手时"
echo "  cd $PKG_DIR && uv sync --extra arm --extra hand"
