#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_URL="https://github.com/QiuYuCode/lidar_localization_ros2.git"
BRANCH="krt-nav2-map-odom"

if ! git ls-remote --exit-code --heads "$REPO_URL" "$BRANCH" >/dev/null; then
  echo "ERROR: remote branch $BRANCH does not exist on $REPO_URL" >&2
  echo "Create and push it from QiuYuCode/lidar_localization_ros2 humble before importing." >&2
  exit 1
fi

if ! command -v vcs >/dev/null; then
  echo "ERROR: vcs command not found. Install python3-vcstool first." >&2
  exit 1
fi

vcs import "$ROOT/src" < "$ROOT/third_party/lidar_localization_ros2.repos"
