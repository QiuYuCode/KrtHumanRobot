#!/usr/bin/env bash
set -euo pipefail
IFACE="${1:?missing interface}"
BITRATE="${2:?missing bitrate}"
IP_BIN="$(command -v ip)"
# udev时序下接口可能刚创建，短重试提高成功率
for _ in {1..15}; do
  if "$IP_BIN" link show "$IFACE" >/dev/null 2>&1; then
    "$IP_BIN" link set "$IFACE" down 2>/dev/null || true
    "$IP_BIN" link set "$IFACE" up type can bitrate "$BITRATE"
    exit 0
  fi
  sleep 0.2
done
exit 1
