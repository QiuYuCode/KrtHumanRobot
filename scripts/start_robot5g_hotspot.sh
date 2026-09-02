#!/usr/bin/env bash
set -euo pipefail

IFACE="wlp5s0"
CON_NAME="Robot5G"
SSID="Robot5G"
PASSWORD="createrobot"
CHANNEL="149"
REG_DOMAIN="CN"

# Wait for NetworkManager and the Wi-Fi device to appear.
for _ in $(seq 1 30); do
  if nmcli -t -f DEVICE dev status | grep -Fxq "$IFACE"; then
    break
  fi
  sleep 1
done

# Best-effort regulatory domain setup for 5GHz channel availability.
iw reg set "$REG_DOMAIN" || true

nmcli radio wifi on || true

# Recreate the hotspot profile to guarantee band/channel/password are correct.
if nmcli -t -f NAME con show | grep -Fxq "$CON_NAME"; then
  nmcli con down "$CON_NAME" >/dev/null 2>&1 || true
  nmcli con delete "$CON_NAME" >/dev/null 2>&1 || true
fi

nmcli dev wifi hotspot \
  ifname "$IFACE" \
  con-name "$CON_NAME" \
  ssid "$SSID" \
  band a \
  channel "$CHANNEL" \
  password "$PASSWORD"

# Best-effort: request higher TX power. Some mt7921e firmware keeps AP mode at 3 dBm.
iw dev "$IFACE" set txpower fixed 2000 || true

# Print final state for journalctl diagnostics.
iw dev "$IFACE" info || true
