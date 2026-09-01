#!/usr/bin/env bash
# Wait for the hardware/network prerequisites of a KrtHumanRobot user service.

set -u

mode="${1:-}"
timeout_s="${KRT_STARTUP_TIMEOUT_S:-90}"
interval_s="${KRT_WAIT_INTERVAL_S:-1}"

log() {
    printf '%s\n' "[krt-wait-ready] $*"
}

if ! [[ "$timeout_s" =~ ^[0-9]+$ ]]; then
    log "KRT_STARTUP_TIMEOUT_S must be a non-negative integer: $timeout_s"
    exit 64
fi

if ! [[ "$interval_s" =~ ^[0-9]+$ ]] || [[ "$interval_s" == "0" && "$timeout_s" != "0" ]]; then
    log "KRT_WAIT_INTERVAL_S must be a positive integer unless timeout is zero: $interval_s"
    exit 64
fi

x86_ready() {
    local address_ready=no

    OBSERVED_IPV4="$(ip -o -4 addr show 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | paste -sd, -)"
    if [[ -z "${DDS_BIND_ADDRESS:-}" ]]; then
        log "KRT_DDS_BIND_ADDRESS or a file:// CYCLONEDDS_URI is required for x86 readiness"
        return 1
    fi

    if grep -Fxq "$DDS_BIND_ADDRESS" <<<"${OBSERVED_IPV4//,/$'\n'}"; then
        address_ready=yes
    fi

    global_network_ready
    can_buses_ready
    [[ "$address_ready" == yes && "$GLOBAL_NETWORK_READY" == yes && "$CAN_BUSES_READY" == yes ]]
}

global_network_ready() {
    GLOBAL_NETWORK_STATE=disabled
    GLOBAL_NETWORK_READY=yes
    if [[ "$REQUIRE_GLOBAL_NETWORK" != yes ]]; then
        return 0
    fi

    GLOBAL_NETWORK_READY=no
    if ! command -v nmcli >/dev/null 2>&1; then
        GLOBAL_NETWORK_STATE=nmcli_missing
        return 1
    fi

    GLOBAL_NETWORK_STATE="$(nmcli -t -f STATE general 2>/dev/null | head -n 1)"
    if [[ "$GLOBAL_NETWORK_STATE" == connected ]]; then
        GLOBAL_NETWORK_READY=yes
        return 0
    fi
    [[ -n "$GLOBAL_NETWORK_STATE" ]] || GLOBAL_NETWORK_STATE=unknown
    return 1
}

can_buses_ready() {
    local can_port can_details can_state can_rx_packets

    CAN_BUSES_READY=yes
    CAN_BUS_STATUS=disabled
    [[ -n "$REQUIRED_CAN_CHANNELS" ]] || return 0

    CAN_BUS_STATUS=
    IFS=',' read -r -a can_ports <<<"$REQUIRED_CAN_CHANNELS"
    for can_port in "${can_ports[@]}"; do
        can_port="${can_port//[[:space:]]/}"
        [[ -n "$can_port" ]] || continue
        can_details="$(ip -details -statistics link show dev "$can_port" 2>/dev/null || true)"
        can_state="$(sed -nE 's/.*can state ([A-Z-]+).*/\1/p' <<<"$can_details" | head -n 1)"
        can_rx_packets="$(awk '/RX:/{getline; print $2; exit}' <<<"$can_details")"
        [[ "$can_rx_packets" =~ ^[0-9]+$ ]] || can_rx_packets=0
        if [[ "$can_state" != ERROR-ACTIVE || "$can_rx_packets" == 0 ]]; then
            CAN_BUSES_READY=no
        fi
        CAN_BUS_STATUS+="${CAN_BUS_STATUS:+,}${can_port}=state:${can_state:-missing},rx_packets:${can_rx_packets}"
    done
}
jetson_ready() {
    local devices

    JETSON_LEFT_READABLE=no
    JETSON_RIGHT_READABLE=no
    JETSON_REALSENSE_STATUS=not_checked
    [[ -r "$JETSON_LEFT_CAMERA" ]] && JETSON_LEFT_READABLE=yes
    [[ -r "$JETSON_RIGHT_CAMERA" ]] && JETSON_RIGHT_READABLE=yes
    [[ "$JETSON_LEFT_READABLE" == yes && "$JETSON_RIGHT_READABLE" == yes ]] || return 1
    command -v rs-enumerate-devices >/dev/null 2>&1 || {
        JETSON_REALSENSE_STATUS=command_missing
        return 1
    }
    devices="$(rs-enumerate-devices -s 2>&1)" || {
        JETSON_REALSENSE_STATUS=probe_failed
        return 1
    }
    if [[ "$devices" == *"Intel RealSense D435"* ]]; then
        JETSON_REALSENSE_STATUS=ready
        return 0
    fi
    JETSON_REALSENSE_STATUS=not_detected
    return 1
}

log_readiness_state() {
    case "$mode" in
        x86)
            log "target_ipv4=${DDS_BIND_ADDRESS:-unset} observed_ipv4=${OBSERVED_IPV4:-none} global_network=${GLOBAL_NETWORK_STATE:-unknown} can_buses=${CAN_BUS_STATUS:-unknown}"
            ;;
        jetson)
            log "left_camera=$JETSON_LEFT_CAMERA readable=$JETSON_LEFT_READABLE right_camera=$JETSON_RIGHT_CAMERA readable=$JETSON_RIGHT_READABLE realsense_d435=$JETSON_REALSENSE_STATUS"
            ;;
    esac
}

case "$mode" in
    x86)
        DDS_BIND_ADDRESS="${KRT_DDS_BIND_ADDRESS:-}"
        if [[ -z "$DDS_BIND_ADDRESS" && "${CYCLONEDDS_URI:-}" == file://* ]]; then
            cyclone_config="${CYCLONEDDS_URI#file://}"
            if [[ -r "$cyclone_config" ]]; then
                DDS_BIND_ADDRESS="$(sed -nE '/<NetworkInterface/ s/.*address="([^"]+)".*/\1/p' "$cyclone_config" | head -n 1)"
            fi
        fi
        REQUIRE_GLOBAL_NETWORK="${KRT_REQUIRE_GLOBAL_NETWORK:-yes}"
        REQUIRED_CAN_CHANNELS="${KRT_CAN_REQUIRED_CHANNELS:-}"
        if [[ "$REQUIRE_GLOBAL_NETWORK" != yes && "$REQUIRE_GLOBAL_NETWORK" != no ]]; then
            log "KRT_REQUIRE_GLOBAL_NETWORK must be yes or no: $REQUIRE_GLOBAL_NETWORK"
            exit 64
        fi
        ready=x86_ready
        ready_message="DDS bind address, global network, and required CAN buses ready"
        waiting_message="waiting for DDS bind address ${DDS_BIND_ADDRESS:-unset}, global network, and required CAN buses"
        ;;
    jetson)
        JETSON_LEFT_CAMERA="${KRT_CAMERA_LEFT_PATH:-/dev/camera_left}"
        JETSON_RIGHT_CAMERA="${KRT_CAMERA_RIGHT_PATH:-/dev/camera_right}"
        ready=jetson_ready
        ready_message="Jetson cameras ready"
        waiting_message="waiting for /dev/camera_left, /dev/camera_right, and Intel RealSense D435"
        ;;
    *)
        log "usage: $0 {x86|jetson}"
        exit 64
        ;;
esac

deadline=$((SECONDS + timeout_s))
last_wait_log=$((SECONDS - 10))
log "startup mode=$mode timeout_s=$timeout_s interval_s=$interval_s"
while true; do
    if "$ready"; then
        log_readiness_state
        log "$ready_message"
        exit 0
    fi

    if (( SECONDS >= deadline )); then
        log_readiness_state
        log "timed out after ${timeout_s}s: $waiting_message"
        exit 1
    fi

    if (( SECONDS - last_wait_log >= 10 )); then
        log_readiness_state
        log "$waiting_message"
        last_wait_log=$SECONDS
    fi
    sleep "$interval_s"
done
