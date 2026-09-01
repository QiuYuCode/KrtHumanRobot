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
    OBSERVED_IPV4="$(ip -o -4 addr show 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | paste -sd, -)"
    if [[ -z "${DDS_BIND_ADDRESS:-}" ]]; then
        log "KRT_DDS_BIND_ADDRESS or a file:// CYCLONEDDS_URI is required for x86 readiness"
        return 1
    fi

    grep -Fxq "$DDS_BIND_ADDRESS" <<<"${OBSERVED_IPV4//,/$'\n'}"
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
            log "target_ipv4=${DDS_BIND_ADDRESS:-unset} observed_ipv4=${OBSERVED_IPV4:-none}"
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
        ready=x86_ready
        ready_message="DDS bind address ready: ${DDS_BIND_ADDRESS:-unset}"
        waiting_message="waiting for DDS bind address ${DDS_BIND_ADDRESS:-unset}"
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
