#!/usr/bin/env bash
# Select a usable local X11 session for RViz.

set -u

rviz_display="${KRT_RVIZ_DISPLAY:-${DISPLAY:-}}"
rviz_xauthority="${KRT_RVIZ_XAUTHORITY:-${XAUTHORITY:-}}"
x11_socket_dir="${KRT_X11_SOCKET_DIR:-/tmp/.X11-unix}"

display_socket_exists() {
    [[ "${1:-}" =~ ^:([0-9]+)$ ]] && [[ -S "$x11_socket_dir/X${BASH_REMATCH[1]}" ]]
}

authority_candidates() {
    printf '%s\n' \
        "${rviz_xauthority:-}" \
        "${XAUTHORITY:-}" \
        "${HOME}/.Xauthority" \
        "/run/user/${UID}/gdm/Xauthority"
}

select_working_authority() {
    local candidate
    while IFS= read -r candidate; do
        if [[ -n "$candidate" && -r "$candidate" ]] && \
            DISPLAY="$rviz_display" XAUTHORITY="$candidate" xdpyinfo >/dev/null 2>&1; then
            printf '%s' "$candidate"
            return 0
        fi
    done < <(authority_candidates)
    return 1
}

select_session_display() {
    local session user type display remote state
    while read -r session user _; do
        [[ "$user" == "${USER:-}" ]] || continue
        type="$(loginctl show-session "$session" -p Type --value 2>/dev/null || true)"
        remote="$(loginctl show-session "$session" -p Remote --value 2>/dev/null || true)"
        state="$(loginctl show-session "$session" -p State --value 2>/dev/null || true)"
        display="$(loginctl show-session "$session" -p Display --value 2>/dev/null || true)"
        if [[ "$type" == x11 && "$remote" != yes && "$state" == active ]] && \
            display_socket_exists "$display"; then
            printf '%s' "$display"
            return 0
        fi
    done < <(loginctl list-sessions --no-legend 2>/dev/null || true)
    return 1
}

select_socket_display() {
    local socket display
    for socket in "$x11_socket_dir"/X*; do
        [[ -S "$socket" ]] || continue
        display=":${socket##*/X}"
        [[ "$display" =~ ^:[0-9]+$ ]] || continue
        rviz_display="$display"
        if select_working_authority >/dev/null; then
            printf '%s' "$display"
            return 0
        fi
    done
    return 1
}

if ! display_socket_exists "$rviz_display"; then
    rviz_display=""
    rviz_xauthority=""
fi
if [[ -n "$rviz_display" ]]; then
    rviz_xauthority="$(select_working_authority || true)"
fi
if [[ -z "$rviz_xauthority" ]]; then
    rviz_display=""
    if display_socket_exists "${DISPLAY:-}"; then
        rviz_display="$DISPLAY"
    elif command -v loginctl >/dev/null 2>&1; then
        rviz_display="$(select_session_display || true)"
    fi
    if [[ -z "$rviz_display" ]]; then
        rviz_display="$(select_socket_display || true)"
    fi
    if [[ -n "$rviz_display" ]]; then
        rviz_xauthority="$(select_working_authority || true)"
    fi
fi
if [[ -n "$rviz_display" && -n "$rviz_xauthority" ]]; then
    export DISPLAY="$rviz_display" XAUTHORITY="$rviz_xauthority"
    printf '%s\n' "[krt-rviz-env] DISPLAY=$DISPLAY XAUTHORITY=$XAUTHORITY"
else
    unset DISPLAY XAUTHORITY
    printf '%s\n' '[krt-rviz-env] no usable local X11 session; RViz may be skipped' >&2
fi
