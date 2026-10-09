#!/usr/bin/env bash
# ROS Humble setup scripts intentionally read optional unset variables.
set -eo pipefail

unset PYTHONPATH PYTHONHOME PYTHONNOUSERSITE CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER
source /opt/ros/humble/setup.bash
source "${KRT_WORKSPACE:?KRT_WORKSPACE is required}/install/setup.bash"

: "${KRT_QUEST_VT_BIN:?Set KRT_QUEST_VT_BIN to the vt environment bin directory}"
export PATH="${KRT_QUEST_VT_BIN}:${PATH}"
export PYTHONNOUSERSITE=1

python3 -c 'from pinocchio import casadi; from ppadb.client import Client'

quest_rviz="${KRT_QUEST_RVIZ:-true}"
if [[ "$quest_rviz" == "true" ]]; then
    source "${KRT_WORKSPACE}/deploy/systemd/krt-rviz-env.sh"
    if [[ -z "${DISPLAY:-}" || -z "${XAUTHORITY:-}" ]]; then
        echo "[krt-quest-teleop] no local X11 session; starting without RViz" >&2
        quest_rviz=false
    fi
fi

exec ros2 launch oculus_reader teleop_double_nero_web.launch.py "rviz:=${quest_rviz}"
