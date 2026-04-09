#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export ROS_DOMAIN_ID=9
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

# ROS setup scripts may reference variables before initialization.
# Temporarily disable nounset to avoid "unbound variable" failures.
set +u
source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"
set -u

exec ros2 launch kistar_hand_bridge hand_bridge.launch.py
