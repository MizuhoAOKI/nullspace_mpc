#!/usr/bin/env bash
set -e -o pipefail

# Configurable defaults
export ROS_DISTRO=${ROS_DISTRO:-noetic}
export ROS_WS=${ROS_WS:-$HOME/nullspace_mpc}

# Source ROS
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

# Source workspace overlay (install > devel)
if [ -f "${ROS_WS}/install/setup.bash" ]; then
    source "${ROS_WS}/install/setup.bash"
elif [ -f "${ROS_WS}/devel/setup.bash" ]; then
    source "${ROS_WS}/devel/setup.bash"
fi

# Enable bash-completion for interactive TTY
if [ -t 0 ] && [ -f /etc/bash_completion ]; then
    source /etc/bash_completion
fi

exec "$@"