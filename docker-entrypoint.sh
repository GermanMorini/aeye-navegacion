#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source workspace if exists
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Set ROS 2 domain ID (default 0)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Allow ROS 2 communication outside localhost
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

echo "========================================="
echo "ROS 2 ${ROS_DISTRO} Environment Ready"
echo "========================================="
echo "Workspace: /ros2_ws"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY}"
echo "========================================="

# Execute command
exec "$@"
