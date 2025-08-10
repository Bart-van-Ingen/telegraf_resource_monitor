#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers
# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "Sourced ROS 2 ${ROS_DISTRO}"

# If the workspace exists, source the workspace
if [ -d "/ros_ws" ]; then
    source /ros_ws/install/setup.bash
    echo "Sourced workspace at /ros_ws"
fi

# Execute the command passed into this entrypoint
exec "$@"
