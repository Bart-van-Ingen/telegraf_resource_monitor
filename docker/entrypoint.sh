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

# Flush shell history after every command instead of at shell exit. A devcontainer rebuild
# (or `docker compose down`) SIGKILLs the container, so an exit-time write never happens and
# the session's history is lost. `history -a` appends new lines immediately; `history -n`
# reads back lines other shells have appended, so Ctrl-R sees them without a restart.
# Only meaningful in the interactive shells that source this file via .bashrc; harmless
# when this script runs as the container ENTRYPOINT.
PROMPT_COMMAND='history -a; history -n'

# Execute the command passed into this entrypoint
exec "$@"
