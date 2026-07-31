#!/bin/bash
set -e

# Test from the colcon workspace root regardless of where this was invoked from
cd /ros_ws

if [ -f install/setup.bash ]; then source install/setup.bash; fi
colcon test --merge-install
colcon test-result --all --verbose