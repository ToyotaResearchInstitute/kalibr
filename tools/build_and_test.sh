#!/bin/bash

set -e

source /opt/ros/${ROS_DISTRO}/setup.bash

echo "=== Building packages ==="
colcon build

echo "=== Build complete. Starting tests ==="
colcon test

echo "=== Tests complete. Showing results ==="
colcon test-result --verbose
