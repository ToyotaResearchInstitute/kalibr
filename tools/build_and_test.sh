#!/bin/bash

set -e

# Currently we build and test the packages already migrated to ROS2 / Ubuntu noble.
packages=(
    "aslam_time" \
    "ethz_apriltag2" \
    "sm_common" \
    "sm_logging" \
    "sm_random" \
)

echo "=== Building packages ==="
colcon build --packages-select ${packages[@]}

echo "=== Build complete. Starting tests ==="
colcon test --packages-select ${packages[@]}

echo "=== Tests complete. Showing results ==="
colcon test-result --verbose
