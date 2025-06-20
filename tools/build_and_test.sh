#!/bin/bash

set -e

# Currently we build and test the packages already migrated to ROS2 / Ubuntu noble.
packages=(
    "aslam_backend" \
    "aslam_backend_expressions" \
    "aslam_cameras" \
    "aslam_cv_backend" \
    "aslam_cv_error_terms" \
    "aslam_cv_serialization" \
    "aslam_time" \
    "aslam_imgproc" \
    "aslam_splines" \
    "aslam_time" \
    "bsplines" \
    "ethz_apriltag2" \
    "incremental_calibration" \
    "sm_boost" \
    "sm_common" \
    "sm_eigen" \
    "sm_kinematics" \
    "sm_logging" \
    "sm_opencv" \
    "sm_property_tree" \
    "sm_random" \
    "sm_timing" \
    "sparse_block_matrix" \
)

echo "=== Building packages ==="
colcon build --packages-select ${packages[@]}

echo "=== Build complete. Starting tests ==="
colcon test --packages-select ${packages[@]}

echo "=== Tests complete. Showing results ==="
colcon test-result --verbose
