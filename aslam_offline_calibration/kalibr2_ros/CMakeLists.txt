cmake_minimum_required(VERSION 3.28.0)

project(kalibr2_ros
  VERSION 0.0.1
  DESCRIPTION "Kalibr ROS interface for reading ROS2 bags"
  LANGUAGES CXX
)

include(FetchContent)

FetchContent_Declare(
  cli11
  GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
  GIT_TAG v2.4.2
)
FetchContent_MakeAvailable(cli11)

find_package(ament_cmake REQUIRED)
find_package(aslam_time REQUIRED)
find_package(kalibr2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Temporal include directories, till encapsulation is done
find_package(incremental_calibration REQUIRED)

# CLI deps
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}
  src/BagReader.cpp
  src/Config.cpp
  src/KalibrToROSConverter.cpp
  src/ROSToYAMLConverter.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  aslam_time::aslam_time
  kalibr2::kalibr2
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  rosbag2_transport::rosbag2_transport
  rosidl_runtime_cpp::rosidl_runtime_cpp
  ${OpenCV_LIBS}
  yaml-cpp
  # Temporal linking
  incremental_calibration::incremental_calibration
)

ament_target_dependencies(
  ${PROJECT_NAME}
  cv_bridge
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
  rosidl_runtime_cpp
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_msgs
)

add_executable(kalibr_calibrate_cameras
  src/CalibrateCameras.cpp
)

target_link_libraries(kalibr_calibrate_cameras
  ${PROJECT_NAME}
  CLI11::CLI11
)

install(
  TARGETS ${PROJECT_NAME} kalibr_calibrate_cameras
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  # Need to use this path for executables as well
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

enable_testing()
add_subdirectory(test)

ament_package()
