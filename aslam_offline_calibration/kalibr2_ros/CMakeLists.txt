cmake_minimum_required(VERSION 3.28.0)

project(kalibr2_ros
  VERSION 0.0.1
  DESCRIPTION "Kalibr ROS interface for reading ROS2 bags"
  LANGUAGES CXX
)

find_package(ament_cmake REQUIRED)
find_package(aslam_time REQUIRED)
find_package(kalibr2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

add_library(${PROJECT_NAME}
  src/BagReader.cpp
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
  ${OpenCV_LIBS}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  cv_bridge
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
  rclcpp
  sensor_msgs
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

enable_testing()
add_subdirectory(test)

ament_package()
