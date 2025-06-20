
cmake_minimum_required(VERSION 3.28.0)

project(aslam_cv_serialization
  VERSION 0.1.0
  LANGUAGES CXX
)

find_package(aslam_cameras REQUIRED)
find_package(aslam_time REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization system)
find_package(OpenCV REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_boost REQUIRED)
find_package(sm_property_tree REQUIRED)
find_package(sm_eigen REQUIRED)
find_package(sm_kinematics REQUIRED)
find_package(sm_opencv REQUIRED)
find_package(sm_logging REQUIRED)
find_package(sm_random REQUIRED)

#common commands for building c++ executables and libraries
include(autogen_cameras.cmake)

add_library(${PROJECT_NAME}
  ${AUTOGEN_CAMERA_CPP_FILES}
  src/FrameBaseSerialization.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC
  Boost::serialization
  Boost::system
  aslam_cameras::aslam_cameras
  aslam_time::aslam_time
  sm_common::sm_common
  sm_boost::sm_boost
  sm_property_tree::sm_property_tree
  sm_eigen::sm_eigen
  sm_kinematics::sm_kinematics
  sm_opencv::sm_opencv
  sm_logging::sm_logging
  sm_random::sm_random
  ${OpenCV_LIBS}
)

target_compile_options(
  ${PROJECT_NAME}
  PUBLIC
  "-Wall"
  "-Wextra"
)

target_compile_features(
  ${PROJECT_NAME}
  PUBLIC
  cxx_std_14
)

set(INSTALL_CMAKEDIR ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/cmake)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${INSTALL_CMAKEDIR})

include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${INSTALL_CMAKEDIR})
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  COMPATIBILITY SameMajorVersion)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION ${INSTALL_CMAKEDIR})
