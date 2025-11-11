cmake_minimum_required(VERSION 3.28.0)

project(aslam_cameras_april
  VERSION 0.0.1
  DESCRIPTION "ASLAM cameras Aprilgrid calibration target"
  LANGUAGES CXX
  )

add_library(${PROJECT_NAME}
  src/GridCalibrationTargetAprilgrid.cpp
  src/GridCalibrationTargetAprilgridv2.cpp
)

find_package(aslam_cameras REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization system)
find_package(Eigen3 REQUIRED)
find_package(ethz_apriltag2 REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_logging REQUIRED)
find_package(sm_boost REQUIRED)
find_package(sm_eigen REQUIRED)
find_package(OpenCV REQUIRED)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  aslam_cameras::aslam_cameras
  Boost::serialization
  Boost::system
  Eigen3::Eigen
  ethz_apriltag2::ethz_apriltag2
  sm_common::sm_common
  sm_logging::sm_logging
  sm_boost::sm_boost
  sm_eigen::sm_eigen
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

enable_testing()
add_subdirectory(test)
