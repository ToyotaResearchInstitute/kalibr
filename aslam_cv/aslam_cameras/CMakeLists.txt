cmake_minimum_required(VERSION 3.28.0)

project(aslam_cameras
  VERSION 0.1.0
  DESCRIPTION "ASLAM Camera Models and Geometry"
  LANGUAGES CXX
)

find_package(Boost REQUIRED COMPONENTS filesystem serialization system)
find_package(Eigen3 REQUIRED)
find_package(aslam_time REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_boost REQUIRED)
find_package(sm_eigen REQUIRED)
find_package(sm_kinematics REQUIRED)
find_package(sm_logging REQUIRED)
find_package(sm_opencv REQUIRED)
find_package(sm_property_tree REQUIRED)
find_package(sm_random REQUIRED)
find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME}
  src/BackProjection.cpp
  src/CameraGeometryBase.cpp
  src/DescriptorBase.cpp
  src/EquidistantDistortion.cpp
  src/FovDistortion.cpp
  src/FrameBase.cpp
  src/GlobalShutter.cpp
  src/GridCalibrationTargetBase.cpp
  src/GridCalibrationTargetCheckerboard.cpp
  src/GridCalibrationTargetCirclegrid.cpp
  src/GridCalibrationTargetObservation.cpp
  src/GridDetector.cpp
  src/Image.cpp
  src/ImageMask.cpp
  src/KeypointBase.cpp
  src/Landmark.cpp
  src/NoDistortion.cpp
  src/NoMask.cpp
  src/RadialTangentialDistortion.cpp
  src/RollingShutter.cpp
  src/Triangulation.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  Boost::filesystem
  Boost::serialization
  Boost::system
  Eigen3::Eigen
  aslam_time::aslam_time
  sm_common::sm_common
  sm_boost::sm_boost
  sm_eigen::sm_eigen
  sm_kinematics::sm_kinematics
  sm_logging::sm_logging
  sm_opencv::sm_opencv
  sm_property_tree::sm_property_tree
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

target_compile_options(${PROJECT_NAME} PUBLIC "-Wno-deprecated-copy") # TODO: Fix...

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
