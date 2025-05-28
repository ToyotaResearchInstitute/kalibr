cmake_minimum_required(VERSION 3.28.0)
project(ethz_apriltag2
  VERSION 0.0.1
  DESCRIPTION "ETHZ AprilTag2 library"
  LANGUAGES CXX
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_library(
  ${PROJECT_NAME}
  src/Edge.cc
  src/FloatImage.cc
  src/GLine2D.cc
  src/GLineSegment2D.cc
  src/Gaussian.cc
  src/GrayModel.cc
  src/Homography33.cc
  src/MathUtil.cc
  src/Quad.cc
  src/Segment.cc
  src/TagDetection.cc
  src/TagDetector.cc
  src/TagFamily.cc
  src/UnionFindSimple.cc
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OpenCV_INCLUDE_DIRS}
)

#library
target_link_libraries(
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

#demo
if(NOT APPLE)
  add_executable(apriltags_demo src/example/apriltags_demo.cpp src/example/Serial.cpp)
  target_link_libraries(apriltags_demo ${PROJECT_NAME} v4l2)
endif()

target_compile_options(
  ${PROJECT_NAME}
  INTERFACE
  "-Wall"
  "-Wextra"
  "-fPIC"
  "-O3"
)

target_compile_features(
  ${PROJECT_NAME}
  INTERFACE
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
