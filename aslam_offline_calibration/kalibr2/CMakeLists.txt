cmake_minimum_required(VERSION 3.28.0)

project(kalibr2
  VERSION 0.0.1
  DESCRIPTION "Kalibr calibration functionality"
  LANGUAGES CXX
)

find_package(aslam_cameras REQUIRED)
find_package(aslam_cameras_april REQUIRED)
find_package(aslam_time REQUIRED)
find_package(aslam_cv_error_terms REQUIRED)
find_package(incremental_calibration REQUIRED)
find_package(OpenCV REQUIRED)
find_package(simple_graph REQUIRED)

add_library(${PROJECT_NAME}
  src/BasicMathUtils.cpp
  src/CameraGraph.cpp
  src/Image.cpp
  src/SynchronizedObservationView.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  aslam_cameras::aslam_cameras
  aslam_cameras_april::aslam_cameras_april
  aslam_time::aslam_time
  aslam_cv_error_terms::aslam_cv_error_terms
  incremental_calibration::incremental_calibration
  ${OpenCV_LIBS}
  simple_graph::simple_graph
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
  cxx_std_17
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
install(DIRECTORY test/data/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/test/data)
add_subdirectory(test)
