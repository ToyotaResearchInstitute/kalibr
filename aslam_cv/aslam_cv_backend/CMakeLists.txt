cmake_minimum_required(VERSION 3.28.0)
project(aslam_cv_backend
  VERSION 0.1.0
  LANGUAGES CXX
)

find_package(aslam_backend REQUIRED)
find_package(aslam_backend_expressions REQUIRED)
find_package(aslam_time REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(sm_boost REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_eigen REQUIRED)
find_package(sm_kinematics REQUIRED)
find_package(sm_logging REQUIRED)
find_package(sm_opencv REQUIRED)
find_package(sm_property_tree REQUIRED)
find_package(sm_timing REQUIRED)
find_package(sparse_block_matrix REQUIRED)


add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(
  ${PROJECT_NAME}
  INTERFACE
  aslam_backend::aslam_backend
  aslam_backend_expressions::aslam_backend_expressions
  aslam_time::aslam_time
  Boost::system
  sm_boost::sm_boost
  sm_common::sm_common
  sm_eigen::sm_eigen
  sm_kinematics::sm_kinematics
  sm_logging::sm_logging
  sm_opencv::sm_opencv
  sm_property_tree::sm_property_tree
  sm_timing::sm_timing
  sparse_block_matrix::sparse_block_matrix
)

target_compile_options(
  ${PROJECT_NAME}
  INTERFACE
  "-Wall"
  "-Wextra"
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
