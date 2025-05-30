cmake_minimum_required(VERSION 3.28.0)

project(aslam_time
  VERSION 0.0.1
  DESCRIPTION "Time utilities for aslam_cv"
  LANGUAGES CXX
)

find_package(Boost REQUIRED COMPONENTS system)

add_library(${PROJECT_NAME}
  src/time.cpp
  src/duration.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

set(OTHER_LIBS)
if(NOT APPLE)
  set(OTHER_LIBS rt)
endif()

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  Boost::system
  ${OTHER_LIBS}
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
