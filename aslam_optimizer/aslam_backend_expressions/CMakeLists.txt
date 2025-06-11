cmake_minimum_required(VERSION 3.28.0)
project(aslam_backend_expressions
  VERSION 0.1.0
  LANGUAGES CXX
)

find_package(aslam_backend REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(sm_boost REQUIRED)
find_package(sm_random REQUIRED)
find_package(sm_kinematics REQUIRED)
find_package(sm_timing REQUIRED)
find_package(sparse_block_matrix REQUIRED)

add_library(${PROJECT_NAME}
  src/ErrorTermEuclidean.cpp
  src/ErrorTermTransformation.cpp
  src/EuclideanDirection.cpp
  src/EuclideanExpression.cpp
  src/EuclideanExpressionNode.cpp
  src/EuclideanPoint.cpp
  src/HomogeneousExpression.cpp
  src/HomogeneousExpressionNode.cpp
  src/HomogeneousPoint.cpp
  src/MappedEuclideanPoint.cpp
  src/MappedHomogeneousPoint.cpp
  src/MappedRotationQuaternion.cpp
  src/MapTransformation.cpp
  src/MatrixBasic.cpp
  src/MatrixExpression.cpp
  src/MatrixExpression.cpp
  src/MatrixExpressionNode.cpp
  src/MatrixExpressionNode.cpp
  src/MatrixTransformation.cpp
  src/RotationExpression.cpp
  src/RotationExpressionNode.cpp
  src/RotationQuaternion.cpp
  src/Scalar.cpp
  src/ScalarExpression.cpp
  src/ScalarExpressionNode.cpp
  src/TransformationBasic.cpp
  src/TransformationExpression.cpp
  src/TransformationExpressionNode.cpp
  src/Vector2RotationQuaternionExpressionAdapter.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC
  aslam_backend::aslam_backend
  Boost::system
  Eigen3::Eigen
  sm_boost::sm_boost
  sm_random::sm_random
  sm_kinematics::sm_kinematics
  sm_timing::sm_timing
  sparse_block_matrix::sparse_block_matrix
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
