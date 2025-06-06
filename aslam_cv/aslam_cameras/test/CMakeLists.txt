find_package(GTest MODULE)

add_executable(${PROJECT_NAME}-test
  PinholeCameraGeometry.cpp
  OmniCameraGeometry.cpp
  EquidistantDistortion.cpp
  ExtendedUnifiedCameraGeometry.cpp
  DoubleSphereCameraGeometry.cpp
  RadialTangentialDistortion.cpp
  EquidistantDistortion.cpp
  FovDistortion.cpp
  Triangulation.cpp
  testFrame.cpp
  GridCalibration.cpp
)

file(COPY ${CMAKE_SOURCE_DIR}/test/testImageCheckerboard.jpg ${CMAKE_SOURCE_DIR}/test/testImageCircleGrid.jpg DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
target_compile_definitions(
  ${PROJECT_NAME}-test PRIVATE
  GTEST_USE_OWN_TR1_TUPLE=0
)

target_link_libraries(
  ${PROJECT_NAME}-test PRIVATE ${PROJECT_NAME} GTest::gmock_main
)

include(CTest)
include(GoogleTest)

gtest_discover_tests(${PROJECT_NAME}-test)
