find_package(GTest MODULE)

add_executable(${PROJECT_NAME}-test
  CameraGraphTest.cpp
  ImageTest.cpp
  SynchronizedObservationViewTest.cpp
)

target_link_libraries(
  ${PROJECT_NAME}-test PRIVATE ${PROJECT_NAME} GTest::gmock_main
)

include(CTest)
include(GoogleTest)

gtest_discover_tests(${PROJECT_NAME}-test)
