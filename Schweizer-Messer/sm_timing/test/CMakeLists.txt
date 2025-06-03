find_package(GTest MODULE)

add_executable(${PROJECT_NAME}-test
  TestNsecTimeUtilities.cpp
  TestTimestampCorrector.cpp
)

target_link_libraries(
  ${PROJECT_NAME}-test
  PRIVATE
  ${PROJECT_NAME}
  GTest::gmock_main
  sm_random::sm_random
)

include(CTest)
include(GoogleTest)

gtest_discover_tests(${PROJECT_NAME}-test)
