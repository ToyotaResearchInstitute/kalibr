find_package(GTest MODULE)

find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(${PROJECT_NAME}-test
  BagReaderTest.cpp
)

target_link_libraries(
  ${PROJECT_NAME}-test ${PROJECT_NAME} GTest::gmock_main
)

target_compile_definitions(${PROJECT_NAME}-test
  PRIVATE
    TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
)

include(CTest)
include(GoogleTest)

gtest_discover_tests(${PROJECT_NAME}-test)
