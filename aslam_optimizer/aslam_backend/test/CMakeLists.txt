find_package(GTest MODULE)

add_executable(${PROJECT_NAME}-test
    CompressedColumnMatrixTest.cpp
    DenseMatrixTest.cpp
    ErrorTermTests.cpp
    JacobianContainer.cpp
    LinearSolverTests.cpp
    MatrixTestHarness.cpp
    SparseMatrixTest.cpp
    TestOptimizationProblem.cpp
    TestOptimizer.cpp
    test_sparse_matrix_functions.cpp
)

target_link_libraries(
  ${PROJECT_NAME}-test PRIVATE ${PROJECT_NAME} GTest::gmock_main
)

include(CTest)
include(GoogleTest)

gtest_discover_tests(${PROJECT_NAME}-test)
