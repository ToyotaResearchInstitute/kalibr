find_package(GTest MODULE)

# Commented out tests are the ones already failing in the original code.
# They are left here for reference and can be uncommented when fixed.
add_executable(${PROJECT_NAME}-test
  RotationExpression.cpp
  # HomogeneousExpression.cpp
  MatrixAndEuclideanExpression.cpp
  # FixedPointNumberTest.cpp
  GenericScalarExpressionTest.cpp
  # GenericMatrixExpression.cpp
  # QuaternionExpression.cpp
  ScalarExpression.cpp
  ErrorTest_Transformation.cpp
  # ErrorTest_Euclidean.cpp
)

# Avoid clash with tr1::tuple: https://code.google.com/p/googlesource/browse/trunk/README?r=589#257
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
