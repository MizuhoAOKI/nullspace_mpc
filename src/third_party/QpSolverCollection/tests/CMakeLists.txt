set(QpSolverCollection_gtest_list
  TestQpSolversEnabled
  TestSampleQP
  )

foreach(NAME IN LISTS QpSolverCollection_gtest_list)
  if(DEFINED CATKIN_DEVEL_PREFIX)
    catkin_add_gtest(${NAME} ${NAME}.cpp)
  else()
    find_package(GTest REQUIRED)
    include(GoogleTest)
    add_executable(${NAME} ${NAME}.cpp)
    target_link_libraries(${NAME} GTest::gtest)
    gtest_discover_tests(${NAME})
  endif()
  if(SKIP_PRIVATE_SOLVER_TEST)
    target_compile_definitions(${NAME} PRIVATE SKIP_PRIVATE_SOLVER_TEST)
  endif()
  if(FORCE_ALL_SOLVER_TEST)
    target_compile_definitions(${NAME} PRIVATE FORCE_ALL_SOLVER_TEST)
  endif()
  target_link_libraries(${NAME} QpSolverCollection)
endforeach()
