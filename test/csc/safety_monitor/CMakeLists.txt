add_executable(test_safety_monitor_task
  ${CMAKE_SOURCE_DIR}/src/csc/safety_monitor/safety_monitor_task.cpp
  test_safety_monitor_task.cpp
)

target_compile_definitions(test_safety_monitor_task PRIVATE UNIT_TESTING)

target_include_directories(test_safety_monitor_task PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/external/protocore/include
  ${CMAKE_SOURCE_DIR}/external/kfplusplus/include
)

target_link_libraries(test_safety_monitor_task PRIVATE protocore kfplusplus gtest gtest_main)

add_test(NAME test_safety_monitor_task COMMAND test_safety_monitor_task)
