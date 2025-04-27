add_executable(test_motion_control_task
  ${CMAKE_SOURCE_DIR}/src/csc/control/motion_control/motion_control_task.cpp
  test_motion_control_task.cpp
  mock_diff_drive.cpp
)

target_compile_definitions(test_motion_control_task PRIVATE UNIT_TESTING)

target_include_directories(test_motion_control_task PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/external/protocore/include
  ${CMAKE_SOURCE_DIR}/external/kfplusplus/include
)

target_link_libraries(test_motion_control_task PRIVATE protocore kfplusplus gtest gtest_main)

add_test(NAME test_motion_control_task COMMAND test_motion_control_task)
