add_executable(test_localization_task
  ${CMAKE_SOURCE_DIR}/src/csc/localization/localization_task.cpp
  test_localization_task.cpp
)

target_compile_definitions(test_localization_task PRIVATE UNIT_TESTING)

target_include_directories(test_localization_task PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/external/protocore/include
  ${CMAKE_SOURCE_DIR}/external/kfplusplus/include
)

target_link_libraries(test_localization_task PRIVATE protocore kfplusplus gtest gtest_main)

add_test(NAME test_localization_task COMMAND test_localization_task)
