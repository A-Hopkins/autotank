add_executable(test_navigation_task
  ${CMAKE_SOURCE_DIR}/src/csc/navigation/navigation_task.cpp
  ${CMAKE_SOURCE_DIR}/src/csc/services/map_service/map_service.cpp
  test_navigation_task.cpp
)

target_compile_definitions(test_navigation_task PRIVATE UNIT_TESTING)

target_include_directories(test_navigation_task PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/external/protocore/include
  ${CMAKE_SOURCE_DIR}/external/kfplusplus/include
)

target_link_libraries(test_navigation_task PRIVATE protocore kfplusplus gtest gtest_main)

add_test(NAME test_navigation_task COMMAND test_navigation_task)
