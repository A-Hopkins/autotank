add_executable(test_map_service
  ${CMAKE_SOURCE_DIR}/src/csc/services/map_service/map_service.cpp
  test_map_service.cpp
)

target_compile_definitions(test_map_service PRIVATE UNIT_TESTING)

target_include_directories(test_map_service PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/external/protocore/include
  ${CMAKE_SOURCE_DIR}/external/kfplusplus/include
)

target_link_libraries(test_map_service PRIVATE protocore kfplusplus gtest gtest_main)

add_test(NAME test_map_service COMMAND test_map_service)
