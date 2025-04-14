cmake_minimum_required(VERSION 3.15)
project(autotank VERSION 0.1.0 LANGUAGES C CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
option(USE_SIM "Build for Gazebo simulation" True)

# Disable external testing and examples to speed up the build.
set(BUILD_TESTING OFF CACHE BOOL "Disable external tests" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable external examples" FORCE)

# Add protocore subdirectory
# Use the compile defintion USE_CUSTOM_MSG_VARIANT to make protocore build with our version of msg_variant_types.h
add_compile_definitions(USE_CUSTOM_MSG_VARIANT)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external/protocore protocore_build)

# Add kfplusplus
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external/kfplusplus kfplusplus_build)

# include directories BEFORE the target is created
# This is necessary to ensure that protocorebuild is built with our version of msg_variant_types.h
target_include_directories(protocore BEFORE PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include/msg"
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/external/kfplusplus/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/external"
)

# Source files
set(SOURCES
  src/main.cpp
  src/csc/sensors/imu/imu_task.cpp
  src/csc/sensors/odom/odom_task.cpp
)

# Conditionally include the appropriate factory implementation
if(USE_SIM)
  # Find the Gazebo Transport library
  find_package(gz-transport14 QUIET REQUIRED OPTIONAL_COMPONENTS log)
  set(GZ_TRANSPORT_VER ${gz-transport14_VERSION_MAJOR})

  message(STATUS "Building for Gazebo simulation")
  add_definitions(-DUSE_SIM)
  list(APPEND SOURCES
      src/hal/gazebo_imu.cpp
      src/hal/gazebo_odom.cpp
      )

  if(gz-transport14_FOUND)
    message(STATUS "Found Gazebo Transport ${GZ_TRANSPORT_VER}")
  else()
    message(FATAL_ERROR "Gazebo Transport not found. Please install Gazebo Transport 14.")
  endif()
else()
  message(STATUS "Building for hardware")
  add_definitions(-DUSE_HARDWARE)
endif()

# Create executable
add_executable(autotank ${SOURCES})
target_link_libraries(autotank PRIVATE protocore)
target_link_libraries(autotank PRIVATE protocore kfplusplus)

# Include directories for headers
target_include_directories(autotank PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/msg
    ${CMAKE_CURRENT_SOURCE_DIR}/external/protocore/include
    ${CMAKE_CURRENT_SOURCE_DIR}/external
)
# Link Gazebo Transport only if simulation is enabled
if(USE_SIM AND gz-transport14_FOUND)
  # Include Gazebo Transport headers
  target_include_directories(autotank PRIVATE ${gz-transport14_INCLUDE_DIRS})
  
  # Link Gazebo Transport
  target_link_libraries(autotank PRIVATE gz-transport${GZ_TRANSPORT_VER})
endif()
