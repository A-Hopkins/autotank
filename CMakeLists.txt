cmake_minimum_required(VERSION 3.15)
project(autotank)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(USE_SIM "Build for Gazebo simulation" ON)

# Source files
set(SOURCES
  src/main.cpp
  src/core/base_task.cpp
  src/core/state_manager.cpp
)

# Conditionally include the appropriate factory implementation
if(USE_SIM)
  message(STATUS "Building for Gazebo simulation")
  add_definitions(-DUSE_SIM)
  list(APPEND SOURCES src/hal/gazebo_factory.cpp)
else()
  message(STATUS "Building for hardware")
  add_definitions(-DUSE_HARDWARE)
endif()

# Create executable
add_executable(autotank ${SOURCES})

# Include directories for headers
target_include_directories(autotank PUBLIC ${CMAKE_SOURCE_DIR}/include)
