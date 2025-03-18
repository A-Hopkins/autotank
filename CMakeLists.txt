cmake_minimum_required(VERSION 3.15)
project(autotank)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(USE_SIM "Build for Gazebo simulation" ON)

# Source files
set(SOURCES
  src/main.cpp
  src/core/task.cpp
  src/core/state_manager.cpp
  src/csc/sensors/imu/imu_task.cpp
)

# Conditionally include the appropriate factory implementation
if(USE_SIM)
  # Find the Gazebo Transport library
  find_package(gz-transport14 QUIET REQUIRED OPTIONAL_COMPONENTS log)
  set(GZ_TRANSPORT_VER ${gz-transport14_VERSION_MAJOR})

  message(STATUS "Building for Gazebo simulation")
  add_definitions(-DUSE_SIM)
  list(APPEND SOURCES src/hal/gazebo_imu.cpp)

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

# Include directories for headers
target_include_directories(autotank PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Link Gazebo Transport only if simulation is enabled
if(USE_SIM AND gz-transport14_FOUND)
  # Include Gazebo Transport headers
  target_include_directories(autotank PRIVATE ${gz-transport14_INCLUDE_DIRS})
  
  # Link Gazebo Transport
  target_link_libraries(autotank PRIVATE gz-transport${GZ_TRANSPORT_VER})
endif()
