include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.17.0
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

# Add subdirectories for different CSC groups
add_subdirectory(csc/control/motion_control)
add_subdirectory(csc/localization)
add_subdirectory(csc/mapping)
add_subdirectory(csc/navigation)
add_subdirectory(csc/safety_monitor)
add_subdirectory(csc/services/map_service)
