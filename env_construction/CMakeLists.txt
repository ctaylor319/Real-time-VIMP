cmake_minimum_required(VERSION 3.8)
project(env_construction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(grid_map REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_octomap REQUIRED)
find_package(grid_map_ros REQUIRED)

# Define a CMake variable named dependencies that lists all
# ROS 2 packages and other dependencies the project requires.
set(dependencies
  rclcpp
  PCL
  Eigen3
  octomap
  octomap_msgs
  grid_map
  grid_map_core
  grid_map_msgs
  grid_map_octomap
  grid_map_ros
)

# Add the specified directories to the list of paths that the compiler
# uses to search for header files. This is important for C++
# projects where you have custom header files that are not located
# in the standard system include paths.
include_directories(dependencies)
include_directories(include/env_construction)

# Tells CMake to create executable targets. Make sure CMake
# knows about the program's dependencies.
add_executable(octomap_to_gridmap src/octomap_to_gridmap.cpp)
ament_target_dependencies(octomap_to_gridmap ${dependencies})

# Install cpp executables
install(
  TARGETS
  octomap_to_gridmap
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
