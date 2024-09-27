cmake_minimum_required(VERSION 3.8)
project(motion_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_subdirectory(VIMP)

# Dependencies
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
find_package(grid_map_sdf REQUIRED)

# Add dependencies to variable
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
  grid_map_sdf
)

set(VIMP_INCLUDE_DIRS
  GaussianVI
  helpers
  instances
  3rdparty)


# Add the specified directories to the list of paths that the compiler
# uses to search for header files. This is important for C++
# projects where you have custom header files that are not located
# in the standard system include paths.
include_directories(dependencies)

# Source code include directory
include_directories(include/motion_planning)

foreach(SUBDIR IN ITEMS ${VIMP_INCLUDE_DIRS})
  include_directories("${CMAKE_CURRENT_SOURCE_DIR}/VIMP/vimp/${SUBDIR}")
endforeach()

# Tells CMake to create executable targets. Make sure CMake
# knows about the program's dependencies.
add_executable(SDF_generation src/SDF_generation.cpp)
ament_target_dependencies(SDF_generation ${dependencies})

# Install cpp executables
install(
  TARGETS
  SDF_generation
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
