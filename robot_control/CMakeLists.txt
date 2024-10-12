cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(mycobot_gazebo REQUIRED)
find_package(xacro REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_core REQUIRED)
find_package(octomap REQUIRED)
find_package(Poco REQUIRED XML Util)

add_compile_definitions("INSTALL_DIR=${CMAKE_INSTALL_PREFIX}")

# Define a CMake variable named dependencies that lists all
# ROS 2 packages and other dependencies the project requires.
set(dependencies
  rclcpp
  std_msgs
  trajectory_msgs
  ros2_control
  ros2_controllers
  gazebo_ros2_control
  gazebo_ros_pkgs
  mycobot_gazebo
  moveit
  moveit_core
  Poco
)

# Add the specified directories to the list of paths that the compiler
# uses to search for header files. This is important for C++
# projects where you have custom header files that are not located
# in the standard system include paths.
include_directories(dependencies)
include_directories(${OCTOMAP_INCLUDE_DIRS})
include_directories(include/robot_control)

link_directories(${OCTOMAP_LIBRARY_DIRS})

# Tells CMake to create executable targets. Make sure CMake
# knows about the program's dependencies.
add_executable(add_time_parameterization src/add_time_parameterization.cpp)
add_executable(robot_control_interface src/robot_control_interface.cpp)
ament_target_dependencies(add_time_parameterization ${dependencies})
ament_target_dependencies(robot_control_interface ${dependencies})

# Install cpp executables
install(
  TARGETS
  add_time_parameterization
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
  robot_control_interface
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
