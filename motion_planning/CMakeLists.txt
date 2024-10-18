cmake_minimum_required(VERSION 3.8)
project(motion_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
  # add_compile_options(-w)
endif()

add_subdirectory(VIMP)

set(BOOST_ROOT "/usr/include/boost")
list(APPEND CMAKE_PREFIX_PATH "${BOOST_ROOT}")

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(octomap REQUIRED)
find_package(Eigen3 3.4.0 REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(grid_map REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_octomap REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_sdf REQUIRED)
find_package(Poco REQUIRED Foundation XML Util)

find_package(Boost 1.70 REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(Boost COMPONENTS timer REQUIRED)
find_package(Boost COMPONENTS date_time REQUIRED)
find_package(Boost COMPONENTS serialization REQUIRED)

find_package(OpenMP)

add_compile_definitions(GTSAM_ENV)
add_compile_definitions(GVI_SUBDUR_ENV)
add_compile_definitions("SOURCE_ROOT=${CMAKE_CURRENT_SOURCE_DIR}/VIMP/vimp")
add_compile_definitions("INSTALL_DIR=${CMAKE_INSTALL_PREFIX}")

# Add dependencies to variable
set(dependencies
  rclcpp
  std_msgs
  control_msgs
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
  OpenMP
  Poco
)

# Add the specified directories to the list of paths that the compiler
# uses to search for header files. This is important for C++
# projects where you have custom header files that are not located
# in the standard system include paths.
include_directories(dependencies)

include_directories(include/motion_planning)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/VIMP/vimp")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/VIMP/vimp/GaussianVI")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/VIMP/vimp/3rdparty/include")
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

set(GPMP2_LIBRARIES "${CMAKE_CURRENT_SOURCE_DIR}/VIMP/vimp/3rdparty/gpmp2/gpmp2/libgpmp2.so.0.3.0")
set(GTSAM_LIBRARIES "${CMAKE_CURRENT_SOURCE_DIR}/VIMP/vimp/3rdparty/gtsam/gtsam/libgtsam.so.4.0.0")

# Tells CMake to create executable targets. Make sure CMake
# knows about the program's dependencies.

add_library(ROBOT_ARM_LIBRARY SHARED src/RobotArm3D.cpp)
ament_target_dependencies(ROBOT_ARM_LIBRARY ${dependencies})

add_library(GVI_ROBOT_LIBRARY SHARED src/GVIMPRobotArm.cpp)
ament_target_dependencies(GVI_ROBOT_LIBRARY ${dependencies})

add_executable(GVIMPImpl src/GVIMPImpl.cpp)
ament_target_dependencies(GVIMPImpl ${dependencies})
target_link_libraries(GVIMPImpl ROBOT_ARM_LIBRARY GVI_ROBOT_LIBRARY ${GTSAM_LIBRARIES} ${GPMP2_LIBRARIES} 
                        OpenMP::OpenMP_CXX ${EIGEN3_LIBRARIES} ${BOOST_LIBRARIES} Boost::serialization 
                        ${libSpGH_LIBRARY} ${Matlab_MEX_LIBRARY})



# Install cpp executables

set(GPMP2_SHARED_LIBRARIES "${CMAKE_CURRENT_SOURCE_DIR}/VIMP/vimp/3rdparty/gpmp2/gpmp2/libgpmp2.so.0")
set(GTSAM_SHARED_LIBRARIES "${CMAKE_CURRENT_SOURCE_DIR}/VIMP/vimp/3rdparty/gtsam/gtsam/libgtsam.so.4")

install(TARGETS ROBOT_ARM_LIBRARY GVI_ROBOT_LIBRARY
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(FILES ${GTSAM_LIBRARIES} ${GTSAM_SHARED_LIBRARIES}
              ${GPMP2_LIBRARIES} ${GPMP2_SHARED_LIBRARIES}
        DESTINATION lib)

install(
  TARGETS
  GVIMPImpl
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES src/sim_config.xml DESTINATION lib)

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

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
