set(_AMENT_PACKAGE_NAME "grid_map_cv")
set(grid_map_cv_VERSION "2.0.0")
set(grid_map_cv_MAINTAINER "Maximilian Wulf <mwulf@anybotics.com>, Yoshua Nava <ynava@anybotics.com>, Ryan Friedman <ryanfriedman5410+grid_map@gmail.com>")
set(grid_map_cv_BUILD_DEPENDS "grid_map_cmake_helpers" "grid_map_core" "cv_bridge" "filters" "pluginlib" "rclcpp" "sensor_msgs")
set(grid_map_cv_BUILDTOOL_DEPENDS "ament_cmake")
set(grid_map_cv_BUILD_EXPORT_DEPENDS "grid_map_core" "cv_bridge" "filters" "pluginlib" "rclcpp" "sensor_msgs")
set(grid_map_cv_BUILDTOOL_EXPORT_DEPENDS )
set(grid_map_cv_EXEC_DEPENDS "grid_map_core" "cv_bridge" "filters" "pluginlib" "rclcpp" "sensor_msgs")
set(grid_map_cv_TEST_DEPENDS "ament_lint_auto" "ament_lint_common" "ament_cmake_gtest")
set(grid_map_cv_GROUP_DEPENDS )
set(grid_map_cv_MEMBER_OF_GROUPS )
set(grid_map_cv_DEPRECATED "")
set(grid_map_cv_EXPORT_TAGS)
list(APPEND grid_map_cv_EXPORT_TAGS "<build_type>ament_cmake</build_type>")
list(APPEND grid_map_cv_EXPORT_TAGS "<filters plugin=\"filter_plugins.xml\"/>")
