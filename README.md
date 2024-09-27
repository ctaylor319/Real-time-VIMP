# Real-Time VIMP
## Overview
Capstone project for Georgia Tech's MS-ROBO program. Implements hzyu17's motion-planning-as-variational-inference (VIMP) algorithm as part of a larger autonomous control package. Utilizes probabilistic map generation, robust motion planning, and standard robotic control algorithms to guarantee robust robotic navigation. Package is written under a ROS2 framework (humble); there may be compiling issues if a more recent ROS version is used.
## Build Requirements
 - moveit2 repository. See the official [documentation](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html) for details on how to install the package. Note that this repo uses ros-humble, which as of writing only had classic moveit for binary installs; thus a binary install is not recommended.
   * You will need **urdf** and **srdf** files so moveit can interpret the physical constraints of your robot. An srdf file can be generated with `moveit_setup_assistant`. Also note that a .urdf.xacro will **NOT** parse correctly; you can download the `ros-${ROS_DISTRO}-xacro` package to convert xacro to urdf files.
 - octomap repository. This can be installed with binary rpm packages (`ros-${ROS_DISTRO}-octomap*`).
 - 3D Lidar sensor, or equivalent depth sensor. 
## Building
  - TBD
  - grid_map needs to be modified. Copy over the grid_map_sdf package in the grid_map repo with the provided grid_map_sdf package within `./build_dependencies/`. Explanation: the humble branch does not have an updated SDF algorithm, so the grid_map_sdf repo from the master branch needs to be copied over. However, the master branch is ROS1, so it isn't compatible with the rest of this repo. The provided grid_map_sdf translates the algorithm into ROS2 for you.

## Launching
  - TBD
  - Depth sensor must publish pointcloud2 messages to the /cloud_in topic.
  - ros2 launch octomap_server2 octomap_server_launch.py launches the octomap service.
