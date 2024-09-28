# Real-Time VIMP
## Overview
Capstone project for Georgia Tech's MS-ROBO program. Implements hzyu17's motion-planning-as-variational-inference (VIMP) algorithm as part of a larger autonomous control package. Utilizes probabilistic map generation, robust motion planning, and standard robotic control algorithms to guarantee robust robotic navigation. Package is written under a ROS2 framework (humble); there may be compiling issues if a more recent ROS version is used.
## Build Requirements
 - moveit2 repository. See the official [documentation](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html) for details on how to install the package. Note that this repo uses ros-humble, which as of writing only had classic moveit for binary installs; thus a binary install is not recommended.
   * You will need **urdf** and **srdf** files so moveit can interpret the physical constraints of your robot. An srdf file can be generated with `moveit_setup_assistant`. Also note that a .urdf.xacro will **NOT** parse correctly; you can download the `ros-${ROS_DISTRO}-xacro` package to convert xacro to urdf files.
 - octomap repository. This can be installed with binary rpm packages (`ros-${ROS_DISTRO}-octomap*`).
 - 3D Lidar sensor, or equivalent depth sensor. 
## Building
  - Clone the repo
    - `cd ~/ros2_ws/src` (or whichever workspace you prefer)
    - `git clone https://github.com/ctaylor319/Real-time-VIMP.git`
  - Install dependencies
    - `vcs import . < dependencies.repos`
    - `rm -rf grid_map/grid_map_sdf && mv -r grid_map_fix/grid_map_sdf/ grid_map/`
    - `sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y`
    - Will also need MATLAB Runtime Library (version R2020b) for VIMP. Check that the MATLAB base path is correct in VIMP's CMakeFiles.txt.
  - Build packages
    - `colcon build`


## Launching
  - TBD
  - Depth sensor must publish pointcloud2 messages to the `/cloud_in` topic.
  - `ros2 launch octomap\_server2 octomap\_server\_launch.py` processes lidar input
