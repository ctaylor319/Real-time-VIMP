# Real-Time VIMP
## Overview
Capstone project for Georgia Tech's MS-ROBO program. Implements hzyu17's motion-planning-as-variational-inference (VIMP) algorithm as part of a larger autonomous control package. Utilizes probabilistic map generation, robust motion planning, and standard robotic control algorithms to guarantee robust robotic navigation. Package is written under a ROS2 framework (humble); there may be compiling issues if a more recent ROS version is used.
## Build Requirements
 - moveit2 repository. See the official [documentation](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html) for details on how to install the package. Note that this repo uses ros-humble, which as of writing only had classic moveit for binary installs; thus, a binary install is not recommended.
   * You will need **urdf** and **srdf** files so moveit can interpret the physical constraints of your robot. An srdf file can be generated with `moveit_setup_assistant`. Also note that a .urdf.xacro will **NOT** parse correctly; you can download the `ros-${ROS_DISTRO}-xacro` package to convert xacro to urdf files.
   * Make sure that both files are created in the right directories, otherwise CMake will not be able to find them. The urdf file should go in `gazebo_simulation/urdf/` and the srdf file should go in `gazebo_simulation/moveit_setup/config`.
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
    - Will also need to install the MATLAB Runtime Library (version R2020b) for VIMP. Check that the MATLAB base path is correct in VIMP's CMakeFiles.txt.
  - Install Poco libraries
    - The github should have already been cloned by vcstool
    - Follow directions on their [github page](https://github.com/pocoproject/poco), making sure that you are installing the libraries, not just building.
  - Update `motion_planning/src/sim_config.xml` parameters as needed
    - goal: goal joint positions of your robot arm
    - urdf: name of the urdf file you're using
    - srdf: name of the srdf file you're using
  - Build packages
    - `colcon build`

## Launching
  - Depth sensor must publish pointcloud2 messages to the `/cloud_in` topic.
  - Current launch sequence:
    - `ros2 launch gazebo_simulation robot_with_control_added_simulation.launch.py`
      - This will start up an empty world in gazebo with pure gravity compensation being added to hold the robot stationary. It also launches environment reconstruction.
    - `ros2 run motion_planning GVIMPImpl`
      - This will run the motion planner with environmental input
    - Current output is the planned path, with control integration in development
