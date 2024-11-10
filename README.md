# Real-Time VIMP
## Overview
Capstone project for Georgia Tech's MS-ROBO program. Implements hzyu17's motion-planning-as-variational-inference (VIMP) algorithm as part of a larger autonomous control package. Utilizes probabilistic map generation, robust motion planning, and standard robotic control algorithms to guarantee robust robotic navigation. Package is written under a ROS2 framework (humble); there may be compiling issues if a more recent ROS version is used.
## Build Requirements
  - Note: For the simulation demo, all config requirements are met. Once all the repositories below have been built/installed, you can continue straight to the launch sequence.
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
    - The github repo should have already been cloned by vcs
    - Follow directions on their [github page](https://github.com/pocoproject/poco), making sure that you are installing the libraries, not just building.
  - Update `motion_planning/src/sim_config.xml` parameters as needed
    - goal: goal joint positions of your robot arm
    - urdf: name of the urdf file you're using
    - srdf: name of the srdf file you're using
  - Build SparseGH_weights file
    - `cd src/motion_planning/VIMP`
    - Follow directions from VIMP's [github page](https://github.com/hzyu17/VIMP/tree/master) to build the repo
    - cd to the build directory you made
    - `./src/generate_sigmapts`
    - `./src/save_SparseGH_weights`
  - Build packages
    - `cd ..`
    - `colcon build`

## Launching
  - Depth sensor must publish PointCloud2 messages to the `input_cloud_topic` LaunchArgument in `octomap_server_launch.py` and have a valid transformation matrix to the base frame.
    - The easiest way to do the first part is to change the default value to whatever topic you are currently publishing to.
      - For example, in `mycobot_280.urdf.xacro` PointCloud2 messages are published to `\ray\pointcloud2` under the `ray_plugin`. Therefore, you would change `\cloud_in` to `\ray\pointcloud2` in the octomap server launch file.
    - If you get a "message filter dropping message" error you have an incomplete transformation to your sensor. You can check your transformation tree in rviz by pulling up the tf plugin. Also note that the base link for the tf matrix must be `world`.
  - Current launch sequence:
    - `ros2 launch six_dof_arm_gazebo six_dof_arm_simulation.launch.py`
      - This will start up an empty world in gazebo with pure gravity compensation being added to hold the robot stationary.
    - In a new tab, `ros2 launch six_dof_arm_controller robot_control_interface.launch.py`
      - This will start the autonomy package.
    - This will have the robot move towards the goal position set in the config file
    - (Optional) In a new tab, `rqt`
      - If you don't have rqt, run `sudo apt-get install ros-${ROS_DISTRO}-rqt*`
      - This will pull up the rqt interface
      - From the top widget, Plugins->Services->Service Caller
        - This will open a GUI that lets you call upon various ROS services
      - In the Service dropdown menu, there are two `robot_runtime_interface` services available, `parameter_tuning` and `set_path`
        - `parameter_tuning` allows you to adjust GVIMP parameter values (but currently only those available by public access functions in the VIMP repo)
        - `set_path` allows you to reset the robot's position back to zero and re-run the simulation
        - These two services provide quick runtime access to test different parameter values
