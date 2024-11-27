# Real-Time VIMP
## Overview
Capstone project for Georgia Tech's MS-ROBO program. Implements hzyu17's motion-planning-as-variational-inference (VIMP) algorithm as part of a larger autonomous control package. Utilizes probabilistic map generation, robust motion planning, and standard robotic control algorithms to guarantee robust robotic navigation. Package is written under a ROS2 framework (humble); there may be compiling issues if a more recent ROS version is used.
## Build Requirements
  - Note: For the simulation demo, all config requirements are met. Once all the repositories below have been built/installed, you can continue straight to the launch sequence.
 - moveit2 repository. See the official [documentation](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html) for details on how to install the package. Note that this repo uses ros-humble, which as of writing only had classic moveit for binary installs; thus, a binary install is not recommended.
   * Because of the size of this repo, you may want to build it in a separate workspace so it only needs to be built once. If you do this, make sure to source it in your `.bashrc` file so it can function as an underlay workspace.
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
    - NOTE: While this should cover all non-VIMP dependencies, rosdep isn't always 100% reliable. If any dependencies are missing, they should either show up as errors when building the CMake or compiling the repo. Either should make it clear which package(s) are missing, from which you can manually install them.
  - Install Poco libraries
    - The github repo should have already been cloned by vcs
    - Follow directions on their [github page](https://github.com/pocoproject/poco), making sure that you are installing the libraries, not just building.
  - Build VIMP
    - `cd src/motion_planning/VIMP`
    - Follow directions from VIMP's [github page](https://github.com/hzyu17/VIMP/tree/master) to build the repo
  - Update `motion_planning/src/sim_config.xml` parameters as needed
    - goal: goal joint positions of your robot arm
    - urdf: name of the urdf file you're using
    - srdf: name of the srdf file you're using
  - Build packages
    - `cd ..`
    - `colcon build`

## Launching
  - Depth sensor must publish PointCloud2 messages to the `input_cloud_topic` LaunchArgument in `octomap_server_launch.py` and have a valid transformation matrix to the base frame.
    - The easiest way to do the first part is to change the default value to whatever topic you are currently publishing to.
      - For example, in `mycobot_280.urdf.xacro` PointCloud2 messages are published to `\ray\pointcloud2` under the `ray_plugin`. Therefore, you would change `\cloud_in` to `\ray\pointcloud2` in the octomap server launch file.
    - If you get a "message filter dropping message" error, you most likely have an incomplete transformation to your sensor. You can check your transformation tree in rviz by pulling up the tf plugin. Also note that the base link for the tf matrix must be `world`.
  - Launch sequence:
    - Launch the simulation
      - Options:
        - `ros2 launch six_dof_arm_gazebo six_dof_arm_simulation.launch.py`
        - `ros2 launch mobile_robot_gazebo mobile_robot_simulation.launch.py`
    - In a new tab, launch the autonomy package
      - Options:
        - `ros2 launch six_dof_arm_controller robot_control_interface.launch.py`
        - `ros2 launch six_dof_arm_controller robot_control_interface_cuda.launch.py`
        - `ros2 launch mobile_robot_controller robot_control_interface.launch.py`
    - (Optional) In a new tab, `rqt`
      - If you don't have rqt, run `sudo apt-get install ros-${ROS_DISTRO}-rqt*`
      - This will pull up the rqt interface
      - From the top widget, Plugins->Services->Service Caller
        - This will open a GUI that lets you call upon various ROS services
      - In the Service dropdown menu, there are two `robot_runtime_interface` services available, `parameter_tuning` and `set_path`
        - `parameter_tuning` allows you to adjust GVIMP parameter values (but currently only those available by public access functions in the VIMP repo)
        - `set_path` allows you to reset the robot's position back to zero and re-run the simulation
        - These two services provide quick runtime access to test different parameter values
