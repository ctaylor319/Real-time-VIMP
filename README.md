# Real-Time VIMP
## Overview
Capstone project for Georgia Tech's MS-ROBO program. Implement's hzyu17's motion-planning-as-variational-inference (VIMP) algorithm as part of a larger autonomous control package. Utilizes probabilistic map generation, robust motion planning, and standard robotic control algorithms to guarantee robust robotic navigation.
## Requirements
 - moveit2 repository. See the official [documentation](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html) for details on how to install the package. Note that this repo uses ros-humble, which as of writing only had classic moveit for binary installs; thus a binary install is not recommended.
   * You will need **urdf** and **srdf** files so moveit can interpret the physical constraints of your robot. An srdf file can be generated with `moveit\_setup\_assistant`. Also note that a .urdf.xacro will **NOT** parse correctly; you can download the `ros-${ROS\_DISTRO}-xacro` package to convert xacro to urdf files.
 - octomap repository. This can be installed with binary rpm packages (`ros-${ROS\_DISTRO}-octomap\*`).
 - 3D Lidar sensor, or equivalent depth sensor. For simulation purposes, this [page](https://www.cplusgears.com/lesson-5-adding-a-lidar.html) offers a fairly comprehensive overview on how to add the sensor to gazebo. Testing was done using the `libgazebo\_ros\_ray\_sensor.so` plugin.
## Building
TBD
