/**
 * Georgia Institute of Technology, 2024
 * File: add_time_parameterization.cpp
 * Author: ctaylor319@gatech.edu
 * Date: 09/09/2024
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/header.hpp"

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/robot_trajectory.h>
#include <urdf_parser/urdf_parser.h>

#include "add_time_parameterization.h"

using namespace std::chrono_literals;

AddTimeParameterization::AddTimeParameterization() : Node("add_time_parameterization"){

    // Default parameters for urdf and srdf paths
    // Note that urdf_path MUST BE a .urdf file, not a .urdf.xacro file. Use the xacro package to convert to .urdf if needed
    std::string default_urdf_path = "/home/ctaylor71023/ros2_ws/src/mycobot_ros2/mycobot_gazebo/urdf/mycobot_280_classic_gazebo.urdf";
    std::string default_srdf_path = "/home/ctaylor71023/ros2_ws/src/mycobot_ros2/mycobot_gazebo/moveit_setup/config/mycobot_280.srdf";

    this->declare_parameter("urdf_path", default_urdf_path);
    this->declare_parameter("srdf_path", default_srdf_path);

    std::string urdf_path = this->get_parameter("urdf_path").as_string();
    std::string srdf_path = this->get_parameter("srdf_path").as_string();

    if ( urdf_path.compare(default_urdf_path) == 0 || srdf_path.compare(default_srdf_path) == 0 ) {
        std::cout << "WARNING: Using default urdf and/or srdf file locations.\nProper usage: ";
        std::cout << "ros2 launch robot_control launch_robot_autonomy.launch.py --ros-args -p";
        std::cout << "urdf_path:=<path to urdf file> -p srdf_path:=<path to srdf file>" << std::endl;
        std::cout << "Alternatively, you can change the default path locations in";
        std::cout << "robot_control/src/add_time_parameterization.cpp" << std::endl;
    }

    _time_param_traj = new trajectory_processing::TimeOptimalTrajectoryGeneration();
    // Create the publisher of the desired arm and gripper goal poses
    _pose_publisher = create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory", 10);

    // Create a timer to periodically call the timerCallback function
    _timer = create_wall_timer(5s, std::bind(&AddTimeParameterization::VIMPCallback, this));

    _frame_id = "base_link";

    // Scraped from moveit source code, really only need max vel/accel from model files for time
    // parameterization but you still need this setup
    urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDFFile(urdf_path);
    srdf::ModelSharedPtr srdf = std::make_shared<srdf::Model>();
    srdf->initFile(*urdf, srdf_path);
    _model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    _group = _model->getJointModelGroup("robot_arm");
    _robot_traj = new robot_trajectory::RobotTrajectory(_model, _group);
    // Set the desired goal poses for the robotic arm.
    _arm_positions = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},  // Home location
        {-1.345, -1.23, 0.264, -0.296, 0.389, -1.5}  // Goal location
    };

    // Keep track of the current trajectory we are executing
    _index = 0;
}

AddTimeParameterization::~AddTimeParameterization()
{

}

void AddTimeParameterization::VIMPCallback()
{
    // Create new JointTrajectory messages for arm and gripper
    auto msg_arm = trajectory_msgs::msg::JointTrajectory();
    msg_arm.header.frame_id = _frame_id;
    msg_arm.joint_names = arm_joints;

    //////////////////////////////// NEW CODE ////////////////////////////////
    // The following receives the path created from our motion planner and time-discretizes it
    // while also checking physical limitations of the robot and adjusting accordingly.
    // This can then be fed into the JointTrajectoryController (or the plugin for the simulation),
    // Which will command the joints.

    // Define a waypoint for the robot and add it to the trajectory. Waypoints can be passed in as vectors,
    // meaning they can be passed in from GVIMP output
    moveit::core::RobotState robot_waypoint(_model);
    robot_waypoint.setToDefaultValues();
    double dt = 0.05; // Time step the algorithm takes for intermediate waypoints, essentially determines how precise you want your time steps

    // ADD WAYPOINTS HERE
    for ( auto waypoint : _arm_positions ) {
        robot_waypoint.setJointGroupPositions(_group, waypoint);
        _robot_traj->addSuffixWayPoint(robot_waypoint, dt);
    }
    
    double max_vel_scaling = 1.0; // Scaling factors range between 0-1 and can slow down trajectory, just keep them normal unless otherwise needed
    double max_accel_scaling = 1.0;
    _time_param_traj->computeTimeStamps(*_robot_traj, max_vel_scaling, max_accel_scaling);
    //////////////////////////////// NEW CODE ////////////////////////////////

    // Create JointTrajectoryPoints for arm and gripper
    auto point_arm = trajectory_msgs::msg::JointTrajectoryPoint();
    point_arm.positions = _arm_positions[_index];
    std::chrono::nanoseconds waypoint_time_from_start(int(_robot_traj->getWayPointDurationFromStart(_index)));
    point_arm.time_from_start = rclcpp::Duration(waypoint_time_from_start);
    msg_arm.points.push_back(point_arm);
    _pose_publisher->publish(msg_arm);

    // Reset the index 
    if ( _index == _arm_positions.size() - 1 ) {
        _index = 0;
    } else {
        _index++;
    }
}

int main ( int argc, char * argv[] )
{

    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);

    // Create an instance of the EnvDataCollect node
    auto node = std::make_shared<AddTimeParameterization>();

    // Spin the node to execute the callbacks
    rclcpp::spin(node);

    // Shutdown the ROS 2 client library
    rclcpp::shutdown();

    return 0;
}