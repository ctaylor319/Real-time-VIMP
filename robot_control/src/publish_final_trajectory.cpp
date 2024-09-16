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

AddTimeParameterization::AddTimeParameterization() : Node("env_data_collect"){
    // Create the publisher of the desired arm and gripper goal poses
    arm_pose_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 1);
    gripper_pose_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/grip_controller/joint_trajectory", 1);

    // Create a timer to periodically call the timerCallback function
    timer_ = create_wall_timer(5s, std::bind(&AddTimeParameterization::timerCallback, this));

    frame_id_ = "base_link";

    // Desired time from the trajectory start to arrive at the trajectory point.
    // Needs to be less than or equal to the timer period above to allow
    // the robotic arm to smoothly transition between points.
    duration_sec_ = 2;
    duration_nanosec_ = 0.5 * 1e9;  // (seconds * 1e9)

    // Set the desired goal poses for the robotic arm.
    arm_positions_ = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},  // Home location
        {-1.345, -1.23, 0.264, -0.296, 0.389, -1.5},  // Goal location
        {-1.345, -1.23, 0.264, -0.296, 0.389, -1.5},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}  // Home location
    };

    gripper_positions_ = {
        {0.0},  // Open gripper
        {0.0},
        {-0.70},  // Close gripper
        {-0.70}
    };

    // Keep track of the current trajectory we are executing
    index_ = 0;
}

AddTimeParameterization::~AddTimeParameterization()
{

}

void AddTimeParameterization::timerCallback()
{
    // Create new JointTrajectory messages for arm and gripper
    auto msg_arm = trajectory_msgs::msg::JointTrajectory();
    msg_arm.header.frame_id = frame_id_;
    msg_arm.joint_names = arm_joints;

    auto msg_gripper = trajectory_msgs::msg::JointTrajectory();
    msg_gripper.header.frame_id = frame_id_;
    msg_gripper.joint_names = gripper_joints;

    //////////////////////////////// NEW CODE ////////////////////////////////
    // The following receives the path created from our motion planner and time-discretizes it
    // while also checking physical limitations of the robot and adjusting accordingly.
    // This can then be fed into the JointTrajectoryController (or the plugin for the simulation),
    // Which will command the joints.

    std::string urdf_path = ""; // Absolute paths
    std::string srdf_path = "";

    // Scraped from moveit source code, really only need max vel/accel from model files for time
    // parameterization but you still need this setup
    moveit::core::RobotModelConstPtr model;
    urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDFFile(urdf_path);
    srdf::ModelSharedPtr srdf = std::make_shared<srdf::Model>();
    srdf->initFile(*urdf, srdf_path);
    model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    robot_trajectory::RobotTrajectory robot_traj(model);

    // Define a waypoint for the robot and add it to the trajectory. Waypoints can be passed in as vectors,
    // meaning they can be passed in from GVIMP output
    moveit::core::RobotState robot_waypoint(model);
    const moveit::core::JointModelGroup* group = model->getJointModelGroup("robot_arm");
    robot_waypoint.setToDefaultValues();
    double dt = 0.05; // Time step the algorithm takes for intermediate waypoints, essentially determines how precise you want your time steps

    // ADD WAYPOINTS HERE
    std::vector<double> waypoint = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    robot_waypoint.setJointGroupPositions(group, waypoint);
    robot_traj.addSuffixWayPoint(robot_waypoint, dt);

    double max_vel_scaling = 1.0; // Scaling factors range between 0-1 and can slow down trajectory, just keep them normal unless otherwise needed
    double max_accel_scaling = 1.0;
    time_param_traj->computeTimeStamps(robot_traj, max_vel_scaling, max_accel_scaling);
    

    //test1->computeTimeStamps(test2)

    //////////////////////////////// NEW CODE ////////////////////////////////

    // Create JointTrajectoryPoints for arm and gripper
    auto point_arm = trajectory_msgs::msg::JointTrajectoryPoint();
    point_arm.positions = arm_positions_[index_];
    point_arm.time_from_start = rclcpp::Duration(duration_sec_, duration_nanosec_);
    msg_arm.points.push_back(point_arm);
    arm_pose_publisher_->publish(msg_arm);

    auto point_gripper = trajectory_msgs::msg::JointTrajectoryPoint();
    point_gripper.positions = gripper_positions_[index_];
    point_gripper.time_from_start = rclcpp::Duration(duration_sec_, duration_nanosec_);
    msg_gripper.points.push_back(point_gripper);
    gripper_pose_publisher_->publish(msg_gripper);

    // Reset the index 
    if (index_ == arm_positions_.size() - 1) {
        index_ = 0;
    } else {
        index_++;
    }
}

int main(int argc, char * argv[])
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