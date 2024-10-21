/**
 * @copyright Georgia Institute of Technology, 2024
 * @file: add_time_parameterization.cpp
 * @author: ctaylor319@gatech.edu
 * @date: 09/16/2024
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <urdf_parser/urdf_parser.h>

#include "robot_control_interface.h"

using namespace std::chrono_literals;

RobotControlInterface::RobotControlInterface() : Node( "robot_control_interface" )
{
    _raw_trajectory_subscriber = create_subscription< trajectory_msgs::msg::JointTrajectory >( "/raw_joint_trajectory", 10,
        std::bind( &RobotControlInterface::TrajectoryCallback, this, std::placeholders::_1 ) );
    _robot_state_subscriber = create_subscription< control_msgs::msg::JointTrajectoryControllerState >( "/arm_controller/controller_state", 10,
        std::bind( &RobotControlInterface::StateCallback, this, std::placeholders::_1 ) );
    _trajectory_publisher = create_publisher< trajectory_msgs::msg::JointTrajectory >( "/arm_controller/joint_trajectory", 10 );
}

RobotControlInterface::~RobotControlInterface()
{
}

void RobotControlInterface::TrajectoryCallback ( trajectory_msgs::msg::JointTrajectory msg )
{
    _trajectory_publisher->publish(msg);
}

void RobotControlInterface::StateCallback ( control_msgs::msg::JointTrajectoryControllerState msg )
{
    _curr_robot_state = msg;
}

int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotControlInterface>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}