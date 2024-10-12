/**
 * @copyright Georgia Institute of Technology, 2024
 * @file: robot_control_interface.h
 * @author: ctaylor319@gatech.edu
 * @date: 09/17/2024
 * @brief: Handles interactions between physical robot, robot controller,
 * and motion planner.
 *
 */

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>

// arm group names
const std::vector<std::string> arm_joints = {
    "link1_to_link2",
    "link2_to_link3",
    "link3_to_link4",
    "link4_to_link5",
    "link5_to_link6",
    "link6_to_link6flange"
};

class RobotControlInterface : public rclcpp::Node
{
public:

    RobotControlInterface();
    ~RobotControlInterface();

private:
    
    /**
     * @brief callback function for when we receive a motion planning message. Adds
     * time stamps to each waypoint and publishes a complete robot trajectory. Motion
     * planning messages are sent whenever we come within a set tolerance of a waypoint.
     * @param msg [in]: vector message containing waypoints from start to goal.
     */
    void TrajectoryCallback(trajectory_msgs::msg::JointTrajectory msg);
    void StateCallback(control_msgs::msg::JointTrajectoryControllerState msg);

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _trajectory_publisher;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr _raw_trajectory_subscriber;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr _robot_state_subscriber;
    // rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr _motion_planner_subscriber;

    control_msgs::msg::JointTrajectoryControllerState _curr_robot_state;

};