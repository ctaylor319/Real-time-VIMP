/**
 * Georgia Institute of Technology, 2024
 * @file: add_time_parameterization.h
 * @brief: receive trajectory from motion_planning pkg and
 *         add an optimal time parameterization
 * @author: ctaylor319@gatech.edu
 * @date: 09/09/2024
 *
 */

#include "rclcpp/rclcpp.hpp"

// Define constants
const std::vector<std::string> arm_joints = {
    "link1_to_link2",
    "link2_to_link3",
    "link3_to_link4",
    "link4_to_link5",
    "link5_to_link6",
    "link6_to_link6flange"
};

const std::vector<std::string> gripper_joints = {
    "gripper_controller"
};

class AddTimeParameterization : public rclcpp::Node
{
public:
    AddTimeParameterization();
    ~AddTimeParameterization();

private:
    void timerCallback();
    // Publishers for arm and gripper joint trajectories
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pose_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pose_publisher_;

    // Timer for periodic callback
    rclcpp::TimerBase::SharedPtr timer_;

    // Frame ID for the joint trajectories
    std::string frame_id_;

    // Duration for each trajectory point
    int duration_sec_;
    int duration_nanosec_;

    // Desired goal poses for the robotic arm and gripper
    std::vector<std::vector<double>> arm_positions_;
    std::vector<std::vector<double>> gripper_positions_;
    
    trajectory_processing::TimeOptimalTrajectoryGeneration* time_param_traj;

    // Index to keep track of the current trajectory point
    size_t index_;
};