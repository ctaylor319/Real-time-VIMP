/**
 * @copyright Georgia Institute of Technology, 2024
 * @file: add_time_parameterization.h
 * @author: ctaylor319@gatech.edu
 * @date: 09/16/2024
 * @brief: Adds time-stamps to the trajectory generated by motion planning before passing
 * the trajectory to the robot control interface to command the robot.
 *
 */

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// arm group names
const std::vector<std::string> arm_joints = {
    "link1_to_link2",
    "link2_to_link3",
    "link3_to_link4",
    "link4_to_link5",
    "link5_to_link6",
    "link6_to_link6flange"
};

class AddTimeParameterization : public rclcpp::Node
{
public:

    AddTimeParameterization();
    ~AddTimeParameterization();

private:
    
    /**
     * @brief callback function for when we receive a motion planning message. Adds
     * time stamps to each waypoint and publishes a complete robot trajectory. Motion
     * planning messages are sent whenever we come within a set tolerance of a waypoint.
     * @param msg [in]: vector message containing waypoints from start to goal.
     */
    void VIMPCallback();

    /**
     * @brief This function comes from the latest moveit release (not available on humble), but
     * we use it to adjust time stamps.
     * @param num_waypoints [in]: desired number of waypoints in the path
     * @param trajectory [in, out]: path that needs time parameterization
     * @param max_velocity_scaling_factor [in]: factor between [0, 1] that can slow down trajectory
     * @param max_acceleration_scaling_factor [in]: factor between [0, 1] that can slow down trajectory
     * @param velocity_limits [in]: sets the velocity limit for each
     */
    bool totgComputeTimeStamps(const size_t num_waypoints, robot_trajectory::RobotTrajectory& trajectory,
                           const double max_velocity_scaling_factor, const double max_acceleration_scaling_factor);

    // Publishers for arm and gripper joint trajectories
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _pose_publisher;

    // Timer for periodic callback
    rclcpp::TimerBase::SharedPtr _timer;

    // Desired goal poses for the robotic arm and gripper
    std::vector<std::vector<double>> _arm_positions;

    std::string _default_urdf_path;
    std::string _default_srdf_path;

    moveit::core::RobotModelConstPtr _model;
    const moveit::core::JointModelGroup* _group;
};