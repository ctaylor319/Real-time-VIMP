/**
 * @copyright Georgia Institute of Technology, 2024
 * @file: GVIMPImpl.h
 * @author: ctaylor319@gatech.edu
 * @date: 10/07/2024
 * @brief: Implements the GVIMP algorithm to construct a
 * path for the robot to follow
 *
 */

#include <rclcpp/rclcpp.hpp>

#include "RobotArm3D.h"
#include "GVIMPRobotArm.h"

class GVIMPImpl : public rclcpp::Node
{
enum RobotState {
    Idle,
    CreatePath,
    ExecutePath,
    ResetArm
};
public:

    // Constructors & Destructors
    GVIMPImpl();
    ~GVIMPImpl();

private:
    
    // Publishers and subscribers
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr _gridmap_subscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _path_status_subscriber;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr _robot_state_subscriber;
    rclcpp::Publisher<motion_planning_msgs::msg::WaypointPath>::SharedPtr _path_publisher;
    rclcpp::Service<motion_planning_msgs::srv::RuntimeParameterInterface>::SharedPtr _parameter_service;
    rclcpp::Service<motion_planning_msgs::srv::RuntimePathInterface>::SharedPtr _path_service;

    /**
     * @brief: callback function for when we receive a grid_map message. Creates path
     * plans when necessary and publishes the resulting Gaussian RV path.
     *
     * @param msg [in]: grid_map message containing full map of surrounding environment.
     */
    void gridMapCallback(grid_map_msgs::msg::GridMap msg);

    /**
     * @brief: callback function for when the robot needs to replan its path. Determined
     * by the controller and the result is passed here.
     *
     * @param msg [in]: boolean message, set to true when a new path plan is needed.
     */
    void pathStatusCallback(std_msgs::msg::Bool msg);

    /**
     * @brief: callback function for the robot's current state. Used to set the start
     * position of the motion planner.
     *
     * @param msg [in]: message containing joint positions & errors.
     */
    void stateCallback(control_msgs::msg::JointTrajectoryControllerState msg);

    /**
     * @brief: callback function for robot parameter change requests.
     *
     * @param request [in]: service request message
     *
     * @param response [out]: service response message
     */
    void handleParameterRequest ( const std::shared_ptr<motion_planning_msgs::srv::RuntimeParameterInterface::Request> request,
                                    std::shared_ptr<motion_planning_msgs::srv::RuntimeParameterInterface::Response> response );

    /**
     * @brief: callback function for robot path change requests.
     *
     * @param request [in]: service request message
     *
     * @param response [out]: service response message
     */
    void handlePathRequest ( const std::shared_ptr<motion_planning_msgs::srv::RuntimePathInterface::Request> request,
                                std::shared_ptr<motion_planning_msgs::srv::RuntimePathInterface::Response> response );

    /**
     * @brief: creates a signed distance field and populates the data into the same
     * data type needed by GVIMP as an input.
     *
     * @param msg [in]: contains the current grid_map of the environment.
     *
     * @return SDF of the surrounding environment
     */
    gpmp2::SignedDistanceField generateSDF(grid_map_msgs::msg::GridMap msg);

    /**
     * @brief: Updates the current state of our robot FSM
     */
    void updateState();

    /**
     * @brief: helper function that extracts the factorized precision matrix
     * of each waypoint from the joint precision matrix and computes the log entropy.
     *
     * @param joint_precision [in]: joint precision of path.
     *
     * @return vector of entropies for each waypoint
     */
    std::vector<double> extractLogEntropyFromJoint(SpMat joint_precision);

    enum RobotState _state;
    std::unique_ptr<vimp::RobotArmMotionPlanner> _path_planner;
    VectorXd _start_pos, _goal_pos;
    bool _new_path_needed, _reset_called;
    motion_planning_msgs::msg::WaypointPath _curr_path;
};