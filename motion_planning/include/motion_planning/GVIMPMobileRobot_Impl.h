/**
 * @copyright Georgia Institute of Technology, 2024
 * @file: GVIMPMobileRobot_Impl.h
 * @author: ctaylor319@gatech.edu
 * @date: 11/24/2024
 * @brief: Implements the GVIMP algorithm to construct a
 * path for the robot to follow
 *
 */

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "MobileRobot.h"
#include "GVIMPMobileRobot.h"

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
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _gridmap_subscriber;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr _occupied_cells_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _pose_subscriber;
    rclcpp::Publisher<motion_planning_msgs::msg::WaypointPath>::SharedPtr _path_publisher;
    rclcpp::Publisher<motion_planning_msgs::msg::VisualizedPath>::SharedPtr _iterative_path_publisher;
    rclcpp::Service<motion_planning_msgs::srv::RuntimeParameterInterface>::SharedPtr _parameter_service;
    rclcpp::Service<motion_planning_msgs::srv::RuntimePathInterface>::SharedPtr _path_service;

    /**
     * @brief: callback function for when we receive a grid_map message. Creates path
     * plans when necessary and publishes the resulting Gaussian RV path.
     * 
     * @param msg [in]: Projected 2D map of the environment (Octomap is by default 3D, even for 2D lidar)
     */
    void gridMapCallback(nav_msgs::msg::OccupancyGrid msg);

    /**
     * @brief: callback function for the robot's current state. Used to set the start
     * position of the motion planner.
     *
     * @param msg [in]: message containing robot position and velocity
     */
    void stateCallback(nav_msgs::msg::Odometry msg);

    /**
     * @brief: callback function for all occupied cells in the octomap. Used
     * to provide a 2D landscape for plotting purposes.
     *
     * @param msg [in]: message containing cell data
     */
    void cellsCallback(visualization_msgs::msg::MarkerArray msg);

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
     * @param msg [in]: contains the current 2D map of the environment
     *
     * @return SDF of the surrounding environment
     */
    gpmp2::PlanarSDF generateSDF(nav_msgs::msg::OccupancyGrid msg);

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
    std::unique_ptr<vimp::GVIMPMobileRobot> _path_planner;
    VectorXd _start_pos, _goal_pos;
    bool _new_path_needed, _reset_called;
    motion_planning_msgs::msg::WaypointPath _curr_path;
    motion_planning_msgs::msg::VisualizedPath _vis_path;
    std::vector<geometry_msgs::msg::Point> _occupied_cells;
    
};