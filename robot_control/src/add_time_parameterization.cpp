/**
 * Georgia Institute of Technology, 2024
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
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
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

    _pose_publisher = create_publisher<trajectory_msgs::msg::JointTrajectory>("/raw_joint_trajectory", 10);

    // Create a timer to periodically call the timerCallback function
    _timer = create_wall_timer(5s, std::bind(&AddTimeParameterization::VIMPCallback, this));

    // Scraped from moveit source code, really only need max vel/accel from model files for time
    // parameterization but you still need this setup
    urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDFFile(urdf_path);
    srdf::ModelSharedPtr srdf = std::make_shared<srdf::Model>();
    srdf->initFile(*urdf, srdf_path);
    _model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    _group = _model->getJointModelGroup("robot_arm");
    
    // testing waypoint positions
    _arm_positions = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},  // Home location
        {-1.345, -1.23, 0.264, -0.296, 0.389, -1.5}  // Goal location
    };
}

AddTimeParameterization::~AddTimeParameterization()
{

}

void AddTimeParameterization::VIMPCallback()
{
    // Create new JointTrajectory messages for arm and gripper
    robot_trajectory::RobotTrajectory robot_traj(_model, _group);
    auto msg_arm = trajectory_msgs::msg::JointTrajectory();
    msg_arm.joint_names = arm_joints;

    // Define a waypoint for the robot and add it to the trajectory. Waypoints can be passed in as vectors,
    // meaning they can be passed in from GVIMP output
    moveit::core::RobotState robot_waypoint(_model);
    robot_waypoint.setToDefaultValues();
    double dt = 0.1; // Default time step between waypoints, updated by totgComputeTimeStamps

    // ADD WAYPOINTS HERE
    for ( auto waypoint : _arm_positions ) {
        robot_waypoint.setJointGroupPositions(_group, waypoint);
        robot_traj.addSuffixWayPoint(robot_waypoint, dt);
    }
    
    double max_vel_scaling = 1.0;
    double max_accel_scaling = 1.0;
    const size_t num_waypoints = 20;
    totgComputeTimeStamps(num_waypoints, robot_traj, max_vel_scaling, max_accel_scaling);

    // Create JointTrajectoryPoints for arm
    for ( size_t i=0; i<robot_traj.getWayPointCount(); ++i ) {
        auto point_arm = trajectory_msgs::msg::JointTrajectoryPoint();

        // data only accessible as Eigen structure, so we first get joint values then convert to vector
        Eigen::VectorXd eWaypoint;
        robot_traj.getWayPoint(i).copyJointGroupPositions(_group, eWaypoint); // 
        point_arm.positions = std::vector<double> ( eWaypoint.data(), eWaypoint.data() + eWaypoint.size() );
        std::chrono::nanoseconds waypoint_time_from_start( uint64_t(robot_traj.getWayPointDurationFromStart(i) * 1e9) );
        std::cout << robot_traj.getWayPointDurationFromStart(i) << std::endl;
        point_arm.time_from_start = rclcpp::Duration(waypoint_time_from_start);
        msg_arm.points.push_back(point_arm);
    }
    _pose_publisher->publish(msg_arm);
}

bool AddTimeParameterization::totgComputeTimeStamps(const size_t num_waypoints, robot_trajectory::RobotTrajectory& trajectory,
                           const double max_velocity_scaling_factor, const double max_acceleration_scaling_factor)
{
    trajectory_processing::TimeOptimalTrajectoryGeneration default_totg(0.05 /* default path tolerance */, 0.1 /* default resample_dt */);
    default_totg.computeTimeStamps(trajectory, max_velocity_scaling_factor, max_acceleration_scaling_factor);
    double optimal_duration = trajectory.getDuration();
    double new_resample_dt = optimal_duration / (num_waypoints - 1);
    trajectory_processing::TimeOptimalTrajectoryGeneration resample_totg(0.05 /* path tolerance */, new_resample_dt);
    resample_totg.computeTimeStamps(trajectory, max_velocity_scaling_factor, max_acceleration_scaling_factor);
    return true;
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