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
#include <motion_planning_msgs/msg/waypoint_path.hpp>

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <urdf_parser/urdf_parser.h>

#include <Poco/Util/AbstractConfiguration.h>
#include <Poco/Util/XMLConfiguration.h>

#include "add_time_parameterization.h"

#define STRING(x) #x
#define XSTRING(x) STRING(x)

using namespace std::chrono_literals;

AddTimeParameterization::AddTimeParameterization() : Node( "add_time_parameterization" )
{
    _path_subscriber = create_subscription<motion_planning_msgs::msg::WaypointPath>( "/vimp_path", 10,
        std::bind( &AddTimeParameterization::VIMPCallback, this, std::placeholders::_1 ) );
    _pose_publisher = create_publisher<trajectory_msgs::msg::JointTrajectory>( "/arm_controller/joint_trajectory", 10 );

    // Read in config parameters
    Poco::Util::AbstractConfiguration *cfg;
    std::string install_dir = XSTRING(INSTALL_DIR);
    try {
        cfg = new Poco::Util::XMLConfiguration( install_dir+"/lib/sim_config.xml" );
    } catch ( const std::exception& e ) {
        std::cout << "Unable to open configuration file: " << e.what() << std::endl;
    }
    std::string urdf_path, srdf_path;
    try {
        urdf_path = install_dir+"/lib/"+cfg->getString("Arm.urdf");
        srdf_path = install_dir+"/lib/"+cfg->getString("Arm.srdf");
    } catch ( const std::exception& e ) {
        std::cout << "Unable to parse robot files: " << e.what() << std::endl;
    }

    // Scraped from moveit source code, really only need max vel/accel from model files for time
    // parameterization but you still need this setup
    urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDFFile( urdf_path );
    srdf::ModelSharedPtr srdf = std::make_shared<srdf::Model>();
    srdf->initFile( *urdf, srdf_path );
    _model = std::make_shared<moveit::core::RobotModel>( urdf, srdf );
    _group = _model->getJointModelGroup( "robot_arm" );
}

AddTimeParameterization::~AddTimeParameterization()
{
    
}

void AddTimeParameterization::VIMPCallback( motion_planning_msgs::msg::WaypointPath msg )
{
    // Create new JointTrajectory messages for arm and gripper
    robot_trajectory::RobotTrajectory robot_traj( _model, _group );
    auto msg_arm = trajectory_msgs::msg::JointTrajectory();
    msg_arm.joint_names = arm_joints;

    // Define a waypoint for the robot and add it to the trajectory. Waypoints can be passed in as vectors,
    // meaning they can be passed in from GVIMP output
    moveit::core::RobotState robot_waypoint( _model );
    robot_waypoint.setToDefaultValues();
    double dt = 0.1; // Default time step between waypoints, updated by totgComputeTimeStamps

    // Add waypoints to robot trajectory
    // Note that each waypoint has position+vel, but we only need position
    std::vector<Eigen::VectorXd> original_waypoints;
    for ( size_t i=0; i<msg.waypoints.size()/msg.waypoint_size; ++i ) {
        std::vector<double> waypoint(msg.waypoint_size/2);
        std::copy( msg.waypoints.begin()+i*msg.waypoint_size, 
            msg.waypoints.begin()+i*msg.waypoint_size+msg.waypoint_size/2, waypoint.begin() );
        original_waypoints.push_back( Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(waypoint.data(), waypoint.size()) );
        robot_waypoint.setJointGroupPositions( _group, waypoint );
        robot_traj.addSuffixWayPoint( robot_waypoint, dt );
    }

    std::vector<double> discWaypointSpeedFactors;
    std::cout << "Speed factors:" << std::endl;
    double minEntropy = ( *std::min_element( msg.waypoint_entropies.begin(), msg.waypoint_entropies.end() ));
    for ( auto it : msg.waypoint_entropies ) {
        discWaypointSpeedFactors.push_back( minEntropy/it ); // Inverse gives the speed-up factor since less time = faster to reach
        std::cout << minEntropy/it << " ";
    }
    std::cout << std::endl;
    
    // Compute time stamps for robot trajectory
    double max_vel_scaling = 1.0;
    double max_accel_scaling = 1.0;
    int num_divisions_per_waypoint = 4;
    const size_t num_waypoints = num_divisions_per_waypoint * ( discWaypointSpeedFactors.size()-1 ) + 1;
    totgComputeTimeStamps( num_waypoints, robot_traj, max_vel_scaling, max_accel_scaling );

    // Create weights for each trajectory point from our discontinuous speed factors
    std::vector<double> contWaypointSpeedFactors = linearize( robot_traj, original_waypoints, discWaypointSpeedFactors );
    double entropy_dilation_term = 0;

    // Create JointTrajectoryPoints for arm
    for ( size_t i=0; i<robot_traj.getWayPointCount(); ++i ) {
        double min_speed_factor = 0.75; // Set a hard limit so we don't command the arm to go faster than its velocity limit
        double speed_factor = std::max(min_speed_factor, contWaypointSpeedFactors[i])-1; // Subtract 1 to get the difference from base speed
        entropy_dilation_term += speed_factor*robot_traj.getWayPointDurationFromStart(i); // Since we're only applying to differences, we must
                                                                                              // sum the differences to get total duration change
        if ( i>0 ) {
            entropy_dilation_term -= speed_factor*robot_traj.getWayPointDurationFromStart(i-1);
        }
        // data only accessible as Eigen objects, so we first get joint values then convert to vector
        auto point_arm = trajectory_msgs::msg::JointTrajectoryPoint();
        Eigen::VectorXd eWaypoint;
        robot_traj.getWayPoint(i).copyJointGroupPositions( _group, eWaypoint );
        point_arm.positions = std::vector<double>( eWaypoint.data(), eWaypoint.data() + eWaypoint.size() );
        std::chrono::nanoseconds waypoint_time_from_start( uint64_t((robot_traj.getWayPointDurationFromStart(i) + entropy_dilation_term) * 1e9) );
        point_arm.time_from_start = rclcpp::Duration( waypoint_time_from_start );
        msg_arm.points.push_back( point_arm );
    }
    _pose_publisher->publish( msg_arm );
}

// Source: Moveit2
bool AddTimeParameterization::totgComputeTimeStamps ( const size_t num_waypoints, robot_trajectory::RobotTrajectory& trajectory,
                           const double max_velocity_scaling_factor, const double max_acceleration_scaling_factor )
{
    trajectory_processing::TimeOptimalTrajectoryGeneration default_totg( 0.05 /* default path tolerance */, 0.1 /* default resample_dt */ );
    default_totg.computeTimeStamps( trajectory, max_velocity_scaling_factor, max_acceleration_scaling_factor );
    double optimal_duration = trajectory.getDuration();
    double new_resample_dt = optimal_duration / ( num_waypoints-1 );
    trajectory_processing::TimeOptimalTrajectoryGeneration resample_totg( 0.05 /* path tolerance */, new_resample_dt );
    resample_totg.computeTimeStamps( trajectory, max_velocity_scaling_factor, max_acceleration_scaling_factor );
    return true;
}

std::vector<double> AddTimeParameterization::linearize ( robot_trajectory::RobotTrajectory traj,
                                                            std::vector<Eigen::VectorXd> original_traj, std::vector<double> data )
{
    std::vector<double> res;

    // Create iterators for the original waypoint path (means and entropies)
    std::vector<Eigen::VectorXd>::const_iterator curr_waypoint_iterator = original_traj.begin();
    std::vector<Eigen::VectorXd>::const_iterator next_waypoint_iterator = curr_waypoint_iterator;
    std::vector<double>::const_iterator curr_data_iterator = data.begin();
    std::vector<double>::const_iterator next_data_iterator = curr_data_iterator;
    ++next_waypoint_iterator;
    ++next_data_iterator;
    std::vector<Eigen::VectorXd>::const_iterator next_next_waypoint_iterator = next_waypoint_iterator;
    ++next_next_waypoint_iterator;
    size_t robot_traj_it = 0;

    // Compute result at each point in traj
    while ( robot_traj_it < traj.size() ) {

        // Load waypoint from the RobotTrajectory into an eigen struct
        Eigen::VectorXd intermediate_waypoint((*curr_waypoint_iterator).size());
        for ( int i=0; i<(*curr_waypoint_iterator).size(); ++i) {
            intermediate_waypoint[i] = traj.getWayPoint(robot_traj_it).getVariablePositions()[i];
        }

        // Compute how far away the waypoint is from each of its parents and normalize
        double curr_waypoint_diff = ( *curr_waypoint_iterator-intermediate_waypoint ).norm();
        double next_waypoint_diff = ( *next_waypoint_iterator-intermediate_waypoint ).norm();

        // If you aren't at the final 2 original waypoints, check for transitions
        if ( next_next_waypoint_iterator != original_traj.end() ) {
            double next_next_waypoint_diff = ( *next_next_waypoint_iterator-intermediate_waypoint ).norm();

            // Increment iterators when you get closer to the next next waypoint. This condition only
            // works assuming some level of curvature between points, which is exactly what TOTG
            // guarantees.
            if ( next_next_waypoint_diff <= curr_waypoint_diff ) {
                curr_waypoint_iterator = next_waypoint_iterator;
                next_waypoint_iterator = next_next_waypoint_iterator;
                ++next_next_waypoint_iterator;
                curr_data_iterator = next_data_iterator;
                ++next_data_iterator;
                curr_waypoint_diff = ( *curr_waypoint_iterator-intermediate_waypoint ).norm();
                next_waypoint_diff = ( *next_waypoint_iterator-intermediate_waypoint ).norm();
            }

            // Compute weighted sum of data values to get interpolated value. Intuition: if
            // next_waypoint_diff is ~= 0, then we are very close to the next waypoint and
            // so the majority of the weight should go to the next data point.
            double sum = curr_waypoint_diff + next_waypoint_diff;
            res.push_back( ((*curr_data_iterator) * (next_waypoint_diff/sum)) + ((*next_data_iterator) * (curr_waypoint_diff/sum)) );
        }
        // Otherwise keep going until you've iterated over every waypoint
        else {
            double sum = curr_waypoint_diff + next_waypoint_diff;
            res.push_back( ((*curr_data_iterator) * (next_waypoint_diff/sum)) + ((*next_data_iterator) * (curr_waypoint_diff/sum)) );
        }

        ++robot_traj_it;
    }

    return res;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<AddTimeParameterization>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}