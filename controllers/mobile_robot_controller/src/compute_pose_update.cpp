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
#include <queue>

#include <motion_planning_msgs/msg/waypoint_path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <urdf_parser/urdf_parser.h>

#include <Poco/Util/AbstractConfiguration.h>
#include <Poco/Util/XMLConfiguration.h>

#include "compute_pose_update.h"

#define STRING(x) #x
#define XSTRING(x) STRING(x)

using namespace std::chrono_literals;

ComputePoseUpdate::ComputePoseUpdate() : Node( "compute_pose_update" ),
_K_p_dist(0.0),
_K_p_angle(0.0),
_K_i_dist(0.0),
_K_i_angle(0.0),
_K_d_dist(0.0),
_K_d_angle(0.0),
_eps(0.0)
{
    _path_subscriber = create_subscription<motion_planning_msgs::msg::WaypointPath>( "/vimp_path", 10,
        std::bind( &ComputePoseUpdate::VIMPCallback, this, std::placeholders::_1 ) );
    _pose_subscriber = create_subscription<nav_msgs::msg::Odometry>( "/odom", 10,
        std::bind( &ComputePoseUpdate::PoseCallback, this, std::placeholders::_1 ) );
    _pose_publisher = create_publisher<geometry_msgs::msg::Twist>( "/cmd_vel", 10 );
}

ComputePoseUpdate::~ComputePoseUpdate()
{

}

void ComputePoseUpdate::VIMPCallback( motion_planning_msgs::msg::WaypointPath msg )
{
    if ( _waypoint_queue.size() > 0 ) {
        std::queue<Eigen::VectorXd> empty_q;
        std::swap(_waypoint_queue, empty_q);
    }
    for ( size_t i=0; i<msg.waypoints.size()/msg.waypoint_size; ++i ) {
        std::vector<double> waypoint(msg.waypoint_size/2);
        std::copy( msg.waypoints.begin()+i*msg.waypoint_size, 
            msg.waypoints.begin()+i*msg.waypoint_size+msg.waypoint_size/2, waypoint.begin() );
        _waypoint_queue.push( Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(waypoint.data(), waypoint.size()) );
    }
    _curr_waypoint = _waypoint_queue.front();
    _waypoint_queue.pop();

    // std::vector<double> discWaypointSpeedFactors;
    // std::cout << "Speed factors:" << std::endl;
    // double minEntropy = ( *std::min_element( msg.waypoint_entropies.begin(), msg.waypoint_entropies.end() ));
    // for ( auto it : msg.waypoint_entropies ) {
    //     discWaypointSpeedFactors.push_back( minEntropy/it ); // Inverse gives the speed-up factor since less time = faster to reach
    //     std::cout << minEntropy/it << " ";
    // }
}

void ComputePoseUpdate::PoseCallback( nav_msgs::msg::Odometry msg )
{
    geometry_msgs::msg::Twist vel_out;
    vel_out.linear.x = 0; vel_out.linear.y = 0;

    Eigen::VectorXd currPos;
    currPos << msg.pose.pose.position.x, msg.pose.pose.position.y;
    Eigen::VectorXd desiredPos = _curr_waypoint.block(0, 0, 1, 2);
    double distError = (desiredPos - currPos).norm();

    double currAngle = Quat2Theta( msg.pose.pose.orientation );
    double desiredAngle = atan2( (desiredPos - currPos)[1], (desiredPos - currPos)[0] );

    // atan2's range is from [0, 2*pi), so we change the range to [-pi/2, pi/2) since having error
    // at the sup/inf of can be problematic
    double angleError = fmod((desiredAngle - currAngle)+M_PI, 2*M_PI)-M_PI;

    double time = msg.header.stamp.sec + double(msg.header.stamp.nanosec)/1e9;
    double dt = time - _prevTime;

    // First make sure we have a path to follow
    if ( _waypoint_queue.size() > 0 ) {
        if ( distError <= _eps && _waypoint_queue.size() > 0 ) {
            Eigen::VectorXd currentVel;
            currentVel << msg.twist.twist.linear.x, msg.twist.twist.linear.y;

            // Get angle to next waypoint
            desiredPos = _waypoint_queue.front().block(0, 0, 1, 2);
            desiredAngle = atan2( (desiredPos - currPos)[1], (desiredPos - currPos)[0] );
            angleError = fmod((desiredAngle - currAngle)+M_PI, 2*M_PI)-M_PI;

            TransitionControl( distError, angleError, currentVel, dt, vel_out );
        }
        else {
            // Default is PID
            PIDControl( distError, angleError, dt, vel_out );
        }
    }
    else if ( _curr_waypoint.size() > 0 ) {
        // Apply PID until goal reached
        PIDControl( distError, angleError, dt, vel_out );
        if ( distError <= _eps ) {
            _curr_waypoint.resize(0);
        }
    }
    _prevTime = time;
    _prevDistError = distError;
    _prevAngleError = angleError;
    _pose_publisher->publish( vel_out );
}

inline double ComputePoseUpdate::Quat2Theta( geometry_msgs::msg::Quaternion q )
{
    return atan2( 2*(q.x*q.y + q.w*q.z), 1 - 2*(q.y*q.y + q.z*q.z) );
}

void ComputePoseUpdate::PIDControl( double distError, double angleError, double dt, geometry_msgs::msg::Twist &twist )
{
    double P_dist = distError; double P_angle = angleError;
    double I_dist = distError*dt; double I_angle = angleError*dt;
    double D_dist = 0; double D_angle = 0;
    if ( dt > 0 ) { // Division by zero check
        D_dist = (distError - _prevDistError)/dt;
        D_angle = (angleError - _prevAngleError)/dt;
    }

    double PID_dist = _K_p_dist*P_dist + _K_i_dist*I_dist + _K_d_dist*D_dist;
    double PID_angle = _K_p_angle*P_angle + _K_i_angle*I_angle + _K_d_angle*D_angle;

    twist.linear.x = PID_dist * cos( angleError );
    twist.linear.y = PID_dist * sin( angleError );
    twist.angular.z = PID_angle;
}

void ComputePoseUpdate::TransitionControl( double distError, double angleError, Eigen::VectorXd currVel, double dt, geometry_msgs::msg::Twist &twist )
{
    double P_angle = angleError;
    double I_angle = angleError*dt;
    double D_angle = 0;
    if ( dt > 0 ) { // Division by zero check
        D_angle = (angleError - _prevAngleError)/dt;
    }
    double PID_angle = _K_p_angle*P_angle + _K_i_angle*I_angle + _K_d_angle*D_angle;

    Eigen::VectorXd desiredVel = _curr_waypoint.block(0, 2, 1, 2);

    twist.linear.x = ( (distError*currVel[0]) + ((_eps-distError)*desiredVel[0]) ) / _eps;
    twist.linear.y = ( (distError*currVel[1]) + ((_eps-distError)*desiredVel[1]) ) / _eps;
    twist.angular.z = PID_angle;

    double max_lin_vel_diff = 0.05;
    if ( (desiredVel - currVel).norm() <= max_lin_vel_diff ) {
        _curr_waypoint = _waypoint_queue.front();
        _waypoint_queue.pop();
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ComputePoseUpdate>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}