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
_K_p_dist(0.8),
_K_p_angle(1.0),
_K_i_dist(0.0),
_K_i_angle(0.0),
_K_d_dist(0.0),
_K_d_angle(0.0),
_mode(ControllerMode::Default),
_eps(0.1)
{
    _path_subscriber = create_subscription<motion_planning_msgs::msg::WaypointPath>( "/vimp_path", 10,
        std::bind( &ComputePoseUpdate::VIMPCallback, this, std::placeholders::_1 ) );
    _pose_subscriber = create_subscription<nav_msgs::msg::Odometry>( "/odom", 10,
        std::bind( &ComputePoseUpdate::PoseCallback, this, std::placeholders::_1 ) );
    _pose_publisher = create_publisher<geometry_msgs::msg::Twist>( "/cmd_vel", 10 );
    _waypoint_queue.push(Eigen::VectorXd{{1.0, 2.0, 0.0, 0.0}});
    _curr_waypoint = Eigen::VectorXd{{0.0, 1.0, 0.1, 0.1}};
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
    vel_out.linear.x = 0; vel_out.linear.y = 0; vel_out.angular.z = 0;

    double time = msg.header.stamp.sec + double(msg.header.stamp.nanosec)/1e9;
    double dt = time - _prevTime;
    double distError = 0;
    double angleError = 0;

    if ( _curr_waypoint.size() > 0 ) {
        Eigen::VectorXd currPos(2);
        currPos << msg.pose.pose.position.x, msg.pose.pose.position.y;
        Eigen::VectorXd desiredPos = _curr_waypoint.segment(0, 2);
        distError = (desiredPos - currPos).norm();

        double currAngle = Quat2Theta( msg.pose.pose.orientation );
        double desiredAngle = atan2( (desiredPos - currPos)[1], (desiredPos - currPos)[0] );

        // Get the transformation matrix from fixed to body frame (all velocities must be in body frame)
        Eigen::MatrixXd T(2, 2);
        T << cos(currAngle), -sin(currAngle),
             sin(currAngle),  cos(currAngle);

        // Extract current velocity
        Eigen::VectorXd currentVel(2);
        currentVel << msg.twist.twist.linear.x, msg.twist.twist.linear.y;

        // atan2's range is from [-pi, pi). Convert to [0, 2pi).
        currAngle = currAngle >= 0 ? currAngle : fmod((currAngle+3*M_PI), 2*M_PI)+M_PI;
        desiredAngle = desiredAngle >= 0 ? desiredAngle : fmod((desiredAngle+3*M_PI), 2*M_PI)+M_PI;

        // Take error and convert back to [-pi, pi) range
        angleError = fmod((desiredAngle - currAngle)+M_PI, 2*M_PI)-M_PI;

        // std::cout << "distError: " << distError << std::endl;
        // std::cout << "currAngle: " << currAngle << std::endl;
        // std::cout << "desiredAngle: " << desiredAngle << std::endl;
        // std::cout << "angleError: " << angleError << std::endl;

        // First make sure we have a path to follow
        if ( _waypoint_queue.size() > 0 ) {
            if ( distError <= _eps && _waypoint_queue.size() > 0 ) {
                if ( _mode == ControllerMode::Transition ) {
                    _mode = ControllerMode::Default;
                    _I_angle = 0;
                    _I_dist = 0;
                }

                // If we're close to waypoint position, create smooth transition to waypoint velocity
                TransitionControl( currentVel, T, dt, vel_out );
            }
            else {
                if ( _mode == ControllerMode::Default ) {
                    _mode = ControllerMode::Transition;
                    _I_angle = 0;
                    _I_dist = 0;
                }

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
    _I_dist += distError*dt; _I_angle += angleError*dt;
    double D_dist = 0; double D_angle = 0;
    if ( dt > 0 ) { // Division by zero check
        D_dist = (distError - _prevDistError)/dt;
        D_angle = (angleError - _prevAngleError)/dt;
    }

    double PID_dist = _K_p_dist*P_dist + _K_i_dist*_I_dist + _K_d_dist*D_dist;
    double PID_angle = _K_p_angle*P_angle + _K_i_angle*_I_angle + _K_d_angle*D_angle;

    if ( abs(PID_dist) >= 0.2 ) {
        PID_dist = 0.2*(PID_dist/abs(PID_dist));
    }
    if ( abs(PID_angle) >= 2.5 ) {
        PID_angle = 2.5*(PID_angle/abs(PID_angle));
    }

    twist.linear.x = PID_dist * cos( angleError );
    twist.linear.y = PID_dist * sin( angleError );
    twist.angular.z = PID_angle;

    // std::cout << "vx: " << twist.linear.x << std::endl;
    // std::cout << "vy: " << twist.linear.y << std::endl;
    // std::cout << "vtheta: " << PID_angle << std::endl << std::endl;
}

void ComputePoseUpdate::TransitionControl( Eigen::VectorXd currVel, Eigen::MatrixXd T, 
                                            double dt, geometry_msgs::msg::Twist &twist )
{
    // Compute the desired orientation of the robot based on waypoint velocity
    Eigen::VectorXd desiredVel = T.transpose()*_curr_waypoint.segment(2, 2);
    Eigen::VectorXd vel_diff = desiredVel - currVel;
    double nextAngleError = atan2(vel_diff[1], vel_diff[0]);

    double P_angle = nextAngleError;
    _I_angle += nextAngleError*dt;
    double D_angle = 0;
    if ( dt > 0 ) { // Division by zero check
        D_angle = (nextAngleError - _prevAngleError)/dt;
    }
    double PID_angle = _K_p_angle*P_angle + _K_i_angle*_I_angle + _K_d_angle*D_angle;


    if ( abs(PID_angle) >= 2.5 ) {
        PID_angle = 2.5*(PID_angle/abs(PID_angle));
    }

    twist.linear.x = desiredVel[0]*cos(nextAngleError);
    twist.linear.y = desiredVel[1]*sin(nextAngleError);
    twist.angular.z = nextAngleError;

    // std::cout << "vx: " << twist.linear.x << std::endl;
    // std::cout << "vy: " << twist.linear.y << std::endl;
    // std::cout << "vtheta: " << nextAngleError << std::endl << std::endl;

    double max_lin_vel_diff = 0.01;
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