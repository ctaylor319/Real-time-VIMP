/**
 * @copyright Georgia Institute of Technology, 2024
 * @file: GVIMPImpl.cpp
 * @author: ctaylor319@gatech.edu
 * @date: 10/07/2024
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <std_msgs/msg/bool.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include "motion_planning_msgs/msg/waypoint_path.hpp"

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_sdf/SignedDistanceField.hpp>
#include <gpmp2/obstacle/SignedDistanceField.h>

#include <Poco/Util/AbstractConfiguration.h>
#include <Poco/Util/XMLConfiguration.h>
#include <Poco/StringTokenizer.h>
#include <Poco/NumberParser.h>

#include "GVIMPImpl.h"

GVIMPImpl::GVIMPImpl() : Node( "GVIMP_Impl" )
{
    _gridmap_subscriber = create_subscription<grid_map_msgs::msg::GridMap>( "/gridmap_binary", 10,
        std::bind( &GVIMPImpl::gridMapCallback, this, std::placeholders::_1 ) );
    _path_status_subscriber = create_subscription<std_msgs::msg::Bool>( "/curr_subpath_complete", 10,
        std::bind( &GVIMPImpl::pathStatusCallback, this, std::placeholders::_1 ) );
    _robot_state_subscriber = create_subscription<control_msgs::msg::JointTrajectoryControllerState>( "/arm_controller/controller_state", 10,
        std::bind( &GVIMPImpl::StateCallback, this, std::placeholders::_1 ) );
    _path_publisher = create_publisher<motion_planning_msgs::msg::WaypointPath>( "/vimp_path", 10 );

    _path_planner = std::unique_ptr<vimp::RobotArmMotionPlanner>( new vimp::RobotArmMotionPlanner() ); // Instance of the motion planner

    // Initialize config file reader
    Poco::Util::AbstractConfiguration *cfg;
    std::string install_dir = XSTRING( INSTALL_DIR );
    try {
        cfg = new Poco::Util::XMLConfiguration( install_dir+"/lib/sim_config.xml" );
    } catch ( const std::exception& e ) {
        std::cout << "Unable to open configuration file: " << e.what() << std::endl;
    }

    // Read in the goal position
    try {
        std::vector<double> goal_vec;
        Poco::StringTokenizer goal_str( cfg->getString("goal"), " " );
        for ( Poco::StringTokenizer::Iterator it = goal_str.begin(); it != goal_str.end(); ++it ) {
            goal_vec.push_back( Poco::NumberParser::parseFloat(*it) );
        }
        _goal_pos = Map<VectorXd, Unaligned>( goal_vec.data(), goal_vec.size() );
    } catch ( const std::exception& e ) {
        std::cout << "Unable to parse goal pose: " << e.what() << std::endl;
    }

    // If the number of joint positions you gave is different then how many the robot model has,
    // throw an exception
    if ( _goal_pos.size() != _path_planner->getRobotSDF().nlinks() ) {
        throw std::invalid_argument("Goal has incorrect number of joint positions");
    }
    _new_path_needed = true;
}

GVIMPImpl::~GVIMPImpl()
{
}

void GVIMPImpl::pathStatusCallback ( std_msgs::msg::Bool msg )
{
    _new_path_needed = msg.data;
}

void GVIMPImpl::StateCallback ( control_msgs::msg::JointTrajectoryControllerState msg )
{
    trajectory_msgs::msg::JointTrajectoryPoint curr_state = msg.feedback;
    std::vector<double> start_pos = curr_state.positions;
    _start_pos = Map<VectorXd>(&start_pos[0], start_pos.size()); // Map the position to an Eigen object
}

void GVIMPImpl::gridMapCallback ( grid_map_msgs::msg::GridMap msg )
{
    // Check if a new path is needed and if the current pose is known
    if ( _new_path_needed && _start_pos.size() > 0 ) {

        // Find new path
        _new_path_needed = false;
        gpmp2::SignedDistanceField sdf = generateSDF( msg );
        std::pair<VectorXd, SpMat> res = _path_planner->findBestPath( _start_pos, _goal_pos, sdf );
        
        // Set message data
        motion_planning_msgs::msg::WaypointPath best_path;
        best_path.waypoint_size = _path_planner->getParams().nx();
        for(int i=0; i<res.first.size(); ++i){
            best_path.waypoints.push_back(res.first[i]);
        }
        _path_publisher->publish( best_path );
    }
}

gpmp2::SignedDistanceField GVIMPImpl::generateSDF ( grid_map_msgs::msg::GridMap msg )
{
    // Read in message data to a grid_map object
    std::string elevationLayer = msg.layers[0];
    grid_map::GridMap gridMap;
    std::vector<std::string> layers{ elevationLayer };
    grid_map::GridMapRosConverter::fromMessage( msg, gridMap, layers, false, false );
    auto& gm_data = gridMap.get( elevationLayer );

    // Replace NaNs with min value of map data
    if ( gm_data.hasNaN() ) {
        const float minMapVal{ gm_data.minCoeffOfFinites() };
        gm_data = gm_data.unaryExpr( [=](float v) { return std::isfinite(v)? v : minMapVal; } );
    }
    const float heightMargin = 0.1;
    const float minVal = gm_data.minCoeffOfFinites() - heightMargin;
    const float maxVal = gm_data.maxCoeffOfFinites() + heightMargin;
    grid_map::SignedDistanceField gm_sdf( gridMap, elevationLayer, minVal, maxVal );

    // Get SDF parameters
    double cell_size = gridMap.getResolution();
    grid_map::Size grid_size = gridMap.getSize();
    grid_map::Position gridOrigin;
    gridMap.getPosition( Vector2i(0, 0), gridOrigin );
    grid_map::Length len = gridMap.getLength();
    gtsam::Point3 origin( gridOrigin.x()-len.x(), gridOrigin.y()-len.y(), minVal );

    // Set up data parsing to a gpmp2 SDF
    int row = 0; int col = 0; int z = 0;
    int y_end = grid_size.y()-1;
    int x_end = grid_size.x()-1;
    std::vector<gtsam::Matrix> gpmp2_data;

    // Since grid_map stores 3D data in a single eigen matrix, we can't just cast it to a gtsam matrix
    // We can use their grid_map_sdf's iteration algorithm to extract the data
    gm_sdf.filterPoints( [&](const grid_map::Position3&, float sdfValue, const grid_map::SignedDistanceField::Derivative3) {
        if ( row == 0 && col == 0 ) {
            gtsam::Matrix temp = gtsam::Matrix::Zero( grid_size.y(), grid_size.x() );
            gpmp2_data.push_back( temp );
        }
        gpmp2_data[z](y_end-row, x_end-col) = static_cast<double>( sdfValue );
        ++col;
        if ( col == grid_size.x() ) {
            col = 0;
            ++row;
        }
        if ( row == grid_size.y() ) {
            row = 0;
            ++z;
        }
    } );
    
    // Parse grid_map_sdf into gpmp2_sdf
    gpmp2::SignedDistanceField gpmp2_sdf( origin, cell_size, gpmp2_data );

    // For testing purposes
    // for ( int z=0; z<7; ++z ) {
    //     for ( int y=0; y<12; ++y ) {
    //         for ( int x=0; x<12; ++x ) {
    //             grid_map::Position3 gmTestPoint( double(x)/10.0, double(y)/10.0, double(z)/10.0 );
    //             gtsam::Point3 gtsamTestPoint( double(x)/10.0, double(y)/10.0, double(z)/10.0 );

    //             std::cout << x << " " << y << " " << z << std::endl;
    //             std::cout << "grid_map test: " << gm_sdf.value(gmTestPoint) << std::endl;
    //             std::cout << "gtsam test: " << gpmp2_sdf.getSignedDistance(gtsamTestPoint) << std::endl;
    //             std::cout << std::endl;
    //         }
    //     }
    // }

    return gpmp2_sdf;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<GVIMPImpl>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}