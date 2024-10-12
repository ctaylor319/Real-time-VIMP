/**
 * @copyright Georgia Institute of Technology, 2024
 * @file: GVIMPImpl.cpp
 * @author: ctaylor319@gatech.edu
 * @date: 09/12/2024
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <std_msgs/msg/bool.hpp>

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
    // Subscribers/Publishers
    _gridmap_subscriber = create_subscription<grid_map_msgs::msg::GridMap>( "/gridmap_binary", 10,
        std::bind( &GVIMPImpl::gridMapCallback, this, std::placeholders::_1 ) );
    _path_status_subscriber = create_subscription<std_msgs::msg::Bool>( "/curr_subpath_complete", 10,
        std::bind( &GVIMPImpl::pathStatusCallback, this, std::placeholders::_1 ) );
    _path_publisher = create_publisher<std_msgs::msg::Float32MultiArray>( "/vimp_path", 10 );

    _path_planner = std::unique_ptr<vimp::RobotArmMotionPlanner>( new vimp::RobotArmMotionPlanner() ); // Instance of the motion planner

    // Read in config parameters
    Poco::Util::AbstractConfiguration *cfg;
    std::string install_dir = XSTRING( INSTALL_DIR );
    try {
        cfg = new Poco::Util::XMLConfiguration( install_dir+"/lib/sim_config.xml" );
    } catch ( const std::exception& e ) {
        std::cout << "Unable to open configuration file: " << e.what() << std::endl;
    }

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

void GVIMPImpl::gridMapCallback ( grid_map_msgs::msg::GridMap msg )
{
    if ( _new_path_needed ) {
        _new_path_needed = false;
        gpmp2::SignedDistanceField sdf = generateSDF( msg );
        std::pair<VectorXd, SpMat> res = _path_planner->findBestPath( _start_pos, _goal_pos, sdf );
        
        std_msgs::msg::Float32MultiArray best_path;
        best_path.data = convertToRosFloat ( res.first );
        _path_publisher->publish( best_path );
    }
}

gpmp2::SignedDistanceField GVIMPImpl::generateSDF ( grid_map_msgs::msg::GridMap msg )
{
    std::cout << "Generating SDF" << std::endl;
    std::string elevationLayer = msg.layers[0];
    grid_map::GridMap gridMap;
    std::vector<std::string> layers{elevationLayer};
    grid_map::GridMapRosConverter::fromMessage( msg, gridMap, layers, false, false );
    auto& gm_data = gridMap.get( elevationLayer );

    // Replace NaNs with min value of map data
    if ( gm_data.hasNaN() ) {
        const float minMapVal{gm_data.minCoeffOfFinites()};
        gm_data = gm_data.unaryExpr( [=](float v) { return std::isfinite(v)? v : minMapVal; } );
    }
    const float heightMargin = 0.1;
    const float minVal = gm_data.minCoeffOfFinites() - heightMargin;
    const float maxVal = gm_data.maxCoeffOfFinites() + heightMargin;
    grid_map::SignedDistanceField gm_sdf( gridMap, elevationLayer, minVal, maxVal );



    double cell_size = gridMap.getResolution();
    grid_map::Size grid_size = gridMap.getSize();
    std::vector<gtsam::Matrix> gpmp2_data;

    // Since grid_map stores 3D data in a single eigen matrix, we can't just cast it to a gtsam matrix
    // We can use their grid_map_sdf's iteration algorithm to extract the data
    int row = 0; int col = 0; int z = 0;
    gm_sdf.filterPoints( [&](const grid_map::Position3&, float sdfValue, const grid_map::SignedDistanceField::Derivative3) {
        if ( row == 0 && col == 0 ) {
            gtsam::Matrix temp = gtsam::Matrix::Zero( grid_size[0], grid_size[1] );
            gpmp2_data.push_back( temp );
        }
        else{
            gpmp2_data[z](row, col) = static_cast<double>( sdfValue );
        }
        // Note: filterPoints has x as the innermost loop, so we must copy that structure
        ++row;
        // std::cout << sdfValue << " ";
        if ( row == grid_size[0] ) {
            row = 0;
            ++col;
            // std::cout << std::endl;
        }
        if ( col == grid_size[1] ) {
            col = 0;
            ++z;
        }  
    } );
    
    gtsam::Point3 origin( 0.0, 0.0, minVal ); // This is how the origin is set in grid_map_sdf

    gpmp2::SignedDistanceField gpmp2_sdf( origin, cell_size, gpmp2_data );

    return gpmp2_sdf;
}

std::vector<float> GVIMPImpl::convertToRosFloat ( VectorXd res )
{
    std::vector<double> first_val;
    Map<VectorXd>(&first_val[0], res.size()) = res;
    std::vector<float> means;
    for ( int i=0; i<res.size(); ++i ) {
        means.push_back( static_cast<float>(first_val[i]) );
    }
    return means;
}

int main(int argc, char * argv[])
{
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);

    // Create an instance of the EnvDataCollect node
    auto node = std::make_shared<GVIMPImpl>();

    // Spin the node to execute the callbacks
    rclcpp::spin(node);

    // Shutdown the ROS 2 client library
    rclcpp::shutdown();

    return 0;
}