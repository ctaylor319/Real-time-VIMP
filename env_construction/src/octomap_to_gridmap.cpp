/**
 * Georgia Institute of Technology, 2024
 * @file: octomap_to_gridmap.cpp
 * @author: ctaylor319@gatech.edu
 * @date: 09/12/2024
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <octomap_msgs/conversions.h>
#include <grid_map_octomap/GridMapOctomapConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include "octomap_to_gridmap.h"

using namespace std::chrono_literals;

MapConversion::MapConversion() : Node("octomap_to_gridmap"){

    _octomap_subscriber = create_subscription< octomap_msgs::msg::Octomap >( "/octomap_full", 10,
        std::bind(&MapConversion::timerCallback, this, std::placeholders::_1) );
    _gridmap_publisher = create_publisher<grid_map_msgs::msg::GridMap>( "/gridmap_full", 10 );

}

MapConversion::~MapConversion()
{

}

void MapConversion::octomapCallback ( octomap_msgs::msg::Octomap msg ) const
{
    grid_map::GridMap gridMap( {"elevation"} ); //create grid map with 3D elevation layer
    gridMap.setBasicLayers( {"elevation"} );
    octomap::OcTree* octomap = nullptr;
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(msg);
    octomap = dynamic_cast<octomap::OcTree*>(tree); // convert octomap message to an OcTree

    grid_map::Position3 min_bound;
    grid_map::Position3 max_bound;
    octomap->getMetricMin( min_bound(0), min_bound(1), min_bound(2) );
    octomap->getMetricMax( max_bound(0), max_bound(1), max_bound(2) );

    bool res = grid_map::GridMapOctomapConverter::fromOctomap( *octomap, "elevation", gridMap, &min_bound, &max_bound );
    if ( res ) {
        auto out_msg = grid_map::GridMapRosConverter::toMessage(gridMap);
        _gridmap_publisher->publish(std::move(out_msg));
    }
    else {
        std::cout << "Failed to convert octomap to grid_map" << std::endl;
    }
}

int main(int argc, char * argv[])
{
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);

    // Create an instance of the EnvDataCollect node
    auto node = std::make_shared<MapConversion>();

    // Spin the node to execute the callbacks
    rclcpp::spin(node);

    // Shutdown the ROS 2 client library
    rclcpp::shutdown();

    return 0;
}