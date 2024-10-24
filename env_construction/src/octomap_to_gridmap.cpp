/**
 * @copyright Georgia Institute of Technology, 2024
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
#include <octomap_msgs/msg/octomap.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <octomap/octomap.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_octomap/GridMapOctomapConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include "octomap_to_gridmap.h"

using namespace std::chrono_literals;

MapConversion::MapConversion() : Node( "octomap_to_gridmap" )
{
    _octomap_subscriber = create_subscription< octomap_msgs::msg::Octomap >( "/octomap_binary", 10,
        std::bind(&MapConversion::octomapCallback, this, std::placeholders::_1) );
    _gridmap_publisher = create_publisher<grid_map_msgs::msg::GridMap>( "/gridmap_binary", 10 );
}

MapConversion::~MapConversion()
{
}

void MapConversion::octomapCallback ( octomap_msgs::msg::Octomap msg ) const
{
    grid_map::GridMap gridMap( {"elevation"} ); //create grid map with 3D elevation layer
    gridMap.setBasicLayers( {"elevation"} );

    octomap::OcTree* octomap = nullptr;
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap( msg );
    if ( tree ) {
        octomap = dynamic_cast<octomap::OcTree*>( tree ); // convert octomap message to an OcTree
    } else {
        std::cout << "Failed to call convert Octomap." << std::endl;
        return;
    }

    // Copy over the bounds of the octomap
    grid_map::Position3 min_bound;
    grid_map::Position3 max_bound;
    octomap->getMetricMin( min_bound(0), min_bound(1), min_bound(2) );
    octomap->getMetricMax( max_bound(0), max_bound(1), max_bound(2) );
    // Convert to grid map, publish if it was successful
    bool res = grid_map::GridMapOctomapConverter::fromOctomap( *octomap, "elevation", gridMap, &min_bound, &max_bound );
    if ( res ) {
        auto out_msg = grid_map::GridMapRosConverter::toMessage( gridMap );
        _gridmap_publisher->publish( std::move(out_msg) );
    }
    else {
        std::cout << "Failed to convert octomap to grid_map" << std::endl;
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MapConversion>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}