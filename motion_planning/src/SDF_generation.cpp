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

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_sdf/SignedDistanceField.hpp>

#include "SDF_generation.h"

using namespace std::chrono_literals;

SDFGeneration::SDFGeneration() : Node("sdf_generation"){

    _gridmap_subscriber = create_subscription< grid_map_msgs::msg::GridMap >( "/gridmap_full", 10,
        std::bind(&SDFGeneration::gridMapCallback, this, std::placeholders::_1) );
    _sdf_publisher = create_publisher< grid_map_msgs::msg::GridMap >( "/sdf_full", 10 );

}

SDFGeneration::~SDFGeneration()
{

}

void SDFGeneration::gridMapCallback ( grid_map_msgs::msg::GridMap msg ) const
{
    std::string elevationLayer = msg.layers[0];
    grid_map::GridMap gridMap;
    std::vector<std::string> layers{elevationLayer};
    grid_map::GridMapRosConverter::fromMessage(msg, gridMap, layers, false, false);
    auto& data = gridMap.get(elevationLayer);

    // Replace NaNs with min value of map data
    if (data.hasNaN()) {
        const float minMapVal{data.minCoeffOfFinites()};
        data = data.unaryExpr([=](float v) { return std::isfinite(v)? v : minMapVal; });
    }
    const float heightMargin = 0.1;
    const float minVal = data.minCoeffOfFinites() - heightMargin;
    const float maxVal = data.maxCoeffOfFinites() + heightMargin;
    grid_map::SignedDistanceField sdf(gridMap, elevationLayer, minVal, maxVal);
    _sdf_publisher->publish(msg);
}

int main(int argc, char * argv[])
{
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);

    // Create an instance of the EnvDataCollect node
    auto node = std::make_shared<SDFGeneration>();

    // Spin the node to execute the callbacks
    rclcpp::spin(node);

    // Shutdown the ROS 2 client library
    rclcpp::shutdown();

    return 0;
}