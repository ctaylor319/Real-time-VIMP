/**
 * Georgia Institute of Technology, 2024
 * @file: octomap_to_gridmap.h
 * @author: ctaylor319@gatech.edu
 * @date: 09/12/2024
 * @brief: converts octomap message into a grid_map message, which is used
 * by motion_planning to create an SDF of the environment
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "octomap/octomap.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"

class MapConversion : public rclcpp::Node
{
public:
    MapConversion();
    ~MapConversion();

private:
    void timerCallback(octomap_msgs::msg::Octomap msg) const;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr _gridmap_publisher;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr _octomap_subscriber;

};