import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  start_env_construction_cmd = Node(
    package="env_construction",
    executable="octomap_to_gridmap"
  )

  start_gvimp_cmd = Node(
    package="motion_planning",
    executable="GVIMPImpl"
  )

  start_time_parameterization_cmd = Node(
    package="robot_control",
    executable="add_time_parameterization"
  )

  start_robot_control_cmd = Node(
    package="robot_control",
    executable="robot_control_interface"
  )

    
  # Create the launch description and populate
  ld = LaunchDescription()

  # Add any actions
  ld.add_action(start_env_construction_cmd)
  ld.add_action(start_gvimp_cmd)
  ld.add_action(start_time_parameterization_cmd)
  #ld.add_action(start_robot_control_cmd)

  return ld



