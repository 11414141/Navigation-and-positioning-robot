#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    start_livox = LaunchConfiguration("start_livox")

    robot_dir = get_package_share_directory("turn_on_wheeltec_robot")
    robot_launch_dir = os.path.join(robot_dir, "launch")
    nav2_dir = get_package_share_directory("wheeltec_nav2")
    nav2_launch_dir = os.path.join(nav2_dir, "launch")
    livox_dir = get_package_share_directory("livox_ros_driver2")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true",
        ),
        DeclareLaunchArgument(
            "map",
            default_value="/home/wheeltec/ws_2dmap/maps/verygood_map.yaml",
            description="Full path to map yaml file",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value="/home/wheeltec/rtk_tools/config/amcl_nav2_params.yaml",
            description="AMCL Nav2 parameter file",
        ),
        DeclareLaunchArgument(
            "start_livox",
            default_value="true",
            description="Start Livox MID360s driver and /scan converter",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_launch_dir, "turn_on_wheeltec_robot.launch.py")),
        ),
        Node(
            package="nav2_waypoint_cycle",
            executable="nav2_waypoint_cycle",
            name="waypoint_cycle",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(livox_dir, "launch_ROS2", "msg_MID360s_launch.py")),
            condition=IfCondition(start_livox),
        ),
        Node(
            package="livox_nav_tools",
            executable="livox_custom_to_pointcloud2",
            name="livox_custom_to_pointcloud2_amcl",
            output="screen",
            condition=IfCondition(start_livox),
            parameters=[{
                "input_topic": "/livox/lidar",
                "output_topic": "/livox/lidar_points",
                "nav_output_topic": "/livox/lidar_points_nav",
                "glim_output_topic": "/livox/lidar_points_glim",
                "output_frame": "livox_frame",
                "scan_topic": "/scan",
                "min_height": 0.0,
                "max_height": 0.15,
                "angle_min": -3.1416,
                "angle_max": 3.1416,
                "angle_increment": 0.0087,
                "range_min": 0.4,
                "range_max": 50.0,
                "use_inf": False,
            }],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher_livox_amcl",
            arguments=[
                "0.14", "0.0", "0.16",
                "0", "0", "0",
                "base_footprint", "livox_frame",
            ],
            condition=IfCondition(start_livox),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, "bringup_launch.py")),
            launch_arguments={
                "map": map_file,
                "use_sim_time": use_sim_time,
                "params_file": params_file,
                "slam": "False",
                "use_composition": "True",
                "autostart": "true",
            }.items(),
        ),
    ])
