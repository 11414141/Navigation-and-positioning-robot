#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("wheeltec_rviz2"),
        "rviz",
        "wheeltec.rviz",
    )
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_config,
            description="RViz config file. Defaults to wheeltec_rviz2/rviz/wheeltec.rviz.",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="myrobot_nav2_rviz",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])
