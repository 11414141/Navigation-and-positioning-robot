import os.path
from datetime import datetime

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')
    session_id = datetime.now().strftime('%Y%m%d_%H%M%S')
    default_session_dir = os.path.join(
        '/home/wheeltec/maps/fast_lio_sessions', session_id)
    default_raw_dir = os.path.join(default_session_dir, 'raw')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    map_file_path = LaunchConfiguration('map_file_path')
    convert_livox_points = LaunchConfiguration('convert_livox_points')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_map_file_path_cmd = DeclareLaunchArgument(
        'map_file_path',
        default_value=os.path.join(default_raw_dir, 'fast_lio_full.pcd'),
        description='Full output PCD path for FAST-LIO full 3D map'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_convert_livox_points_cmd = DeclareLaunchArgument(
        'convert_livox_points', default_value='true',
        description='Publish /livox/lidar_points helper PointCloud2 for FAST-LIO RViz'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {
                        'use_sim_time': use_sim_time,
                        'map_file_path': map_file_path,
                    }],
        output='screen'
    )
    # 原：FAST-LIO RViz只能显示PointCloud2结果点云
    # 改：旁路转换/livox/lidar(CustomMsg)为/livox/lidar_points(PointCloud2)，仅用于RViz显示原始雷达点云
    livox_custom_to_pointcloud2_node = Node(
        package='fast_lio',
        executable='livox_custom_to_pointcloud2',
        condition=IfCondition(convert_livox_points),
        parameters=[{
            # 原：无原始雷达点云PointCloud2显示话题
            # 改：将/livox/lidar(CustomMsg)旁路转换为/livox/lidar_points(PointCloud2)。
            'input_topic': '/livox/lidar',
            'output_topic': '/livox/lidar_points',
            # 原：原始点云frame_id为livox_frame，FAST-LIO RViz固定帧为camera_init时缺少直接TF
            # 改：输出到body帧，并应用LiDAR->IMU平移外参，沿用FAST-LIO的camera_init->body TF显示。
            'output_frame': 'body',
            'lidar_to_body_translation': [-0.011, -0.02329, 0.04412],
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_map_file_path_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_convert_livox_points_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(fast_lio_node)
    ld.add_action(livox_custom_to_pointcloud2_node)
    ld.add_action(rviz_node)

    return ld
