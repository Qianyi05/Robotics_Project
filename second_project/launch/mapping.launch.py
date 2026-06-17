from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('second_project')

    slam_config = os.path.join(
        pkg_share,
        'config',
        'slam_toolbox_mapping.yaml'
    )

    rviz_config = os.path.join(
        pkg_share,
        'rviz',
        'mapping.rviz'
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time': True,

            # TF tree:
            # UGV_odom -> UGV_base_link -> rslidar
            'target_frame': 'UGV_base_link',
            'transform_tolerance': 0.5,

            # Height slicing from 3D PointCloud2 to 2D LaserScan
            # Filter out ground points and vehicle-body noise
            'min_height': 0.05,
            'max_height': 1.2,

            # 360-degree laser scan
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,

            'scan_time': 0.1,
            'range_min': 0.5,
            'range_max': 30.0,
            'use_inf': True,
        }],
        remappings=[
            ('cloud_in', '/ugv/rslidar_points'),
            ('scan', '/base_scan'),
        ]
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': True}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),

        pointcloud_to_laserscan_node,
        slam_toolbox_node,
        rviz_node
    ])