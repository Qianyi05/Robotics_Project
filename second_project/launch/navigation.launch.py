from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('second_project')
    nav2_share = FindPackageShare('nav2_bringup')

    world_file = PathJoinSubstitution([
        pkg_share,
        'world',
        'scout_mini.world'
    ])

    map_file = PathJoinSubstitution([
        pkg_share,
        'map',
        'task1_map.yaml'
    ])

    nav2_params = PathJoinSubstitution([
        pkg_share,
        'config',
        'nav2_params.yaml'
    ])

    rviz_config = PathJoinSubstitution([
        pkg_share,
        'rviz',
        'nav2_config.rviz'
    ])

    # Start Stage simulator.
    # Stage publishes /clock, robot odometry, laser scan, and TF related to odom/base_link.
    stage_node = Node(
        package='stage_ros2',
        executable='stage_ros2',
        name='stage',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'world_file': world_file}
        ]
    )

    # Start Nav2 bringup.
    # map -> odom should be provided by AMCL, not by a static transform.
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                nav2_share,
                'launch',
                'bringup_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'map': map_file,
            'params_file': nav2_params,
            'autostart': 'True'
        }.items()
    )

    # Start RViz.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    # Start the CSV-driven goal controller after Nav2 has time to activate.
    goal_publisher_node = Node(
        package='second_project',
        executable='goal_publisher_node',
        name='goal_publisher_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        stage_node,

        # Wait a few seconds so Stage and /clock start first.
        TimerAction(
            period=3.0,
            actions=[nav2_bringup]
        ),

        # Start RViz after Nav2 begins launching.
        TimerAction(
            period=6.0,
            actions=[rviz_node]
        ),

        # Send goals from csv/goals.csv once Nav2 should be ready.
        TimerAction(
            period=10.0,
            actions=[goal_publisher_node]
        )
    ])
