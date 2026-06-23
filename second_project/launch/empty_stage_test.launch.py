from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_file = PathJoinSubstitution([
        FindPackageShare('second_project'),
        'world',
        'empty_test.world'
    ])

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

    return LaunchDescription([
        stage_node
    ])
