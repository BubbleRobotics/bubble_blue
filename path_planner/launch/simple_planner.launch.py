from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_planner',
            executable='simple_planner',
            name='simple_planner',
            output='screen',
        ),
    ])
