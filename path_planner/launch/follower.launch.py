from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_planner',
            executable='follower_node',
            name='follower_node',
            output='screen',
            parameters=[{
                'publish_rate_hz': 20.0,
                'set_mode_on_start': 'MANUAL',
                'arm_on_start': False,
            }],
        ),
    ])
