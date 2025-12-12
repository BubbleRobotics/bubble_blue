from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_planner',
            executable='follower_live_test',
            name='follower_node_live_test',
            output='screen',
            parameters=[{
                'publish_rate_hz': 20.0,
                'set_mode': 'GUIDED',
                'arm_on_start': True,
            }],
        ),
    ])
