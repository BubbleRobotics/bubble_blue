from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    test_ego = LaunchConfiguration('test_ego')

    return LaunchDescription([
        DeclareLaunchArgument(
            'test_ego',
            default_value='false',
            description='Run ego test mode'
        ),

        Node(
            package='path_planner',
            executable='run_test_MT',
            name='run_test_MT',
            output='screen',
            parameters=[{
                'test_ego': test_ego,
            }],
        ),
    ])