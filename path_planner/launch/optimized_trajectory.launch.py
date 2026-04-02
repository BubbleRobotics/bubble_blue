from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    test_ego = LaunchConfiguration('test_ego')
    use_current_disturbances = LaunchConfiguration('use_current_disturbances')

    return LaunchDescription([
        DeclareLaunchArgument(
            'test_ego',
            default_value='false',
            description='Run ego test mode'
        ),
        DeclareLaunchArgument(
            'use_current_disturbances',
            default_value='false',
            description='Whether to use current disturbances in the optimized trajectory node'
        ),

        Node(
            package='path_planner',
            executable='optimal_trajectory',
            name='optimal_trajectory',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'test_ego': test_ego,
                'use_current_disturbances': use_current_disturbances
            }],
        ),
    ])