from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    test_ego = LaunchConfiguration('test_ego')
    test_opt = LaunchConfiguration('test_opt')
    test_case_id = LaunchConfiguration('test_case_id')

    return LaunchDescription([
        DeclareLaunchArgument(
            'test_ego',
            default_value='false',
            description='Run ego test mode'
        ),
        DeclareLaunchArgument(
            'test_opt',
            default_value='false',
            description='Run optimized test mode'
        ),
        DeclareLaunchArgument(
            'test_case_id',
            default_value="1",
            description='Test ID to run'
        ),

        Node(
            package='path_planner',
            executable='run_test_MT',
            name='run_test_MT',
            output='screen',
            parameters=[{
                'test_ego': test_ego,
                'test_opt': test_opt,
                'test_case_id': test_case_id,
            }],
        ),
    ])