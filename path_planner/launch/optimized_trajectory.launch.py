from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    test_ego = LaunchConfiguration('test_ego')
    use_known_currents = LaunchConfiguration('use_known_currents')
    use_disturbance_currents = LaunchConfiguration('use_disturbance_currents')

    return LaunchDescription([
        DeclareLaunchArgument(
            'test_ego',
            default_value='false',
            description='Run ego test mode'
        ),
        DeclareLaunchArgument(
            'use_known_currents',
            default_value='false',
            description='Whether to use known currents in the optimized trajectory node'
        ),
        DeclareLaunchArgument(
            'use_disturbance_currents',
            default_value='false',
            description='Whether to use disturbance currents in the optimized trajectory node'
        ),

        Node(
            package='path_planner',
            executable='optimal_trajectory',
            name='optimal_trajectory',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'test_ego': test_ego,
                'use_known_currents': use_known_currents,
                'use_disturbance_currents': use_disturbance_currents,
            }],
        ),
    ])