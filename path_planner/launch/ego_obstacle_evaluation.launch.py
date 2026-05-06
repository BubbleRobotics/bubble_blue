from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_planner',
            executable='ego_obstacle_evaluation',
            name='ego_obstacle_evaluation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'world_name': 'underwater_world',
                'model_name': 'bluerov2_heavy',
                'goal_tolerance_xyz': 0.1,
                'settle_time_sec': 2.0,
                'record_bag': True,
                'bag_output_dir': '/home/ubuntu/ws_blue/evaluation/EGO_data',
            }],
        ),
    ])