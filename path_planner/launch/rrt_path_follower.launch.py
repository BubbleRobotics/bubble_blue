from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
                Node(
            package='path_planner',
            executable='rrt_path_follower',
            name='rrt_path_follower',
            output='screen',
            parameters=[{
                'mavlink_url': 'udp:127.0.0.1:14550',
                'set_mode': 'GUIDED',
                'arm': True,
                'goal': [11.0, 13.45, -5.2],
                'bounds': [-5.0, -5.0, -10.0, 20.0, 20.0, 1.0],
                'step_size': 0.30,
                'edge_res': 0.05,
                'goal_bias': 0.10,
                'max_iters': 30000,
                'reach_thresh': 0.10,
                'final_yaw_deg': -15.6923,
                'near_goal_radius': 2.0,
                'safety_buffer': 0.15,
            }],
        ),
    ])
