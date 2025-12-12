from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Launch configurations
    use_sim_time    = LaunchConfiguration('use_sim_time', default='false')
    namespace       = LaunchConfiguration('namespace', default='')
    timeout_sec     = LaunchConfiguration('timeout_sec', default='90.0')
    use_follower    = LaunchConfiguration('use_follower', default='true')
    use_rrt_planner = LaunchConfiguration('use_rrt_planner', default='false')

    # Nodes
    wait = Node(
        package='path_planner',
        executable='wait_mavros_ready',
        name='wait_mavros_ready',
        namespace=namespace,
        parameters=[{
            'use_sim_time': use_sim_time,
            'timeout_sec': timeout_sec,
        }],
        output='screen',
    )

    follower = Node(
        package='path_planner',
        executable='follower_node',
        name='follower_node',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_rate_hz': 20.0,
            'set_mode': 'GUIDED',
            'arm_on_start': False,
        }],
        # Only start if use_follower is true
        condition=IfCondition(use_follower),
        # respawn=True,  # optional
    )

    planner = Node(
        package='path_planner',
        executable='path_planner',
        name='path_planner',
        namespace=namespace,
        output='screen',
        # Only start if use_rrt_planner is true
        condition=IfCondition(use_rrt_planner),
        # respawn=True,  # optional
    )

    # Event: when the wait node exits, start follower/planner (if their conditions are true)
    start_after_wait = RegisterEventHandler(
        OnProcessExit(
            target_action=wait,
            on_exit=[follower, planner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',    default_value='true'),
        DeclareLaunchArgument('namespace',       default_value=''),
        DeclareLaunchArgument('timeout_sec',     default_value='90.0'),
        DeclareLaunchArgument('use_follower',    default_value='true',  choices=['true', 'false']),
        DeclareLaunchArgument('use_rrt_planner', default_value='false', choices=['true', 'false']),
        wait,
        start_after_wait,
    ])
