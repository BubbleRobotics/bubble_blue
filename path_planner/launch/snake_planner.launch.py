from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---------- Launch args ----------
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    set_mode_on_start = LaunchConfiguration('set_mode_on_start')
    arm_on_start = LaunchConfiguration('arm_on_start')

    odom_frame = LaunchConfiguration('odom_frame')
    body_frame = LaunchConfiguration('body_frame')
    assume_odom_is_enu = LaunchConfiguration('assume_odom_is_enu')

    # PID gains
    pid_x_kp = LaunchConfiguration('pid_x_kp')
    pid_x_ki = LaunchConfiguration('pid_x_ki')
    pid_x_kd = LaunchConfiguration('pid_x_kd')

    pid_y_kp = LaunchConfiguration('pid_y_kp')
    pid_y_ki = LaunchConfiguration('pid_y_ki')
    pid_y_kd = LaunchConfiguration('pid_y_kd')

    pid_z_kp = LaunchConfiguration('pid_z_kp')
    pid_z_ki = LaunchConfiguration('pid_z_ki')
    pid_z_kd = LaunchConfiguration('pid_z_kd')

    pid_yaw_kp = LaunchConfiguration('pid_yaw_kp')
    pid_yaw_ki = LaunchConfiguration('pid_yaw_ki')
    pid_yaw_kd = LaunchConfiguration('pid_yaw_kd')

    # Limits
    i_limit_pos = LaunchConfiguration('i_limit_pos')
    i_limit_yaw = LaunchConfiguration('i_limit_yaw')
    v_max_xy = LaunchConfiguration('v_max_xy')
    v_max_z = LaunchConfiguration('v_max_z')
    yaw_rate_max = LaunchConfiguration('yaw_rate_max')

    # Optional topic remaps (in case you use namespaced systems)
    ref_topic = LaunchConfiguration('ref_topic')
    odom_topic = LaunchConfiguration('odom_topic')

    return LaunchDescription([
        DeclareLaunchArgument('publish_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('set_mode_on_start', default_value='GUIDED'),
        DeclareLaunchArgument('arm_on_start', default_value='false'),

        DeclareLaunchArgument('odom_frame', default_value='map'),
        DeclareLaunchArgument('body_frame', default_value='body_link'),
        DeclareLaunchArgument('assume_odom_is_enu', default_value='true'),

        DeclareLaunchArgument('pid_x_kp', default_value='1.0'),
        DeclareLaunchArgument('pid_x_ki', default_value='0.0'),
        DeclareLaunchArgument('pid_x_kd', default_value='0.2'),

        DeclareLaunchArgument('pid_y_kp', default_value='1.0'),
        DeclareLaunchArgument('pid_y_ki', default_value='0.0'),
        DeclareLaunchArgument('pid_y_kd', default_value='0.2'),

        DeclareLaunchArgument('pid_z_kp', default_value='1.0'),
        DeclareLaunchArgument('pid_z_ki', default_value='0.0'),
        DeclareLaunchArgument('pid_z_kd', default_value='0.2'),

        DeclareLaunchArgument('pid_yaw_kp', default_value='2.0'),
        DeclareLaunchArgument('pid_yaw_ki', default_value='0.0'),
        DeclareLaunchArgument('pid_yaw_kd', default_value='0.2'),

        DeclareLaunchArgument('i_limit_pos', default_value='1.0'),
        DeclareLaunchArgument('i_limit_yaw', default_value='1.0'),
        DeclareLaunchArgument('v_max_xy', default_value='0.6'),
        DeclareLaunchArgument('v_max_z', default_value='0.4'),
        DeclareLaunchArgument('yaw_rate_max', default_value='0.8'),

        DeclareLaunchArgument('ref_topic', default_value='/ego_planner/pos_cmd'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered'),

        Node(
            package='path_planner',
            executable='snake_planner',
            name='snake_planner',
            output='screen',
            parameters=[{
                'publish_rate_hz': publish_rate_hz,
                'set_mode_on_start': set_mode_on_start,
                'arm_on_start': arm_on_start,

                'odom_frame': odom_frame,
                'body_frame': body_frame,
                'assume_odom_is_enu': assume_odom_is_enu,

                'pid_x.kp': pid_x_kp,
                'pid_x.ki': pid_x_ki,
                'pid_x.kd': pid_x_kd,

                'pid_y.kp': pid_y_kp,
                'pid_y.ki': pid_y_ki,
                'pid_y.kd': pid_y_kd,

                'pid_z.kp': pid_z_kp,
                'pid_z.ki': pid_z_ki,
                'pid_z.kd': pid_z_kd,

                'pid_yaw.kp': pid_yaw_kp,
                'pid_yaw.ki': pid_yaw_ki,
                'pid_yaw.kd': pid_yaw_kd,

                'i_limit_pos': i_limit_pos,
                'i_limit_yaw': i_limit_yaw,
                'v_max_xy': v_max_xy,
                'v_max_z': v_max_z,
                'yaw_rate_max': yaw_rate_max,
                'use_sim_time': True,
            }],
            remappings=[
                # follower input reference
                ('/ego_planner/pos_cmd', ref_topic),

                # EKF feedback
                ('/odometry/filtered', odom_topic),

                # leave mavros topics as-is by default
                # ('/mavros/setpoint_raw/local', '/mavros/setpoint_raw/local'),
            ],
        ),
    ])