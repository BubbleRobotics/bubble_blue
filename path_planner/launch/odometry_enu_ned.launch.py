from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    in_odom = LaunchConfiguration("in_odom")
    out_odom = LaunchConfiguration("out_odom")

    return LaunchDescription([
        DeclareLaunchArgument(
            "in_odom",
            default_value="/odometry/filtered",
            description="Input Odometry topic (NED)",
        ),
        DeclareLaunchArgument(
            "out_odom",
            default_value="/odometry/filtered_enu",
            description="Output Odometry topic (ENU)",
        ),
        Node(
            package="path_planner",
            executable="odometry_ned_enu",
            name="odometry_ned_enu",
            output="screen",
            parameters=[{
                "in_odom": in_odom,
                "out_odom": out_odom,
            }],
        ),
    ])