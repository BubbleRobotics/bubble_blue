import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("path_planner"),
        "config",
        "seabed_scan.yaml",
    )

    params_file = LaunchConfiguration("params_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_config,
                description="YAML config for the seabed scan planner",
            ),
            Node(
                package="path_planner",
                executable="seabed_scan_planner",
                name="seabed_scan_planner",
                output="screen",
                parameters=[
                    params_file,
                ],
            ),
        ]
    )
