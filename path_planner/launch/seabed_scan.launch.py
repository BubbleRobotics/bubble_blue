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
    scan_width_m = LaunchConfiguration("scan_width_m")
    scan_height_m = LaunchConfiguration("scan_height_m")
    lane_spacing_m = LaunchConfiguration("lane_spacing_m")
    fixed_depth_down_m = LaunchConfiguration("fixed_depth_down_m")
    use_current_depth = LaunchConfiguration("use_current_depth")
    hold_current_yaw = LaunchConfiguration("hold_current_yaw")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_config,
                description="YAML config for the seabed scan planner",
            ),
            DeclareLaunchArgument("scan_width_m", default_value="10.0"),
            DeclareLaunchArgument("scan_height_m", default_value="10.0"),
            DeclareLaunchArgument("lane_spacing_m", default_value="0.5"),
            DeclareLaunchArgument("fixed_depth_down_m", default_value="2.0"),
            DeclareLaunchArgument("use_current_depth", default_value="true"),
            DeclareLaunchArgument("hold_current_yaw", default_value="true"),
            Node(
                package="path_planner",
                executable="seabed_scan_planner",
                name="seabed_scan_planner",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "scan_width_m": scan_width_m,
                        "scan_height_m": scan_height_m,
                        "lane_spacing_m": lane_spacing_m,
                        "fixed_depth_down_m": fixed_depth_down_m,
                        "use_current_depth": use_current_depth,
                        "hold_current_yaw": hold_current_yaw,
                    },
                ],
            ),
        ]
    )
