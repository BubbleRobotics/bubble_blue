from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Adjustable delays
    sim_settle_time = LaunchConfiguration("sim_settle_time")
    controller_delay = LaunchConfiguration("controller_delay")
    ego_delay = LaunchConfiguration("ego_delay")

    # Package share paths
    bb_bringup_share = FindPackageShare("bb_bringup")
    path_planner_share = FindPackageShare("path_planner")
    blue_demos_share = FindPackageShare("blue_demos")
    ego_planner_share = FindPackageShare("ego_planner")
    sim_disturb_share = FindPackageShare("simulation_disturbances")


    # STEP 2 - Call initialize service after sim settles
    initialize = ExecuteProcess(
        cmd=[
            "ros2", "service", "call",
            "/initialize",
            "std_srvs/srv/Trigger",
            "{}"
        ],
        output="screen",
        shell=False,
    )


    # STEP 3 - NED -> ENU conversion
    odom_ned_enu = TimerAction(
        period=sim_settle_time,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [path_planner_share, "/launch/odometry_ned_enu.launch.py"]
                )
            )
        ],
    )

    # STEP 4 - AUV controller (sim)
    controllers = TimerAction(
        period=controller_delay,
        condition=IfCondition(use_sim),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('blue_demos'),
                    '/control_integration/launch/bluerov2_heavy_controllers.launch.py'
                ])
            )
        ]
    )

    # STEP 5 - ego_viz
    ego_viz = TimerAction(
        period=ego_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [ego_planner_share, "/launch/rviz.launch.py"]
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time
                }.items(),
            )
        ],
    )

    # STEP 6 - ego_planner
    ego_planner = TimerAction(
        period=ego_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [ego_planner_share, "/launch/single_run_in_sim.launch.py"]
                )
            )
        ],
    )

    simulate_current = TimerAction(
        period=ego_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [sim_disturb_share, "/launch/simulate_current.launch.py"]
                )
            )
        ],
    )

    optimized_trajectory = TimerAction(
        period=ego_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [path_planner_share, "/launch/optimized_trajectory.launch.py"]
                )
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("sim_settle_time", default_value="5.0"),
        DeclareLaunchArgument("controller_delay", default_value="10.0"),
        DeclareLaunchArgument("ego_delay", default_value="10.0"),

        initialize,
        odom_ned_enu,
        controllers,
        simulate_current,
    ])