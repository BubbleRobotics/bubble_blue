from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="path_planner",
            executable="simple_accel_controller",
            name="simple_accel_controller",
            output="screen",
            parameters=[{
                "rate_hz": 50.0,

                # --- Gains ---
                # Horizontal position -> thrust (body forward/right)
                "kp_xy": 0.7,
                "ki_xy": 0.2,
                "kd_xy": 0.5,

                # Vertical (Down axis in NED) position -> thrust
                "kp_z": 0.7,
                "ki_z": 0.2,
                "kd_z": 0.5,

                "i_limit_xy": 2.0,
                "i_limit_z": 2.0,

                # --- Reference frame ---
                # Set True only if /test/controller/reference is ENU (x=E,y=N,z=Up)
                "reference_is_enu": False,

                # --- RC channel mapping (COMMON ArduSub defaults; verify!) ---
                # OverrideRCIn.channels is 0-based:
                # CH1->0, CH2->1, CH3->2, CH4->3, CH5->4, CH6->5, ...
                "ch_throttle_vertical": 2,  # CH3: heave (up/down)
                "ch_yaw": 3,                # CH4: yaw
                "ch_forward": 4,            # CH5: surge (forward)
                "ch_lateral": 5,            # CH6: sway (lateral)

                # --- PWM ---
                "pwm_center": 1500,
                "pwm_min": 1100,
                "pwm_max": 1900,

                # --- Controller output -> PWM scaling ---
                # Increase if it feels weak, decrease if too aggressive.
                "scale_forward_pwm": 120.0,
                "scale_lateral_pwm": 120.0,
                "scale_vertical_pwm": 120.0,

                # --- Optional yaw hold ---
                "enable_yaw_hold": True,
                "kp_yaw": 1.0,
                "scale_yaw_pwm": 120.0,
                "use_sim_time": True, # TODO change this for real tests
            }]
        )
    ])
