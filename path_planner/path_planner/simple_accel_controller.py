#!/usr/bin/env python3
from typing import Optional
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, PointStamped
from mavros_msgs.msg import OverrideRCIn
from quadrotor_msgs.msg import PositionCommand
# Import mavutil
from pymavlink import mavutil

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """
    Returns yaw (rad) from quaternion.
    Assumes standard quaternion -> yaw extraction.
    Works for typical NED/ENU as long as quaternion represents vehicle orientation in the odom frame.
    """
    # yaw (Z axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class SimpleAUVController(Node):
    """
    ArduSub AUV controller:
      - odom: NED (x=N, y=E, z=Down)
      - ref: Pose (assumed NED by default)
      - publishes OverrideRCIn at 40Hz

    Key difference vs multirotor:
      - We command *forward* and *lateral* thrust directly (surge/sway channels),
        not roll/pitch.
      - Convert world-frame (N,E) position error into BODY frame (forward/right)
        using current yaw from odometry orientation.

    BODY axes used:
      forward = +X_body
      right   = +Y_body
      down    = +Z_body  (heave positive down, matches NED sign convention)
    """

    def __init__(self):
        super().__init__("simple_auv_controller")

        # --- Gains: position -> "thrust command" (PID-ish) ---
        # Horizontal (forward/right)
        self.declare_parameter("kp_xy", 1.0)
        self.declare_parameter("ki_xy", 0.0)
        self.declare_parameter("kd_xy", 0.4)

        # Vertical (down)
        self.declare_parameter("kp_z", 1.0)
        self.declare_parameter("ki_z", 0.0)
        self.declare_parameter("kd_z", 0.4)

        # Integrator clamps
        self.declare_parameter("i_limit_xy", 2.0)
        self.declare_parameter("i_limit_z", 2.0)

        # Reference frame toggle (if your reference Pose is ENU)
        self.declare_parameter("reference_is_enu", False)

        # --- RC channels (COMMON ArduSub defaults; verify on your setup) ---
        # 1500 neutral, >1500 positive, <1500 negative (for most Sub configs)
        self.declare_parameter("ch_throttle_vertical", 2)  # CH3 -> index 2
        self.declare_parameter("ch_yaw", 3)                # CH4 -> index 3
        self.declare_parameter("ch_forward", 4)            # CH5 -> index 4
        self.declare_parameter("ch_lateral", 5)            # CH6 -> index 5

        # PWM params
        self.declare_parameter("pwm_center", 1500)
        self.declare_parameter("pwm_min", 1100)
        self.declare_parameter("pwm_max", 1900)

        # Scaling from "thrust command" to PWM delta
        self.declare_parameter("scale_forward_pwm", 120.0)
        self.declare_parameter("scale_lateral_pwm", 120.0)
        self.declare_parameter("scale_vertical_pwm", 120.0)

        # Optional: simple yaw hold to reference yaw (taken from reference orientation)
        self.declare_parameter("enable_yaw_hold", False)
        self.declare_parameter("kp_yaw", 1.0)
        self.declare_parameter("scale_yaw_pwm", 120.0)

        # Publish rate
        self.declare_parameter("rate_hz", 50.0)

        # State
        self.odom: Optional[Odometry] = None
        self.ref_pose: Optional[PositionCommand] = None

        self.int_f = 0.0   # forward integrator (body)
        self.int_r = 0.0   # right integrator (body)
        self.int_d = 0.0   # down integrator (world down)

        self.last_time: Optional[Time] = None
        self._clicked_point_sub = self.create_subscription(PointStamped, '/ego_planner/clicked_point', self._clicked_point_cb, 10)
        self._goal_point_pub = self.create_publisher(PoseStamped, 'ego_planner/move_base_simple/goal', 10)
        # ROS I/O
        self.sub_odom = self.create_subscription(Odometry, "odometry/filtered", self.on_odom, 10)
        self.sub_ref = self.create_subscription(PositionCommand, "ego_planner/pos_cmd", self.on_ref, 10)
        self.pub_rc = self.create_publisher(OverrideRCIn, "mavros/rc/override", 10)

        rate_hz = float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(1.0 / rate_hz, self.on_timer)
        self.get_logger().info(f"SimpleAUVController started @ {rate_hz:.1f} Hz")
        # Create the connection
        
        #self.master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
        self.master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
        # Wait a heartbeat before sending commands
        self.master.wait_heartbeat()

    def _clicked_point_cb(self, msg:PointStamped):
        goal_point = PoseStamped()
        
        goal_point.pose.position.x = msg.point.x
        goal_point.pose.position.y = msg.point.y
        goal_point.pose.position.z = -1.0 # TODO change to 3D capability, for now selecting in 3D is not possible (only able to select on reference points)
        self.get_logger().info(f"Published new goal (ENU) to EGO planner. x: {msg.point.x}, y: {msg.point.y}, z: {goal_point.pose.position.z}")
        self._goal_point_pub.publish(goal_point)

    def on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def on_ref(self, msg: PositionCommand) -> None:
        self.ref_pose = PositionCommand()
        self.ref_pose.position.x = msg.position.y
        self.ref_pose.position.y = msg.position.x
        self.ref_pose.position.z = -msg.position.z
        self.ref_pose.velocity.x = msg.velocity.x
        self.ref_pose.velocity.y = msg.velocity.y
        self.ref_pose.velocity.z = msg.velocity.z
        self.ref_pose.acceleration.x = msg.acceleration.x
        self.ref_pose.acceleration.y = msg.acceleration.y
        self.ref_pose.acceleration.z = msg.acceleration.z


    def on_timer(self) -> None:

        if self.odom is None or self.ref_pose is None:
            return

        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 0.5:
            return

        # Params
        kp_xy = float(self.get_parameter("kp_xy").value)
        ki_xy = float(self.get_parameter("ki_xy").value)
        kd_xy = float(self.get_parameter("kd_xy").value)

        kp_z = float(self.get_parameter("kp_z").value)
        ki_z = float(self.get_parameter("ki_z").value)
        kd_z = float(self.get_parameter("kd_z").value)

        i_lim_xy = float(self.get_parameter("i_limit_xy").value)
        i_lim_z = float(self.get_parameter("i_limit_z").value)

        ch_vert = int(self.get_parameter("ch_throttle_vertical").value)
        ch_yaw = int(self.get_parameter("ch_yaw").value)
        ch_fwd = int(self.get_parameter("ch_forward").value)
        ch_lat = int(self.get_parameter("ch_lateral").value)

        pwm_center = int(self.get_parameter("pwm_center").value)
        pwm_min = int(self.get_parameter("pwm_min").value)
        pwm_max = int(self.get_parameter("pwm_max").value)

        scale_fwd = float(self.get_parameter("scale_forward_pwm").value)
        scale_lat = float(self.get_parameter("scale_lateral_pwm").value)
        scale_vert = float(self.get_parameter("scale_vertical_pwm").value)

        enable_yaw_hold = bool(self.get_parameter("enable_yaw_hold").value)
        kp_yaw = float(self.get_parameter("kp_yaw").value)
        scale_yaw = float(self.get_parameter("scale_yaw_pwm").value)

        # Feedback (NED)
        p = self.odom.pose.pose.position
        v = self.odom.twist.twist.linear
        p_n_meas, p_e_meas, p_d_meas = p.x, p.y, p.z
        v_n_meas, v_e_meas, v_d_meas = v.x, v.y, v.z

        # Current yaw (rad)
        q = self.odom.pose.pose.orientation
        p_yaw_meas = yaw_from_quat(q.x, q.y, q.z, q.w)
        v_yaw_meas = self.odom.twist.twist.angular.z

        # Reference (NED)
        p_n_ref = self.ref_pose.position.x
        p_e_ref = self.ref_pose.position.y
        p_d_ref = self.ref_pose.position.z

        v_n_ref = self.ref_pose.velocity.x
        v_e_ref = self.ref_pose.velocity.y
        v_d_ref = self.ref_pose.velocity.z

        a_n_ref = self.ref_pose.acceleration.x
        a_e_ref = self.ref_pose.acceleration.y
        a_d_ref = self.ref_pose.acceleration.z


        # World-frame position error (NED)
        error_p_n = p_n_ref - p_n_meas
        error_p_e = p_e_ref - p_e_meas
        error_p_d = p_d_ref - p_d_meas  # + means "go more DOWN"

        # Convert (en, ee) into BODY frame (forward/right)
        # For NED with yaw about Down axis:
        # forward_err =  cos(yaw)*en + sin(yaw)*ee
        # right_err   = -sin(yaw)*en + cos(yaw)*ee
        cy = math.cos(p_yaw_meas)
        sy = math.sin(p_yaw_meas)
        error_p_f = cy * error_p_n + sy * error_p_e
        error_p_r = -sy * error_p_n + cy * error_p_e

        # Convert velocity (vn, ve) into BODY frame for damping
        v_f_meas = cy * v_n_meas + sy * v_e_meas
        v_r_meas = -sy * v_n_meas + cy * v_e_meas

        # Integrators (clamped)
        self.int_f = clamp(self.int_f + error_p_f * dt, -i_lim_xy, i_lim_xy)
        self.int_r = clamp(self.int_r + error_p_r * dt, -i_lim_xy, i_lim_xy)
        self.int_d = clamp(self.int_d + error_p_d * dt, -i_lim_z, i_lim_z)

        # "Thrust-like" commands (dimensionless-ish)
        # (P on pos, D on velocity, optional I)
        u_fwd = kp_xy * error_p_f + ki_xy * self.int_f + kd_xy * (0.0 - v_f_meas)
        u_lat = kp_xy * error_p_r + ki_xy * self.int_r + kd_xy * (0.0 - v_r_meas)
        u_ver = kp_z  * error_p_d + ki_z  * self.int_d + kd_z  * (0.0 - v_d_meas)  # down axis

        # Map to PWM (1500 neutral)
        fwd_pwm = pwm_center + int(scale_fwd * u_fwd)
        lat_pwm = pwm_center + int(scale_lat * u_lat)
        vert_pwm = pwm_center + int(-scale_vert * u_ver) # Since down positive, 
        # z too low (error_d positive) should result in downward thrust (lower PWM)

        # Yaw: optional simple hold using reference orientation yaw
        yaw_pwm = pwm_center
        if enable_yaw_hold:
            p_yaw_ref = self.ref_pose.yaw
            v_yaw_ref = self.ref_pose.yaw_dot
            yaw_err = math.atan2(math.sin(p_yaw_ref - p_yaw_meas), math.cos(p_yaw_ref - p_yaw_meas))
            yaw_dot_err = v_yaw_ref - v_yaw_meas
            u_yaw = kp_yaw * yaw_err #+ ki_yaw  * self.int_yaw + kd_yaw  * (0.0 - v_yaw_meas)  # down axis
            yaw_pwm = pwm_center + int(scale_yaw * u_yaw)

        # Clamp
        fwd_pwm = int(clamp(fwd_pwm, pwm_min, pwm_max))
        lat_pwm = int(clamp(lat_pwm, pwm_min, pwm_max))
        vert_pwm = int(clamp(vert_pwm, pwm_min, pwm_max))
        yaw_pwm = int(clamp(yaw_pwm, pwm_min, pwm_max))

        """
        out = OverrideRCIn()
        out.channels = [0] * 18
        out.channels[ch_fwd] = fwd_pwm
        out.channels[ch_lat] = lat_pwm
        out.channels[ch_vert] = vert_pwm
        out.channels[ch_yaw] = yaw_pwm
        self.pub_rc.publish(out)
        """
        rc_channel_values = [65535 for _ in range(18)]
        # Initialize all channels to 65535 (ignore)
        # Then set the channels you want to override
        # 1500 is neutral for these channels
        # Min and Max values are 1100 and 1900
        """rc_channel_values[0] = 1500 # Roll
        rc_channel_values[1] = 1500 # Pitch"""
        rc_channel_values[2] = vert_pwm # Throttle (Up, Down)
        rc_channel_values[3] = yaw_pwm # Yaw
        rc_channel_values[4] = fwd_pwm # Forward, Backward
        rc_channel_values[5] = lat_pwm # Left, Right
        
        self.master.mav.rc_channels_override_send(
            self.master.target_system,                # target_system
            self.master.target_component,             # target_component
            *rc_channel_values)  
        return

def main(args=None):
    rclpy.init(args=args)
    node = SimpleAUVController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
