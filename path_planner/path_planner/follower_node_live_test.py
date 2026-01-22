#!/usr/bin/env python3
import math
import time
import threading
from dataclasses import dataclass
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseWithCovarianceStamped
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import State as MavState
from mavros_msgs.srv import CommandBool, SetMode
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry

from tf_transformations import euler_from_quaternion, quaternion_matrix

import tf2_ros


@dataclass
class PIDGains:
    kp: float
    ki: float
    kd: float


class PIDAxis:
    """Simple PID with integral clamp and derivative on error."""
    def __init__(self, gains: PIDGains, i_limit: float):
        self.g = gains
        self.i_limit = abs(i_limit)
        self.i = 0.0
        self.prev_e = None

    def reset(self):
        self.i = 0.0
        self.prev_e = None

    def step(self, e: float, dt: float) -> float:
        if dt <= 1e-6:
            return 0.0

        # integral
        self.i += e * dt
        self.i = max(-self.i_limit, min(self.i_limit, self.i))

        # derivative (on error)
        if self.prev_e is None:
            de = 0.0
        else:
            de = (e - self.prev_e) / dt
        self.prev_e = e

        return self.g.kp * e + self.g.ki * self.i + self.g.kd * de


class BodyPIDFollower(Node):
    """
    - Reference: /drone_0_planner/pos_cmd (PositionCommand), interpreted in 'map'
    - Feedback:  /mavros/vision_pose/pose_cov (PoseWithCovarianceStamped), assumed in 'map'
    - Output:    /mavros/setpoint_raw/local (PositionTarget) using FRAME_BODY_NED and velocity setpoints
    """

    def __init__(self):
        super().__init__('body_pid_follower')

        # ---------------- Params ----------------
        self.declare_parameters(
            namespace='',
            parameters=[
                ('publish_rate_hz', 20.0),

                # frames
                ('odom_frame', 'map'),
                ('body_frame', 'base_link_fsd'),

                # PID gains (position -> body-velocity)
                ('pid_x.kp', 1.0), ('pid_x.ki', 0.0), ('pid_x.kd', 0.2),
                ('pid_y.kp', 1.0), ('pid_y.ki', 0.0), ('pid_y.kd', 0.2),
                ('pid_z.kp', 1.0), ('pid_z.ki', 0.0), ('pid_z.kd', 0.2),

                # yaw controller (yaw -> yaw_rate)
                ('pid_yaw.kp', 2.0), ('pid_yaw.ki', 0.0), ('pid_yaw.kd', 0.2),

                # clamps
                ('i_limit_pos', 1.0),
                ('i_limit_yaw', 1.0),
                ('v_max_xy', 0.6),   # m/s
                ('v_max_z', 0.4),    # m/s (body z-down)
                ('yaw_rate_max', 0.8),  # rad/s

                # mavros startup
                ('set_mode_on_start', False),
                ('mode_to_set', 'GUIDED'),
                ('arm_on_start', False),

                # If your odom is ENU and you want MAVLink NED, set True and apply conversion below.
                ('assume_odom_is_enu', True),
            ],
        )

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.body_frame = str(self.get_parameter('body_frame').value)
        self.assume_odom_is_enu = bool(self.get_parameter('assume_odom_is_enu').value)

        # PID
        self.pid_x = PIDAxis(PIDGains(
            float(self.get_parameter('pid_x.kp').value),
            float(self.get_parameter('pid_x.ki').value),
            float(self.get_parameter('pid_x.kd').value),
        ), i_limit=float(self.get_parameter('i_limit_pos').value))

        self.pid_y = PIDAxis(PIDGains(
            float(self.get_parameter('pid_y.kp').value),
            float(self.get_parameter('pid_y.ki').value),
            float(self.get_parameter('pid_y.kd').value),
        ), i_limit=float(self.get_parameter('i_limit_pos').value))

        self.pid_z = PIDAxis(PIDGains(
            float(self.get_parameter('pid_z.kp').value),
            float(self.get_parameter('pid_z.ki').value),
            float(self.get_parameter('pid_z.kd').value),
        ), i_limit=float(self.get_parameter('i_limit_pos').value))

        self.pid_yaw = PIDAxis(PIDGains(
            float(self.get_parameter('pid_yaw.kp').value),
            float(self.get_parameter('pid_yaw.ki').value),
            float(self.get_parameter('pid_yaw.kd').value),
        ), i_limit=float(self.get_parameter('i_limit_yaw').value))

        self.v_max_xy = float(self.get_parameter('v_max_xy').value)
        self.v_max_z = float(self.get_parameter('v_max_z').value)
        self.yaw_rate_max = float(self.get_parameter('yaw_rate_max').value)

        # ---------------- State ----------------
        self._is_connected = False
        self._armed = False
        self._mode_set = False

        self._have_pose = False
        self._have_ref = False

        self.current_odom_msg: Optional[Odometry] = None        
        self.ref_cmd: Optional[PositionCommand] = None

        self._last_time = self.get_clock().now()

        # ---------------- TF2 ----------------
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=2.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------- ROS I/O ----------------
        qos = QoSProfile(depth=10)

        self._state_sub = self.create_subscription(MavState, '/mavros/state', self._state_cb, qos)
        self._odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self._odom_cb,
            qos
        )


        self._cmd_sub = self.create_subscription(PositionCommand, '/drone_0_planner/pos_cmd', self._ref_cb, qos)

        self._setpoint_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', qos)

        self._set_mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self._arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')

        # Controller timer
        self._ctrl_timer = self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self._control_step)

        # ---------------- Wait for MAVROS heartbeat ----------------
        self.get_logger().info('Waiting for MAVROS heartbeat...')
        start_time = time.time()
        while not self._is_connected and (time.time() - start_time) < 50.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self._is_connected:
            raise RuntimeError('No heartbeat from MAVROS within timeout')
        self.get_logger().info('Connected to MAVROS')
        # Optional: set mode + arm
        set_mode_on_start = bool(self.get_parameter('set_mode_on_start').value)
        mode_to_set = str(self.get_parameter('mode_to_set').value)
        arm_on_start = bool(self.get_parameter('arm_on_start').value)

        if set_mode_on_start:
            while not self._mode_set:
                self._mode_set = self._set_mode(mode_to_set)
        if arm_on_start:
            while not self._armed:
                self._armed = self._arm(True)

        self.get_logger().info('BodyPIDFollower ready.')

    # ---------------- Callbacks ----------------
    def _state_cb(self, msg: MavState):
        if msg.connected and not self._is_connected:
            self._is_connected = True
            

    def _odom_cb(self, msg: Odometry):
        self.current_odom_msg = msg
        self._have_pose = True


    def _ref_cb(self, msg: PositionCommand):
        self.ref_cmd = msg
        self._have_ref = True

    # ---------------- Control ----------------
    def _control_step(self):
        if not self._have_pose or not self._have_ref:
            return

        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now
        if dt <= 1e-6:
            return

        odom = self.current_odom_msg
        pose = odom.pose.pose

        ref = self.ref_cmd

        # --- Current position in map frame---
        p_cur = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)

        # --- Reference position in map frame ---
        p_ref = np.array([float(ref.position.x), float(ref.position.y), float(ref.position.z)], dtype=float)

        # error in map frame
        e_odom = p_ref - p_cur

        # --- Rotation map frame -> body frame using tf2 (preferred) ---
        R_odom_to_body = None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.body_frame,  # target
                self.odom_frame,  # source
                self.get_clock().now()
            )
            q = tf.transform.rotation
            T = quaternion_matrix([q.x, q.y, q.z, q.w])
            R_odom_to_body = T[:3, :3]
        except Exception:
            # fallback: use pose orientation (works if pose is body orientation in map)
            q = pose.orientation
            T = quaternion_matrix([q.x, q.y, q.z, q.w])
            # pose quaternion is typically body->map; we want map->body
            # If your pose is indeed body in map, transpose gives inverse rotation.
            R_odom_to_body = T[:3, :3].T

        # rotate error into body frame
        e_body = R_odom_to_body @ e_odom

        # ---------------- OPTIONAL ENU->NED fix ----------------
        # If your odometry in map frame is ENU but you want to command MAVLink BODY_NED (FRD),
        # you may need axis swaps/sign changes.
        #
        # ENU: x=East, y=North, z=Up
        # NED: x=North, y=East, z=Down
        #
        # A common conversion for a vector in ENU -> NED:
        #   [n, e, d] = [y, x, -z]
        #
        if self.assume_odom_is_enu:
            e_body = np.array([e_body[1], e_body[0], -e_body[2]], dtype=float)

        # --- PID: position error -> desired body velocity ---
        vx = self.pid_x.step(float(e_body[0]), dt)
        vy = self.pid_y.step(float(e_body[1]), dt)
        vz = self.pid_z.step(float(e_body[2]), dt)

        # saturate velocities
        vxy = math.hypot(vx, vy)
        if vxy > self.v_max_xy and vxy > 1e-6:
            s = self.v_max_xy / vxy
            vx *= s
            vy *= s
        vz = max(-self.v_max_z, min(self.v_max_z, vz))

        # --- Yaw control (reference yaw -> yaw_rate) ---
        # Current yaw from pose quaternion
        q_cur = pose.orientation
        _, _, yaw_cur = euler_from_quaternion([q_cur.x, q_cur.y, q_cur.z, q_cur.w])

        yaw_ref = float(ref.yaw) if ref.yaw is not None else yaw_cur
        e_yaw = self._wrap_pi(yaw_ref - yaw_cur)
        yaw_rate = self.pid_yaw.step(e_yaw, dt)
        yaw_rate = max(-self.yaw_rate_max, min(self.yaw_rate_max, yaw_rate))

        # --- Publish MAVROS setpoint_raw/local in BODY_NED ---
        pt = PositionTarget()
        pt.header.stamp = now.to_msg()
        pt.header.frame_id = self.odom_frame 
        pt.coordinate_frame = PositionTarget.FRAME_BODY_NED

        # We command: velocity + yaw_rate only
        pt.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW
        )
        pt.velocity.x = float(vx)
        pt.velocity.y = float(vy)
        pt.velocity.z = float(vz)
        pt.yaw_rate = float(yaw_rate)

        self._setpoint_pub.publish(pt)

    @staticmethod
    def _wrap_pi(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    # ---------------- MAVROS services ----------------
    def _set_mode(self, mode: str) -> bool:
        if not self._set_mode_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Set mode service unavailable')
            return False
        req = SetMode.Request()
        req.custom_mode = mode
        fut = self._set_mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        ok = bool(res and res.mode_sent)
        self.get_logger().info(f'Set mode {mode}: {"OK" if ok else "FAIL"}')
        return ok

    def _arm(self, value: bool) -> bool:
        if not self._arm_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Arming service unavailable')
            return False
        req = CommandBool.Request()
        req.value = value
        fut = self._arm_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        ok = bool(res and res.success)
        self.get_logger().info(f'Arm({value}): {"OK" if ok else "FAIL"}')
        return ok


def main(args=None):
    rclpy.init(args=args)
    node = BodyPIDFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()