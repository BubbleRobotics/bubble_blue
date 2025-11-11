#!/usr/bin/env python3
"""
ROS 2 follower node that sends setpoint_raw commands to ArduPilot via MAVROS.
- Supports POSITION, VELOCITY and THRUST (acceleration) control modes.
- Uses the same topics/services as the reference file:
    * Publishes to   /mavros/setpoint_raw/local   (mavros_msgs/PositionTarget)
    * Subscribes to  /mavros/vision_pose/pose_cov (geometry_msgs/PoseWithCovarianceStamped)
    * Subscribes to  /mavros/state                 (mavros_msgs/State)
    * Calls services /mavros/set_mode (mavros_msgs/SetMode) and /mavros/cmd/arming (mavros_msgs/CommandBool)

This node exposes three user-facing subscriptions you can feed from your planner or teleop:
    * /follower/cmd_position : geometry_msgs/PoseStamped
          - position in local frame and yaw in orientation (yaw extracted from quaternion)
    * /follower/cmd_velocity : geometry_msgs/TwistStamped
          - linear.{x,y,z} are velocities in local frame (m/s)
          - angular.z is yaw rate (rad/s). If you prefer absolute yaw, use /follower/cmd_position
    * /follower/cmd_thrust   : geometry_msgs/Vector3Stamped
          - vector.x/y/z are accelerations/thrust (m/s^2) in LOCAL_NED frame (as used by MAVROS PositionTarget)

Notes
-----
* This node does **not** include any planner. It only relays incoming commands to the FCU.
* It maintains a steady publishing rate so the FCU continues to receive setpoints (important for OFFBOARD/GUIDED).
* Coordinate frame is PositionTarget.FRAME_LOCAL_NED, matching the reference code.
* For thrust/acceleration control, we set the acceleration fields (afx/afy/afz) and ignore pos/vel/yaw/yaw_rate.
* If you need body-frame thrust, you can change `coordinate_frame` to FRAME_BODY_NED.

Author: You
"""

import math
import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import State as MavState
from mavros_msgs.srv import CommandBool, SetMode
from tf_transformations import euler_from_quaternion


class SetpointRawFollower(Node):
    def __init__(self):
        super().__init__('setpoint_raw_follower')

        # -------- Parameters --------
        self.declare_parameters(
            namespace='',
            parameters=[
                ('publish_rate_hz', 20.0),        # stream setpoints to FCU
                ('set_mode_on_start', 'GUIDED'),  # set to '' to skip
                ('arm_on_start', True),
            ],
        )

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.set_mode_on_start = str(self.get_parameter('set_mode_on_start').value)
        self.arm_on_start = bool(self.get_parameter('arm_on_start').value)

        # -------- State / Buffers --------
        self._is_connected = False
        self._current_yaw = 0.0
        self._last_cmd: Optional[PositionTarget] = None
        self._last_cmd_lock = threading.Lock()

        # -------- QoS --------
        qos = QoSProfile(depth=10)

        # -------- Subscriptions to MAVROS --------
        self._state_sub = self.create_subscription(MavState, '/mavros/state', self._state_cb, qos)
        self._pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov', self._pose_cb, qos)

        # -------- Publisher to MAVROS --------
        self._setpoint_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', qos)

        # -------- Services to MAVROS --------
        self._set_mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self._arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')

        # -------- User-facing command topics --------
        self._pos_cmd_sub = self.create_subscription(PoseStamped, '/follower/cmd_position', self._cmd_position_cb, qos)
        self._vel_cmd_sub = self.create_subscription(TwistStamped, '/follower/cmd_velocity', self._cmd_velocity_cb, qos)
        self._thr_cmd_sub = self.create_subscription(Vector3Stamped, '/follower/cmd_thrust', self._cmd_thrust_cb, qos)

        # -------- Timers --------
        self._pub_timer = self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self._publish_latest)

        # -------- Wait for MAVROS heartbeat (non-blocking loop with timeout) --------
        self.get_logger().info('Waiting for MAVLink heartbeat (MAVROS state)...')
        start_time = time.time()
        timeout = 10.0
        while not self._is_connected and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self._is_connected:
            raise RuntimeError('No heartbeat from MAVROS within timeout')
        self.get_logger().info('Connected to MAVLink vehicle.')

        # -------- Optional: set mode + arm --------
        if self.set_mode_on_start:
            self._set_mode(self.set_mode_on_start)
        if self.arm_on_start:
            self._arm(True)

        self.get_logger().info('SetpointRawFollower ready. Awaiting commands...')

    # =====================
    # MAVROS Callbacks
    # =====================
    def _state_cb(self, msg: MavState):
        if msg.connected and not self._is_connected:
            self._is_connected = True

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        # Track current yaw so we can keep it when not commanded explicitly
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quat)
        self._current_yaw = yaw

    # =====================
    # Command Callbacks
    # =====================
    def _cmd_position_cb(self, msg: PoseStamped):
        """Absolute position + yaw (from quaternion)."""
        pt = PositionTarget()
        pt.header = msg.header
        pt.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        # Ignore velocity, accel, yaw_rate
        pt.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        # Position
        pt.position.x = msg.pose.position.x
        pt.position.y = msg.pose.position.y
        pt.position.z = msg.pose.position.z
        # Yaw
        q = msg.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        pt.yaw = float(yaw)
        self._stash_cmd(pt)

    def _cmd_velocity_cb(self, msg: TwistStamped):
        """Velocity command + yaw rate (angular.z)."""
        pt = PositionTarget()
        pt.header = msg.header
        pt.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        # Ignore position, accel; use yaw_rate instead of absolute yaw
        pt.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW
        )
        # Velocity (m/s)
        pt.velocity.x = msg.twist.linear.x
        pt.velocity.y = msg.twist.linear.y
        pt.velocity.z = msg.twist.linear.z
        # Yaw rate (rad/s)
        pt.yaw_rate = msg.twist.angular.z
        self._stash_cmd(pt)

    def _cmd_thrust_cb(self, msg: Vector3Stamped):
        """Acceleration/Thrust command (m/s^2). Yaw is held at current value."""
        pt = PositionTarget()
        pt.header = msg.header
        pt.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        # Ignore position, velocity, yaw, yaw_rate; use acceleration fields
        pt.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        )
        pt.acceleration_or_force.x = msg.vector.x
        pt.acceleration_or_force.y = msg.vector.y
        pt.acceleration_or_force.z = msg.vector.z
        # Keep current yaw (not used by FCU since ignored, but document intent)
        pt.yaw = float(self._current_yaw)
        self._stash_cmd(pt)

    # =====================
    # Helpers
    # =====================
    def _stash_cmd(self, pt: PositionTarget):
        # Standardize header.frame_id to "map" for consistency with reference code
        if not pt.header.frame_id:
            pt.header.frame_id = 'map'
        with self._last_cmd_lock:
            self._last_cmd = pt

    def _publish_latest(self):
        with self._last_cmd_lock:
            cmd = self._last_cmd
        if cmd is None:
            # Send a hold command at the current yaw to keep FCU happy
            hold = PositionTarget()
            hold.header.stamp = self.get_clock().now().to_msg()
            hold.header.frame_id = 'map'
            hold.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            hold.type_mask = (
                PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE
            )
            hold.position.x = 0.0
            hold.position.y = 0.0
            hold.position.z = 0.0
            hold.yaw = float(self._current_yaw)
            self._setpoint_pub.publish(hold)
            return
        # Update timestamp and publish
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(f"Sent latest command {cmd}")
        self._setpoint_pub.publish(cmd)

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
    node = SetpointRawFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
