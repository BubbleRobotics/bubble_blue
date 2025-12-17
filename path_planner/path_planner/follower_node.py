#!/usr/bin/env python3
"""
ROS 2 follower node that sends setpoint_raw commands to ArduPilot via MAVROS.
- Supports POSITION and VELOCITY (acceleration) control modes.
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
    

Notes
-----
* This node does **not** include any planner. It only relays incoming commands to the FCU.
* It maintains a steady publishing rate so the FCU continues to receive setpoints (important for OFFBOARD/GUIDED).
* Coordinate frame is PositionTarget.FRAME_LOCAL_NED, matching the reference code.

Author: You
"""

import math
import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TwistStamped, Vector3Stamped, Pose, PoseWithCovariance
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import State as MavState
from mavros_msgs.srv import CommandBool, SetMode
from quadrotor_msgs.msg import PositionCommand
from tf_transformations import euler_from_quaternion
from path_planner_interfaces.msg import Path
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped

class SetpointRawFollower(Node):
    def __init__(self):
        super().__init__('setpoint_raw_follower')
        self.get_logger().info('Starting SetpointRawFollower node...')
        # -------- Parameters --------
        self.declare_parameters(
            namespace='',
            parameters=[
                ('publish_rate_hz', 20.0),        # stream setpoints to FCU
                ('set_mode_on_start', 'GUIDED'),  # set to '' to skip
                ('arm_on_start', False),
            ],
        )
        self._position_received = False
        self._active = False
        self._first_hold = True
        self._armed = False
        self._mode_set = False
        self.current_pose = PoseWithCovariance()
        self.current_waypoint = Pose()
        self.current_waypoint.position.z = -0.5
        self.current_path = Path()
        self.waypoint_threshold = 0.1
        self.controller_active = 0 # 0: No controller active, 1: position controller, 2: velocity controller
        self.last_position = PositionTarget()
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.set_mode_on_start = str(self.get_parameter('set_mode_on_start').value)
        self.arm_on_start = bool(self.get_parameter('arm_on_start').value)
        self.first = True
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
        self._clicked_point_sub = self.create_subscription(PointStamped, '/clicked_point', self._clicked_point_cb, qos)
        self._cmd_sub = self.create_subscription(PositionCommand, '/drone_0_planning/pos_cmd', self._cmd_cb_ego_planner, qos)
        self._cmd_sub_live = self.create_subscription(PositionCommand, '/drone_0_planning/pos_cmd_live', self._cmd_cb_ego_planner_live, qos)
        self.ref_depth_sub = self.create_subscription(Float32, '/ref_depth', self._depth_cb, qos)
        self.current_depth_desired = -2.0
        # -------- Publisher to MAVROS --------
        self._setpoint_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', qos)
        self._goal_point_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', qos)
        # -------- Services to MAVROS --------
        self._set_mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self._arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        
        # -------- Timers --------
        #self._pub_timer = self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self._publish_latest)

        # -------- Wait for MAVROS heartbeat (non-blocking loop with timeout) --------
        self.get_logger().info('Waiting for MAVLink heartbeat (MAVROS state)...')
        start_time = time.time()
        timeout = 50.0
        while not self._is_connected and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self._is_connected:
            raise RuntimeError('No heartbeat from MAVROS within timeout')
        self.get_logger().info('Connected to MAVLink vehicle.')

        # -------- Optional: set mode + arm --------
        if self.set_mode_on_start:
            
            while not self._mode_set:
                self._mode_set = self._set_mode(self.set_mode_on_start)
        if self.arm_on_start:
            
            while not self._armed:
                self._armed = self._arm(True)
        
        
        self.get_logger().info('SetpointRawFollower ready. Awaiting commands...')

    def _clicked_point_cb(self, msg:PointStamped):
        goal_point = PoseStamped()
        
        goal_point.pose.position.x = msg.point.x
        goal_point.pose.position.y = msg.point.y
        goal_point.pose.position.z = -1.0 # TODO change to 3D capability, for now selecting in 3D is not possible (only able to select on reference points)
        self.get_logger().info(f"Published new goal (ENU) to EGO planner. x: {msg.point.x}, y: {msg.point.y}, z: {goal_point.pose.position.z}")
        self._goal_point_pub.publish(goal_point)

    def _depth_cb(self, msg:Float32):
        self.current_depth_desired = msg.data
        return
    # =====================
    # MAVROS Callbacks
    # =====================
    def _state_cb(self, msg: MavState):
        if msg.connected and not self._is_connected:
            self._is_connected = True
    def _pose_cb(self, msg: PoseWithCovarianceStamped):

        self.current_pose = msg.pose
        self._position_received = True
        #if self._active:
            
            # Check if waypoint has to be changed
            #self._check_waypoint_reached()


    # =====================
    # Command Callbacks
    # =====================
    def _cmd_cb_ego_planner_live(self, msg:PositionCommand):
        """Update path."""

        # For very close points, the ego-planner only outputs position commands.
        # Since spamming pure position commands will stall the ArduSub controller, only send this once.
        if msg.velocity.x == 0.0 and msg.velocity.y == 0.0 and msg.velocity.z == 0.0:
            if self.first:
                self.get_logger().info("First time going to position control!")
                self.first = True
                # Once close to goal, use position control
                self.goto_position(x_east_m=msg.position.x,y_north_m=msg.position.y,up_m=msg.position.z, yaw_deg=math.degrees(msg.yaw))
                return
            else:
                return
        # Otherwise, use position + velocity control     
        self.first = True
        pt = PositionTarget()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = "map"
        pt.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # Start with accel ignored (we don't use them)
        mask = (
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ
        )

        # ---- Position part ----
        if msg.position.x is None or msg.position.y is None or msg.position.z is None:
            # We are NOT commanding position → ignore PX/PY/PZ
            mask |= (
                PositionTarget.IGNORE_PX |
                PositionTarget.IGNORE_PY |
                PositionTarget.IGNORE_PZ
            )
        else:
            pt.position.x = float(msg.position.x)
            pt.position.y = float(msg.position.y)
            pt.position.z = float(msg.position.z)

        # ---- Velocity part ----
        if msg.velocity.x is None or msg.velocity.y is None or msg.velocity.z is None:
            # We are NOT commanding velocity → ignore VX/VY/VZ
            mask |= (
                PositionTarget.IGNORE_VX |
                PositionTarget.IGNORE_VY |
                PositionTarget.IGNORE_VZ
            )
        else:
            pt.velocity.x = float(msg.velocity.x)
            pt.velocity.y = float(msg.velocity.y)
            pt.velocity.z = float(msg.velocity.z)

        # ---- Yaw / yaw rate ----
        if msg.yaw is not None:
            pt.yaw = float(msg.yaw)
            # Using absolute yaw → ignore yaw_rate
            mask |= PositionTarget.IGNORE_YAW_RATE
        elif msg.yaw_dot is not None:
            pt.yaw_rate = float(msg.yaw_dot)
            # Using yaw rate → ignore absolute yaw
            mask |= PositionTarget.IGNORE_YAW
        else:
            # Not commanding any yaw
            mask |= PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE

        pt.type_mask = mask

        self.goto_pos_vel(pos_target=pt)

    def _cmd_cb_ego_planner(self, msg:PositionCommand):
        """Update path."""

        # For very close points, the ego-planner only outputs position commands.
        # Since spamming pure position commands will stall the ArduSub controller, only send this once.
        if msg.velocity.x == 0.0 and msg.velocity.y == 0.0 and msg.velocity.z == 0.0:
            if self.first:
                self.get_logger().info("First time going to position control!")
                self.first = False
                # Once close to goal, use position control
                self.goto_position(x_east_m=msg.position.x,y_north_m=msg.position.y,up_m=msg.position.z, yaw_deg=math.degrees(msg.yaw))
                return
            else:
                return
        # Otherwise, use position + velocity control     
        self.first = True
        pt = PositionTarget()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = "map"
        pt.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # Start with accel ignored (we don't use them)
        mask = (
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ
        )

        # ---- Position part ----
        if msg.position.x is None or msg.position.y is None or msg.position.z is None:
            # We are NOT commanding position → ignore PX/PY/PZ
            mask |= (
                PositionTarget.IGNORE_PX |
                PositionTarget.IGNORE_PY |
                PositionTarget.IGNORE_PZ
            )
        else:
            pt.position.x = float(msg.position.x)
            pt.position.y = float(msg.position.y)
            pt.position.z = float(msg.position.z)

        # ---- Velocity part ----
        if msg.velocity.x is None or msg.velocity.y is None or msg.velocity.z is None:
            # We are NOT commanding velocity → ignore VX/VY/VZ
            mask |= (
                PositionTarget.IGNORE_VX |
                PositionTarget.IGNORE_VY |
                PositionTarget.IGNORE_VZ
            )
        else:
            pt.velocity.x = float(msg.velocity.x)
            pt.velocity.y = float(msg.velocity.y)
            pt.velocity.z = float(msg.velocity.z)

        # ---- Yaw / yaw rate ----
        if msg.yaw is not None:
            pt.yaw = float(msg.yaw)
            # Using absolute yaw → ignore yaw_rate
            mask |= PositionTarget.IGNORE_YAW_RATE
        elif msg.yaw_dot is not None:
            pt.yaw_rate = float(msg.yaw_dot)
            # Using yaw rate → ignore absolute yaw
            mask |= PositionTarget.IGNORE_YAW
        else:
            # Not commanding any yaw
            mask |= PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE

        pt.type_mask = mask

        self.goto_pos_vel(pos_target=pt)
        
    def _cmd_position(self, msg: Pose):
        """Absolute position + yaw (from quaternion)."""
    
        pt = PositionTarget()
        
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        # Ignore velocity, accel, yaw_rate
        pt.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        # Position
        pt.position = msg.position
        # Yaw
        q = msg.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        pt.yaw = float(yaw)
        self._stash_cmd(pt)


    # =====================
    # Helpers
    # =====================

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
       
    def goto_position(self, x_east_m: float, y_north_m: float, up_m: float, yaw_deg: Optional[float] = 0.0) -> None:
  
        msg = PositionTarget()
        # Use local NED frame (MAVROS translates correctly)
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # Type mask (bits = ignore fields)
        # We want to send ONLY position + yaw
        # ignore velocity, accel, yaw_rate
        msg.type_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        # Desired position (NED)
        msg.position.x = float(x_east_m)    # North
        msg.position.y = float(y_north_m)    # East
        msg.position.z = float(up_m)        # Down

        # Desired yaw (rad)
        msg.yaw = float(math.radians(yaw_deg))
        self._setpoint_pub.publish(msg)
    def goto_pos_vel(self, pos_target: PositionTarget
    ) -> None:
        """
        Send a PositionTarget that can contain:
        - position only
        - velocity only
        - position + velocity
        - yaw OR yaw_rate (not both at once)

        Any argument left as None will be ignored via type_mask.
        """
        
        # Feed into the existing pipeline
        self._setpoint_pub.publish(pos_target)


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
