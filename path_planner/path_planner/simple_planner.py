#!/usr/bin/env python3
"""
RRT path follower node (ROS 2, rclpy).
- Plans in local ENU (z up) with simple obstacle models.
- Sends position setpoints to the FCU via pymavlink (LOCAL_NED).
- Uses MAVROS services to set mode / arm.

Author: Luis Blunschi (adapted to ROS2 node)
"""

import math
import numpy as np
import time
import threading
import random
from dataclasses import dataclass
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.parameter import Parameter
from mavros_msgs.srv import CommandBool, SetMode
from pymavlink import mavutil
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from robot_localization.srv import SetPose
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from path_planner_interfaces.srv import InitiatePath, SetObstacles
from path_planner_interfaces.msg import Sphere, AABB, OrientedBox, Path
from std_srvs.srv import Trigger
from std_msgs.msg import Header, Float32
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import State as mavState
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
# ==============
# ROS2 Node
# ==============

class SimplePlannerNode(Node):
    def __init__(self):
        super().__init__('simple_planner')
        self.get_logger().info('Starting SimplePlannerNode...')
        # ---- parameters (declare + get) ----
        self._running = False

        self.direction = "None"
        self.last_direction = "None"
        # MAVROS subscribers
        #self._pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov', self._pose_cb, 10)

        # MAVROS setpoint publishers
        self.vel_local_pub = self.create_publisher(PositionCommand,'/pilot_planner/pos_cmd',10)
        self.depth_sub = self.create_subscription(Odometry, 'model/bluerov2/odometry', self._odom_callback, 10)
        # Service to plan + follow path
        self.path_initializer = self.create_service(
            Trigger,
            'test/initialize',
            self._initialize
        )
        self.path_stopper = self.create_service(Trigger,
            'test/stop',
            self._stop
        )
        
        # Create a subscription to listen for depth image
        self._state_sub = self.create_subscription(
            Image, '/camera_d455/depth/image_raw', self._image_callback, 10
        )
        # TODO cleanup parameters
        self._depth_pub = self.create_publisher(Float32, '/ref_depth', 10)
        self.index = 0
        self.desired_z = -3.6
        self.current_z = 0.0
        self.desired_depth = 0.4
        self.current_depth = 0.0
        self.to_lower_z = False
        self.z_step_size = 0.2

    def _image_callback(self, msg: Image):
        self.index += 1
        if self._running:
            
            self.current_depth = self._compute_average_depth(msg)
            grad_x, grad_y = self._compute_depth_gradients(msg, msg.width, msg.height)
            edges_x, edges_y, edges = self._detect_depth_hard_edges(msg, msg.width, msg.height, threshold_m=0.10)
            avg_x_edgex, avg_y_edgex = self._compute_average_edge_position(edges_x)
            avg_x_edgey, avg_y_edgey = self._compute_average_edge_position(edges_y)
            if self.index % 10 == 0:
                self.get_logger().info(f"Average Depth: {self.current_depth:.2f} m")
            
            if self.current_depth == np.inf:
                return  # No valid depth data
            # Simple avoidance logic based on depth gradient
            # TODO clean up these params
            
            edge_position_reference_left = 0.35
            edge_position_reference_right = 0.65


            cmd = PositionCommand()
            cmd.header = Header()
            cmd.header.stamp = self.get_clock().now().to_msg()

            # Forward/backward position control to maintain distance
            cmd.position.x = (self.current_depth-self.desired_depth)
            
            # z - position control
            cmd.position.z = (self.desired_z - self.current_z)

            # TODO see how this can be done better, for now just set a very small forward velocity
            # In order to avoid the ArduSub controller from stopping the vehicle 
            # When spamming pure position commands
            cmd.velocity.x = 0.0000001
            
         
            # For now, known orientation TODO improve logic with gradients
            cmd.yaw_dot = 0.0 #grad_x*0.1
            cmd.yaw = -1.8447
            
            # If too few edges detected, assume no edges
            if np.sum(edges_x) < 20:
                avg_x_edgex = 0.5*msg.width

            # Snake pattern control logic
            # Start at top left corner, go right until edge detected, go down a bit, go left until edge detected, go down a bit, repeat
            if avg_x_edgex is not None:

                # Starting to approach edge: Move edge to 40% or 60% of image width (center used for distance)
                if self.direction == "None":
                    # Robot was moving right: Inspection element is on left side of edge
                    if self.last_direction == "Right":
                        cmd.position.y = -(avg_x_edgex/msg.width - edge_position_reference_right)*0.5
                    # Robot was moving left or just initialized: Inspection element is on right side of edge
                    else:# self.last_direction == "Left":
                        cmd.position.y = -(avg_x_edgex/msg.width - edge_position_reference_left)*0.5
                    if (abs(self.current_depth  - self.desired_depth) < 0.05):
                        # If we still need to lower the z value (have not arrived at edge)
                        if self.to_lower_z:
                            if ((self.last_direction == "Right" and avg_x_edgex/msg.width < 0.70 and avg_x_edgex/msg.width > 0.60)
                            or (self.last_direction == "Left" and avg_x_edgex/msg.width < 0.40 and avg_x_edgex/msg.width > 0.30)
                            or (self.last_direction == "None" and avg_x_edgex/msg.width < 0.40 and avg_x_edgex/msg.width > 0.30)):
                               
                                self.to_lower_z = False
                                self.desired_z -= self.z_step_size
                                depth = Float32()
                                depth.data = self.desired_z
                                self._depth_pub.publish(depth)
                                self.get_logger().info(f"Published new depth: {self.desired_z}")
                        else:
                            # If depth and z value are close enough to reference, start lateral movement
                            if (abs(self.current_depth  - self.desired_depth) < 0.05 and 
                                abs(self.current_z  - self.desired_z) < 0.05):
                                if self.last_direction == "Right" and avg_x_edgex/msg.width < 0.70 and avg_x_edgex/msg.width > 0.60 :
                                    self.direction = "Left"
                                    self.get_logger().info("Starting LEFT MOVEMENT")
                            
                                elif self.last_direction == "Left" and avg_x_edgex/msg.width < 0.40 and avg_x_edgex/msg.width > 0.30:
                                    self.direction = "Right"
                                    self.get_logger().info("Starting RIGHT MOVEMENT")
                                    
                                elif self.last_direction == "None" and avg_x_edgex/msg.width < 0.40 and avg_x_edgex/msg.width > 0.30:
                                    self.direction = "Right"
                                    self.get_logger().info("Starting RIGHT MOVEMENT")

                elif self.direction == "Left":
                    cmd.position.y = 0.2
                    
                    if avg_x_edgex is not None:
                        # If we detect an edge at the left corner of the image, stop the movement
                        if avg_x_edgex/msg.width < 0.2 and self.last_direction != "Left":
                            self.last_direction = self.direction
                            self.direction = "None"
                            self.get_logger().info("Finishing LEFT MOVEMENT")
                            self.get_logger().info(f"number of x edges: {np.sum(edges_x)}")
                            self.to_lower_z = True
                            
                elif self.direction == "Right":
                    cmd.position.y = -0.2
                    if avg_x_edgex is not None:
                        # If we detect an edge at the right corner of the image, stop the movement
                        if avg_x_edgex/msg.width > 0.8 and self.last_direction != "Right":
                            self.last_direction = self.direction
                            self.direction = "None"
                            self.get_logger().info("Finishing RIGHT MOVEMENT")
                            self.get_logger().info(f"number of x edges: {np.sum(edges_x)}")
                            self.to_lower_z = True

                else:
                    self.get_logger().warn(f"Direction {self.direction} is an invalid state!")
                    cmd.position.y = 0.0
            else:
                self.get_logger().info(f"AVG X Edge is None{avg_x_edgex}")
                cmd.position.y = 0.0
    
        else:
            # Just stand still for now TODO improve logic
            cmd = PositionCommand()
            cmd.header = Header()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.position.x = 0.0
            cmd.position.y = 0.0
            cmd.position.z = 0.0   
            cmd.velocity.x = 0.0001 # Constant forward speed
            cmd.velocity.y = 0.0  # Move away from obstacles
            cmd.velocity.z = 0.0  # Adjust depth based on gradient
            cmd.yaw = -1.8447
            cmd.yaw_dot = 0.0 #grad_x*0.1
        
        self.vel_local_pub.publish(cmd)   


    def _initialize(self, request, response):
        self.get_logger().info("Path initialization requested")
        # TODO implement path initialization
        response.success = True
        response.message = "Path initialized"
        self._running = True
        self.direction = "None"
        self.last_direction = "None"
        self.desired_z = -3.6
        depth = Float32()
        depth.data = self.desired_z
        self._depth_pub.publish(depth)
        self.get_logger().info(f"Published new depth: {self.desired_z}")
        return response
    
    def _stop(self, request, response):
        self.get_logger().info("Path stopping requested")
        # TODO implement path stopping
        response.success = True
        response.message = "Path stopped"
        
        self._running = False

        # Simple avoidance logic based on depth gradient
        cmd = PositionCommand()
        cmd.header = Header()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.position.x = 0.0
        cmd.position.y = 0.0
        cmd.position.z = 0.0   
        cmd.velocity.x = 0.0001 # Constant forward speed
        cmd.velocity.y = 0.0  # Move away from obstacles
        cmd.velocity.z = 0.0  # Adjust depth based on gradient
        cmd.yaw = -1.8447
        cmd.yaw_dot = 0.0 #grad_x*0.1

        self.vel_local_pub.publish(cmd)
        self.get_logger().info("STOPPING ALL MOVEMENT")
        return response

    def _odom_callback(self, msg: Odometry):
        self.current_z = msg.pose.pose.position.z
        return

    def _detect_depth_hard_edges(
        self,
        depth_image: Image,
        width: int,
        height: int,
        threshold_m: float = 0.10,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Detect hard edges in a depth image.

        An edge is detected where neighboring pixels differ in depth
        by more than `threshold_m` (e.g. 0.10 = 10 cm).

        Args:
            depth_image: Image object with raw float32 depth buffer.
            width, height: Dimensions of the depth image.
            threshold_m: Depth difference threshold in meters.

        Returns:
            edges_x: bool array (H, W), edges along x-direction
                    (between left/right neighbors).
            edges_y: bool array (H, W), edges along y-direction
                    (between top/bottom neighbors).
            edges_combined: bool array (H, W), edges_x OR edges_y.
        """
        # Interpret raw data as float32 array and reshape to 2D (H, W)
        depth = np.frombuffer(depth_image.data, dtype=np.float32).reshape(height, width)

        # Valid values: > 0 and finite (no NaN, no ±inf)
        valid = (depth > 0.0) & np.isfinite(depth)

        # --- X-direction edges (left-right neighbors) ---
        # Differences between neighboring columns
        dx = np.abs(depth[:, 1:] - depth[:, :-1])

        # Only consider pairs where both pixels are valid
        valid_x = valid[:, 1:] & valid[:, :-1]
        edges_x_inner = (dx > threshold_m) & valid_x

        # Pad to original shape (no edge info for first column)
        edges_x = np.zeros_like(depth, dtype=bool)
        edges_x[:, 1:] = edges_x_inner

        # --- Y-direction edges (top-bottom neighbors) ---
        dy = np.abs(depth[1:, :] - depth[:-1, :])
        valid_y = valid[1:, :] & valid[:-1, :]
        edges_y_inner = (dy > threshold_m) & valid_y

        edges_y = np.zeros_like(depth, dtype=bool)
        edges_y[1:, :] = edges_y_inner

        # Combined edge map
        edges_combined = edges_x | edges_y

        return edges_x, edges_y, edges_combined


    def _compute_average_depth(self, depth_image: Image) -> float:

        # Convert raw data to float32 array and reshape to image dimensions
        depth_array = np.frombuffer(depth_image.data, dtype=np.float32)
        depth_array = depth_array.reshape((depth_image.height, depth_image.width))

        # --- Compute central 5% × 5% crop ---
        h, w = depth_image.height, depth_image.width

        crop_h = int(h * 0.05)
        crop_w = int(w * 0.05)

        # Ensure at least 1 pixel
        crop_h = max(crop_h, 1)
        crop_w = max(crop_w, 1)

        # Coordinates of crop
        top = (h - crop_h) // 2
        left = (w - crop_w) // 2
        bottom = top + crop_h
        right = left + crop_w

        central_patch = depth_array[top:bottom, left:right].ravel()

        # Filter valid depth values
        valid = central_patch[
            (central_patch > 0.0) &
            np.isfinite(central_patch)
        ]

        # No valid values found → return 0
        if valid.size == 0:
            return 0.0

        return float(valid.mean())


    
    def _compute_depth_gradients(self, depth_image: Image, width: int, height: int) -> tuple[np.ndarray, np.ndarray]:
        """
        Compute depth edges (gradients) in x and y directions.

        Returns:
            gx: gradient in x-direction (horizontal, increasing column index)
            gy: gradient in y-direction (vertical, increasing row index)
        """
        # Interpret raw data as float32 array and reshape to 2D (H, W)
        depth = np.frombuffer(depth_image.data, dtype=np.float32).reshape(height, width)

        # Mask invalid values: non-positive, NaN or infinite -> set to 0
        valid_mask = (depth > 0.0) & np.isfinite(depth)
        depth = np.where(valid_mask, depth, 0.0)

        # np.gradient returns (d/dy, d/dx) = (along axis 0, along axis 1)
        gy, gx = np.gradient(depth)

        return np.average(gx), np.average(gy)

    import numpy as np

    def _compute_average_edge_position(self, edges_x: np.ndarray):
        """
        Compute the average pixel position of all True values in edges_x.

        Args:
            edges_x: Boolean array (H, W) marking detected horizontal edges.

        Returns:
            (avg_x, avg_y): Floats giving the centroid of all edge pixels.
                            Returns (None, None) if no edge pixels exist.
        """
        # Find all edge pixel indices
        ys, xs = np.where(edges_x)

        if xs.size == 0:
            return None, None

        avg_x = float(xs.mean())
        avg_y = float(ys.mean())

        return avg_x, avg_y



def main(args=None):
    rclpy.init(args=args)
    node = SimplePlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
