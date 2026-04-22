#!/usr/bin/env python3
import math
from typing import Optional


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry
from traj_utils.msg import SnakeYaw
from transforms3d.euler import quat2euler
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped as PoseStampted
from std_msgs.msg import String
from enum import Enum


class SnakeState(Enum):  
    IDLE = 0
    CHECK_TAG_INIT = 1
    MOVE_TO_TAG_END = 2
    MOVE_TO_LINE_START = 3
    WAIT_AT_LINE_START = 4
    MOVE_TO_LINE_END = 5
    WAIT_AT_LINE_END = 6
    STEP_DEPTH = 7
    FINISHED = 8


class BodyPIDFollower(Node):

    def __init__(self):
        super().__init__('snake_planner')
        self.current_odom: Odometry = Odometry()
        self.current_goal: Optional[PoseStampted] = None
        # ---------------- ROS I/O ----------------
        qos = QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )

        self.execute_snake_srv = self.create_service(
            Trigger,
            '/snake_planner/execute_snake_path',
            self.execute_snake_path_callback
        )
        self.stop_snake_srv = self.create_service(
            Trigger,
            '/snake_planner/stop_snake_path',
            self.stop_snake_path_callback
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered_enu',
            self._odom_cb,
            qos
        )

        qos = QoSProfile(depth=10)
        self.goal_pub = self.create_publisher(
            PoseStampted,
            '/ego_planner/move_base_simple/goal',
            qos
        )
        self.snake_status_pub = self.create_publisher(
            String,
            '/snake_planner/status',
            qos
        )
        self.snake_yaw_pub = self.create_publisher(
            SnakeYaw, 'planning/snake_yaw', qos
            )
        self.reset_first_client = self.create_client(Trigger, '/follower/reset_first')
        # ==== SNAKE FSM ADDED ====
        self.snake_active = False
        self.snake_state = SnakeState.IDLE

        # snake path parameters (filled when service is called)
        self.top_left = None
        self.top_right = None
        self.bottom_left = None
        self.bottom_right = None
        self.depth_step = None
        self.current_depth = None
        self.max_depth = None
        self.going_right = None
        self.checked_apriltags = False
        self.vel_small = False

        # Timer to drive the snake path (non-blocking)
        self.snake_timer = self.create_timer(0.1, self._snake_timer_cb)
        # Timer to resend goal periodically (in case of failures or disturbances)
        self.goal_timer = self.create_timer(5.0, self.send_goal_callback)
        self.snake_yaw_to_use = 0.0
        self.width_of_ocean_ecostructure = 1.0
 
        # ==== END SNAKE FSM ADDED ====

    def _odom_cb(self, msg: Odometry):
        self.current_odom = msg
        if (self.current_odom.twist.twist.linear.x ** 2 +
            self.current_odom.twist.twist.linear.y ** 2 +
            self.current_odom.twist.twist.linear.z ** 2) < 0.05 ** 2:
            self.vel_small = True
        else:
            self.vel_small = False

    def send_goal_callback(self):
        if self.current_goal:
            goal_msg = PoseStampted()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose = self.current_goal.pose
            self.goal_pub.publish(goal_msg)
        return
    
    def goto_position(self, x_east, y_north, z_down, yaw_deg=None):
        goal_msg = PoseStampted()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x_east
        goal_msg.pose.position.y = y_north
        goal_msg.pose.position.z = -z_down 
        self.current_goal = goal_msg
        
        self.goal_pub.publish(goal_msg)
    

    def reached_goal(self, x_east_goal, y_north_goal, z_down_goal, yaw_goal, threshold=0.25):
        """
        Check if current position is within threshold of target
        threshold in meters
        """
        pos_msg = self.current_odom.pose.pose.position

        if pos_msg is None:
            return False, None

        x_east_current = pos_msg.x
        y_north_current = pos_msg.y
        z_down_current = -pos_msg.z

        x_dist = (x_east_goal - x_east_current)
        y_dist = (y_north_goal - y_north_current)
        depth_dist = z_down_goal - z_down_current

        """_, _, yaw_current = quat2euler([
            self.current_odom.pose.pose.orientation.w,
            self.current_odom.pose.pose.orientation.x,
            self.current_odom.pose.pose.orientation.y,
            self.current_odom.pose.pose.orientation.z
            
        ])
        yaw_dist = yaw_goal - yaw_current

        # wrap to [-pi, pi]
        yaw_dist = (yaw_dist + math.pi) % (2 * math.pi) - math.pi
        # Scale down yaw distance to be comparable to position distances (tuning parameter)
        yaw_dist_scaled = yaw_dist * 0.1"""
        # For now, ignore yaw since yaw control is not very good at endpoints and we do not want to stall the path following if yaw is not perfect 
        # (since a slight yaw error is not detrimental to the inspection task)
        yaw_dist_scaled = 0.0
        total_dist = math.sqrt(
            x_dist**2 + y_dist**2 + depth_dist**2 + yaw_dist_scaled**2
        )

        self.get_logger().info(f"Goal: x={x_east_goal:.2f}, y={y_north_goal:.2f}, depth={z_down_goal:.2f} m")
        self.get_logger().info(f"Current: x={x_east_current:.2f}, y={y_north_current:.2f}, depth={z_down_current:.2f} m")
        self.get_logger().info(
            f"Current: dist_x={x_dist:.7f}, dist_y={y_dist:.7f}, "
            f"dist_depth={depth_dist:.2f} m, dist_yaw={yaw_dist_scaled:.7f}, total_dist={total_dist:.2f} m"
        )

        """if total_dist < threshold:
            request = Trigger.Request()

            future = self.reset_first_client.call_async(request)

            # Optionally spin until response arrives
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                response = future.result()
                self.get_logger().info(
                    f"Success: {response.success}, message: {response.message}"
                )
            else:
                self.get_logger().error('Resetting first position indicator in follower node failed!')"""
        return total_dist < threshold, total_dist

    # =======================
    # SERVICE
    # =======================
    def execute_snake_path_callback(self, request, response):
        """
        Trigger service: start the snake path in the background.
        Returns quickly.
        """
        if self.snake_active:
            response.success = False
            response.message = "Snake path already running."
            return response

        yaw_msg = SnakeYaw()
        roll, pitch, yaw = quat2euler([
            self.current_odom.pose.pose.orientation.w,
            self.current_odom.pose.pose.orientation.x,
            self.current_odom.pose.pose.orientation.y,
            self.current_odom.pose.pose.orientation.z,

        ])
        self.snake_yaw_to_use = yaw
        yaw_msg.snake_yaw = self.snake_yaw_to_use
        yaw_msg.use_snake_yaw = True

        self.snake_yaw_pub.publish(yaw_msg)

        self.execute_snake_path()  # just initializes the FSM state & params

        response.success = True
        response.message = "Snake path execution started."
        return response
    def stop_snake_path_callback(self, request, response):
        """
        Trigger service: start the snake path in the background.
        Returns quickly.
        """
        if not self.snake_active:
            response.success = False
            response.message = "Snake path already inactive."
            return response

        yaw_msg = SnakeYaw()
        yaw_msg.snake_yaw = 0.0
        yaw_msg.use_snake_yaw = False

        self.snake_yaw_pub.publish(yaw_msg)

        x_curr = self.current_odom.pose.pose.position.x
        y_curr = self.current_odom.pose.pose.position.y
        depth_curr = -self.current_odom.pose.pose.position.z

        self.goto_position(x_curr, y_curr, depth_curr, yaw_deg=self.snake_yaw_to_use)
        response.success = True
        response.message = "Snake path execution stopped."
        return response
    # =======================
    # SNAKE PATH INITIALIZER
    # =======================
    def execute_snake_path(self):
        """
        Initialize snake path parameters and start FSM.
        This no longer blocks or uses time.sleep().
        """
        self.get_logger().info("Initializing snake path...")

        # -------------------------
        # Snake path parameters
        # -------------------------
        """self.top_left = {"x": 10.92, "y": 13.55, "depth": 3.25, "yaw": 105.6923}
        self.top_right = {"x": 10.7,  "y": 12.63, "depth": 3.25, "yaw": 105.6923}
        self.bottom_left = {"x": 10.92, "y": 13.56, "depth": 5.0, "yaw": 105.6923}
        self.bottom_right = {"x": 10.7,  "y": 12.63, "depth": 5.0, "yaw": 105.6923}"""

        x_right = math.sin(self.snake_yaw_to_use)*self.width_of_ocean_ecostructure
        y_right = -math.cos(self.snake_yaw_to_use)*self.width_of_ocean_ecostructure

        self.top_left = {"x": 0.0, "y": 0.0, "depth": 0.2}
        self.top_right = {"x": x_right, "y": y_right, "depth": 0.2}
        self.bottom_left = {"x": 0.0, "y": 0.0, "depth": 1.0}
        self.bottom_right = {"x": x_right, "y": y_right, "depth": 1.0}

        self.depth_step = 0.2
        self.current_depth = self.top_left["depth"]
        self.max_depth = self.bottom_left["depth"]
        self.going_right = True
        self.checked_apriltags = False

        # Start FSM at "go to tag check position"
        self.snake_active = True
        self.snake_state = SnakeState.MOVE_TO_LINE_START
        self.get_logger().info("Snake path FSM started.")

    # =======================
    # SNAKE FSM TIMER
    # =======================
    def _snake_timer_cb(self):
        """
        Called periodically by self.snake_timer.
        Advances the snake state machine in small steps.
        """
        if not self.snake_active:
            return

        if self.snake_state == SnakeState.FINISHED:
            self.get_logger().info("Snake path finished.")
            self.snake_active = False
            self.snake_state = SnakeState.IDLE
            self.snake_status_pub.publish(String(data="IDLE"))
            # TODO also reset yaw
            return

        # 1) Move to bottom_left first (AprilTag check region)
        if self.snake_state == SnakeState.CHECK_TAG_INIT:
            self.get_logger().info("Checking for AprilTags before starting snake path...")
            x_end = self.bottom_left["x"]
            y_end = self.bottom_left["y"]
            depth_end = self.bottom_left["depth"]

            # Command motion once toward bottom_left
            self.goto_position(x_end, y_end, depth_end, yaw_deg=self.snake_yaw_to_use)
            self.snake_state = SnakeState.MOVE_TO_TAG_END
            self.snake_status_pub.publish(String(data="MOVE_TO_TAG_END"))
            return

        if self.snake_state == SnakeState.MOVE_TO_TAG_END:
            x_end = self.bottom_left["x"]
            y_end = self.bottom_left["y"]
            depth_end = self.bottom_left["depth"]

            reached, dist = self.reached_goal(
                x_end, y_end, depth_end, self.snake_yaw_to_use, threshold=0.25
            )
            if reached:
                if self.vel_small:
                    self.checked_apriltags = True
                    self.get_logger().info("AprilTag region reached, starting first pass.")
                    self.snake_state = SnakeState.MOVE_TO_LINE_START
                    self.snake_status_pub.publish(String(data="MOVE_TO_LINE_START"))
            return

        # 2) Decide line direction at current depth
        if self.snake_state == SnakeState.MOVE_TO_LINE_START:
            if self.going_right:
                x_start = self.top_left["x"]
                y_start = self.top_left["y"]
            else:
                x_start = self.top_right["x"]
                y_start = self.top_right["y"]

            self.get_logger().info(
                f"Moving to start of line at depth {self.current_depth:.2f} m"
            )
            self.goto_position(x_start, y_start, self.current_depth, yaw_deg=self.snake_yaw_to_use)
            self.snake_state = SnakeState.WAIT_AT_LINE_START
            self.snake_status_pub.publish(String(data="WAIT_AT_LINE_START"))
            return

        if self.snake_state == SnakeState.WAIT_AT_LINE_START:
            if self.going_right:
                x_start = self.top_left["x"]
                y_start = self.top_left["y"]
            else:
                x_start = self.top_right["x"]
                y_start = self.top_right["y"]

            reached, dist = self.reached_goal(
                x_start, y_start, self.current_depth, self.snake_yaw_to_use, threshold=0.05
            )
            if reached:
                if self.vel_small:
                    self.get_logger().info("Reached start of line, moving to end.")
                    self.snake_state = SnakeState.MOVE_TO_LINE_END
                    self.snake_status_pub.publish(String(data="MOVE_TO_LINE_END"))
            return

        if self.snake_state == SnakeState.MOVE_TO_LINE_END:
            if self.going_right:
                x_end = self.top_right["x"]
                y_end = self.top_right["y"]
            else:
                x_end = self.top_left["x"]
                y_end = self.top_left["y"]

            self.get_logger().info("Moving to end of line.")
            self.goto_position(x_end, y_end, self.current_depth, yaw_deg=self.snake_yaw_to_use)
            self.snake_state = SnakeState.WAIT_AT_LINE_END
            self.snake_status_pub.publish(String(data="WAIT_AT_LINE_END"))
            return

        if self.snake_state == SnakeState.WAIT_AT_LINE_END:
            # Determine which end we just commanded (current "end" based on going_right)
            if self.going_right:
                x_end = self.top_right["x"]
                y_end = self.top_right["y"]
            else:
                x_end = self.top_left["x"]
                y_end = self.top_left["y"]

            reached, dist = self.reached_goal(
                x_end, y_end, self.current_depth, self.snake_yaw_to_use, threshold=0.1
            )
            if reached and self.vel_small:
                
                # step depth
                self.get_logger().info("Reached end of line, stepping depth.")
                self.snake_state = SnakeState.STEP_DEPTH
                self.snake_status_pub.publish(String(data="STEP_DEPTH"))
            return


        if self.snake_state == SnakeState.STEP_DEPTH:
            self.current_depth += self.depth_step
            if self.current_depth > self.max_depth:
                self.get_logger().info(
                    f"Max depth {self.max_depth:.2f} m reached, finishing snake."
                )
                self.snake_state = SnakeState.FINISHED
                self.snake_status_pub.publish(String(data="FINISHED SNAKE PATH"))
                #self.execute_snake_path()
                return

            self.going_right = not self.going_right
            self.get_logger().info(
                f"Next pass at depth {self.current_depth:.2f} m, "
                f"direction: {'right' if self.going_right else 'left'}"
            )
            self.snake_state = SnakeState.MOVE_TO_LINE_START
            self.snake_status_pub.publish(String(data="MOVE_TO_LINE_START"))
            return


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