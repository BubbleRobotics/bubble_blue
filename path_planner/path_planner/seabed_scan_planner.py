#!/usr/bin/env python3
import math
from collections import deque
from enum import Enum
from typing import Deque, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from tf_transformations import euler_from_quaternion
from traj_utils.msg import SnakeYaw


class ScanState(Enum):
    IDLE = 0
    MOVE_TO_WAYPOINT = 1
    ALIGN_TO_NEXT_WAYPOINT = 2
    FINISHED = 3


class SeabedScanPlanner(Node):
    def __init__(self):
        super().__init__("seabed_scan_planner")

        self.declare_parameter("odom_topic", "/odometry/filtered_enu")
        self.declare_parameter("goal_topic", "/ego_planner/move_base_simple/goal")
        self.declare_parameter("status_topic", "/seabed_scan_planner/status")
        self.declare_parameter("yaw_topic", "planning/snake_yaw")
        self.declare_parameter("goal_frame_id", "odom")
        self.declare_parameter("scan_width_m", 10.0)
        self.declare_parameter("scan_height_m", 10.0)
        self.declare_parameter("lane_spacing_m", 0.5)
        self.declare_parameter("goal_tolerance_m", 0.10)
        self.declare_parameter("goal_check_xy_only", False)
        self.declare_parameter("stop_speed_threshold_mps", 0.05)
        self.declare_parameter("fixed_depth_down_m", 2.0)
        self.declare_parameter("use_current_depth", True)
        self.declare_parameter("use_initial_seabed_distance", False)
        self.declare_parameter("initial_distance_topic", "/min_distance")
        self.declare_parameter("target_distance_from_seabed_m", 2.0)
        self.declare_parameter("initial_distance_timeout_s", 1.0)
        self.declare_parameter("initial_distance_average_samples", 10)
        self.declare_parameter("goal_refresh_period_s", 5.0)
        self.declare_parameter("control_period_s", 0.1)
        self.declare_parameter("hold_current_yaw", True)
        self.declare_parameter("use_goal_facing_yaw_near_waypoint", False)
        self.declare_parameter("goal_facing_yaw_switch_distance_m", 1.0)
        self.declare_parameter("pre_rotate_to_next_waypoint", False)
        self.declare_parameter("yaw_alignment_tolerance_deg", 10.0)

        odom_topic = self.get_parameter("odom_topic").value
        goal_topic = self.get_parameter("goal_topic").value
        status_topic = self.get_parameter("status_topic").value
        yaw_topic = self.get_parameter("yaw_topic").value
        self.goal_frame_id = self.get_parameter("goal_frame_id").value
        initial_distance_topic = self.get_parameter("initial_distance_topic").value

        self.scan_width_m = float(self.get_parameter("scan_width_m").value)
        self.scan_height_m = float(self.get_parameter("scan_height_m").value)
        self.lane_spacing_m = float(self.get_parameter("lane_spacing_m").value)
        self.goal_tolerance_m = float(self.get_parameter("goal_tolerance_m").value)
        self.goal_check_xy_only = bool(self.get_parameter("goal_check_xy_only").value)
        self.stop_speed_threshold_mps = float(self.get_parameter("stop_speed_threshold_mps").value)
        self.fixed_depth_down_m = float(self.get_parameter("fixed_depth_down_m").value)
        self.use_current_depth = bool(self.get_parameter("use_current_depth").value)
        self.use_initial_seabed_distance = bool(
            self.get_parameter("use_initial_seabed_distance").value
        )
        self.target_distance_from_seabed_m = float(
            self.get_parameter("target_distance_from_seabed_m").value
        )
        self.initial_distance_timeout_s = float(
            self.get_parameter("initial_distance_timeout_s").value
        )
        self.initial_distance_average_samples = max(
            1, int(self.get_parameter("initial_distance_average_samples").value)
        )
        self.goal_refresh_period_s = float(self.get_parameter("goal_refresh_period_s").value)
        self.control_period_s = float(self.get_parameter("control_period_s").value)
        self.hold_current_yaw = bool(self.get_parameter("hold_current_yaw").value)
        self.use_goal_facing_yaw_near_waypoint = bool(
            self.get_parameter("use_goal_facing_yaw_near_waypoint").value
        )
        self.goal_facing_yaw_switch_distance_m = max(
            0.0, float(self.get_parameter("goal_facing_yaw_switch_distance_m").value)
        )
        self.pre_rotate_to_next_waypoint = bool(
            self.get_parameter("pre_rotate_to_next_waypoint").value
        )
        self.yaw_alignment_tolerance_rad = math.radians(
            max(0.0, float(self.get_parameter("yaw_alignment_tolerance_deg").value))
        )

        qos = QoSProfile(depth=10)
        best_effort_qos = QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )

        self.current_odom: Optional[Odometry] = None
        self.current_goal: Optional[PoseStamped] = None
        self.scan_waypoints: List[Tuple[float, float, float]] = []
        self.current_waypoint_index = 0
        self.scan_state = ScanState.IDLE
        self.scan_active = False
        self.fixed_yaw_rad = 0.0
        self.segment_yaw_rad: Optional[float] = None
        self.last_goal_facing_yaw_rad = 0.0
        self.pending_waypoint_index: Optional[int] = None
        self.target_alignment_yaw_rad: Optional[float] = None
        self.latest_min_distance_m: Optional[float] = None
        self.latest_min_distance_stamp = None
        self.min_distance_samples: Deque[Tuple[float, object]] = deque(
            maxlen=self.initial_distance_average_samples
        )
        self._last_snake_yaw_use: Optional[bool] = None
        self._last_snake_yaw_rad: Optional[float] = None

        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            best_effort_qos,
        )
        self.min_distance_sub = self.create_subscription(
            Float32,
            initial_distance_topic,
            self.min_distance_callback,
            best_effort_qos,
        )
        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, qos)
        self.status_pub = self.create_publisher(String, status_topic, qos)
        self.snake_yaw_pub = self.create_publisher(SnakeYaw, yaw_topic, qos)

        self.execute_scan_srv = self.create_service(
            Trigger,
            "/seabed_scan_planner/execute_scan",
            self.execute_scan_callback,
        )
        self.stop_scan_srv = self.create_service(
            Trigger,
            "/seabed_scan_planner/stop_scan",
            self.stop_scan_callback,
        )

        self.control_timer = self.create_timer(self.control_period_s, self.control_loop)
        self.goal_timer = self.create_timer(self.goal_refresh_period_s, self.republish_goal)

        self.get_logger().info(
            "Seabed scan planner ready: width=%.2f m, height=%.2f m, spacing=%.2f m"
            % (self.scan_width_m, self.scan_height_m, self.lane_spacing_m)
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.current_odom = msg

    def min_distance_callback(self, msg: Float32) -> None:
        sample = float(msg.data)
        stamp = self.get_clock().now()
        self.latest_min_distance_m = sample
        self.latest_min_distance_stamp = stamp
        self.min_distance_samples.append((sample, stamp))

    def execute_scan_callback(self, request, response):
        del request
        if self.scan_active:
            response.success = False
            response.message = "Seabed scan already running."
            return response

        if self.current_odom is None:
            response.success = False
            response.message = "No odometry yet."
            return response

        self.fixed_yaw_rad = self.get_current_yaw()
        self.segment_yaw_rad = None
        self.last_goal_facing_yaw_rad = self.fixed_yaw_rad
        if self.hold_current_yaw:
            self.publish_snake_yaw(self.fixed_yaw_rad, True)

        origin_x = self.current_odom.pose.pose.position.x
        origin_y = self.current_odom.pose.pose.position.y
        if self.use_current_depth:
            depth_down = -self.current_odom.pose.pose.position.z
        else:
            depth_down = self.fixed_depth_down_m

        self.scan_waypoints = self.build_raster_waypoints(origin_x, origin_y, depth_down)
        self.scan_waypoints = self.trim_redundant_start_waypoints(self.scan_waypoints)
        self.apply_initial_seabed_distance_if_enabled()
        if not self.scan_waypoints:
            response.success = False
            response.message = "Generated scan is empty after removing waypoints already at the current pose."
            return response

        self.current_waypoint_index = 0
        self.scan_state = ScanState.MOVE_TO_WAYPOINT
        self.scan_active = True
        self.publish_current_waypoint()
        self.status_pub.publish(String(data="MOVE_TO_WAYPOINT"))

        response.success = True
        response.message = f"Seabed scan started with {len(self.scan_waypoints)} waypoints."
        return response

    def stop_scan_callback(self, request, response):
        del request
        if not self.scan_active:
            response.success = False
            response.message = "Seabed scan already inactive."
            return response

        self.finish_scan("STOPPED")
        response.success = True
        response.message = "Seabed scan stopped."
        return response

    def control_loop(self) -> None:
        if not self.scan_active:
            return

        if self.current_odom is None:
            return

        if self.scan_state == ScanState.ALIGN_TO_NEXT_WAYPOINT:
            self.control_alignment_to_next_waypoint()
            return

        if self.scan_state != ScanState.MOVE_TO_WAYPOINT:
            return

        self.update_yaw_behavior()

        reached, distance = self.reached_current_waypoint()
        if not reached:
            return

        if not self.is_vehicle_slow():
            return

        self.get_logger().info(
            "Reached waypoint %d/%d (residual %.2f m)"
            % (self.current_waypoint_index + 1, len(self.scan_waypoints), distance)
        )
        next_waypoint_index = self.current_waypoint_index + 1
        if next_waypoint_index >= len(self.scan_waypoints):
            self.finish_scan("FINISHED")
            return

        if self.pre_rotate_to_next_waypoint:
            self.start_alignment_to_next_waypoint(next_waypoint_index)
            return

        self.current_waypoint_index = next_waypoint_index
        self.publish_current_waypoint()
        self.status_pub.publish(String(data="MOVE_TO_WAYPOINT"))

    def republish_goal(self) -> None:
        if self.current_goal is None or not self.scan_active:
            return

        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = self.goal_frame_id
        goal_msg.pose = self.current_goal.pose
        self.goal_pub.publish(goal_msg)

    def publish_current_waypoint(self) -> None:
        x_east, y_north, z_down = self.scan_waypoints[self.current_waypoint_index]
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = self.goal_frame_id
        goal_msg.pose.position.x = x_east
        goal_msg.pose.position.y = y_north
        goal_msg.pose.position.z = -z_down
        goal_msg.pose.orientation.w = 1.0
        self.current_goal = goal_msg
        self.goal_pub.publish(goal_msg)
        self.update_yaw_behavior()
        self.get_logger().info(
            "Sending waypoint %d/%d -> x=%.2f, y=%.2f, depth=%.2f"
            % (
                self.current_waypoint_index + 1,
                len(self.scan_waypoints),
                x_east,
                y_north,
                z_down,
            )
        )

    def finish_scan(self, status: str) -> None:
        self.scan_active = False
        self.scan_state = ScanState.FINISHED
        self.current_goal = None
        self.pending_waypoint_index = None
        self.target_alignment_yaw_rad = None
        self.segment_yaw_rad = None
        self.status_pub.publish(String(data=status))
        self.publish_snake_yaw(0.0, False)

    def reached_current_waypoint(self) -> Tuple[bool, float]:
        x_err, y_err, z_err = self.get_current_waypoint_errors()
        if self.goal_check_xy_only:
            distance = math.sqrt(x_err * x_err + y_err * y_err)
        else:
            distance = math.sqrt(x_err * x_err + y_err * y_err + z_err * z_err)
        return distance < self.goal_tolerance_m, distance

    def is_vehicle_slow(self) -> bool:
        vel = self.current_odom.twist.twist.linear
        speed = math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z)
        return speed < self.stop_speed_threshold_mps

    def apply_initial_seabed_distance_if_enabled(self) -> None:
        if not self.use_initial_seabed_distance or not self.scan_waypoints:
            return

        if self.current_odom is None:
            return

        if self.latest_min_distance_m is None or self.latest_min_distance_stamp is None:
            self.get_logger().warn(
                "Initial seabed-distance mode enabled, but no min_distance has been received yet. "
                "Using nominal scan depth for the first waypoint."
            )
            return

        now = self.get_clock().now()
        valid_samples = [
            value
            for value, stamp in self.min_distance_samples
            if (now - stamp).nanoseconds / 1e9 <= self.initial_distance_timeout_s
        ]
        if not valid_samples:
            self.get_logger().warn(
                "No recent min_distance samples are available (timeout %.2f s). "
                "Using nominal scan depth for the first waypoint."
                % self.initial_distance_timeout_s
            )
            return

        if len(valid_samples) < self.initial_distance_average_samples:
            self.get_logger().warn(
                "Only %d/%d recent min_distance samples available. "
                "Using their average for the first waypoint depth."
                % (len(valid_samples), self.initial_distance_average_samples)
            )

        avg_min_distance_m = sum(valid_samples) / len(valid_samples)

        current_depth_down = -self.current_odom.pose.pose.position.z
        desired_depth_down = (
            current_depth_down
            + avg_min_distance_m
            - self.target_distance_from_seabed_m
        )
        x_goal, y_goal, _ = self.scan_waypoints[0]
        self.scan_waypoints[0] = (x_goal, y_goal, desired_depth_down)
        self.get_logger().info(
            "Adjusted first waypoint depth from seabed distance: current_depth=%.2f m, "
            "avg_min_distance=%.2f m over %d sample(s), target_clearance=%.2f m "
            "-> first_waypoint_depth=%.2f m"
            % (
                current_depth_down,
                avg_min_distance_m,
                len(valid_samples),
                self.target_distance_from_seabed_m,
                desired_depth_down,
            )
        )

    def get_current_yaw(self) -> float:
        q = self.current_odom.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def get_current_waypoint_errors(self) -> Tuple[float, float, float]:
        x_goal, y_goal, z_goal = self.scan_waypoints[self.current_waypoint_index]
        pos = self.current_odom.pose.pose.position
        x_err = x_goal - pos.x
        y_err = y_goal - pos.y
        z_err = z_goal - (-pos.z)
        return x_err, y_err, z_err

    def compute_goal_facing_yaw(self) -> float:
        x_err, y_err, _ = self.get_current_waypoint_errors()
        planar_distance = math.hypot(x_err, y_err)
        if planar_distance > 1e-6:
            self.last_goal_facing_yaw_rad = math.atan2(y_err, x_err)
        return self.last_goal_facing_yaw_rad

    def compute_yaw_to_waypoint(self, waypoint_index: int) -> float:
        x_goal, y_goal, _ = self.scan_waypoints[waypoint_index]
        pos = self.current_odom.pose.pose.position
        dx = x_goal - pos.x
        dy = y_goal - pos.y
        planar_distance = math.hypot(dx, dy)
        if planar_distance > 1e-6:
            return math.atan2(dy, dx)
        return self.get_current_yaw()

    def angle_diff(self, target_yaw: float, current_yaw: float) -> float:
        return math.atan2(
            math.sin(target_yaw - current_yaw),
            math.cos(target_yaw - current_yaw),
        )

    def start_alignment_to_next_waypoint(self, waypoint_index: int) -> None:
        self.pending_waypoint_index = waypoint_index
        self.target_alignment_yaw_rad = self.compute_yaw_to_waypoint(waypoint_index)
        self.segment_yaw_rad = self.target_alignment_yaw_rad
        self.current_goal = None
        self.scan_state = ScanState.ALIGN_TO_NEXT_WAYPOINT
        self.publish_snake_yaw(self.target_alignment_yaw_rad, True)
        self.status_pub.publish(String(data="ALIGN_TO_NEXT_WAYPOINT"))
        self.get_logger().info(
            "Aligning to waypoint %d/%d before moving: yaw_target=%.1f deg"
            % (
                waypoint_index + 1,
                len(self.scan_waypoints),
                math.degrees(self.target_alignment_yaw_rad),
            )
        )

    def control_alignment_to_next_waypoint(self) -> None:
        if self.pending_waypoint_index is None or self.target_alignment_yaw_rad is None:
            self.scan_state = ScanState.MOVE_TO_WAYPOINT
            return

        self.publish_snake_yaw(self.target_alignment_yaw_rad, True)
        yaw_error = self.angle_diff(self.target_alignment_yaw_rad, self.get_current_yaw())
        if abs(yaw_error) > self.yaw_alignment_tolerance_rad:
            return

        self.current_waypoint_index = self.pending_waypoint_index
        self.pending_waypoint_index = None
        self.target_alignment_yaw_rad = None
        self.scan_state = ScanState.MOVE_TO_WAYPOINT
        self.publish_current_waypoint()
        self.status_pub.publish(String(data="MOVE_TO_WAYPOINT"))
        self.get_logger().info(
            "Yaw aligned within %.1f deg, publishing waypoint %d/%d"
            % (
                math.degrees(self.yaw_alignment_tolerance_rad),
                self.current_waypoint_index + 1,
                len(self.scan_waypoints),
            )
        )

    def update_yaw_behavior(self) -> None:
        if not self.scan_active or self.current_odom is None or not self.scan_waypoints:
            return

        use_snake_yaw = False
        yaw_rad = 0.0

        if self.use_goal_facing_yaw_near_waypoint:
            x_err, y_err, _ = self.get_current_waypoint_errors()
            planar_distance = math.hypot(x_err, y_err)
            if planar_distance <= self.goal_facing_yaw_switch_distance_m:
                use_snake_yaw = True
                yaw_rad = self.compute_goal_facing_yaw()

        if not use_snake_yaw and self.segment_yaw_rad is not None:
            use_snake_yaw = True
            yaw_rad = self.segment_yaw_rad

        if not use_snake_yaw and self.hold_current_yaw:
            use_snake_yaw = True
            yaw_rad = self.fixed_yaw_rad

        self.publish_snake_yaw(yaw_rad, use_snake_yaw)

    def publish_snake_yaw(self, yaw_rad: float, use_snake_yaw: bool) -> None:
        if self._last_snake_yaw_use == use_snake_yaw:
            if not use_snake_yaw:
                return
            if self._last_snake_yaw_rad is not None:
                yaw_delta = math.atan2(
                    math.sin(yaw_rad - self._last_snake_yaw_rad),
                    math.cos(yaw_rad - self._last_snake_yaw_rad),
                )
                if abs(yaw_delta) < 1e-3:
                    return

        yaw_msg = SnakeYaw()
        yaw_msg.snake_yaw = yaw_rad
        yaw_msg.use_snake_yaw = use_snake_yaw
        self.snake_yaw_pub.publish(yaw_msg)
        self._last_snake_yaw_use = use_snake_yaw
        self._last_snake_yaw_rad = yaw_rad if use_snake_yaw else None

    def build_raster_waypoints(self, origin_x: float, origin_y: float, depth_down: float) -> List[Tuple[float, float, float]]:
        spacing = max(self.lane_spacing_m, 1e-3)
        width = max(self.scan_width_m, 0.0)
        height = max(self.scan_height_m, 0.0)

        y_offsets = []
        current_y = 0.0
        while current_y < height + 1e-9:
            y_offsets.append(min(current_y, height))
            current_y += spacing
        if not y_offsets:
            y_offsets.append(0.0)
        if abs(y_offsets[-1] - height) > 1e-9:
            y_offsets.append(height)

        waypoints: List[Tuple[float, float, float]] = []
        for row_idx, y_offset in enumerate(y_offsets):
            row_y = origin_y + y_offset
            x_start = origin_x
            x_end = origin_x + width
            if row_idx % 2 == 0:
                waypoints.append((x_start, row_y, depth_down))
                if width > 0.0:
                    waypoints.append((x_end, row_y, depth_down))
            else:
                waypoints.append((x_end, row_y, depth_down))
                if width > 0.0:
                    waypoints.append((x_start, row_y, depth_down))
        return waypoints

    def trim_redundant_start_waypoints(
        self, waypoints: List[Tuple[float, float, float]]
    ) -> List[Tuple[float, float, float]]:
        if self.current_odom is None:
            return waypoints

        pos = self.current_odom.pose.pose.position
        current_depth_down = -pos.z

        first_non_redundant_idx = 0
        for idx, (x_goal, y_goal, z_goal) in enumerate(waypoints):
            distance = math.sqrt(
                (x_goal - pos.x) ** 2
                + (y_goal - pos.y) ** 2
                + (z_goal - current_depth_down) ** 2
            )
            if distance >= self.goal_tolerance_m:
                first_non_redundant_idx = idx
                break
        else:
            return []

        if first_non_redundant_idx > 0:
            self.get_logger().info(
                "Skipping %d initial waypoint(s) already at the current pose."
                % first_non_redundant_idx
            )

        return waypoints[first_non_redundant_idx:]


def main(args=None):
    rclpy.init(args=args)
    node = SeabedScanPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
