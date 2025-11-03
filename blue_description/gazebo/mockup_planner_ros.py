"""ROS2 ROV snake-pattern planner using MAVROS topics and services."""

import math
import time
import rclpy
from rclpy.node import Node

from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import GlobalPositionTarget, State
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header


class ROVSnakePlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        # --- MAVROS state ---
        self.state = None
        self.global_fix = None
        self.rel_alt = 0.0

        # --- Subscribers ---
        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_cb, 10)

        # --- Publishers ---
        self.target_pub = self.create_publisher(GlobalPositionTarget,
                                                '/mavros/setpoint_position/global', 10)

        # --- Service clients ---
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.get_logger().info("ROV Planner node started. Waiting for MAVROS...")

        # Wait for services
        for client, name in [(self.arm_client, 'arming'), (self.mode_client, 'set_mode')]:
            if not client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f"Service /mavros/{name} unavailable.")
                return

        # Arm and set mode
        self.arm_and_mode(True, "GUIDED")

        # --- Path definition ---
        self.top_left =  {"lat": 41.35846806, "lon": 2.185427772, "depth": 3.75}
        self.top_right = {"lat": 41.358501, "lon": 2.1854, "depth": 3.75}
        self.bottom_left = {"lat": 41.35846806, "lon": 2.185427772, "depth": 5.15}
        self.bottom_right = {"lat": 41.358501, "lon": 2.1854, "depth": 5.15}

        self.depth_step = 0.1
        self.current_depth = self.top_left["depth"]
        self.max_depth = self.bottom_left["depth"]
        self.going_right = True

        self.timer = self.create_timer(2.0, self.run_pattern)

    # --- Callbacks ---
    def state_cb(self, msg: State):
        self.state = msg

    def gps_cb(self, msg: NavSatFix):
        self.global_fix = msg

    # --- Utilities ---
    def arm_and_mode(self, arm=True, mode="GUIDED"):
        # Set mode
        req = SetMode.Request()
        req.custom_mode = mode
        fut = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if fut.result() and fut.result().mode_sent:
            self.get_logger().info(f"Mode set to {mode}")

        # Arm
        req = CommandBool.Request()
        req.value = arm
        fut = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if fut.result() and fut.result().success:
            self.get_logger().info("Vehicle armed")

    def publish_target(self, lat, lon, depth):
        msg = GlobalPositionTarget()
        msg.header = Header(stamp=self.get_clock().now().to_msg())
        msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_INT
        msg.type_mask = int(0b110111111000)  # use position only
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = -depth  # positive down convention
        self.target_pub.publish(msg)

    # --- Pattern logic ---
    def run_pattern(self):
        if not self.global_fix:
            self.get_logger().warn("Waiting for GPS fix...")
            return

        if self.current_depth > self.max_depth:
            self.get_logger().info("Snake pattern complete!")
            self.destroy_timer(self.timer)
            return

        lon_start = self.top_left["lon"] if not self.going_right else self.top_right["lon"]
        lon_end = self.top_right["lon"] if not self.going_right else self.top_left["lon"]

        # Publish to both ends for demonstration
        self.get_logger().info(
            f"Depth {self.current_depth:.2f} m pass from {lon_start:.6f} → {lon_end:.6f}"
        )

        self.publish_target(self.top_left["lat"], lon_start, self.current_depth)
        time.sleep(3.0)
        self.publish_target(self.top_left["lat"], lon_end, self.current_depth)
        time.sleep(3.0)

        self.current_depth += self.depth_step
        self.going_right = not self.going_right


def main(args=None):
    rclpy.init(args=args)
    node = ROVSnakePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
