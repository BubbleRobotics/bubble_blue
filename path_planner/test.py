#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped


class ThrustCommander(Node):
    def __init__(self):
        super().__init__('thrust_commander')

        # Publisher to the follower node
        self.pub = self.create_publisher(Vector3Stamped,
                                         '/follower/cmd_thrust',
                                         10)

        # Timer: send thrust at 10 Hz
        self.timer = self.create_timer(0.1, self.send_thrust)

    def send_thrust(self):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"   # follower standardizes to 'map'

        # === Example thrust command (m/s^2) ===
        msg.vector.x = -20.0    # Forward acceleration
        msg.vector.y = 0.0     # Lateral acceleration
        msg.vector.z = 0.0     # Upward thrust (positive is UP in LOCAL_NED)

        # Publish
        self.pub.publish(msg)
        self.get_logger().info(f"Sent thrust: {msg.vector}")


def main(args=None):
    rclpy.init(args=args)
    node = ThrustCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
