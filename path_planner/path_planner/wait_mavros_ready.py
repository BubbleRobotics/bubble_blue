# path_planner/wait_mavros_ready.py
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped  # or use nav_msgs/Odometry if that's what you have

class WaitMavrosReady(Node):
    def __init__(self):
        super().__init__('wait_mavros_ready')
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.state_ok = False
        self.pose_seen = False

        self.state_sub = self.create_subscription(State, '/mavros/state', self._state_cb, 10)
        # If you use /mavros/local_position/odom instead, change type & topic accordingly
        self.pose_sub  = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._pose_cb, qos)

        # safety timeout (optional): exit nonzero if not ready after N seconds
        self.declare_parameter('timeout_sec', 60.0)
        timeout = float(self.get_parameter('timeout_sec').value)
        self.create_timer(0.2, self._check)
        self._deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=timeout)

    def _state_cb(self, msg: State):
        # “connected” just means MAVROS ↔ FCU OK; we still wait for position
        self.state_ok = bool(msg.connected)

    def _pose_cb(self, _msg: PoseStamped):
        self.pose_seen = True

    def _check(self):
        if self.state_ok and self.pose_seen:
            self.get_logger().info('MAVROS ready: connected and position available.')
            rclpy.shutdown()
            sys.exit(0)
        if self.get_clock().now() > self._deadline:
            self.get_logger().error('Timeout waiting for MAVROS readiness.')
            rclpy.shutdown()
            sys.exit(2)

def main():
    rclpy.init()
    rclpy.spin(WaitMavrosReady())

if __name__ == '__main__':
    main()
