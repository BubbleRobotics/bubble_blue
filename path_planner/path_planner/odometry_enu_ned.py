"""Converts odometry/filtered from NED to ENU and publish it to mavros/odometry/out and mavros/local_position/odom from ENU to NED"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import tf_transformations as tft
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class NedToEnuOdom(Node):
    def __init__(self):
        super().__init__('ned_to_enu_odom')

        self.declare_parameter("in_odom", "/odometry/filtered")
        self.declare_parameter("out_odom", "/mavros/odometry/out")

        in_odom = self.get_parameter("in_odom").value
        out_odom = self.get_parameter("out_odom").value

        self.sub = self.create_subscription(Odometry, in_odom, self.cb, 10)
        self.pub = self.create_publisher(Odometry, out_odom, 10)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub_mav = self.create_subscription(
            Odometry,
            "/mavros/local_position/odom",
            self.cb_mav,
            qos
        )
        self.pub_mav = self.create_publisher(Odometry, "/mavros/local_position/odom_ned", 10)


        # NED -> ENU axis swap/flip for vectors: [xE,yN,zU]^T = M * [xN,yE,zD]^T
        self.M = np.array([[0, 1, 0],
                           [1, 0, 0],
                           [0, 0,-1]])

        self.get_logger().info(f"Converting {in_odom} (NED) -> {out_odom} (ENU)")

    def cb(self, msg: Odometry):
        out = Odometry()

        # Header / frames
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "map"
        out.child_frame_id = "base_link"

        # Position (NED -> ENU): x<->y, z flips sign
        p = msg.pose.pose.position
        out.pose.pose.position.x = p.y
        out.pose.pose.position.y = p.x
        out.pose.pose.position.z = -p.z

        # Velocity (NED -> ENU): x<->y, z flips sign
        v = msg.twist.twist.linear
        out.twist.twist.linear.x = v.y
        out.twist.twist.linear.y = v.x
        out.twist.twist.linear.z = -v.z

        # Angular velocity: also needs frame conversion (NED -> ENU)
        w = msg.twist.twist.angular
        out.twist.twist.angular.x = w.y
        out.twist.twist.angular.y = w.x
        out.twist.twist.angular.z = -w.z

        # Orientation (NED -> ENU)
        # Convert rotation matrix by: R_enu = M * R_ned * M^T
        q = msg.pose.pose.orientation
        q_ned = [q.x, q.y, q.z, q.w]
        R_ned_4 = tft.quaternion_matrix(q_ned)
        R_ned = R_ned_4[:3, :3]

        R_enu = self.M @ R_ned @ self.M.T

        R_enu_4 = np.eye(4)
        R_enu_4[:3, :3] = R_enu
        q_enu = tft.quaternion_from_matrix(R_enu_4)

        out.pose.pose.orientation.x = q_enu[0]
        out.pose.pose.orientation.y = q_enu[1]
        out.pose.pose.orientation.z = q_enu[2]
        out.pose.pose.orientation.w = q_enu[3]

        # Covariances
        out.pose.covariance = msg.pose.covariance
        out.twist.covariance = msg.twist.covariance

        self.pub.publish(out)

    def cb_mav(self, msg: Odometry):
        out = Odometry()

        # Header
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "map_ned"
        out.child_frame_id = "base_link_frd"  # or base_link if you prefer

        # ------------------
        # Position ENU -> NED
        # ------------------
        p = msg.pose.pose.position
        out.pose.pose.position.x = p.y
        out.pose.pose.position.y = p.x
        out.pose.pose.position.z = -p.z

        # ------------------
        # Linear velocity ENU -> NED
        # ------------------
        v = msg.twist.twist.linear
        out.twist.twist.linear.x = v.y
        out.twist.twist.linear.y = v.x
        out.twist.twist.linear.z = -v.z

        # ------------------
        # Angular velocity ENU -> NED
        # ------------------
        w = msg.twist.twist.angular
        out.twist.twist.angular.x = w.y
        out.twist.twist.angular.y = w.x
        out.twist.twist.angular.z = -w.z

        # ------------------
        # Orientation ENU -> NED
        # R_ned = M * R_enu * M^T
        # ------------------
        q = msg.pose.pose.orientation
        q_enu = [q.x, q.y, q.z, q.w]

        R_enu_4 = tft.quaternion_matrix(q_enu)
        R_enu = R_enu_4[:3, :3]

        R_ned = self.M @ R_enu @ self.M.T

        R_ned_4 = np.eye(4)
        R_ned_4[:3, :3] = R_ned

        q_ned = tft.quaternion_from_matrix(R_ned_4)

        out.pose.pose.orientation.x = q_ned[0]
        out.pose.pose.orientation.y = q_ned[1]
        out.pose.pose.orientation.z = q_ned[2]
        out.pose.pose.orientation.w = q_ned[3]

        # ------------------
        # Covariances
        # ------------------
        out.pose.covariance = msg.pose.covariance
        out.twist.covariance = msg.twist.covariance

        self.pub_mav.publish(out)
        
def main():
    rclpy.init()
    node = NedToEnuOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
