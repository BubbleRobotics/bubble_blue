"""
Converts odometry/filtered (NED source) to a TF-based map->base_link_fsd odom for controller,
while forwarding the original message unchanged to mavros/odometry/out.
Also republishes mavros/local_position/odom (unchanged by default).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from rclpy.time import Time

import numpy as np
import tf_transformations as tft

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped

import tf2_ros
from tf2_ros import TransformException


class NedToEnuOdom(Node):
    def __init__(self):
        super().__init__('ned_to_enu_odom_sim')

        self.declare_parameter("in_odom", "/odometry/filtered")
        self.declare_parameter("out_odom", "/mavros/odometry/out")

        in_odom = self.get_parameter("in_odom").value
        out_odom = self.get_parameter("out_odom").value

        # Subscriber QoS (sensor-ish)
        sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Publisher QoS (reliable)
        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(Odometry, in_odom, self.cb, sub_qos)

        # ORIGINAL message unchanged goes here
        self.pub = self.create_publisher(Odometry, out_odom, pub_qos)

        # TF-converted map->base_link_fsd message goes here
        self.pub_controller = self.create_publisher(
            Odometry, "/integral_sliding_mode_controller/system_state", pub_qos
        )

        # MAVROS odom passthrough (unchanged by default)
        self.sub_mav = self.create_subscription(
            Odometry, "/mavros/local_position/odom", self.cb_mav, sub_qos
        )
        self.pub_mav = self.create_publisher(
            Odometry, "/mavros/local_position/odom_odom_frame", sub_qos
        )

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(f"Forwarding original: {in_odom} -> {out_odom}")
        self.get_logger().info("Publishing TF-transformed odom to /integral_sliding_mode_controller/system_state")

    # ----------------- small helpers -----------------

    def _tf_time_from_header(self, stamp_msg):
        if stamp_msg.sec == 0 and stamp_msg.nanosec == 0:
            return Time()  # latest
        return Time.from_msg(stamp_msg)

    def _apply_transform_to_pose(self, pose_in: PoseStamped, tf_msg: TransformStamped) -> PoseStamped:
        """pose_out = TF(target<-source) * pose_in"""
        out = PoseStamped()
        out.header.stamp = pose_in.header.stamp
        out.header.frame_id = tf_msg.header.frame_id  # target frame

        # Transform matrix
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        T = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0, 3] = t.x
        T[1, 3] = t.y
        T[2, 3] = t.z

        # Position
        p = pose_in.pose.position
        p_out = T @ np.array([p.x, p.y, p.z, 1.0])
        out.pose.position.x = float(p_out[0])
        out.pose.position.y = float(p_out[1])
        out.pose.position.z = float(p_out[2])

        # Orientation: q_out = q_T * q_in
        qi = pose_in.pose.orientation
        q_in = [qi.x, qi.y, qi.z, qi.w]
        q_T = [q.x, q.y, q.z, q.w]
        q_out = tft.quaternion_multiply(q_T, q_in)

        out.pose.orientation.x = float(q_out[0])
        out.pose.orientation.y = float(q_out[1])
        out.pose.orientation.z = float(q_out[2])
        out.pose.orientation.w = float(q_out[3])

        return out

    def _rotate_vector_by_quat(self, x, y, z, qx, qy, qz, qw):
        R = tft.quaternion_matrix([qx, qy, qz, qw])
        vx, vy, vz, _ = (R @ [x, y, z, 0.0])
        return float(vx), float(vy), float(vz)

    def _transform_twist(self, twist_in: TwistStamped, tf_msg: TransformStamped) -> TwistStamped:
        """
        Rotate linear/angular vectors using TF rotation.
        (No translational velocity terms; correct for pure frame rotation use.)
        """
        q = tf_msg.transform.rotation

        out = TwistStamped()
        out.header.stamp = twist_in.header.stamp
        out.header.frame_id = tf_msg.header.frame_id  # target frame

        lx = twist_in.twist.linear.x
        ly = twist_in.twist.linear.y
        lz = twist_in.twist.linear.z
        ax = twist_in.twist.angular.x
        ay = twist_in.twist.angular.y
        az = twist_in.twist.angular.z

        out.twist.linear.x, out.twist.linear.y, out.twist.linear.z = self._rotate_vector_by_quat(
            lx, ly, lz, q.x, q.y, q.z, q.w
        )
        out.twist.angular.x, out.twist.angular.y, out.twist.angular.z = self._rotate_vector_by_quat(
            ax, ay, az, q.x, q.y, q.z, q.w
        )
        return out

    # ----------------- callbacks -----------------

    def cb(self, msg: Odometry):
        # A) publish original unchanged
        self.pub.publish(msg)

        # B) publish TF-computed map -> base_link_fsd
        target_parent = "map"
        target_child = "base_link_fsd"

        tf_time = self._tf_time_from_header(msg.header.stamp)

        # Pose in source frame (msg.header.frame_id)
        pose_in = PoseStamped()
        pose_in.header = msg.header
        pose_in.pose = msg.pose.pose

        try:
            tf_pose = self.tf_buffer.lookup_transform(
                target_parent,              # target
                pose_in.header.frame_id,    # source
                tf_time,
                timeout=Duration(seconds=0.02)
            )
            pose_out = self._apply_transform_to_pose(pose_in, tf_pose)
        except TransformException as ex:
            self.get_logger().warn(f"Pose TF failed ({pose_in.header.frame_id} -> {target_parent}): {ex}")
            return

        # Twist is usually expressed in child frame
        twist_source_frame = msg.child_frame_id if msg.child_frame_id else msg.header.frame_id

        twist_in = TwistStamped()
        twist_in.header.stamp = msg.header.stamp
        twist_in.header.frame_id = twist_source_frame
        twist_in.twist = msg.twist.twist

        try:
            tf_twist = self.tf_buffer.lookup_transform(
                target_child,               # target
                twist_in.header.frame_id,   # source
                tf_time,
                timeout=Duration(seconds=0.02)
            )
            twist_out = self._transform_twist(twist_in, tf_twist)
        except TransformException as ex:
            self.get_logger().warn(f"Twist TF failed ({twist_in.header.frame_id} -> {target_child}): {ex}")
            return

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = target_parent
        out.child_frame_id = target_child

        out.pose.pose = pose_out.pose
        out.twist.twist = twist_out.twist

        # keep incoming covariances
        out.pose.covariance = msg.pose.covariance
        out.twist.covariance = msg.twist.covariance

        self.pub_controller.publish(out)

    def cb_mav(self, msg: Odometry):
        # Publish unchanged by default (NO aliasing / mutation)
        self.pub_mav.publish(msg)


def main():
    rclpy.init()
    node = NedToEnuOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
