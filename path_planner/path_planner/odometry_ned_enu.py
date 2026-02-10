"""Converts odometry/filtered from NED to ENU and publish it to mavros/odometry/out and mavros/local_position/odom from ENU to NED"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import tf_transformations as tft
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, PoseWithCovarianceStamped, Vector3Stamped
import tf2_ros
import tf2_geometry_msgs

class NedToEnuOdom(Node):
    def __init__(self):
        super().__init__('odometry_ned_enu')

        self.declare_parameter("in_odom", "/odometry/filtered")
        self.declare_parameter("out_odom", "/odometry/filtered_enu")

        in_odom = self.get_parameter("in_odom").value
        out_odom = self.get_parameter("out_odom").value

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # TF2 setup
        self.pose_target_frame = "odom"
        self.twist_source_frame = "base_link_fsd"
        self.twist_target_frame = "base_link"

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(Odometry, in_odom, self.cb, qos)
        self.pub = self.create_publisher(Odometry, out_odom, 10)

        self.sub_de = self.create_subscription(Odometry, "/baro/odom", self.cb_de, qos)
        self.pub_de = self.create_publisher(Odometry, "/baro/odom_enu", 10)

        self.get_logger().info(f"Starting Conversion: {in_odom} (NED) -> {out_odom} (ENU)")

    def cb(self, msg: Odometry):

        odom_in = Odometry()
        odom_in.header = msg.header
        odom_in.pose = msg.pose
        odom_in.twist = msg.twist
        odom_in.child_frame_id = msg.child_frame_id
        
        pose_in = PoseWithCovarianceStamped()
        pose_in.pose = odom_in.pose
        pose_in.header = odom_in.header
        
        twist_in = odom_in.twist
        
        try:
            # Lookup transform target <- source at the message time
            t_frame = self.tf_buffer.lookup_transform(
                self.pose_target_frame,
                odom_in.header.frame_id,
                rclpy.time.Time.from_msg(odom_in.header.stamp),
                timeout=Duration(seconds=0.1),
            )

            pose_out = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(pose_in, t_frame)
            twist_out = self.twist_transform(odom_in.header.stamp, twist_in)
            
            odom_out = Odometry()
            odom_out.header = odom_in.header
            odom_out.header.frame_id = self.pose_target_frame
            odom_out.child_frame_id = self.twist_target_frame
            odom_out.pose = pose_out.pose
            odom_out.twist = twist_out

            self.pub.publish(odom_out)
    
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warn(
                f"TF transform {pose_in.header.frame_id} -> {self.pose_target_frame} unavailable: {ex}"
            )
            return
        
        
    def cb_de(self, msg: Odometry):
        odom_in = Odometry()
        odom_in.header = msg.header
        odom_in.pose = msg.pose
        odom_in.twist = msg.twist
        odom_in.child_frame_id = msg.child_frame_id
        
        pose_in = PoseWithCovarianceStamped()
        pose_in.pose = odom_in.pose
        pose_in.header = odom_in.header
        
        twist_in = odom_in.twist
        
        try:
            # Lookup transform target <- source at the message time
            t_frame = self.tf_buffer.lookup_transform(
                self.pose_target_frame,
                odom_in.header.frame_id,
                rclpy.time.Time.from_msg(odom_in.header.stamp),
                timeout=Duration(seconds=0.1),
            )

            pose_out = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(pose_in, t_frame)
            twist_out = self.twist_transform(odom_in.header.stamp, twist_in)
            
            odom_out = Odometry()
            odom_out.header = odom_in.header
            odom_out.header.frame_id = self.pose_target_frame
            odom_out.child_frame_id = self.twist_target_frame
            odom_out.pose = pose_out.pose
            odom_out.twist = twist_out

            self.pub_de.publish(odom_out)
    
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warn(
                f"TF transform {pose_in.header.frame_id} -> {self.pose_target_frame} unavailable: {ex}"
            )
            return
    
    def twist_transform(self, msg_stamp, twist:TwistWithCovariance):
        """
        Rotate twist vectors from base_link_frd to base_link using TF.

        """


        v_in = Vector3Stamped()
        v_in.header.stamp = msg_stamp
        v_in.header.frame_id = self.twist_source_frame
        v_in.vector = twist.twist.linear

        w_in = Vector3Stamped()
        w_in.header.stamp = msg_stamp
        w_in.header.frame_id = self.twist_source_frame
        w_in.vector = twist.twist.angular

        t = self.tf_buffer.lookup_transform(
            self.twist_target_frame,
            self.twist_source_frame,
            rclpy.time.Time.from_msg(msg_stamp),
            timeout=Duration(seconds=0.1),
        )
        covariance_in = PoseWithCovarianceStamped()
        covariance_in.pose.covariance = twist.covariance
        covariance_in.header.frame_id = self.twist_source_frame

        v_out = tf2_geometry_msgs.do_transform_vector3(v_in, t)
        w_out = tf2_geometry_msgs.do_transform_vector3(w_in, t)
        cov_out = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(covariance_in, t)

        twist_out = TwistWithCovariance()
        twist_out.twist.linear = v_out.vector
        twist_out.twist.angular = w_out.vector
        twist_out.covariance = cov_out.pose.covariance

        return twist_out
    
        
def main():
    rclpy.init()
    node = NedToEnuOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()