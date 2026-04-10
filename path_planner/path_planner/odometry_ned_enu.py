"""Converts odometry/filtered from NED to ENU and publish it to mavros/odometry/out and mavros/local_position/odom from ENU to NED"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from geometry_msgs.msg import TwistWithCovariance, PoseWithCovarianceStamped, Vector3Stamped, PoseWithCovariance
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from tf_transformations import quaternion_matrix, quaternion_multiply

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
            # Transform odom_ned frame into odom
            t_world = self.tf_buffer.lookup_transform(
                self.pose_target_frame,
                odom_in.header.frame_id,
                rclpy.time.Time.from_msg(odom_in.header.stamp),
                timeout=Duration(seconds=0.1),
            )

            pose_world_old_child = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(
                pose_in, t_world
            )

            # Transform child frame from original child_frame_id to base_link
            t_old_new = self.tf_buffer.lookup_transform(
                odom_in.child_frame_id,          # old child frame
                self.twist_target_frame,         # new child frame
                rclpy.time.Time.from_msg(odom_in.header.stamp),
                timeout=Duration(seconds=0.1),
            )

            pose_world_new_child = self.convert_pose_child_frame(
                pose_world_old_child.pose,
                t_old_new
            )

            # Transform twist into base_link
            twist_out = self.twist_transform(odom_in.header.stamp, twist_in)

            odom_out = Odometry()
            odom_out.header = odom_in.header
            odom_out.header.frame_id = self.pose_target_frame
            odom_out.child_frame_id = self.twist_target_frame
            odom_out.pose = pose_world_new_child
            odom_out.twist = twist_out

            self.pub.publish(odom_out)

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warn(f"TF transform failed: {ex}")
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
    
    def convert_pose_child_frame(self, pose_old_child: PoseWithCovariance, t_old_new):
        """
        Convert pose of old_child in world frame into pose of new_child in same world frame.

        pose_old_child: pose of old child frame in world
        t_old_new: TF transform old_child <- new_child
        """

        p_world_old = np.array([
            pose_old_child.pose.position.x,
            pose_old_child.pose.position.y,
            pose_old_child.pose.position.z,
        ])

        q_world_old = np.array([
            pose_old_child.pose.orientation.x,
            pose_old_child.pose.orientation.y,
            pose_old_child.pose.orientation.z,
            pose_old_child.pose.orientation.w,
        ])

        p_old_new = np.array([
            t_old_new.transform.translation.x,
            t_old_new.transform.translation.y,
            t_old_new.transform.translation.z,
        ])

        q_old_new = np.array([
            t_old_new.transform.rotation.x,
            t_old_new.transform.rotation.y,
            t_old_new.transform.rotation.z,
            t_old_new.transform.rotation.w,
        ])

        R_world_old = quaternion_matrix(q_world_old)[:3, :3]

        # T_world_new = T_world_old * T_old_new
        p_world_new = p_world_old + R_world_old @ p_old_new
        q_world_new = quaternion_multiply(q_world_old, q_old_new)

        out = PoseWithCovariance()
        out.pose.position.x = float(p_world_new[0])
        out.pose.position.y = float(p_world_new[1])
        out.pose.position.z = float(p_world_new[2])

        out.pose.orientation.x = float(q_world_new[0])
        out.pose.orientation.y = float(q_world_new[1])
        out.pose.orientation.z = float(q_world_new[2])
        out.pose.orientation.w = float(q_world_new[3])

        out.covariance = pose_old_child.covariance
        return out

    def twist_transform(self, msg_stamp, twist:TwistWithCovariance):
        """
        Rotate twist vectors from base_link_fsd to base_link using TF.

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