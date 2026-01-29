#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration
import numpy as np
from sensor_msgs_py import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import tf_transformations as tft # make sure `python3 -m pip install tf-transformations` in the image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class BlueROVOdometryToNED(Node):
    def __init__(self):
        super().__init__('bluerov2_ned')

        # -----------------
        # POINT CLOUD → MAP
        # -----------------
        self.point_cloud_target_frame = 'odom'  # global frame you want
        qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT
            )
        # Subscribe to camera point cloud (in realsense_d455_link / depth frame)
        self.sub_cloud = self.create_subscription(
            PointCloud2,
            '/camera_d455/depth/image_raw/points',
            self.cloud_callback,
            qos
        )

        # Publish transformed cloud in map frame
        self.pub_cloud_map = self.create_publisher(
            PointCloud2,
            '/stereo/point_cloud',
            10
        )

        self.get_logger().info(
            f"BlueROVOdometryToNED running. "
            f"Transforming /camera_d455/depth/image_raw/points -> {self.point_cloud_target_frame} "
            f"on /camera_d455/depth/image_raw/points_map"
        )

        self.tf_buffer = None
        self.tf_listener = None
        # delay listener
        self.create_timer(0.1, self.init_tf)
        self.count = 0


    def init_tf(self):
        if self.tf_listener is None:
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT
            )

            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(
                self.tf_buffer,
                self,
                qos=qos
            )
            self.get_logger().info("TF listener initialized (BEST_EFFORT)")
    # ------------- ODOM HELPERS -------------



    # ------------- POINT CLOUD CALLBACK -------------

    def cloud_callback(self, msg: PointCloud2):
        self.count += 1
        
        self.get_logger().info(f"PointCloud2 callback received #{self.count}")
        source_frame = msg.header.frame_id  # e.g. "realsense_d455_link"
        pc_time = rclpy.time.Time.from_msg(msg.header.stamp)
        now = self.get_clock().now()
        
        if self.tf_buffer is None:
            #self.get_logger().warn("TF not ready yet")
            return
        
        try:
            # Prefer transform at the cloud timestamp
            transform = self.tf_buffer.lookup_transform(
                self.point_cloud_target_frame,
                source_frame,
                pc_time,
                timeout=Duration(seconds=0.2),
            )
            #self.get_logger().warn(f"########################################################\n\nActually found TF at stamp for {self.target_frame} <- {source_frame} at time {pc_time.nanoseconds/1e9:.6f}")

        except (ExtrapolationException, LookupException, ConnectivityException) as e:
            #self.get_logger().warn(
            #    f"TF transform {self.point_cloud_target_frame} <- {source_frame} failed: {e}"
            #)
            return

        DOWNSAMPLE_STRIDE = 10  # keep 1 out of every 10 points

        points_xyz = pc2.read_points_numpy(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        )


        if points_xyz.size == 0:
            #self.get_logger().info("Point cloud has no valid points")
            return

        # --------------------------
        # 2) Limit range to 5 meters
        # --------------------------
        MAX_RANGE = 5.0  # meters
        max_range_sq = MAX_RANGE * MAX_RANGE

        # squared distance from sensor origin
        dist_sq = np.einsum('ij,ij->i', points_xyz, points_xyz)
        points_xyz = points_xyz[dist_sq <= max_range_sq]

        if points_xyz.shape[0] == 0:
            #self.get_logger().info("Point cloud has no valid points2")
            return

        points_xyz = points_xyz[::DOWNSAMPLE_STRIDE]

        if points_xyz.shape[0] == 0:
            #self.get_logger().info("Point cloud has no valid points3")
            return

        # 3) Build 4x4 transform matrix from TransformStamped
        t = transform.transform.translation
        q = transform.transform.rotation

        # quaternion: (x, y, z, w)
        T = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0, 3] = t.x
        T[1, 3] = t.y
        T[2, 3] = t.z

        # 4) Apply transform to all points
        n = points_xyz.shape[0]
        homog = np.ones((n, 4), dtype=np.float32)
        homog[:, :3] = points_xyz.astype(np.float32)

        transformed = (T @ homog.T).T[:, :3]  # shape (N, 3)

        # 5) Create a new PointCloud2 in map frame, XYZ only
        header = msg.header
        header.frame_id = self.point_cloud_target_frame  # "odom"

        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]

        # pc2.create_cloud() expects an iterable of point tuples or a Nx3 array
        points_list = transformed.tolist()
        cloud_map = pc2.create_cloud(header, fields, points_list)

        # 6) Publish
        self.pub_cloud_map.publish(cloud_map)



def main(args=None):
    rclpy.init(args=args)
    node = BlueROVOdometryToNED()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()