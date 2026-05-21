#!/usr/bin/env python3
"""
NOTE: this node should be launched via:
ros2 launch path_planner optimized_trajectory.launch.py

ROS 2 node that orchestrates automated repeated evaluation runs comparing EGO-Planner,
SCP Optimized, and Hierarchical trajectory following across four disturbance conditions.

For each run the node:
  1. Loads and rebases the optimized trajectory CSV to the local odom frame.
  2. Pauses the simulation, teleports the robot to the trajectory start pose,
     and reinitialises the state estimator.
  3. Starts a ROS 2 bag recording, then triggers the appropriate control mode:
       - EGO:          publishes the final waypoint to /ego_planner/move_base_simple/goal
       - Optimized:    publishes the full OptimizedTrajectory to planning/optimized_trajectory
       - Hierarchical: periodically feeds velocity-annotated waypoints to EGO-Planner
                       using a lookahead of 5 s along the optimized trajectory
  4. Optionally activates known-current or disturbance-current plugins.
  5. On goal reached, stops the bag and automatically advances to the next
     planner type or disturbance condition.

Evaluation cycles through 4 conditions (no current, disturbance, known current,
known current + disturbance) × 3 planners (EGO, Optimized, Hierarchical),
running nr_per_process repetitions each.

Services exposed:
  /start_test    (std_srvs/Trigger)   starts or advances the evaluation sequence

Parameters:
  test_ego                   (default: False)
  use_known_currents         (default: False)
  use_disturbance_currents   (default: False)
  ego_waypoint_update_period (default: 1 s)   period for hierarchical waypoint updates
"""

import math
import subprocess
import time
from pathlib import Path

import pandas as pd
import rclpy
from geometry_msgs.msg import Point, Vector3, Twist, PoseStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf_transformations import quaternion_from_euler
from traj_utils.msg import OptimizedTrajectory, TrajectorySample
from nav_msgs.msg import Odometry
from traj_utils.srv import VelAccCmd
class StartTestService(Node):
    def __init__(self):
        super().__init__("optimal_trajectory")
        
        self.declare_parameter("test_ego", False)
        self.test_ego = self.get_parameter("test_ego").value
        
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_z = 0.0

        self.nr_ego_done = 0
        self.nr_opt_done = 0
        self.nr_combined_done = 0
        self.nr_per_process = 1

        self.declare_parameter("use_known_currents", False)
        self.use_known_currents = self.get_parameter("use_known_currents").value #use_current_disturbances
        self.declare_parameter("use_disturbance_currents", False)
        self.use_disturbance_currents = self.get_parameter("use_disturbance_currents").value

        # Sampling period for sending updated EGO waypoints from the optimized trajectory
        self.declare_parameter("ego_waypoint_update_period", 1)
        self.ego_waypoint_update_period = float(
            self.get_parameter("ego_waypoint_update_period").value
        )

        self.evalation_test = True
        # First case: No Currents & EGO
        self.evaluation_test_case = 1
        self.test_ego = True
        self.test_opt = False
        self.use_known_currents = False
        self.use_disturbance_currents = False

        self.bag_process = None

        self.bag_topics = [
            "/clock",
            "/odometry/filtered",
            "/odometry/filtered_enu",
            "/model/bluerov2_heavy/odometry",
            "/current_disturbance/current",
            "/position_cmd",
            "/adaptive_integral_terminal_sliding_mode_controller/reference",
            "/adaptive_integral_terminal_sliding_mode_controller/status",
            "/thruster_1_controller/status",
            "/thruster_2_controller/status",
            "/thruster_3_controller/status",
            "/thruster_4_controller/status",
            "/thruster_5_controller/status",
            "/thruster_6_controller/status",
            "/thruster_7_controller/status",
            "/thruster_8_controller/status",
        ]

        self.bag_output_dir = "/home/ubuntu/ws_blue/evaluation/data"

        self.service_group = MutuallyExclusiveCallbackGroup()
        self.client_group = ReentrantCallbackGroup()

        self.trajectory_name_without_current = "Without_Current_MT" # For test case 0 and 1
        self.trajectory_name_with_current = "With_Current_MT"
        self.trajectory_name = self.trajectory_name_without_current
        # self.trajectory_name = "With_Current_Heavy" # for test case 2 and 3
        self.csv_file_path = Path(
            f"/home/ubuntu/ws_blue/src/blue/path_planner/optimized_trajectories/{self.trajectory_name}.csv"
        )

        # State for timer-driven EGO waypoint updates
        self.ego_waypoint_timer = None
        self.active_optimized_trajectory = None
        self.next_waypoint_index = 0

        # Compensate for estimator initialization bias.
        # If the estimator starts at +90 deg, rotate the commanded trajectory by -90 deg.
        self.init_yaw_bias_rad = -math.pi / 2.0

        # World pose where the robot should be placed in Gazebo before each run
        self.start_world_x = -4.0
        self.start_world_y = 0.0
        self.start_world_z = 0.0
        self.start_world_qx = 0.0
        self.start_world_qy = 0.0
        self.start_world_qz = 0.0
        self.start_world_qw = 1.0

        self.goal_x = 999
        self.goal_y = 999
        self.goal_z = 999

        self.pub = self.create_publisher(
            OptimizedTrajectory,
            "planning/optimized_trajectory",
            10,
        )

        self.cmd_state_pub = self.create_publisher(
            String,
            "controller_state",
            10,
        )

        self.initialize_client = self.create_client(
            Trigger,
            "/initialize",
            callback_group=self.client_group,
        )

        self.start_known_currents_client = self.create_client(
            Trigger,
            "/current_disturbances/start_known_currents",
            callback_group=self.client_group,
        )

        self.end_known_currents_client = self.create_client(
            Trigger,
            "/current_disturbances/end_known_currents",
            callback_group=self.client_group,
        )

        self.start_disturbance_currents_client = self.create_client(
            Trigger,
            "/current_disturbances/start_disturbance_currents",
            callback_group=self.client_group,
        )

        self.end_disturbance_currents_client = self.create_client(
            Trigger,
            "/current_disturbances/end_disturbance_currents",
            callback_group=self.client_group,
        )

        self.srv = self.create_service(
            Trigger,
            "/start_test",
            self.handle_start_test,
            callback_group=self.service_group,
        )

        self.vel_reference_pub = self.create_publisher(
            Twist,
            "/adaptive_integral_terminal_sliding_mode_controller/reference",
            10
        )

        self.waypoint_pub = self.create_publisher(
            PoseStamped,
            "/ego_planner/move_base_simple/goal",
            10
        )

        self.velocity_waypoint_pub = self.create_publisher(
            Odometry,
            "/ego_planner/move_base_simple/goal_with_velocity",
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odometry/filtered_enu",
            self.odom_callback,
            10
        )

        self.reset_ego_fsm_client = self.create_client(
            Trigger,
            "/ego_planner/reset_ego_replan_fsm",
            callback_group=self.client_group,
        )
        self.reset_traj_server_client = self.create_client(
            Trigger,
            "/ego_traj_server/reset_traj_server",
            callback_group=self.client_group,
        )

        self.reset_odometry_viz_client = self.create_client(
            Trigger,
            "/odometry_visualization/reset_odometry_visualization",
            callback_group=self.client_group,
        )

        self.set_vel_acc_client = self.create_client(
            VelAccCmd,
            "ego_planner/set_vel_acc_cmd",
            callback_group=self.client_group
        )

        self.get_logger().info("Service /start_test is ready.")

    @staticmethod
    def wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def call_command(self, cmd, label):
        self.get_logger().info(f"{label}...")
        try:
            result = subprocess.run(
                cmd,
                check=True,
                capture_output=True,
                text=True,
            )

            stdout = result.stdout.strip()
            stderr = result.stderr.strip()

            if stdout:
                self.get_logger().info(stdout)
            if stderr:
                self.get_logger().warn(stderr)

            return True, stdout

        except subprocess.CalledProcessError as e:
            stdout = e.stdout.strip() if e.stdout else ""
            stderr = e.stderr.strip() if e.stderr else ""
            self.get_logger().error(f"{label} failed.")
            if stdout:
                self.get_logger().error(stdout)
            if stderr:
                self.get_logger().error(stderr)
            return False, stderr or stdout or "unknown error"

    def start_bag_recording(self):
        if self.bag_process is not None and self.bag_process.poll() is None:
            self.get_logger().warn("ros2 bag recording is already running.")
            return True

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        if self.test_ego:
            bag_name = f"{self.bag_output_dir}/ego_bag_{self.trajectory_name}_current_known_{self.use_known_currents}_dist_{self.use_disturbance_currents}_{timestamp}"
        elif self.test_opt:
            bag_name = f"{self.bag_output_dir}/opt_bag_{self.trajectory_name}_current_known_{self.use_known_currents}_dist_{self.use_disturbance_currents}_{timestamp}"
        else:
            bag_name = f"{self.bag_output_dir}/comb_bag_{self.trajectory_name}_current_known_{self.use_known_currents}_dist_{self.use_disturbance_currents}_{timestamp}"

        cmd = [
            "ros2", "bag", "record",
            "-o", bag_name,
            *self.bag_topics,
        ]

        self.get_logger().info(f"Starting ros2 bag recording: {' '.join(cmd)}")

        try:
            self.bag_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to start ros2 bag recording: {e}")
            self.bag_process = None
            return False
    
    def stop_bag_recording(self):
        if self.bag_process is None:
            return

        if self.bag_process.poll() is None:
            self.get_logger().info("Stopping ros2 bag recording...")
            self.bag_process.terminate()

            try:
                self.bag_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.get_logger().warn("ros2 bag did not stop gracefully, killing it.")
                self.bag_process.kill()

        self.bag_process = None

    def wait_for_service_client(self, client, service_name: str, timeout_sec=5.0):
        self.get_logger().info(f"Waiting for {service_name} service...")
        available = client.wait_for_service(timeout_sec=timeout_sec)
        if not available:
            self.get_logger().error(f"{service_name} service not available.")
            return False
        return True

    def call_trigger_client(self, client, service_name: str, timeout_sec=10.0):
        request = Trigger.Request()
        future = client.call_async(request)

        start_time = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start_time > timeout_sec:
                self.get_logger().error(f"{service_name} service call timed out.")
                return False, "timeout"
            time.sleep(0.05)

        if not future.done():
            self.get_logger().error(f"{service_name} future did not complete.")
            return False, "future did not complete"

        response = future.result()
        if response is None:
            self.get_logger().error(f"Failed to call {service_name}.")
            return False, "future.result() is None"

        if response.success:
            self.get_logger().info(f"{service_name} succeeded: {response.message}")
            return True, response.message

        self.get_logger().error(f"{service_name} failed: {response.message}")
        return False, response.message

    def wait_for_initialize_service(self, timeout_sec=5.0):
        self.get_logger().info("Waiting for /initialize service...")
        available = self.initialize_client.wait_for_service(timeout_sec=timeout_sec)
        if not available:
            self.get_logger().error("/initialize service not available.")
            return False
        return True

    def call_initialize_service(self, timeout_sec=10.0):
        request = Trigger.Request()
        future = self.initialize_client.call_async(request)

        start_time = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start_time > timeout_sec:
                self.get_logger().error("/initialize service call timed out.")
                return False, "timeout"
            time.sleep(0.05)

        if not future.done():
            self.get_logger().error("/initialize future did not complete.")
            return False, "future did not complete"

        response = future.result()
        if response is None:
            self.get_logger().error("Failed to call /initialize.")
            return False, "future.result() is None"

        if response.success:
            self.get_logger().info(f"/initialize succeeded: {response.message}")
            return True, response.message

        self.get_logger().error(f"/initialize failed: {response.message}")
        return False, response.message

    def load_trajectory_dataframe(self) -> pd.DataFrame:
        if not self.csv_file_path.exists():
            raise FileNotFoundError(f"CSV file not found: {self.csv_file_path}")

        df = pd.read_csv(self.csv_file_path)

        required_cols = ["t", "x", "y", "z", "yaw", "u", "v", "w", "r"]
        missing = [c for c in required_cols if c not in df.columns]
        if missing:
            raise ValueError(f"Missing required columns: {missing}")

        return df

    def rebase_trajectory_to_origin(self, df: pd.DataFrame) -> pd.DataFrame:
        """
        Rebase trajectory so the first sample becomes:
        position = (0, 0, 0)
        yaw = 0, plus optional fixed compensation

        XY positions are translated and rotated by:
            -(yaw0 + init_yaw_bias_rad)

        Z is translated only.
        Body-frame velocities (u,v,w) and yaw rate (r) are left unchanged.
        """
        rebased = df.copy()

        x0 = float(df.iloc[0]["x"])
        y0 = float(df.iloc[0]["y"])
        z0 = float(df.iloc[0]["z"])
        yaw0 = float(df.iloc[0]["yaw"])

        q = quaternion_from_euler(0.0, 0.0, yaw0)

        self.start_world_x = x0
        self.start_world_y = y0
        self.start_world_z = z0
        self.start_world_qx = q[0]
        self.start_world_qy = q[1]
        self.start_world_qz = q[2]
        self.start_world_qw = q[3]
        
        theta = -(self.init_yaw_bias_rad)

        c = math.cos(theta)
        s = math.sin(theta)

        dx = df["x"].astype(float) - x0
        dy = df["y"].astype(float) - y0

        rebased["x"] = c * dx - s * dy
        rebased["y"] = s * dx + c * dy
        rebased["z"] = df["z"].astype(float) # z - given in absolute terms, so no rebase needed

        rebased["yaw"] = df["yaw"].astype(float).apply(
            lambda yaw: self.wrap_angle(yaw - self.init_yaw_bias_rad)
        )
        self.get_logger().info(f"initial yaw: {yaw0:.4f} rad, init_yaw_bias_rad: {self.init_yaw_bias_rad:.4f} rad, total theta: {theta} rad")

        rebased["t"] = df["t"].astype(float) - float(df.iloc[0]["t"])

        self.goal_x = rebased["x"].iloc[-1]
        self.goal_y = rebased["y"].iloc[-1]
        self.goal_z = rebased["z"].iloc[-1]


        return rebased

    def stop_ego_waypoint_updates(self):
        if self.ego_waypoint_timer is not None:
            self.ego_waypoint_timer.cancel()
            self.destroy_timer(self.ego_waypoint_timer)
            self.ego_waypoint_timer = None

        self.active_optimized_trajectory = None
        self.next_waypoint_index = 0
        if hasattr(self, "set_vel_acc_client") and self.set_vel_acc_client is not None:
            if self.set_vel_acc_client.service_is_ready():
                req = VelAccCmd.Request()
                req.max_velocity = float(0.4) # back to default value

                future = self.set_vel_acc_client.call_async(req)

                def _vel_acc_response_cb(fut):
                    try:
                        resp = fut.result()
                        self.get_logger().info(
                            f"Set planner vel/acc: success={resp.success}, "
                            f"message='{resp.message}', "
                            f"max_velocity={req.max_velocity:.3f}"
                        )
                    except Exception as e:
                        self.get_logger().warn(
                            f"Failed calling ego_planner/set_vel_acc_cmd: {e}"
                        )

                future.add_done_callback(_vel_acc_response_cb)
            else:
                self.get_logger().warn(
                    "Service ego_planner/set_vel_acc_cmd not ready, skipping velocity update."
                )
        else:
            self.get_logger().warn(
                "set_vel_acc_client not initialized, skipping velocity update."
            )

    def start_ego_waypoint_updates(self, optimized_trajectory_msg: OptimizedTrajectory):
        self.stop_ego_waypoint_updates()

        if not optimized_trajectory_msg.points:
            self.get_logger().warn("Optimized trajectory is empty, cannot start EGO waypoint updates.")
            return

        self.active_optimized_trajectory = optimized_trajectory_msg
        self.next_waypoint_index = max(1, int(round((5.0) / self.active_optimized_trajectory.dt)))

        self.ego_waypoint_timer = self.create_timer(
            self.ego_waypoint_update_period,
            self.publish_next_ego_velocity_waypoint,
            callback_group=self.client_group,
        )

        self.get_logger().info(
            f"Started EGO waypoint updates every {self.ego_waypoint_update_period:.2f} s."
        )

    def publish_next_ego_velocity_waypoint(self):
        if self.active_optimized_trajectory is None:
            return

        points = self.active_optimized_trajectory.points
        if not points:
            self.stop_ego_waypoint_updates()
            return

        # Need valid odometry to choose the closest point
        if not hasattr(self, "odom_x") or not hasattr(self, "odom_y") or not hasattr(self, "odom_z"):
            self.get_logger().warn("Current odometry not available yet, skipping waypoint update.")
            return

        # Find closest trajectory point to current vehicle position
        closest_index = 0
        closest_dist_sq = float("inf")

        for i, pt in enumerate(points):
            dx = float(pt.position.x) - float(self.odom_x)
            dy = float(pt.position.y) - float(self.odom_y)
            dz = float(pt.position.z) - float(self.odom_z)
            dist_sq = dx * dx + dy * dy + dz * dz

            if dist_sq < closest_dist_sq:
                closest_dist_sq = dist_sq
                closest_index = i

        # Choose a point 5 seconds ahead of the closest point
        lookahead_time = 5.0

        step = 1
        if self.active_optimized_trajectory.dt > 0.0:
            step = max(1, int(round(lookahead_time / self.active_optimized_trajectory.dt)))

        target_index = min(closest_index + step, len(points) - 1)
        sample = points[target_index]

        # ------------------------------------------------------------
        # Compute segment average and maximum speed from closest_index
        # to target_index inclusive
        # ------------------------------------------------------------
        segment_points = points[closest_index:target_index + 1]

        segment_speeds = []
        for pt in segment_points:
            u_seg = float(pt.body_velocity.x)
            v_seg = float(pt.body_velocity.y)
            w_seg = float(pt.body_velocity.z)
            speed = math.sqrt(u_seg * u_seg + v_seg * v_seg + w_seg * w_seg)
            segment_speeds.append(speed)

        if segment_speeds:
            avg_segment_speed = sum(segment_speeds) / len(segment_speeds)
            max_segment_speed = max(segment_speeds)
        else:
            avg_segment_speed = 0.0
            max_segment_speed = 0.0

        # ------------------------------------------------------------
        # Push max velocity to EGO planner via service
        # ------------------------------------------------------------
        if hasattr(self, "set_vel_acc_client") and self.set_vel_acc_client is not None:
            if self.set_vel_acc_client.service_is_ready():
                req = VelAccCmd.Request()
                req.max_velocity = float(max_segment_speed)

                future = self.set_vel_acc_client.call_async(req)

                def _vel_acc_response_cb(fut):
                    try:
                        resp = fut.result()
                        self.get_logger().info(
                            f"Set planner vel/acc: success={resp.success}, "
                            f"message='{resp.message}', "
                            f"max_velocity={req.max_velocity:.3f}"
                        )
                    except Exception as e:
                        self.get_logger().warn(
                            f"Failed calling ego_planner/set_vel_acc_cmd: {e}"
                        )

                future.add_done_callback(_vel_acc_response_cb)
            else:
                self.get_logger().warn(
                    "Service ego_planner/set_vel_acc_cmd not ready, skipping velocity update."
                )
        else:
            self.get_logger().warn(
                "set_vel_acc_client not initialized, skipping velocity update."
            )

        # Body-frame velocity at target waypoint
        u = float(sample.body_velocity.x)
        v = float(sample.body_velocity.y)
        w = float(sample.body_velocity.z)

        # Yaw of the trajectory sample
        yaw = float(sample.yaw)

        # Rotate body-frame velocity into odom/world frame
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        vx_odom = cy * u - sy * v
        vy_odom = sy * u + cy * v
        vz_odom = w

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"

        msg.pose.pose.position.x = float(sample.position.x)
        msg.pose.pose.position.y = float(sample.position.y)
        msg.pose.pose.position.z = float(sample.position.z)

        msg.twist.twist.linear.x = vx_odom
        msg.twist.twist.linear.y = vy_odom
        msg.twist.twist.linear.z = vz_odom

        self.velocity_waypoint_pub.publish(msg)

        self.get_logger().info(
            f"Published EGO velocity waypoint: "
            f"closest_idx={closest_index}, target_idx={target_index}, "
            f"pos=({sample.position.x:.3f}, {sample.position.y:.3f}, {sample.position.z:.3f}), "
            f"body_vel=({u:.3f}, {v:.3f}, {w:.3f}), "
            f"odom_vel=({vx_odom:.3f}, {vy_odom:.3f}, {vz_odom:.3f}), "
            f"yaw={yaw:.3f}, "
            f"dist_to_closest={math.sqrt(closest_dist_sq):.3f}, "
            f"segment_avg_speed={avg_segment_speed:.3f}, "
            f"segment_max_speed={max_segment_speed:.3f}"
        )

        if target_index == len(points) - 1:
            self.get_logger().info("Finished publishing all sampled EGO velocity waypoints.")
            self.stop_ego_waypoint_updates()
            return

        
    def odom_callback(self, msg: Odometry):
        # check if goal is reached and if so, stop the the recording
        
        pos = msg.pose.pose.position
        self.odom_x = pos.x
        self.odom_y = pos.y
        self.odom_z = pos.z
        goal_reached = (
            abs(pos.x - self.goal_x) < 0.05 and
            abs(pos.y - self.goal_y) < 0.05 and
            abs(pos.z - self.goal_z) < 0.1
        )
        
        if goal_reached:
            if self.nr_ego_done >= self.nr_per_process and self.nr_opt_done >= self.nr_per_process and self.nr_combined_done >= self.nr_per_process:
                self.get_logger().info("All tests completed, Stopping.")
                # self.goal_x = 999
                # self.goal_y = 999
                # self.goal_z = 999
                # self.test_ego = True
                # self.nr_ego_done = 0
                # self.nr_opt_done = 0
                # self.stop_bag_recording()
                # return
            self.get_logger().info("Goal reached, stopping ros2 bag recording...")
            self.stop_bag_recording()
            self.stop_ego_waypoint_updates()
            self.get_logger().info("Restarting the test with the next trajectory...")
            trigger_request = Trigger.Request()
            self.handle_start_test(trigger_request, Trigger.Response())


    def dataframe_to_optimized_trajectory(
        self,
        df: pd.DataFrame,
        traj_id: int = 0,
        frame_id: str = "",
    ) -> OptimizedTrajectory:
        msg = OptimizedTrajectory()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.traj_id = traj_id

        if len(df) >= 2:
            msg.dt = float(df.iloc[1]["t"] - df.iloc[0]["t"])
        else:
            msg.dt = 0.0

        msg.points = []

        for row in df.itertuples(index=False):
            sample = TrajectorySample()

            sample.t = float(row.t)

            sample.position = Point()
            sample.position.x = float(row.x)
            sample.position.y = float(row.y)
            sample.position.z = float(row.z)

            sample.yaw = float(row.yaw)

            sample.body_velocity = Vector3()
            sample.body_velocity.x = float(row.u)
            sample.body_velocity.y = float(row.v)
            sample.body_velocity.z = float(row.w)

            sample.yaw_dot = float(row.r)

            msg.points.append(sample)

        return msg

    def handle_start_test(self, request, response):

        if self.evalation_test:
            if self.nr_ego_done >= self.nr_per_process and self.nr_opt_done >= self.nr_per_process and self.nr_combined_done >= self.nr_per_process:
                self.evaluation_test_case += 1
                self.nr_ego_done = 0
                self.nr_opt_done = 0
                self.nr_combined_done = 0
                self.test_ego = True
                if self.evaluation_test_case >= 5:
                    self.get_logger().info("All tests completed, Stopping.")
                    self.goal_x = 999
                    self.goal_y = 999
                    self.goal_z = 999
                    self.nr_ego_done = 0
                    self.nr_opt_done = 0
                    self.nr_combined_done = 0
                    self.test_ego = True
                    self.test_opt = False
                    self.evaluation_test_case = 1
                    self.evalation_test = True
                    # First case: No Currents & EGO
                    self.use_known_currents = False
                    self.use_disturbance_currents = False
                    self.trajectory_name = self.trajectory_name_without_current
                        # self.trajectory_name = "With_Current_Heavy" # for test case 2 and 3
                    self.csv_file_path = Path(
                        f"/home/ubuntu/ws_blue/src/blue/path_planner/optimized_trajectories/{self.trajectory_name}.csv"
                    )
                    response.success = True
                    response.message = "All tests completed."

                    
                else:
                    if self.evaluation_test_case == 2:
                        self.use_known_currents = False
                        self.use_disturbance_currents = True
                    elif self.evaluation_test_case == 3:
                        self.use_known_currents = True
                        self.use_disturbance_currents = False
                        self.trajectory_name = self.trajectory_name_with_current # Switch to trajectory with known currents for test case 3 and 4
                        self.csv_file_path = Path(
                            f"/home/ubuntu/ws_blue/src/blue/path_planner/optimized_trajectories/{self.trajectory_name}.csv"
                        )
                    elif self.evaluation_test_case == 4:
                        self.use_known_currents = True
                        self.use_disturbance_currents = True

                    self.get_logger().info(f"\n\n\nStarting evaluation test case {self.evaluation_test_case}/4\n\n\n")
            
        if self.nr_ego_done >= self.nr_per_process and self.nr_opt_done >= self.nr_per_process:
            self.test_ego = False
            self.test_opt = False
            self.nr_combined_done += 1
            self.get_logger().info(f"\n######################################### \n \n \n Starting hierarchical trajectory test [{self.nr_combined_done}/{self.nr_per_process}] \n \n \n#########################################")

        elif self.nr_ego_done >= self.nr_per_process:
            self.test_ego = False
            self.test_opt = True
            self.nr_opt_done += 1
            self.get_logger().info(f"\n######################################### \n \n \n Starting optimized trajectory test [{self.nr_opt_done}/{self.nr_per_process}] \n \n \n#########################################")
        else:
            self.nr_ego_done += 1
            self.get_logger().info(f"\n######################################### \n \n \n Starting EGO trajectory test [{self.nr_ego_done}/{self.nr_per_process}] \n \n \n#########################################")
        

        del request
        t0 = time.time()

        try:
            self.get_logger().info("Loading trajectory CSV...")
            trajectory_df = self.load_trajectory_dataframe()

            self.get_logger().info("Rebasing trajectory to local origin...")
            rebased_df = self.rebase_trajectory_to_origin(trajectory_df)

            optimized_trajectory_msg = self.dataframe_to_optimized_trajectory(
                rebased_df,
                traj_id=0,
                frame_id="",
            )

        except Exception as e:
            response.success = False
            response.message = f"Failed to load/rebase trajectory: {e}"
            self.get_logger().error(response.message)
            return response

        pause_cmd = [
            "gz", "service",
            "-s", "/world/underwater_world/control",
            "--reqtype", "gz.msgs.WorldControl",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", "pause: true"
        ]

        set_pose_cmd = [
            "gz", "service",
            "-s", "/world/underwater_world/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req",
            (
                f'name: "bluerov2_heavy", '
                f'position: {{x: {self.start_world_x}, y: {self.start_world_y}, z: {self.start_world_z}}}, '
                f'orientation: {{x: {self.start_world_qx}, y: {self.start_world_qy}, '
                f'z: {self.start_world_qz}, w: {self.start_world_qw}}}'
            )
        ]

        start_cmd = [
            "gz", "service",
            "-s", "/world/underwater_world/control",
            "--reqtype", "gz.msgs.WorldControl",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", "pause: false"
        ]

        self.stop_bag_recording()

        self.stop_ego_waypoint_updates()

        # Reset the ego planner FSM and trajectory server to ensure a clean state before starting the test
        if self.wait_for_service_client(
            self.reset_ego_fsm_client,
            "/ego_planner/reset_ego_replan_fsm",
            timeout_sec=1.0,
        ):
            ok, msg = self.call_trigger_client(
                self.reset_ego_fsm_client,
                "/ego_planner/reset_ego_replan_fsm",
                timeout_sec=5.0,
            )
            if not ok:
                self.get_logger().warn(f"Resetting ego planner FSM failed: {msg}")

        if self.wait_for_service_client(
            self.reset_traj_server_client,
            "/ego_traj_server/reset_traj_server",
            timeout_sec=1.0,
        ):
            ok, msg = self.call_trigger_client(
                self.reset_traj_server_client,
                "/ego_traj_server/reset_traj_server",
                timeout_sec=5.0,
            )
            if not ok:
                self.get_logger().warn(f"Resetting trajectory server failed: {msg}")

        # Turn off all currents before starting the test to ensure a clean state, in case they were left on from a previous test
        if self.wait_for_service_client(
            self.end_known_currents_client,
            "/current_disturbances/end_known_currents",
            timeout_sec=1.0,
        ):
            ok, msg = self.call_trigger_client(
                self.end_known_currents_client,
                "/current_disturbances/end_known_currents",
                timeout_sec=5.0,
            )
            if not ok:
                self.get_logger().warn(f"Ending disturbances failed: {msg}")
        if self.wait_for_service_client(
            self.end_disturbance_currents_client,
            "/current_disturbances/end_disturbance_currents",
            timeout_sec=1.0,
        ):
            ok, msg = self.call_trigger_client(
                self.end_disturbance_currents_client,
                "/current_disturbances/end_disturbance_currents",
                timeout_sec=5.0,
            )
            if not ok:
                self.get_logger().warn(f"Ending disturbances failed: {msg}")

        # Stop the EGO trajectory server output
        cmd_state_msg = String()
        cmd_state_msg.data = "other"
        self.cmd_state_pub.publish(cmd_state_msg)

        # Stop the velocity controller
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.vel_reference_pub.publish(vel_cmd)

        # Pause the simulation
        ok, msg = self.call_command(pause_cmd, "Pausing simulation")
        if not ok:
            response.success = False
            response.message = f"Pause failed: {msg}"
            return response


        # Set robot pose in gazebo
        ok, msg = self.call_command(set_pose_cmd, "Setting robot pose")
        if not ok:
            response.success = False
            response.message = f"Set pose failed: {msg}"
            return response
    
        # Initialize the state estimation
        if not self.wait_for_initialize_service(timeout_sec=2.0):
            response.success = False
            response.message = "/initialize service not available"
            return response

        ok, msg = self.call_initialize_service(timeout_sec=10.0)
        if not ok:
            response.success = False
            response.message = f"Initialize failed: {msg}"
            return response

        # Reset the odometry visualization to clear old paths
        if self.wait_for_service_client(
            self.reset_odometry_viz_client,
            "/odometry_visualization/reset_odometry_visualization",
            timeout_sec=1.0,
        ):
            ok, msg = self.call_trigger_client(
                self.reset_odometry_viz_client,
                "/odometry_visualization/reset_odometry_visualization",
                timeout_sec=5.0,
            )
            if not ok:
                self.get_logger().warn(f"Resetting odometry visualization failed: {msg}")

        # Start the simulation again
        ok, msg = self.call_command(start_cmd, "Restarting simulation")
        if not ok:
            response.success = False
            response.message = f"Restart failed: {msg}"
            return response

        if not self.wait_for_initialize_service(timeout_sec=2.0):
            response.success = False
            response.message = "/initialize service not available"
            return response

        ok, msg = self.call_initialize_service(timeout_sec=10.0)
        if not ok:
            response.success = False
            response.message = f"Initialize failed: {msg}"
            return response
        
        time.sleep(5.0)

        if not self.start_bag_recording():
            response.success = False
            response.message = "Failed to start ros2 bag recording"
            return response

        if self.test_opt:
            # Publish optimized trajectory to be followed
            self.get_logger().info("Publishing optimized trajectory...")
            self.pub.publish(optimized_trajectory_msg)
        
        elif self.test_ego:  
            self.get_logger().info("Publishing new waypoint to the EGO planner...")
            last_point = optimized_trajectory_msg.points[-1]
            last_pos = last_point.position
            waypoint_msg = PoseStamped()

            waypoint_msg.header.frame_id = "odom"
            waypoint_msg.pose.position.x = last_pos.x
            waypoint_msg.pose.position.y = last_pos.y
            waypoint_msg.pose.position.z = last_pos.z
            self.waypoint_pub.publish(waypoint_msg)
        else:
            self.get_logger().info("Start publishing sampled waypoints with velocity to EGO Planner...")
            # self.get_logger().info("Publishing new waypoint to the EGO planner...")
            # last_point = optimized_trajectory_msg.points[-1]
            # last_pos = last_point.position
            # waypoint_msg = PoseStamped()

            # waypoint_msg.header.frame_id = "odom"
            # waypoint_msg.pose.position.x = last_pos.x
            # waypoint_msg.pose.position.y = last_pos.y
            # waypoint_msg.pose.position.z = last_pos.z
            # self.waypoint_pub.publish(waypoint_msg)
            self.start_ego_waypoint_updates(optimized_trajectory_msg)

        time.sleep(0.05)

        # Set the controller mode to auv_controller, so that the EGO trajectory server can start publishing velocity commands
        cmd_state_msg = String()
        cmd_state_msg.data = "auv_controller"
        self.cmd_state_pub.publish(cmd_state_msg)
        
        if self.use_known_currents:
            if self.wait_for_service_client(
                self.start_known_currents_client,
                "/current_disturbances/start_known_currents",
                timeout_sec=1.0,
            ):
                ok, msg = self.call_trigger_client(
                    self.start_known_currents_client,
                    "/current_disturbances/start_known_currents",
                    timeout_sec=5.0,
                )
                if not ok:
                    self.get_logger().warn(f"Starting disturbances failed: {msg}")

        if self.use_disturbance_currents:
            if self.wait_for_service_client(
                self.start_disturbance_currents_client,
                "/current_disturbances/start_disturbance_currents",
                timeout_sec=1.0,
            ):
                ok, msg = self.call_trigger_client(
                    self.start_disturbance_currents_client,
                    "/current_disturbances/start_disturbance_currents",
                    timeout_sec=5.0,
                )
                if not ok:
                    self.get_logger().warn(f"Starting disturbances failed: {msg}")

        elapsed = time.time() - t0
        
        response.success = True
        response.message = (
            f"Test start sequence completed successfully in {elapsed:.3f} s."
        )
        self.get_logger().info(response.message)
        return response

    def destroy_node(self):
        self.get_logger().info("Shutting down node, stopping rosbag...")
        self.stop_ego_waypoint_updates()
        self.stop_bag_recording()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StartTestService()
    executor = MultiThreadedExecutor(num_threads=4)

    try:
        executor.add_node(node)
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()