#!/usr/bin/env python3
"""
NOTE: this node should be launched via its launch file via:

ros2 launch path_planner ego_obstacle_evaluation.launch.py

ROS 2 node that orchestrates automated repeated evaluation runs of the EGO-Planner
in Gazebo across 9 obstacle scenarios (Cases 1–9).

For each run the node:
  1. Pauses the simulation and teleports the robot to the scenario start pose.
  2. Clears all obstacle models and spawns the scenario-specific SDF obstacle set.
  3. Resets the EGO-Planner FSM, trajectory server, and odometry visualisation.
  4. Resumes simulation, activates the AUV controller, and publishes the goal.
  5. Starts a ROS 2 bag recording of all relevant topics.
  6. Monitors /odometry/filtered_enu for goal reaching, /bluerov2_heavy/collision
     for collisions, and a watchdog timer for timeouts.
  7. On any terminal event (success / collision / timeout), stops the bag and
     automatically restarts the scenario until nr_runs_total runs are completed.

Scenarios (Cases 1–9) cover: single cube, corridor, uniform clutter, varied clutter,
vertical triangle gate, dense sphere field, pylon frame, dock piles (cylindrical),
and square piles. Triggering case 8 automatically sequences cases 8 and 9.
Case 8 is the sense pillar feld / Pier traversal test case

Services exposed:
  /start_test_case_1  to  /start_test_case_9   (std_srvs/Trigger)

Parameters:
  world_name            (default: underwater_world)
  model_name            (default: bluerov2_heavy)
  goal_tolerance_xyz    (default: 0.1 m)
  settle_time_sec       (default: 2.0 s)
  record_bag            (default: True)
  bag_output_dir        (default: /home/ubuntu/ws_blue/evaluation/EGO_data)
  visualize_start_goal  (default: False)  spawns colored sphere markers in Gazebo
  nr_runs_total         (default: 50)
  case_to_run           (default: 8)
  run_timeout_sec       (default: 240.0 s)
"""
import subprocess
import tempfile
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf_transformations import quaternion_from_euler
from ros_gz_interfaces.msg import Contacts

@dataclass
class Scenario:
    name: str
    start_world_x: float
    start_world_y: float
    start_world_z: float
    start_yaw_rad: float
    goal_local_x: float
    goal_local_y: float
    goal_local_z: float
    obstacle_model_name: str
    obstacle_sdf: str


class StartEgoScenarioService(Node):
    def __init__(self):
        super().__init__("ego_obstacle_evaluation")

        self.declare_parameter("world_name", "underwater_world")
        self.declare_parameter("model_name", "bluerov2_heavy")
        self.declare_parameter("goal_tolerance_xyz", 0.1)
        self.declare_parameter("settle_time_sec", 2.0)
        self.declare_parameter("record_bag", True)
        self.declare_parameter("bag_output_dir", "/home/ubuntu/ws_blue/evaluation/EGO_data")
        self.declare_parameter("visualize_start_goal", False)
        self.declare_parameter("nr_runs_total", 50)
        self.declare_parameter("case_to_run", "8")
        self.declare_parameter("run_timeout_sec", 240.0)
        


        self.world_name = str(self.get_parameter("world_name").value)
        self.model_name = str(self.get_parameter("model_name").value)
        self.goal_tolerance_xyz = float(self.get_parameter("goal_tolerance_xyz").value)
        self.settle_time_sec = float(self.get_parameter("settle_time_sec").value)
        self.record_bag = bool(self.get_parameter("record_bag").value)
        self.bag_output_dir = str(self.get_parameter("bag_output_dir").value)
        self.visualize_start_goal = bool(self.get_parameter("visualize_start_goal").value)
        self.nr_runs_total = int(self.get_parameter("nr_runs_total").value)
        self.case_to_run = str(self.get_parameter("case_to_run").value)
        self.run_timeout_sec = float(self.get_parameter("run_timeout_sec").value)

        # ── Bag recording state ────────────────────────────────────────────
        self.run_count = 0
        self.bag_process = None
        self.active_case_nr = None
        self.timeout_timer = None

        self.scenario_sequence = []
        self.sequence_index = 0

        

        self.bag_topics = [
            "/clock",
            "/odometry/filtered",
            "/odometry/filtered_enu",
            "/model/bluerov2_heavy/odometry",
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
            "/bluerov2_heavy/collision",
            "/ego_evaluation/test_status",
            "/current_disturbance/current",
            "/position_cmd"
        ]

        self.service_group = MutuallyExclusiveCallbackGroup()
        self.client_group = ReentrantCallbackGroup()

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_z = 0.0

        self.goal_x = 999.0
        self.goal_y = 999.0
        self.goal_z = 999.0
        self.active_scenario_name = ""
        self.test_running = False

        self.contact_cooldown = False

        self.all_obstacle_model_names = [
            "tc1_obstacles",
            "tc2_obstacles",
            "tc3_obstacles",
            "tc4_obstacles",
            "tc5_obstacles",
            "tc6_obstacles",
            "tc7_obstacles",
            "tc8_obstacles",
            "tc9_obstacles",
        ]

        self.cmd_state_pub = self.create_publisher(String, "controller_state", 10)
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

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odometry/filtered_enu",
            self.odom_callback,
            10
        )
        self.test_status_pub = self.create_publisher(String, "/ego_evaluation/test_status", 10)

        self.initialize_client = self.create_client(
            Trigger, "/initialize", callback_group=self.client_group
        )
        self.reset_ego_fsm_client = self.create_client(
            Trigger, "/ego_planner/reset_ego_replan_fsm", callback_group=self.client_group
        )
        self.reset_traj_server_client = self.create_client(
            Trigger, "/ego_traj_server/reset_traj_server", callback_group=self.client_group
        )
        self.reset_odometry_viz_client = self.create_client(
            Trigger, "/odometry_visualization/reset_odometry_visualization",
            callback_group=self.client_group,
        )

        self.scenarios = {
            "case_1_single_cube": Scenario(
                name="case_1_single_cube",
                start_world_x=-26.0,
                start_world_y=20.0,
                start_world_z=-5.0,
                start_yaw_rad=0.0,
                goal_local_x=0.0,
                goal_local_y=-12.0,
                goal_local_z=-5.0,
                obstacle_model_name="tc1_obstacles",
                obstacle_sdf=self.build_case_1_sdf(),
            ),
            "case_2_corridor": Scenario(
                name="case_2_corridor",
                start_world_x=12.0,
                start_world_y=20.0,
                start_world_z=-5.0,
                start_yaw_rad=0.0,
                goal_local_x=0.0,
                goal_local_y=-16.0,
                goal_local_z=-5.0,
                obstacle_model_name="tc2_obstacles",
                obstacle_sdf=self.build_case_2_sdf(),
            ),
            "case_3_clutter_same_size": Scenario(
                name="case_3_clutter_same_size",
                start_world_x=-28.0,
                start_world_y=-20.0,
                start_world_z=-5.0,
                start_yaw_rad=0.0,
                goal_local_x=0.0,
                goal_local_y=-16.0,
                goal_local_z=-5.0,
                obstacle_model_name="tc3_obstacles",
                obstacle_sdf=self.build_case_3_sdf(),
            ),
            "case_4_clutter_varied": Scenario(
                name="case_4_clutter_varied",
                start_world_x=12.0,
                start_world_y=-20.0,
                start_world_z=-5.0,
                start_yaw_rad=0.0,
                goal_local_x=0.0,
                goal_local_y=-16.0,
                goal_local_z=-5.0,
                obstacle_model_name="tc4_obstacles",
                obstacle_sdf=self.build_case_4_sdf(),
            ),
            "case_5_vertical_triangle": Scenario(
                name="case_5_vertical_triangle",
                start_world_x=-31.0,
                start_world_y=50.0,
                start_world_z=-5.5,
                start_yaw_rad=0.0,
                goal_local_x=0.0,
                goal_local_y=-26.0,
                goal_local_z=-5.5,
                obstacle_model_name="tc5_obstacles",
                obstacle_sdf=self.build_case_5_sdf(),
            ),
            "case_6_dense_spheres": Scenario(
                name="case_6_dense_spheres",
                start_world_x=8.5,
                start_world_y=50.0,
                start_world_z=-5.0,
                start_yaw_rad=0.0,
                goal_local_x=0.0,
                goal_local_y=-22.0,
                goal_local_z=-5.0,
                obstacle_model_name="tc6_obstacles",
                obstacle_sdf=self.build_case_6_sdf(),
            ),
            "case_7_pylon": Scenario(
                name="case_7_pylon",
                start_world_x=-31.0,
                start_world_y=80.0,
                start_world_z=-5.0,
                start_yaw_rad=0.0,
                goal_local_x=0.0,
                goal_local_y=-22.0,
                goal_local_z=-5.0,
                obstacle_model_name="tc7_obstacles",
                obstacle_sdf=self.build_case_7_sdf(),
            ),
            "case_8_dock_piles": Scenario(
                name="case_8_dock_piles",
                start_world_x=6.0,
                start_world_y=70.0,
                start_world_z=-5.0,
                start_yaw_rad=0.0,
                goal_local_x=10.0,
                goal_local_y=-14.0,
                goal_local_z=-5.0,
                obstacle_model_name="tc8_obstacles",
                obstacle_sdf=self.build_case_8_sdf(),
            ),
            "case_9_square_piles": Scenario(
                name="case_9_square_piles",
                start_world_x=6.0,
                start_world_y=70.0,
                start_world_z=-5.0,
                start_yaw_rad=0.0,
                goal_local_x=10.0,
                goal_local_y=-14.0,
                goal_local_z=-5.0,
                obstacle_model_name="tc9_obstacles",
                obstacle_sdf=self.build_case_9_sdf(),
            ),
        }

        # Map scenario name to case number for bag naming
        self.scenario_case_nr = {
            "case_1_single_cube":       1,
            "case_2_corridor":          2,
            "case_3_clutter_same_size": 3,
            "case_4_clutter_varied":    4,
            "case_5_vertical_triangle": 5,
            "case_6_dense_spheres":     6,
            "case_7_pylon":             7,
            "case_8_dock_piles":        8,
            "case_9_square_piles":      9,
        }

        self.case1_srv = self.create_service(
            Trigger, "/start_test_case_1", self.handle_start_test_case_1,
            callback_group=self.service_group
        )
        self.case2_srv = self.create_service(
            Trigger, "/start_test_case_2", self.handle_start_test_case_2,
            callback_group=self.service_group
        )
        self.case3_srv = self.create_service(
            Trigger, "/start_test_case_3", self.handle_start_test_case_3,
            callback_group=self.service_group
        )
        self.case4_srv = self.create_service(
            Trigger, "/start_test_case_4", self.handle_start_test_case_4,
            callback_group=self.service_group
        )
        self.case5_srv = self.create_service(
            Trigger, "/start_test_case_5", self.handle_start_test_case_5,
            callback_group=self.service_group
        )
        self.case6_srv = self.create_service(
            Trigger, "/start_test_case_6", self.handle_start_test_case_6,
            callback_group=self.service_group
        )
        self.case7_srv = self.create_service(
            Trigger, "/start_test_case_7", self.handle_start_test_case_7,
            callback_group=self.service_group
        )
        self.case8_srv = self.create_service(
            Trigger, "/start_test_case_8", self.handle_start_test_case_8,
            callback_group=self.service_group
        )
        self.case9_srv = self.create_service(
            Trigger, "/start_test_case_9", self.handle_start_test_case_9,
            callback_group=self.service_group
        )
        self.contact_sub = self.create_subscription(
            Contacts,
            "/bluerov2_heavy/collision", 
            self.contact_callback,
            10
        )
        self.collision_detected = False


        self.get_logger().info(
            "EGO obstacle evaluation service ready with services: "
            "/start_test_case_1 to /start_test_case_9"
        )

    def continue_or_finish_sequence(self):
        """Advance to next scenario in the sequence, or finish."""
        self.run_count = 0
        self.sequence_index += 1

        if self.sequence_index < len(self.scenario_sequence):
            next_name = self.scenario_sequence[self.sequence_index]
            self.active_scenario_name = next_name
            self.get_logger().info(
                f"Switching to {next_name} "
                f"for runs 1/{self.nr_runs_total}..."
            )
            scenario = self.scenarios[next_name]
            dummy_response = type("Response", (), {"success": False, "message": ""})()
            self.run_scenario(scenario, dummy_response)
        else:
            self.get_logger().info("Completed all scenarios in the sequence. Done.")
            self.active_scenario_name = None
            self.scenario_sequence = []
            self.sequence_index = 0
            
    # --------- collision detection callback ---------
    def contact_callback(self, msg: Contacts):
        if not self.test_running or self.collision_detected or self.contact_cooldown:
            return

        ignored = {
            "sand_heightmap",
            "coast_waves",
            "ground_plane",
        }

        MIN_CONTACT_DEPTH = 1e-4

        for contact in msg.contacts:
            col1 = contact.collision1.name
            col2 = contact.collision2.name

            if any(ign in col1 or ign in col2 for ign in ignored):
                continue

            max_depth = max(contact.depths) if contact.depths else 0.0
            if max_depth < MIN_CONTACT_DEPTH:
                self.get_logger().debug(
                    f"Ignoring shallow contact ({max_depth:.2e} m): {col1} <-> {col2}"
                )
                continue

            self.collision_detected = True
            self.test_running = False
            self.cancel_timeout_timer()
            self.get_logger().warn(
                f"Collision detected for {self.active_scenario_name}: "
                f"{col1} <-> {col2} — run stopped. (depth: {max_depth:.4f} m)"
            )
            self.publish_test_status(
                f"STOPPED:collision:{self.active_scenario_name}:"
                f"{col1}<->{col2}"
            )
            time.sleep(0.2)
            self.stop_bag_recording()
            self.run_count += 1

            if self.run_count < self.nr_runs_total:
                self.get_logger().info(
                    f"Restarting {self.active_scenario_name} "
                    f"for run {self.run_count + 1}/{self.nr_runs_total}..."
                )
                scenario = self.scenarios[self.active_scenario_name]
                dummy_response = type("Response", (), {"success": False, "message": ""})()
                self.run_scenario(scenario, dummy_response)
            else:
                self.get_logger().info(
                    f"Completed all {self.nr_runs_total} runs "
                    f"for {self.active_scenario_name}."
                )
                self.continue_or_finish_sequence()

            break
                
    # ---------- scenario execution and timeout handling ---------
    def start_timeout_timer(self):
        """Start a watchdog timer that cancels the run if goal is not reached in time."""
        self.cancel_timeout_timer()
        self.timeout_timer = self.create_timer(
            self.run_timeout_sec,
            self.handle_run_timeout,
            callback_group=self.client_group,
        )

    def cancel_timeout_timer(self):
        """Cancel and destroy the watchdog timer if it is running."""
        if self.timeout_timer is not None:
            self.timeout_timer.cancel()
            self.destroy_timer(self.timeout_timer)
            self.timeout_timer = None

    def handle_run_timeout(self):
        """Called if the robot does not reach the goal within run_timeout_sec."""
        self.cancel_timeout_timer()
        if not self.test_running:
            return

        self.run_count += 1

        self.get_logger().warn(
            f"Run timed out for {self.active_scenario_name} "
            f"after {self.run_timeout_sec:.0f}s "
            f"[run {self.run_count}/{self.nr_runs_total}]."
        )

        self.test_running = False

        self.publish_test_status(
            f"STOPPED:timeout:{self.active_scenario_name}:"
            f"run_{self.run_count}_of_{self.nr_runs_total}"
        )
        time.sleep(0.2)
        self.stop_bag_recording()

        if self.run_count < self.nr_runs_total:
            self.get_logger().info(
                f"Restarting {self.active_scenario_name} "
                f"for run {self.run_count + 1}/{self.nr_runs_total}..."
            )
            scenario = self.scenarios[self.active_scenario_name]
            dummy_response = type(
                "Response", (), {"success": False, "message": ""}
            )()
            self.run_scenario(scenario, dummy_response)
        else:
            self.get_logger().info(
                f"Completed all {self.nr_runs_total} runs "
                f"for {self.active_scenario_name}."
            )
            self.continue_or_finish_sequence()

    # ---------- logger helpers ----------
    def publish_test_status(self, status: str):
        msg = String()
        msg.data = status
        self.test_status_pub.publish(msg)
        self.get_logger().info(f"Test status: {status}")
        
    def start_bag_recording(self, case_nr: int):
        """Start a ros2 bag recording for the given case number."""
        if not self.record_bag:
            return True

        if self.bag_process is not None and self.bag_process.poll() is None:
            self.get_logger().warn("Bag recording already running, stopping it first.")
            self.stop_bag_recording()

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        bag_name = f"{self.bag_output_dir}/EGO_CASE_{case_nr}_{timestamp}"

        cmd = [
            "ros2", "bag", "record",
            "-o", bag_name,
            *self.bag_topics,
        ]

        self.get_logger().info(f"Starting bag recording: {bag_name}")

        try:
            self.bag_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            self.active_case_nr = case_nr
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to start bag recording: {e}")
            self.bag_process = None
            return False

    def stop_bag_recording(self):
        """Gracefully stop the running bag recording."""
        if self.bag_process is None:
            return

        if self.bag_process.poll() is None:
            self.get_logger().info("Stopping bag recording...")
            self.bag_process.terminate()
            try:
                self.bag_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.get_logger().warn("Bag did not stop gracefully, killing it.")
                self.bag_process.kill()

        self.bag_process = None
        self.active_case_nr = None

    def rotate_goal_plus_90(self, x: float, y: float, z: float):
        return -x, -y, z

    # ---------- SDF builders ----------

    def wrap_model(self, model_name: str, body: str) -> str:
        return f"""<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="{model_name}">
    <static>true</static>
    <link name="obstacles">
{body}
    </link>
  </model>
</sdf>
"""

    def box_block(self, name: str, x: float, y: float, z: float,
                  sx: float, sy: float, sz: float,
                  roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> str:
        return f"""      <collision name="{name}_collision">
        <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      </collision>
      <visual name="{name}_visual">
        <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.3 0.3 0.3 1</diffuse>
          <specular>0.3 0.3 0.3 1</specular>
        </material>
        <cast_shadows>false</cast_shadows>
      </visual>"""

    def sphere_block(self, name: str, x: float, y: float, z: float, r: float) -> str:
        return f"""      <collision name="{name}_collision">
        <pose>{x} {y} {z} 0 0 0</pose>
        <geometry><sphere><radius>{r}</radius></sphere></geometry>
      </collision>
      <visual name="{name}_visual">
        <pose>{x} {y} {z} 0 0 0</pose>
        <geometry><sphere><radius>{r}</radius></sphere></geometry>
        <material>
          <ambient>0.2 0.7 0.3 1</ambient>
          <diffuse>0.2 0.7 0.3 1</diffuse>
          <specular>0.2 0.7 0.3 1</specular>
        </material>
        <cast_shadows>false</cast_shadows>
      </visual>"""

    def marker_sphere_block(self, name: str, x: float, y: float, z: float,
                            r: float, color: tuple) -> str:
        """Colored sphere marker for start/goal visualization — visual only, no collision."""
        cr, cg, cb = color
        return f"""      <visual name="{name}_visual">
            <pose>{x} {y} {z} 0 0 0</pose>
            <geometry><sphere><radius>{r}</radius></sphere></geometry>
            <material>
            <ambient>{cr} {cg} {cb} 1</ambient>
            <diffuse>{cr} {cg} {cb} 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <emissive>{cr} {cg} {cb} 0.5</emissive>
            </material>
            <cast_shadows>false</cast_shadows>
        </visual>"""

    def build_case_1_sdf(self) -> str:
        body = "\n".join([
            self.box_block("cube1", -20, 20, -5, 2.0, 2.0, 20.0),
        ])
        return self.wrap_model("tc1_obstacles", body)

    def build_case_2_sdf(self) -> str:
        body = "\n".join([
            self.box_block("wall_l", 20, 15.75, -5, 20, 0.5, 20),
            self.box_block("wall_r", 20, 24.25, -5, 20, 0.5, 20),
            self.box_block("arm1", 16, 21, -5, 1.5, 6.0, 20),
            self.box_block("arm2", 20, 19, -5, 1.5, 6.0, 20),
            self.box_block("arm3", 24, 21, -5, 1.5, 6.0, 20),
        ])
        return self.wrap_model("tc2_obstacles", body)

    def build_case_3_sdf(self) -> str:
        body = "\n".join([
            self.box_block("c31", -22, -22, -5, 1.6, 1.6, 20),
            self.box_block("c32", -19, -18, -5, 1.6, 1.6, 20),
            self.box_block("c33", -16, -22, -5, 1.6, 1.6, 20),
            self.box_block("c34", -13, -18, -5, 1.6, 1.6, 20),
            self.box_block("c35", -10, -22, -5, 1.6, 1.6, 20),
        ])
        return self.wrap_model("tc3_obstacles", body)

    def build_case_4_sdf(self) -> str:
        body = "\n".join([
            self.box_block("v41", 17, -22, -5, 1.2, 2.4, 20, yaw=0.3),
            self.box_block("v42", 20, -18, -5, 2.0, 1.0, 20, yaw=-0.5),
            self.box_block("v43", 23, -22, -5, 1.5, 2.8, 20, yaw=0.7),
            self.box_block("v44", 26, -18, -5, 2.5, 1.2, 20, yaw=-0.2),
        ])
        return self.wrap_model("tc4_obstacles", body)

    def build_case_5_sdf(self) -> str:
        body = "\n".join([
            self.box_block("w1", -20, 45.5, -5, 24, 0.5, 20),
            self.box_block("w2", -20, 54.5, -5, 24, 0.5, 20),
            self.box_block("b1", -28, 50, -6.0, 5.0, 7.0, 0.6, pitch=0.55),
            self.box_block("b2", -23, 50, -4.0, 5.0, 7.0, 0.6, pitch=-0.55),
            self.box_block("b3", -18, 49, -5.5, 4.5, 6.0, 0.6, pitch=0.45, yaw=0.25),
            self.box_block("tri_l", -13.2, 48.8, -5.2, 4.5, 0.7, 0.7, pitch=0.55),
            self.box_block("tri_r", -13.2, 51.2, -5.2, 4.5, 0.7, 0.7, pitch=-0.55),
            self.box_block("tri_t", -11.2, 50.0, -3.3, 2.6, 0.7, 0.7, pitch=1.57),
            self.box_block("b4", -7, 50, -5.8, 4.0, 7.0, 0.6, pitch=-0.45),
        ])
        return self.wrap_model("tc5_obstacles", body)

    def build_case_6_sdf(self) -> str:
        spheres = [
    (10.500, 45.275, -9.777, 0.820),
    (23.534, 45.892, -8.913, 0.868),
    (10.596, 45.219, -7.495, 0.711),
    (13.977, 45.650, -6.455, 0.513),
    (21.785, 45.809, -5.994, 0.610),
    (23.963, 45.340, -4.845, 0.903),
    (16.732, 45.093, -3.903, 0.979),
    (22.075, 45.807, -2.270, 0.924),
    (29.462, 45.379, -1.448, 0.768),
    (22.370, 45.862, -0.423, 0.915),
    (10.916, 46.228, -9.711, 0.852),
    (14.656, 46.101, -8.722, 0.540),
    (17.297, 46.370, -7.790, 0.818),
    (28.733, 46.648, -6.391, 0.633),
    (24.583, 46.163, -5.621, 0.586),
    (22.800, 46.557, -4.315, 0.995),
    (25.520, 46.229, -3.968, 0.921),
    (15.355, 46.211, -2.057, 0.658),
    (16.294, 46.655, -1.604, 0.938),
    (19.177, 46.265, -0.753, 0.957),
    (15.255, 47.585, -9.102, 0.781),
    (14.386, 47.998, -8.490, 0.700),
    (10.942, 47.110, -7.373, 0.545),
    (18.443, 47.064, -6.618, 0.896),
    (20.582, 47.971, -5.139, 0.998),
    (24.414, 47.682, -4.463, 0.506),
    (22.819, 47.112, -3.565, 0.633),
    (29.076, 47.876, -2.737, 0.727),
    (13.573, 47.913, -1.129, 0.750),
    (22.779, 47.609, -0.847, 0.649),
    (20.788, 48.779, -9.470, 0.881),
    (16.483, 48.019, -8.071, 0.500),
    (26.633, 48.308, -7.942, 0.939),
    (28.939, 48.086, -6.514, 0.939),
    (25.212, 48.766, -5.872, 0.535),
    (20.996, 48.265, -4.128, 0.738),
    (14.236, 48.539, -3.270, 0.712),
    (16.234, 48.995, -2.350, 0.601),
    (20.352, 48.121, -1.775, 0.719),
    (21.766, 48.230, -0.780, 0.669),
    (22.622, 49.229, -9.095, 0.535),
    (11.417, 49.238, -8.331, 0.930),
    (12.646, 49.936, -7.429, 0.607),
    (25.692, 49.807, -6.810, 0.736),
    (18.621, 49.424, -5.533, 0.548),
    (23.467, 49.984, -4.902, 0.865),
    (16.786, 49.862, -3.751, 0.701),
    (18.972, 49.422, -2.721, 0.595),
    (28.465, 49.443, -1.139, 0.625),
    (11.012, 49.999, -0.164, 0.775),
    (28.527, 50.849, -9.834, 0.984),
    (14.275, 50.401, -8.941, 0.743),
    (29.706, 50.265, -7.216, 0.689),
    (18.460, 50.957, -6.005, 0.728),
    (24.368, 50.155, -5.703, 0.778),
    (21.584, 50.542, -4.252, 0.984),
    (21.684, 50.503, -3.147, 0.529),
    (29.216, 50.080, -2.814, 0.579),
    (23.504, 50.235, -1.880, 0.798),
    (14.924, 50.595, -0.381, 0.945),
    (21.673, 51.523, -9.065, 0.710),
    (24.324, 51.239, -8.604, 0.602),
    (16.000, 51.316, -7.248, 0.836),
    (19.166, 51.998, -6.004, 0.536),
    (14.263, 51.265, -5.067, 0.537),
    (27.585, 51.370, -4.842, 0.940),
    (24.071, 51.612, -3.013, 0.917),
    (10.156, 51.817, -2.701, 0.827),
    (28.779, 51.134, -1.885, 0.832),
    (21.064, 51.272, -0.395, 0.554),
    (14.072, 52.634, -9.736, 0.859),
    (28.107, 52.846, -8.908, 0.744),
    (15.534, 52.004, -7.229, 0.712),
    (15.239, 52.741, -6.448, 0.819),
    (10.193, 52.075, -5.117, 0.714),
    (20.912, 52.835, -4.417, 0.952),
    (12.549, 52.308, -3.101, 0.574),
    (27.214, 52.899, -2.790, 0.898),
    (12.056, 52.780, -1.116, 0.625),
    (22.413, 52.155, -0.070, 0.703),
    (29.524, 53.811, -9.119, 0.932),
    (24.731, 53.332, -8.069, 0.512),
    (27.281, 53.811, -7.733, 0.901),
    (12.162, 53.872, -6.141, 0.894),
    (26.332, 53.460, -5.695, 0.611),
    (14.552, 53.024, -4.807, 0.898),
    (27.287, 53.967, -3.721, 0.664),
    (17.994, 53.981, -2.464, 0.821),
    (12.307, 53.970, -1.821, 0.970),
    (15.309, 53.108, -0.565, 0.981),
    (16.274, 54.606, -9.489, 0.864),
    (21.532, 54.255, -8.291, 0.693),
    (28.512, 54.538, -7.281, 0.501),
    (23.413, 54.364, -6.930, 0.871),
    (16.604, 54.314, -5.152, 0.832),
    (16.006, 54.309, -4.592, 0.860),
    (15.913, 54.127, -3.580, 0.701),
    (23.546, 54.903, -2.384, 0.970),
    (20.959, 54.000, -1.713, 0.650),
    (21.600, 54.655, -0.535, 0.715),
        ]
        parts = [
            self.box_block("wall_l", 20, 44.75, -5, 24, 0.5, 20),
            self.box_block("wall_r", 20, 55.25, -5, 24, 0.5, 20),
        ]
        for i, (x, y, z, r) in enumerate(spheres, start=1):
            parts.append(self.sphere_block(f"s{i}", x, y, z, r))
        return self.wrap_model("tc6_obstacles", "\n".join(parts))

    def build_case_7_sdf(self) -> str:
        parts = [
            # Optional corridor walls
            self.box_block("wall_l", -20, 74.75, -5, 26, 0.5, 20),
            self.box_block("wall_r", -20, 85.25, -5, 26, 0.5, 20),

            # --------------------------------------------------
            # Pylon outer frame: 4 legs
            # --------------------------------------------------
            self.box_block("leg_fl", -20.0, 78.5, -5.0, 0.5, 0.5, 20),
            self.box_block("leg_fr", -20.0, 81.5, -5.0, 0.5, 0.5, 20),
            self.box_block("leg_rl", -14.0, 78.5, -5.0, 0.5, 0.5, 20),
            self.box_block("leg_rr", -14.0, 81.5, -5.0, 0.5, 0.5, 20),

            # --------------------------------------------------
            # Horizontal frame members
            # --------------------------------------------------
            self.box_block("front_low",  -17.0, 78.5, -8.0, 6.0, 0.35, 0.35),
            self.box_block("rear_low",   -17.0, 81.5, -8.0, 6.0, 0.35, 0.35),
            self.box_block("left_low",   -20.0, 80.0, -8.0, 0.35, 3.0, 0.35),
            self.box_block("right_low",  -14.0, 80.0, -8.0, 0.35, 3.0, 0.35),

            self.box_block("front_mid",  -17.0, 78.5, -5.0, 6.0, 0.35, 0.35),
            self.box_block("rear_mid",   -17.0, 81.5, -5.0, 6.0, 0.35, 0.35),
            self.box_block("left_mid",   -20.0, 80.0, -5.0, 0.35, 3.0, 0.35),
            self.box_block("right_mid",  -14.0, 80.0, -5.0, 0.35, 3.0, 0.35),

            self.box_block("front_top",  -17.0, 78.5, -2.0, 6.0, 0.35, 0.35),
            self.box_block("rear_top",   -17.0, 81.5, -2.0, 6.0, 0.35, 0.35),
            self.box_block("left_top",   -20.0, 80.0, -2.0, 0.35, 3.0, 0.35),
            self.box_block("right_top",  -14.0, 80.0, -2.0, 0.35, 3.0, 0.35),

            # --------------------------------------------------
            # Diagonal lattice braces
            # --------------------------------------------------
            self.box_block("diag_front_1", -17.0, 78.5, -6.5, 6.5, 0.25, 0.25, pitch=0.45),
            self.box_block("diag_front_2", -17.0, 78.5, -3.5, 6.5, 0.25, 0.25, pitch=-0.45),

            self.box_block("diag_rear_1",  -17.0, 81.5, -6.5, 6.5, 0.25, 0.25, pitch=0.45),
            self.box_block("diag_rear_2",  -17.0, 81.5, -3.5, 6.5, 0.25, 0.25, pitch=-0.45),

            self.box_block("diag_left_1",  -20.0, 80.0, -6.5, 0.25, 3.2, 0.25, roll=0.45),
            self.box_block("diag_left_2",  -20.0, 80.0, -3.5, 0.25, 3.2, 0.25, roll=-0.45),

            self.box_block("diag_right_1", -14.0, 80.0, -6.5, 0.25, 3.2, 0.25, roll=0.45),
            self.box_block("diag_right_2", -14.0, 80.0, -3.5, 0.25, 3.2, 0.25, roll=-0.45),

            # --------------------------------------------------
            # Interior cross obstacle to make traversal harder
            # --------------------------------------------------
            self.box_block("inner_cross_x", -17.0, 80.0, -5.0, 3.0, 0.25, 0.25),
            self.box_block("inner_cross_y", -17.0, 80.0, -5.0, 0.25, 1.8, 0.25),

            # Slightly offset inner diagonal beam
            self.box_block("inner_diag", -16.2, 79.4, -4.2, 2.8, 0.25, 0.25, pitch=0.5, yaw=0.35),
        ]

        return self.wrap_model("tc7_obstacles", "\n".join(parts))

    def build_case_8_sdf(self) -> str:
        parts = []

        pillar_diameter = 0.5
        start_x = 10.0
        nr_x = 4
        spacing_x = 2.0
        nr_y = 6
        start_y = 70.0
        spacing_y = 2.0

        row_xs = [start_x + i * spacing_x for i in range(nr_x)]
        pile_ys = [start_y + j * spacing_y for j in range(nr_y)]

        idx = 1
        for rx in row_xs:
            for py in pile_ys:
                parts.append(
                    self.cylinder_block(
                        f"p{idx}", rx, py, -5.0,
                        pillar_diameter / 2, 12.0
                    )
                )
                idx += 1

        if self.visualize_start_goal:
            # ── Start marker (green sphere, visual only) ───────────────────────
            start_world_x = 6.0
            start_world_y = 70.0
            start_world_z = -5.0
            parts.append(
                self.marker_sphere_block(
                    "start_marker",
                    start_world_x, start_world_y, start_world_z,
                    r=0.35,
                    color=(0.0, 0.9, 0.0),
                )
            )

            # ── Goal marker (red sphere, visual only) ──────────────────────────
            # rotate_goal_plus_90(10.0, -14.0, -5.0) -> (-10.0, 14.0, -5.0)
            # The odom frame is initialized with a +90 deg offset relative to
            # the world frame, so the mapping from rotated local goal to world
            # coordinates is non-trivial. Empirically verified to be:
            #   world_x = start_world_x + goal_local_y_rot
            #   world_y = start_world_y - goal_local_x_rot
            goal_local_x_rot = -10.0
            goal_local_y_rot =  14.0
            goal_world_x = start_world_x + goal_local_y_rot   # = 20.0
            goal_world_y = start_world_y - goal_local_x_rot   # = 80.0
            goal_world_z = -5.0
            parts.append(
                self.marker_sphere_block(
                    "goal_marker",
                    goal_world_x, goal_world_y, goal_world_z,
                    r=0.35,
                    color=(0.9, 0.0, 0.0),
                )
            )

        return self.wrap_model("tc8_obstacles", "\n".join(parts))
    def build_case_9_sdf(self) -> str:
        parts = []

        square_side = 0.5
        start_x = 10.0
        nr_x = 4
        spacing_x = 2.0
        nr_y = 6
        start_y = 70.0
        spacing_y = 2.0

        row_xs = [start_x + i * spacing_x for i in range(nr_x)]
        pile_ys = [start_y + j * spacing_y for j in range(nr_y)]

        idx = 1
        for rx in row_xs:
            for py in pile_ys:
                parts.append(
                    self.box_block(
                        f"p{idx}", rx, py, -5.0,
                        square_side, square_side, 12.0,
                    )
                )
                idx += 1

        if self.visualize_start_goal:
            start_world_x = 6.0
            start_world_y = 70.0
            start_world_z = -5.0
            parts.append(
                self.marker_sphere_block(
                    "start_marker",
                    start_world_x, start_world_y, start_world_z,
                    r=0.35, color=(0.0, 0.9, 0.0),
                )
            )
            goal_local_x_rot = -10.0
            goal_local_y_rot =  14.0
            goal_world_x = start_world_x + goal_local_y_rot
            goal_world_y = start_world_y - goal_local_x_rot
            goal_world_z = -5.0
            parts.append(
                self.marker_sphere_block(
                    "goal_marker",
                    goal_world_x, goal_world_y, goal_world_z,
                    r=0.35, color=(0.9, 0.0, 0.0),
                )
            )

        return self.wrap_model("tc9_obstacles", "\n".join(parts))
    # ---------- helpers ----------

    def cylinder_block(self, name: str, x: float, y: float, z: float,
                       radius: float, length: float,
                       roll: float = 0.0, pitch: float = 0.0,
                       yaw: float = 0.0) -> str:
        return f"""      <collision name="{name}_collision">
            <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
            <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
        </collision>
        <visual name="{name}_visual">
            <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
            <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
            <material>
            <ambient>0.45 0.35 0.25 1</ambient>
            <diffuse>0.45 0.35 0.25 1</diffuse>
            <specular>0.45 0.35 0.25 1</specular>
            </material>
            <cast_shadows>false</cast_shadows>
        </visual>"""

    def call_command(self, cmd, label, accept_failure=False):
        self.get_logger().info(f"{label}...")
        try:
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            if result.stdout.strip():
                self.get_logger().info(result.stdout.strip())
            if result.stderr.strip():
                self.get_logger().warn(result.stderr.strip())
            return True, result.stdout.strip()
        except subprocess.CalledProcessError as e:
            stdout = e.stdout.strip() if e.stdout else ""
            stderr = e.stderr.strip() if e.stderr else ""
            if accept_failure:
                if stdout:
                    self.get_logger().info(stdout)
                if stderr:
                    self.get_logger().warn(stderr)
                return True, stderr or stdout or "ignored failure"
            self.get_logger().error(f"{label} failed.")
            if stdout:
                self.get_logger().error(stdout)
            if stderr:
                self.get_logger().error(stderr)
            return False, stderr or stdout or "unknown error"

    def wait_for_service_client(self, client, service_name: str, timeout_sec=5.0):
        return client.wait_for_service(timeout_sec=timeout_sec)

    def call_trigger_client(self, client, service_name: str, timeout_sec=10.0):
        request = Trigger.Request()
        future = client.call_async(request)
        start_time = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start_time > timeout_sec:
                return False, "timeout"
            time.sleep(0.05)
        response = future.result()
        if response is None:
            return False, "future.result() is None"
        return bool(response.success), response.message

    def publish_stop_commands(self):
        msg = String()
        msg.data = "other"
        self.cmd_state_pub.publish(msg)
        vel = Twist()
        self.vel_reference_pub.publish(vel)

    def activate_controller(self):
        msg = String()
        msg.data = "auv_controller"
        self.cmd_state_pub.publish(msg)

    def wait_for_goal_subscriber(self, timeout_sec=3.0):
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout_sec:
            if self.waypoint_pub.get_subscription_count() > 0:
                return True
            time.sleep(0.1)
        self.get_logger().warn("No subscribers detected on goal topic.")
        return False

    def publish_goal(self, x, y, z, repeats=10, period_sec=0.2):
        for _ in range(repeats):
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = "odom"
            goal.pose.position.x = float(x)
            goal.pose.position.y = float(y)
            goal.pose.position.z = float(z)
            goal.pose.orientation.w = 1.0
            self.waypoint_pub.publish(goal)
            time.sleep(period_sec)

    def delete_model_if_present(self, model_name: str):
        cmd = [
            "gz", "service",
            "-s", f"/world/{self.world_name}/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", f'name: "{model_name}", type: MODEL'
        ]
        ok, msg = self.call_command(
            cmd, f"Removing model {model_name}", accept_failure=True
        )
        return ok, msg

    def clear_all_obstacles(self):
        for model_name in self.all_obstacle_model_names:
            self.delete_model_if_present(model_name)

    def spawn_obstacle_model(self, model_name: str, sdf_text: str):
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".sdf", delete=False
        ) as f:
            f.write(sdf_text)
            sdf_path = f.name

        cmd = [
            "gz", "service",
            "-s", f"/world/{self.world_name}/create",
            "--reqtype", "gz.msgs.EntityFactory",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000",
            "--req",
            f'sdf_filename: "{sdf_path}", name: "{model_name}"'
        ]
        return self.call_command(cmd, f"Spawning obstacle model {model_name}")

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.odom_x = pos.x
        self.odom_y = pos.y
        self.odom_z = pos.z

        if not self.test_running:
            return

        # Check if Euclidean distance to goal is within tolerance

        # Check if Euclidean distance without np
        goal_reached = ((self.odom_x - self.goal_x) ** 2 +
                        (self.odom_y - self.goal_y) ** 2 +
                        (self.odom_z - self.goal_z) ** 2) < self.goal_tolerance_xyz ** 2

        if goal_reached:
            self.cancel_timeout_timer()
            self.test_running = False
            self.run_count += 1

            self.publish_test_status(
                f"STOPPED:goal_reached:{self.active_scenario_name}:"
                f"run_{self.run_count}_of_{self.nr_runs_total}"
            )
            time.sleep(0.2)  # let the bag capture the status message
            self.stop_bag_recording()

            self.get_logger().info(
                f"Goal reached for {self.active_scenario_name} "
                f"[run {self.run_count}/{self.nr_runs_total}]."
            )

            if self.run_count < self.nr_runs_total:
                # Automatically restart the same scenario
                self.get_logger().info(
                    f"Restarting {self.active_scenario_name} "
                    f"for run {self.run_count + 1}/{self.nr_runs_total}..."
                )
                scenario = self.scenarios[self.active_scenario_name]
                dummy_response = type(
                    "Response", (), {"success": False, "message": ""}
                )()
                self.run_scenario(scenario, dummy_response)
            else:
                self.get_logger().info(
                    f"Completed all {self.nr_runs_total} runs "
                    f"for {self.active_scenario_name}."
                )
                self.continue_or_finish_sequence()
    
    def _clear_contact_cooldown(self):
        self.contact_cooldown = False
        self.destroy_timer(self._cooldown_timer)
        self._cooldown_timer = None

    # ---------- main scenario runner ----------

    def run_scenario(self, scenario: Scenario, response: Trigger.Response):
        
        self.active_scenario_name = scenario.name
        case_nr = self.scenario_case_nr[scenario.name]

        self.goal_x, self.goal_y, self.goal_z = self.rotate_goal_plus_90(
            scenario.goal_local_x,
            scenario.goal_local_y,
            scenario.goal_local_z,
        )
        self.test_running = False

        self.get_logger().info(
            f"Scenario goal raw=({scenario.goal_local_x:.3f}, "
            f"{scenario.goal_local_y:.3f}, {scenario.goal_local_z:.3f}), "
            f"rotated=({self.goal_x:.3f}, {self.goal_y:.3f}, {self.goal_z:.3f})"
        )

        q = quaternion_from_euler(0.0, 0.0, scenario.start_yaw_rad)

        pause_cmd = [
            "gz", "service",
            "-s", f"/world/{self.world_name}/control",
            "--reqtype", "gz.msgs.WorldControl",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", "pause: true"
        ]
        start_cmd = [
            "gz", "service",
            "-s", f"/world/{self.world_name}/control",
            "--reqtype", "gz.msgs.WorldControl",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", "pause: false"
        ]
        set_pose_cmd = [
            "gz", "service",
            "-s", f"/world/{self.world_name}/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req",
            (
                f'name: "{self.model_name}", '
                f'position: {{x: {scenario.start_world_x}, '
                f'y: {scenario.start_world_y}, '
                f'z: {scenario.start_world_z}}}, '
                f'orientation: {{x: {q[0]}, y: {q[1]}, '
                f'z: {q[2]}, w: {q[3]}}}'
            )
        ]

        # Stop any previous bag recording before starting a new scenario
        self.stop_bag_recording()
        self.publish_stop_commands()

        # Pause
        ok, msg = self.call_command(pause_cmd, "Pausing simulation")
        if not ok:
            response.success = False
            response.message = f"Pause failed: {msg}"
            return response

        # Move robot FIRST while paused
        ok, msg = self.call_command(set_pose_cmd, "Setting robot pose")
        if not ok:
            response.success = False
            response.message = f"Set pose failed: {msg}"
            return response

        # Then clear and spawn obstacles
        self.clear_all_obstacles()
        ok, msg = self.spawn_obstacle_model(
            scenario.obstacle_model_name,
            scenario.obstacle_sdf,
        )
        if not ok:
            response.success = False
            response.message = f"Spawn obstacles failed: {msg}"
            return response

        if self.wait_for_service_client(
            self.reset_ego_fsm_client,
            "/ego_planner/reset_ego_replan_fsm", 2.0
        ):
            self.call_trigger_client(
                self.reset_ego_fsm_client,
                "/ego_planner/reset_ego_replan_fsm", 5.0
            )
        if self.wait_for_service_client(
            self.reset_traj_server_client,
            "/ego_traj_server/reset_traj_server", 2.0
        ):
            self.call_trigger_client(
                self.reset_traj_server_client,
                "/ego_traj_server/reset_traj_server", 5.0
            )
        if self.wait_for_service_client(
            self.reset_odometry_viz_client,
            "/odometry_visualization/reset_odometry_visualization", 2.0
        ):
            self.call_trigger_client(
                self.reset_odometry_viz_client,
                "/odometry_visualization/reset_odometry_visualization", 5.0
            )
        if self.wait_for_service_client(
            self.initialize_client, "/initialize", 2.0
        ):
            ok, msg = self.call_trigger_client(
                self.initialize_client, "/initialize", 10.0
            )
            if not ok:
                response.success = False
                response.message = f"Initialize failed: {msg}"
                return response
        else:
            response.success = False
            response.message = "/initialize service not available"
            return response

        ok, msg = self.call_command(start_cmd, "Restarting simulation")
        if not ok:
            response.success = False
            response.message = f"Restart failed: {msg}"
            return response

        time.sleep(self.settle_time_sec)

        if self.wait_for_service_client(
            self.initialize_client, "/initialize", 2.0
        ):
            ok, msg = self.call_trigger_client(
                self.initialize_client, "/initialize", 10.0
            )
            if not ok:
                response.success = False
                response.message = f"Initialize failed: {msg}"
                return response
        else:
            response.success = False
            response.message = "/initialize service not available"
            return response

        # Start bag recording before activating the controller
        if not self.start_bag_recording(case_nr):
            self.get_logger().warn(
                "Failed to start bag recording, continuing without it."
            )

        self.activate_controller()
        time.sleep(1.0)
        self.wait_for_goal_subscriber(3.0)
        self.publish_goal(self.goal_x, self.goal_y, self.goal_z)

        self.contact_cooldown = True
        self._cooldown_timer = self.create_timer(
            5.0, self._clear_contact_cooldown, callback_group=self.client_group
        )
        self.collision_detected = False
        self.test_running = True

        self.publish_test_status(
            f"STARTED:{scenario.name}:run_{self.run_count + 1}_of_{self.nr_runs_total}"
        )
        self.start_timeout_timer()
        response.success = True
        response.message = f"{scenario.name} started"
        return response

    # ---------- service handlers ----------

    def handle_start_test_case_1(self, request, response):
        del request
        return self.run_scenario(self.scenarios["case_1_single_cube"], response)

    def handle_start_test_case_2(self, request, response):
        del request
        return self.run_scenario(self.scenarios["case_2_corridor"], response)

    def handle_start_test_case_3(self, request, response):
        del request
        return self.run_scenario(self.scenarios["case_3_clutter_same_size"], response)

    def handle_start_test_case_4(self, request, response):
        del request
        return self.run_scenario(self.scenarios["case_4_clutter_varied"], response)

    def handle_start_test_case_5(self, request, response):
        del request
        return self.run_scenario(self.scenarios["case_5_vertical_triangle"], response)

    def handle_start_test_case_6(self, request, response):
        del request
        return self.run_scenario(self.scenarios["case_6_dense_spheres"], response)

    def handle_start_test_case_7(self, request, response):
        del request
        return self.run_scenario(self.scenarios["case_7_pylon"], response)

    def handle_start_test_case_8(self, request, response):
        del request
        self.scenario_sequence = ["case_8_dock_piles", "case_9_square_piles"]
        self.sequence_index = 0
        self.run_count = 0
        self.active_scenario_name = self.scenario_sequence[0]
        return self.run_scenario(self.scenarios[self.active_scenario_name], response)
    
    def handle_start_test_case_9(self, request, response):
        del request
        return self.run_scenario(self.scenarios["case_9_square_piles"], response)
    
    def destroy_node(self):
        self.get_logger().info("Shutting down, stopping bag recording...")
        self.cancel_timeout_timer()
        self.stop_bag_recording()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StartEgoScenarioService()
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