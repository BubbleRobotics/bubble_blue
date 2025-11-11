#!/usr/bin/env python3
"""
RRT path follower node (ROS 2, rclpy).
- Plans in local ENU (z up) with simple obstacle models.
- Sends position setpoints to the FCU via pymavlink (LOCAL_NED).
- Uses MAVROS services to set mode / arm.

Author: Luis Blunschi (adapted to ROS2 node)
"""

import math
import time
import threading
import random
from dataclasses import dataclass
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.parameter import Parameter
from mavros_msgs.srv import CommandBool, SetMode
from pymavlink import mavutil
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from robot_localization.srv import SetPose
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from path_planner_interfaces.srv import InitiatePath, SetObstacles
from path_planner_interfaces.msg import Sphere, AABB, OrientedBox
from std_srvs.srv import Trigger
from std_msgs.msg import Header
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import State as mavState
from tf_transformations import euler_from_quaternion


# ============
# Data types
# ============

@dataclass
class Bounds3D:
    minx: float; miny: float; minz: float
    maxx: float; maxy: float; maxz: float

@dataclass
class State:
    x: float; y: float; z: float
    yaw: float = 0.0

@dataclass
class NodeT:
    s: State
    parent: int

"""
@dataclass
class Sphere:
    cx: float; cy: float; cz: float; r: float

@dataclass
class AABB:
    minx: float; miny: float; minz: float
    maxx: float; maxy: float; maxz: float

@dataclass
class OrientedBox:
    cx: float; cy: float; cz: float
    sx: float; sy: float; sz: float
    yaw: float  # rad"""

@dataclass
class RobotBox:
    hx: float
    hy: float
    hz: float
    z_offset: float



# ==============
# ROS2 Node
# ==============

class RrtPathFollowerNode(Node):
    def __init__(self):
        super().__init__('rrt_path_follower')

        # ---- parameters (declare + get) ----
        self.declare_parameters(
            namespace='',
            parameters=[
                ('mavlink_url', 'udp:127.0.0.1:14550'),
                ('set_mode', 'GUIDED'),
                ('arm', True),
                ('goal', [11.0, 13.45, -5.2]),
                ('bounds', [-10.0, -10.0, -10.0, 20.0, 20.0, 1.0]),  # minx,miny,minz,maxx,maxy,maxz
                ('step_size', 0.30),
                ('edge_res', 0.05),
                ('goal_bias', 0.10),
                ('max_iters', 30000),
                ('reach_thresh', 0.10),
                ('final_yaw_deg', -15.6923),
                ('near_goal_radius', 2.0),
                ('safety_buffer', 0.15)
            ],
        )

        self._build_obstacles() # Build standard obstacles and robot box
        
        self.bounds_list = self.get_parameter('bounds').get_parameter_value().double_array_value
        self.bounds = Bounds3D(*self.bounds_list)
        self.step      = float(self.get_parameter('step_size').value)
        self.edge_res  = float(self.get_parameter('edge_res').value)
        self.goal_bias = float(self.get_parameter('goal_bias').value)
        self.max_iters = int(self.get_parameter('max_iters').value)
        self.threshold    = float(self.get_parameter('reach_thresh').value)
        self.final_yaw = float(self.get_parameter('final_yaw_deg').value)
        self.near_goal = float(self.get_parameter('near_goal_radius').value)
        self.safety_buffer = float(self.get_parameter('safety_buffer').value)

        self.q_start = State(0.0, 0.0, -1)
        goal_xyz = self.get_parameter('goal').get_parameter_value().double_array_value
        self.q_goal  = State(goal_xyz[0], goal_xyz[1], goal_xyz[2], 0.0)

        self.get_logger().info(f'RRT path from start ENU ({self.q_start.x}, {self.q_start.y}, {self.q_start.z})')
        self.get_logger().info(f'            to goal ENU ({self.q_goal.x},  {self.q_goal.y},  {self.q_goal.z})')
        
        self.aabbs_added = []

        self.mavlink_url: str = self.get_parameter('mavlink_url').get_parameter_value().string_value
        
        # MAVROS subscribers
        # QoS: small queue, best-effort is fine for pose

        self.current_pose = State(0.0, 0.0, 0.0, 0.0)
        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov', self._pose_cb, 10)
        self._last_pose = None
        self._pose_lock = threading.Lock()
        # MAVROS setpoint publishers
        self.setpoint_local_pub = self.create_publisher(PositionTarget,'/mavros/setpoint_raw/local',10)
        # MAVROS service clients
        self.set_mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_cli      = self.create_client(CommandBool, '/mavros/cmd/arming')
        # --- worker management ---
        self._worker = None
        self._cancel_evt = threading.Event()
        self._worker_lock = threading.Lock()
        
        # Service to plan + follow path
        self.path_follow_srv = self.create_service(
            InitiatePath,
            'path_planner/follow_rrt_path',
            self.handle_follow_rrt_path
        )
        self.obstacle_adder_srv = self.create_service(
            SetObstacles,
            'path_planner/add_obstacles',
            self.handler_add_obstacles
        )

        self.obstacle_setter_srv = self.create_service(
            SetObstacles,
            'path_planner/set_obstacles',
            self.handler_set_obstacles
        )

        self.replanner_srv = self.create_service(
            Trigger,
            'path_planner/replan',
            self.handler_replan
        )
        
        
        self._is_connected = False

        # Create a subscription to listen for MAVROS heartbeat (state)
        self._state_sub = self.create_subscription(
            mavState, '/mavros/state', self.state_callback, 10
        )

        self.get_logger().info('Waiting for MAVLink heartbeat (MAVROS state)...')

        # Non-blocking loop to wait for heartbeat and process callbacks
        start_time = time.time()
        timeout = 10  # Timeout in seconds

        while not self._is_connected and (time.time() - start_time) < timeout:
            rclpy.spin_once(self)  # Process messages (this calls the callback)
            self.get_logger().info(f'Waiting for heartbeat...{self._is_connected}')
            time.sleep(0.5)

        if self._is_connected:
            self.get_logger().info('Connected to MAVLink vehicle.')
        else:
            self.get_logger().warn(f"Heartbeat not received within {timeout} seconds.")
            # Handle timeout (e.g., exit or try again)
            raise ValueError("Path Planner Node Timed out - No Heartbeat from MAVROS")

        
        self.q_start = self.current_pose
        
        self.destroy_subscription(self._state_sub)

        self.get_logger().info('Path planner: rrt_path_follower node initialized.')


    def state_callback(self, msg: mavState):
        # Check if the connection is established
        
        if msg.connected and not self._is_connected:
            self._is_connected = True
            

    def _start_new_worker(self):
        """Cancel any existing worker and start a new one."""
   
        self.goto_position(self.current_pose.x,
                            self.current_pose.y,
                            self.current_pose.z,
                            self.current_pose.yaw)

        with self._worker_lock:
            # 1) ask the current worker to stop
            if self._worker and self._worker.is_alive():
                self._cancel_evt.set()
                self.get_logger().info("Stopping previous worker...")
                self._worker.join(timeout=2.0)
                if self._worker.is_alive():
                    self.get_logger().warn("Previous worker did not stop within timeout; it will be abandoned.")
            # 2) clear cancel flag and start a new worker
            self._cancel_evt.clear()
            self._worker = threading.Thread(target=self._run_wrapper, daemon=True)
            self._worker.start()

    def _should_cancel(self) -> bool:
        return self._cancel_evt.is_set()
    
    def _run_wrapper(self):
        try:
            self._run()
        except Exception:
            self.get_logger().warn("Worker crashed")
        finally:
            # nothing to cleanup right now
            pass

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        _,_,yaw = euler_from_quaternion(quat)
        self.current_pose = State(x=position.x, y=position.y, z=position.z, yaw=yaw)
        
            
        
    # ===== service handler =====
    

    def handle_follow_rrt_path(self, request: InitiatePath.Request, response: InitiatePath.Response):
        goal_x = request.pose.position.x
        goal_y = request.pose.position.y
        goal_z = request.pose.position.z
        
        
        self.get_logger().info(
            "\n\n##################################################################\n"
        )
        self.get_logger().info(
            f"Service request: follow RRT path to ({goal_x}, {goal_y}, {goal_z})"
        )
        self.q_goal = State(goal_x, goal_y, goal_z)

        self._start_new_worker()

        # Send simple acknowledgment
        response.success = True
        return response

    def handler_add_obstacles(self, request: SetObstacles.Request, response: SetObstacles.Response):
        self.get_logger().info(
            "\n\n##################################################################\n"
        )
        self.get_logger().info(
            "Service request: Add Obstacle)"
        )
        
        self.aabbs_added = [] #For now do not add them to the aabbs becuase they cannot be removed again
        # Change obstacles and for now just replan
        for _, sphere in enumerate(request.spheres):
            self.spheres.append(sphere)

        for _, aabb in enumerate(request.aabbs):
            self.aabbs_added.append(aabb)

        for _, oriented_box in enumerate(request.oriented_boxes):
            self.oriented_boxes.append(oriented_box)    

        self.get_logger().info("Added a new obstacle!")
        # Start worker thread to run the mission
        self._start_new_worker()

        # Send simple acknowledgment
        response.success = False
        return response

    def handler_set_obstacles(self, request: SetObstacles.Request, response: SetObstacles.Response):
        # Change obstacles and for now just replan
        self.get_logger().info(
            "\n\n##################################################################\n"
        )
        self.get_logger().info(
            "Service request: Set Obstacle)"
        )
        
        self.spheres = request.spheres
        self.aabbs = request.aabbs
        self.oriented_boxes = request.oriented_boxes
        

        # Start worker thread to run the mission
        self._start_new_worker()

        # Send simple acknowledgment
        response.success = True
        return response
    
    def handler_replan(self, request: Trigger.Request, response: Trigger.Response):
        
        self.get_logger().info(
            "\n\n##################################################################\n"
        )
        self.get_logger().info(
            "Service request: Replan"
        )
        # Start worker thread to run the mission
        self._start_new_worker()

        # Send simple acknowledgment
        response.success = True
        return response
    
    # ===== MAVROS helpers =====

    def set_mode(self, mode: str) -> bool:
        if not self.set_mode_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Set mode service unavailable')
            return False
        req = SetMode.Request()
        req.custom_mode = mode
        fut = self.set_mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        ok = bool(res and res.mode_sent)
        self.get_logger().info(f'Set mode {mode}: {"OK" if ok else "FAIL"}')
        return ok

    def arm(self, value: bool) -> bool:
        if not self.arm_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Arming service unavailable')
            return False
        req = CommandBool.Request()
        req.value = value
        fut = self.arm_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        ok = bool(res and res.success)
        self.get_logger().info(f'Arm({value}): {"OK" if ok else "FAIL"}')
        return ok


    def goto_position(self, x_east_m: float, y_north_m: float, up_m: float, yaw_deg: Optional[float] = 0.0) -> None:
  
        msg = PositionTarget()
        # Use local NED frame (MAVROS translates correctly)
        msg.header.frame_id = "map"

        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # Type mask (bits = ignore fields)
        # We want to send ONLY position + yaw
        # ignore velocity, accel, yaw_rate
        msg.type_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        # Desired position (NED)
        msg.position.x = float(x_east_m)    # North
        msg.position.y = float(y_north_m)    # East
        msg.position.z = float(up_m)        # Down

        # Desired yaw (rad)
        msg.yaw = float(math.radians(yaw_deg))
        self.setpoint_local_pub.publish(msg)
         
    # ==================
    # Geometry helpers
    # ==================
    def reached_goal(self, x_goal_e: float, y_goal_n: float, up_goal: float, yaw_goal_deg: float) -> Tuple[bool, float]:
        
    
        dx = x_goal_e - self.current_pose.x
        dy = y_goal_n - self.current_pose.y
        dz = up_goal - self.current_pose.z + 0.15
        dyaw = (yaw_goal_deg - self.current_pose.yaw) / 100.0
        total = math.sqrt(dx*dx + dy*dy + dz*dz + 0.0*dyaw*dyaw)
        self.get_logger().info(f"Current: dx={dx:.7f}, dy={dy:.7f}, dz={dz:.2f} m, dyaw={dyaw}, total_dist={total:.2f} m")
        return (total < self.threshold, total)

    def dist_xyz(self, a: State, b: State) -> float:
        dx = a.x - b.x
        dy = a.y - b.y
        dz = a.z - b.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def in_bounds(self, p: State) -> bool:
        return (self.bounds.minx <= p.x <= self.bounds.maxx and
                self.bounds.miny <= p.y <= self.bounds.maxy and
                self.bounds.minz <= p.z <= self.bounds.maxz)

    def collides_robot(self, p_base: State) -> bool:
        """
        Collision check with a safety buffer (meters). Implements a Minkowski sum:
        - Spheres: inflate obstacle radius by robot sphere radius + buffer.
        - aabbs:   expand each face by robot half-size + buffer.
        - oriented_boxes:    expand each half-extent by robot half-size + buffer (in OBB frame).
        """
        pc = State(p_base.x, p_base.y, p_base.z + self.rb.z_offset, 0.0)

        # Treat robot as a bounding sphere for sphere-vs-robot checks
        r_robot_sphere = math.sqrt( self.rb.hx* self.rb.hx +  self.rb.hy*self.rb.hy + self.rb.hz*self.rb.hz) + self.safety_buffer

        # === spheres (inflate by robot sphere radius + buffer) ===
        for sp in self.spheres:
            dx = pc.x - sp.cx
            dy = pc.y - sp.cy 
            dz = pc.z - sp.cz
            if (dx*dx + dy*dy + dz*dz) <= (sp.r + r_robot_sphere) ** 2:
                return True

        # === AABBs (inflate by robot half-size + buffer) ===
        inf_x = self.rb.hx + self.safety_buffer
        inf_y = self.rb.hy + self.safety_buffer
        inf_z = self.rb.hz + self.safety_buffer
        for b in self.aabbs:
            if (b.minx - inf_x <= pc.x <= b.maxx + inf_x and
                b.miny - inf_y <= pc.y <= b.maxy + inf_y and
                b.minz - inf_z <= pc.z <= b.maxz + inf_z):
                return True
        for b in self.aabbs_added:
            if (b.minx - inf_x <= pc.x <= b.maxx + inf_x and
                b.miny - inf_y <= pc.y <= b.maxy + inf_y and
                b.minz - inf_z <= pc.z <= b.maxz + inf_z):
                return True
            
        # === oriented_boxes (inflate by robot half-size + buffer in local frame) ===
        for obb in self.oriented_boxes:
            dx = pc.x - obb.cx
            dy = pc.y - obb.cy
            dz = pc.z - obb.cz
            c, s = math.cos(obb.yaw), math.sin(obb.yaw)
            lx =  c*dx + s*dy
            ly = -s*dx + c*dy
            lz = dz

            # obstacle half-sizes
            hx_o, hy_o, hz_o = obb.sx * 0.5, obb.sy * 0.5, obb.sz * 0.5
            # inflate by robot half-size + buffer
            hx_inf = hx_o + self.rb.hx + self.safety_buffer
            hy_inf = hy_o + self.rb.hy + self.safety_buffer
            hz_inf = hz_o + self.rb.hz + self.safety_buffer

            if abs(lx) <= hx_inf and abs(ly) <= hy_inf and abs(lz) <= hz_inf:
                return True

        return False


    def edge_free(self, a: State, b: State) -> bool:
        L = self.dist_xyz(a, b)
        steps = max(1, int(math.ceil(L / self.edge_res)))
        for step_i in range(steps + 1):
            t = step_i / steps
            p = State(a.x + t*(b.x - a.x),
                    a.y + t*(b.y - a.y),
                    a.z + t*(b.z - a.z), 0.0)
            if not self.in_bounds(p): 
                return False
            if self.collides_robot(p): 
                return False
        return True

    def optimize_path_shortcut(self, path: List[State], max_passes: int = 10) -> List[State]:
        if len(path) < 3:
            return path
        pts = list(path)
        for _ in range(max_passes):
            changed = False
            idx = 0
            while idx < len(pts) - 2:
                jumped = False
                for j in range(len(pts) - 1, idx + 1, -1):
                    if j == idx + 1:
                        continue
                    if self.edge_free(pts[idx], pts[j]):
                        del pts[idx + 1:j]
                        changed = True
                        jumped = True
                        break
                if not jumped:
                    idx += 1
            if not changed:
                break
        return pts

    def nearest(self, tree: List[NodeT], q: State) -> int:
        """FIXED: choose the index with min distance."""
        best = float("inf")
        idx = -1
        for tree_idx, n in enumerate(tree):
            d = self.dist_xyz(n.s, q)
            if d < best:
                best = d
                idx = tree_idx
        return idx

    def steer_toward(self, qn: State, qt: State) -> State:
        dx, dy, dz = qt.x - qn.x, qt.y - qn.y, qt.z - qn.z
        d = math.sqrt(dx*dx + dy*dy + dz*dz)
        if d < 1e-9:
            return State(qt.x, qt.y, qt.z, 0.0)
        t = min(self.step / d, 1.0)
        return State(qn.x + t*dx, qn.y + t*dy, qn.z + t*dz, 0.0)

    def connect_toward(self, tree: List[NodeT], q_target: State) -> int:
        idx = self.nearest(tree, q_target)
        qn = tree[idx].s
        while True:
            qnext = self.steer_toward(qn, q_target)
            if not self.edge_free(qn, qnext):
                return idx
            tree.append(NodeT(qnext, idx))
            idx = len(tree) - 1
            qn = qnext
            if self.dist_xyz(qn, q_target) < 1e-9:
                return idx
            if self.dist_xyz(qn, q_target) <= self.step:
                qnext2 = self.steer_toward(qn, q_target)
                if not self.edge_free(qn, qnext2):
                    return idx
                tree.append(NodeT(qnext2, idx))
                return len(tree) - 1

    def backtrack(self, tree: List[NodeT], idx: int) -> List[State]:
        path: List[State] = []
        while idx >= 0:
            path.append(tree[idx].s)
            idx = tree[idx].parent
        return list(reversed(path))


    # ===== RRT planning =====

    def _build_obstacles(self):

        self.spheres: List[Sphere] = []

        self.aabbs: List[AABB] = []
        self.aabbs.append(AABB(minx=2 - 0.25, miny=-20, minz=-12,  maxx=2 + 0.25, maxy= 20,  maxz=-4.5))
        self.aabbs.append(AABB(minx=2 - 0.25, miny=-20, minz=-2.5, maxx=2 + 0.25, maxy=20,   maxz=2))
        self.aabbs.append(AABB(minx=2 - 0.25, miny=-20, minz=-12,  maxx=2 + 0.25, maxy=-1.5, maxz=1))
        self.aabbs.append(AABB(minx=2 - 0.25, miny=1.5, minz=-12,  maxx=2 + 0.25, maxy=20,   maxz=1))

        self.oriented_boxes: List[OrientedBox] = [
            OrientedBox(cx=11.57346922, cy=12.78198424, cz=-4.25, sx=1.04922147, sy=0.25,  sz=1.5,  yaw=-1.844678),
            OrientedBox(cx=10.722990,   cy=3.746699,    cz=-5.0,  sx=50.0,       sy=3.25,  sz=16.0, yaw=-1.844678),
        ]

        #robot = RobotBox(hx=0.457*0.5, hy=0.338*0.5, hz=0.25*0.5, z_offset=0.0)
        worst_case_box_len = math.sqrt((0.457/2)**2+(0.338/2)**2)
        self.get_logger().info(f"Worst case box: {worst_case_box_len}")
        self.rb = RobotBox(hx=worst_case_box_len, hy=worst_case_box_len, hz=0.25*0.5, z_offset=0.0) # For now yaw is not taken into account in collision checker, thus the worst case bounding box is assumed
        

    def _rrt_plan(self) -> List[State]:
        

        start_collision = self.collides_robot(self.q_start)
        goal_collision = self.collides_robot(self.q_goal)
        self.get_logger().info(f'Start collision: {start_collision}')
        self.get_logger().info(f'Goal collision: {goal_collision}')
        start_out_of_bounds = not self.in_bounds(self.q_start)
        goal_out_of_bounds = not self.in_bounds(self.q_goal)
        self.get_logger().info(f'Start out of bounds: {start_out_of_bounds}')
        self.get_logger().info(f'Goal out of bounds: {goal_out_of_bounds}')

        if start_collision or goal_collision or start_out_of_bounds or goal_out_of_bounds:
            self.get_logger().error('Path impossible!')
            return []

        Ta: List[NodeT] = [NodeT(self.q_start, -1)]
        Tb: List[NodeT] = [NodeT(self.q_goal,  -1)]
        rng = random.Random()

        def sample() -> State:
            if rng.random() < self.goal_bias:
                return State(self.q_goal.x, self.q_goal.y, self.q_goal.z, 0.0)
            return State(
                rng.uniform(self.bounds.minx, self.bounds.maxx),
                rng.uniform(self.bounds.miny, self.bounds.maxy),
                rng.uniform(self.bounds.minz, self.bounds.maxz),
                0.0
            )

        success = False
        meet_a = meet_b = -1
        for iter in range(self.max_iters):
            if self._should_cancel():
                break
            if iter % 1000 == 0:
                self.get_logger().info(f'RRT iter {iter}, |Ta|={len(Ta)}, |Tb|={len(Tb)}')
            q_rand = sample()
            new_a = self.connect_toward(Ta, q_rand)
            q_new_a = Ta[new_a].s
            new_b = self.connect_toward(Tb, q_new_a)
            if self.dist_xyz(Tb[new_b].s, q_new_a) < self.step * 0.5:
                success = True
                meet_a = new_a
                meet_b = new_b
                break
            Ta, Tb = Tb, Ta  # swap

        if not success:
            self.get_logger().error('RRT failed to find a path')
            return []

        path_a = self.backtrack(Ta, meet_a)
        path_b = self.backtrack(Tb, meet_b)
        path_b.reverse()
        if path_a and path_b and self.dist_xyz(path_a[-1], path_b[0]) < 1e-6:
            path_b = path_b[1:]
        path = path_a + path_b

        # downsample a bit
        if len(path) > 2:
            filtered = [path[0]]
            for i in range(1, len(path)-1):
                if i % 2 == 0:
                    filtered.append(path[i])
            filtered.append(path[-1])
            path = filtered

        # shortcut
        path = self.optimize_path_shortcut(path)

        if path and self.dist_xyz(path[0], self.q_start) > self.dist_xyz(path[-1], self.q_start):
            path = list(reversed(path))
        self.get_logger().info(f'Planned path with {len(path)} waypoints')
        return path

    # ===== mission thread =====

    def _run(self) -> None:
        # parameters
        self.q_start = self.current_pose
        self.goto_position(self.current_pose.x,
                            self.current_pose.y,
                            self.current_pose.z,
                            self.current_pose.yaw)

        self.get_logger().info(f'RRT path from start ENU ({self.q_start.x}, {self.q_start.y}, {self.q_start.z})')
        self.get_logger().info(f'            to goal ENU ({self.q_goal.x},  {self.q_goal.y},  {self.q_goal.z})')

        # mode + arm
        if self.get_parameter('set_mode').value:
            self.set_mode(self.get_parameter('set_mode').value)
        if self.get_parameter('arm').value:
            self.get_logger().info("Arming")
            self.arm(True)
            

        self.get_logger().info("Started planning path")
        # plan
        path = self._rrt_plan()
        if not path:
            return
        self.get_logger().info("Finished planning path")
        # execute
        failed = False
        for path_i, wp in enumerate(path):
            # choose yaw
            if failed:
                break
            if self._should_cancel():
                break
            dgoal = self.dist_xyz(wp, self.q_goal)
            yaw_cmd = self.final_yaw if dgoal < self.near_goal else 0.0
            
            self.goto_position(wp.x, wp.y, wp.z, yaw_cmd)
            self.get_logger().info(f'[{path_i+1}/{len(path)}] goto ENU x={wp.x:.2f} y={wp.y:.2f} z={wp.z:.2f} yaw={yaw_cmd:.1f}')

            start_t = time.time()
            last_dist = None
            stuck_cnt = 0
            while True:
                try:
                    ok, dist = self.reached_goal(wp.x, wp.y, wp.z, yaw_cmd)
                except Exception:
                    ok, dist = False, 1e9

                if last_dist is None:
                    last_dist = dist
                else:
                    if abs(last_dist - dist) < 0.01:
                        stuck_cnt += 1
                    else:
                        stuck_cnt = 0
                    last_dist = dist

                if ok:
                    self.get_logger().info(f'  reached (err≈{dist:.2f} m)')
                    break
                if time.time() - start_t > 180.0 or stuck_cnt > 100:
                    self.get_logger().warn(f'  time since start_t:{time.time() - start_t} and stuck_cnt: {stuck_cnt}')
                    self.get_logger().warn(f'  timeout/stuck (err≈{dist:.2f} m), continue')
                    self.get_logger().warn("Aborting mission!")
                    failed = True
                    break
                time.sleep(0.3)
        if not failed:
            self.get_logger().info('\n\nRRT path complete\n\n##################################################################\n')
        else:
            self.get_logger().info('\n\nProcess died or failed\n\n#############################################################\n')
def main(args=None):
    rclpy.init(args=args)
    node = RrtPathFollowerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
