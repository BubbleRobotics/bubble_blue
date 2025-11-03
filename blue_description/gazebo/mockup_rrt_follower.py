"""Luis Blunschi 31.10.2025
RRT path follower: generate a 3D path (ENU meters, z up) and fly it via MAVLink/ROS2.
- Obstacles: two oriented boxes (from SDF), plus a big wall at x=2, z∈[-7,1], y∈[-2,20].
- Start/Goal (ENU): start=(0,0,0), goal=(11.35433787, 13.38846827, -5)
- Bounds: x∈[-2,20], y∈[-2,20], z∈[-10,1] (z up)
"""
import pymap3d as pm
from pymavlink import mavutil
import math
import time
from mavros_msgs.srv import CommandBool, SetMode
import rclpy
from dataclasses import dataclass
from typing import List, Tuple
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# =========================
# MAVLink / ROS2 UTILITIES
# =========================

master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print("Connected to ROV")
time.sleep(0.5)

def set_mode(mode="GUIDED"):
    rclpy.init()
    node = rclpy.create_node('temp_set_mode_node')
    client = node.create_client(SetMode, '/mavros/set_mode')
    if not client.wait_for_service(timeout_sec=5.0):
        print("Set mode service not available!")
        node.destroy_node(); rclpy.shutdown(); return False
    req = SetMode.Request(); req.custom_mode = mode
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    node.destroy_node(); rclpy.shutdown()
    if result and result.mode_sent:
        print(f"Flight mode set to: {mode}"); return True
    print("Failed to set flight mode"); return False

def arm_vehicle_and_set_mode(arm=False, mode="MANUAL"):
    print(f"Setting mavros mode to {mode}")
    rclpy.init()
    node = rclpy.create_node('temp_arming_node')

    set_mode_client = node.create_client(SetMode, '/mavros/set_mode')
    if not set_mode_client.wait_for_service(timeout_sec=5.0):
        print("Set mode service not available!")
        node.destroy_node(); rclpy.shutdown(); return False
    req = SetMode.Request(); req.custom_mode = mode
    fut = set_mode_client.call_async(req)
    rclpy.spin_until_future_complete(node, fut)
    res = fut.result()
    if not (res and res.mode_sent):
        print("Failed to set flight mode"); node.destroy_node(); rclpy.shutdown(); return False
    print(f"Flight mode set to: {mode}")
    print("Arming vehicle" if arm else "Disarming vehicle")
    arming_client = node.create_client(CommandBool, '/mavros/cmd/arming')
    if not arming_client.wait_for_service(timeout_sec=5.0):
        print('Arming service not available!')
        node.destroy_node(); rclpy.shutdown(); return False
    req2 = CommandBool.Request(); req2.value = arm
    fut2 = arming_client.call_async(req2)
    rclpy.spin_until_future_complete(node, fut2)
    res2 = fut2.result()
    node.destroy_node(); rclpy.shutdown()
    print('Arming successful' if (res2 and res2.success) else 'Arming failed')
    return bool(res2 and res2.success)

def set_yaw(yaw_deg, speed_deg_per_s=10, relative=False):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0, yaw_deg, speed_deg_per_s, 0, 1 if relative else 0, 0, 0, 0
    )
    _ = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)

def goto_position(x_east_m, y_north_m, up_m, yaw_deg=0.0):
    x_north_m = y_north_m
    y_east_m = x_east_m
    depth_m = -up_m  # NED: positive down

    master.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms (can be 0)
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,   # local NED (absolute)
        int(0b110111111000),                   # use position + yaw; ignore vel/accel/yaw_rate
        x_north_m,                             # meters North of local origin
        y_east_m,                              # meters East of local origin
        depth_m,                               # NED: positive down (depth)
        0, 0, 0,                               # vx, vy, vz (ignored by mask)
        0, 0, 0,                               # ax, ay, az (ignored)
        math.radians(yaw_deg),                 # yaw (rad, absolute)
        0                                      # yaw_rate (ignored)
    )

    # optional: also send a yaw hold
    if yaw_deg is not None:
        set_yaw(yaw_deg)

def reached_goal(x_goal_east_m, y_goal_north_m, up_goal_m, yaw_goal_deg, threshold=0.15):
    pos_msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)

    

    if pos_msg is not None:
        # Convert from NED to ENU
        x_current_east_m = pos_msg.y  # meters North
        y_current_north_m = pos_msg.x  # meters East
        up_current_m = -pos_msg.z  # meters up (negative down)

    else:
        raise ValueError("No LOCAL_POSITION_NED message received")
    _ = master.recv_match(type='ATTITUDE', blocking=False, timeout=0.2)  # yaw ignored

    
    """if not pos_msg: return False, None
    lat_current = pos_msg.lat / 1e7
    lon_current = pos_msg.lon / 1e7"""

    # relative_alt is mm up; depth positive down
    #depth_current = -pos_msg.relative_alt / 1000.0
    # meters per degree (local)
    """m_per_deg_lat = 111000.0
    m_per_deg_lon = 111000.0 * math.cos(math.radians(lat_goal))"""
    x_dist = (x_goal_east_m - x_current_east_m) 
    y_dist = (y_goal_north_m - y_current_north_m)
    depth_dist = (up_goal_m - up_current_m) 
    yaw_dist = 0.0  # yaw completely ignored
    total = math.sqrt(x_dist**2 + y_dist**2 + depth_dist**2 + yaw_dist**2)
    return total < threshold, total

# =========================
# LOCAL ENU RRT PLANNER
# =========================

@dataclass
class Bounds3D:
    minx: float; miny: float; minz: float
    maxx: float; maxy: float; maxz: float

@dataclass
class State:
    x: float; y: float; z: float; yaw: float = 0.0  # yaw unused, stays 0

@dataclass
class Node:
    s: State; parent: int

@dataclass
class Sphere:
    cx: float; cy: float; cz: float; r: float

@dataclass
class AABB:
    minx: float; miny: float; minz: float; maxx: float; maxy: float; maxz: float

@dataclass
class OrientedBox:
    cx: float; cy: float; cz: float
    sx: float; sy: float; sz: float  # full sizes
    yaw: float

@dataclass
class RobotBox:
    hx: float; hy: float; hz: float; z_offset: float  # half sizes + center offset

def dist_xyz(a: State, b: State) -> float:
    dx = a.x - b.x; dy = a.y - b.y; dz = a.z - b.z
    return math.sqrt(dx*dx + dy*dy + dz*dz)

def in_bounds(p: State, b: Bounds3D) -> bool:
    return (b.minx <= p.x <= b.maxx) and (b.miny <= p.y <= b.maxy) and (b.minz <= p.z <= b.maxz)

def collides_robot(p_base: State, rb: RobotBox,
                   spheres: List[Sphere], aabbs: List[AABB], obbs: List[OrientedBox]) -> bool:
    """Collision check in ENU (z up). Yaw ignored; robot treated as axis-aligned box (via inflation)."""
    # Robot center (z up) with URDF offset
    pc = State(p_base.x, p_base.y, p_base.z + rb.z_offset, 0.0)

    
    r_sphere = math.sqrt(rb.hx*rb.hx + rb.hy*rb.hy + rb.hz*rb.hz)

    # Spheres
    for sp in spheres:
        dx = pc.x - sp.cx; dy = pc.y - sp.cy; dz = pc.z - sp.cz
        if (dx*dx + dy*dy + dz*dz) <= (sp.r + r_sphere)**2:
            return True

    # Axis-aligned boxes
    for b in aabbs:
        if (pc.x >= b.minx - rb.hx and pc.x <= b.maxx + rb.hx and
            pc.y >= b.miny - rb.hy and pc.y <= b.maxy + rb.hy and
            pc.z >= b.minz - rb.hz and pc.z <= b.maxz + rb.hz):
            return True

    # Oriented boxes (treat as AABBs in world; yaw ignored)
    for obb in obbs:
        # obb: cx, cy, cz, sx, sy, sz (full sizes), yaw (radians, about +z)
        dx = pc.x - obb.cx
        dy = pc.y - obb.cy
        dz = pc.z - obb.cz

        c = math.cos(obb.yaw)
        s = math.sin(obb.yaw)
        # world -> box-local (R^T)
        lx =  c*dx + s*dy
        ly = -s*dx + c*dy
        lz =  dz  # assuming only yaw about z

        hx, hy, hz = obb.sx*0.5, obb.sy*0.5, obb.sz*0.5
        if (abs(lx) <= hx and abs(ly) <= hy and abs(lz) <= hz):
            return True

    return False

def edge_free(a: State, b: State, bounds: Bounds3D, rb: RobotBox,
              spheres: List[Sphere], aabbs: List[AABB], obbs: List[OrientedBox], res: float) -> bool:
    L = dist_xyz(a, b)
    steps = max(1, int(math.ceil(L / res)))
    for i in range(steps+1):
        t = i / steps
        p = State(a.x + t*(b.x - a.x),
                  a.y + t*(b.y - a.y),
                  a.z + t*(b.z - a.z),
                  0.0)  # yaw ignored
        if not in_bounds(p, bounds): return False
        if collides_robot(p, rb, spheres, aabbs, obbs): return False
    return True

def nearest(tree: List[Node], q: State) -> int:
    best = 1e18; idx = -1
    for i, n in enumerate(tree):
        d = dist_xyz(n.s, q)
        if d < best: best = d; idx = i
    return idx

def steer_toward(qn: State, qt: State, max_step: float) -> State:
    dx = qt.x - qn.x; dy = qt.y - qn.y; dz = qt.z - qn.z
    d = math.sqrt(dx*dx + dy*dy + dz*dz)
    if d < 1e-9:
        return State(qt.x, qt.y, qt.z, 0.0)
    t = min(max_step / d, 1.0)
    return State(qn.x + t*dx, qn.y + t*dy, qn.z + t*dz, 0.0)

def connect_toward(tree: List[Node], q_target: State,
                   step: float, edge_res: float,
                   bounds: Bounds3D, rb: RobotBox,
                   spheres: List[Sphere], aabbs: List[AABB], obbs: List[OrientedBox]) -> int:
    idx = nearest(tree, q_target)
    qn = tree[idx].s
    while True:
        qnext = steer_toward(qn, q_target, step)
        if not edge_free(qn, qnext, bounds, rb, spheres, aabbs, obbs, edge_res):
            return idx
        tree.append(Node(qnext, idx))
        idx = len(tree) - 1
        qn = qnext
        if dist_xyz(qn, q_target) < 1e-9: return idx
        if dist_xyz(qn, q_target) <= step:
            qnext2 = steer_toward(qn, q_target, step)
            if not edge_free(qn, qnext2, bounds, rb, spheres, aabbs, obbs, edge_res):
                return idx
            tree.append(Node(qnext2, idx))
            return len(tree) - 1

def backtrack(tree: List[Node], idx: int) -> List[State]:
    path = []
    while idx >= 0:
        path.append(tree[idx].s)
        idx = tree[idx].parent
    return list(reversed(path))

def generate_rrt_path(q_start:State,q_goal:State,step_size:float,bounds:Bounds3D) -> List[State]:
    
    # Obstacles
    spheres: List[Sphere] = []
    aabbs: List[AABB] = []
    # Big wall at x=2, y∈[-2,20], z∈[-7,1], thickness 0.2 in x
    aabbs.append(AABB(2 - 0.25, -20, -12, 2 + 0.25, 20, -4.5))
    aabbs.append(AABB(2 - 0.25, -20, -2.5, 2 + 0.25, 20, 2))
    aabbs.append(AABB(2 - 0.25, -20, -12, 2 + 0.25, -1.5, 1))
    aabbs.append(AABB(2 - 0.25, 1.5, -12, 2 + 0.25, 20, 1))

    obbs: List[OrientedBox] = []
    # Oriented boxes (ENU z up: use SDF z as-is)
    obbs.append(OrientedBox(11.57346922, 12.78198424, -4.25, 1.04922147, 0.25, 1.5, -1.844678))
    obbs.append(OrientedBox(10.722990,    3.746699,   -5.0,  50.0, 3.25,  16.0, -1.844678))

    # Robot box (URDF box 0.457 x 0.338 x 0.065; center offset +0.06 in z up)
    robot = RobotBox(hx=0.457*0.5, hy=0.338*0.5, hz=0.25*0.5, z_offset=0.00)

    # Planner params
    step = step_size
    edge_res = 0.05
    goal_bias = 0.10
    max_iters = 30000

    # Trees
    Ta: List[Node] = [Node(q_start, -1)]
    Tb: List[Node] = [Node(q_goal,  -1)]
    rng = random.Random()

    def sample() -> State:
        if rng.random() < goal_bias:
            return State(q_goal.x, q_goal.y, q_goal.z, 0.0)
        return State(
            rng.uniform(bounds.minx, bounds.maxx),
            rng.uniform(bounds.miny, bounds.maxy),
            rng.uniform(bounds.minz, bounds.maxz),
            0.0
        )
    
    tes = collides_robot(q_goal, robot, spheres, aabbs, obbs)  # test goal collision
    print(f"Goal collision: {tes}")

    success = False; meet_a = -1; meet_b = -1
    for i in range(max_iters):
        if i % 1000 == 0:
            print(f"RRT iteration {i}, Ta size={len(Ta)}, Tb size={len(Tb)}")
        q_rand = sample()
        new_a = connect_toward(Ta, q_rand, step, edge_res, bounds, robot, spheres, aabbs, obbs)
        q_new_a = Ta[new_a].s
        new_b = connect_toward(Tb, q_new_a, step, edge_res, bounds, robot, spheres, aabbs, obbs)
        if dist_xyz(Tb[new_b].s, q_new_a) < step*0.5:
            success = True; meet_a = new_a; meet_b = new_b; break
        Ta, Tb = Tb, Ta  # swap

    if not success:
        print("RRT failed to find a path within iteration limit.")
        return []

    path_a = backtrack(Ta, meet_a)
    path_b = backtrack(Tb, meet_b)
    path_b.reverse()
    if path_a and path_b and dist_xyz(path_a[-1], path_b[0]) < 1e-6:
        path_b = path_b[1:]
    path = path_a + path_b

    # (Optional) Drop every Nth waypoint to reduce chatter
    if len(path) > 2:
        filtered = [path[0]]
        SKIP = 2
        for i in range(1, len(path)-1):
            if i % SKIP == 0:
                filtered.append(path[i])
        filtered.append(path[-1])
        path = filtered

    print(f"Generated path with {len(path)} states.")

    if path and dist_xyz(path[0], q_start) > dist_xyz(path[-1], q_start):
        path = list(reversed(path))
    return path

# =========================
# FOLLOW THE RRT PATH
# =========================

def enu_to_geodetic(lat0: float, lon0: float, alt_0:float, east: float, north: float, up:float) -> Tuple[float, float]:
    """Small-angle ENU meters -> lat/lon degrees around (lat0, lon0)."""
    m_per_deg_lat = 111000.0
    m_per_deg_lon = 111000.0 * math.cos(math.radians(lat0))
    lat = lat0 + (north / m_per_deg_lat)
    lon = lon0 + (east  / m_per_deg_lon)
    test = pm.enu2geodetic(east, north, up, lat0, lon0, alt_0)
    return test[0], test[1], test[2]

def geodetic_to_enu(lat0: float, lon0: float, alt_0:float, lat: float, lon: float, alt:float) -> Tuple[float, float]:
    """Small-angle lat/lon degrees -> ENU meters around (lat0, lon0)."""
    m_per_deg_lat = 111000.0
    m_per_deg_lon = 111000.0 * math.cos(math.radians(lat0))
    north = (lat - lat0) * m_per_deg_lat
    east  = (lon - lon0) * m_per_deg_lon
    test = pm.geodetic2enu(lat, lon, alt, lat0, lon0, alt_0)
    return test[0], test[1], test[2]

def _aabb_edges(aabb):
    xs = [aabb.minx, aabb.maxx]; ys = [aabb.miny, aabb.maxy]; zs = [aabb.minz, aabb.maxz]
    corners = [(x,y,z) for x in xs for y in ys for z in zs]
    eid = [(0,1),(0,2),(0,4),(1,3),(1,5),(2,3),(2,6),(3,7),(4,5),(4,6),(5,7),(6,7)]
    return [(corners[i], corners[j]) for (i,j) in eid]

def _obb_edges(obb):
    hx, hy, hz = obb.sx*0.5, obb.sy*0.5, obb.sz*0.5
    lc = [(x,y,z) for x in (-hx,hx) for y in (-hy,hy) for z in (-hz,hz)]
    c, s = math.cos(obb.yaw), math.sin(obb.yaw)
    def rotZ(p):
        x,y,z = p
        return (c*x - s*y, s*x + c*y, z)
    wc = [(obb.cx + rx, obb.cy + ry, obb.cz + rz) for (rx,ry,rz) in map(rotZ, lc)]
    eid = [(0,1),(0,2),(0,4),(1,3),(1,5),(2,3),(2,6),(3,7),(4,5),(4,6),(5,7),(6,7)]
    return [(wc[i], wc[j]) for (i,j) in eid]

def plot_rrt_path_3d(path, bounds, aabbs, obbs, q_start, q_goal, save_path="rrt_path_3d.png"):
    if not path:
        print("No path to plot.")
        return
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    xs = [p.x for p in path]
    ys = [p.y for p in path]
    zs = [p.z for p in path]  # z is UP (ENU)
    ax.plot(xs, ys, zs, linewidth=2)

    ax.scatter([q_start.x], [q_start.y], [q_start.z], marker='o', s=40)
    ax.scatter([q_goal.x],  [q_goal.y],  [q_goal.z],  marker='^', s=60)

    for a in aabbs:
        for (p0, p1) in _aabb_edges(a):
            ax.plot([p0[0],p1[0]], [p0[1],p1[1]], [p0[2],p1[2]], linewidth=1)

    for o in obbs:
        for (p0, p1) in _obb_edges(o):
            ax.plot([p0[0],p1[0]], [p0[1],p1[1]], [p0[2],p1[2]], linewidth=1)

    bbox = AABB(bounds.minx, bounds.miny, bounds.minz, bounds.maxx, bounds.maxy, bounds.maxz)
    for (p0,p1) in _aabb_edges(bbox):
        ax.plot([p0[0],p1[0]],[p0[1],p1[1]],[p0[2],p1[2]], linewidth=0.5)

    ax.set_xlabel("X (east, m)")
    ax.set_ylabel("Y (north, m)")
    ax.set_zlabel("Z (up, m)")
    ax.set_title("RRT Path (ENU)")

    ax.set_xlim(bounds.minx, bounds.maxx)
    ax.set_ylim(bounds.miny, bounds.maxy)
    ax.set_zlim(bounds.minz, bounds.maxz)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    try:
        plt.show()
    except Exception:
        pass
    print(f"Saved 3D path plot to {save_path}")

def main():
        # Grab home as reference for ENU->LLA mapping
    """home = master.recv_match(type='HOME_POSITION', blocking=True, timeout=3)
    if not home:
        raise RuntimeError("No HOME_POSITION received; cannot convert ENU to WGS84.")
    lat0 = home.latitude / 1e7
    lon0 = home.longitude / 1e7
    alt0 = home.altitude / 1000.0
    print(f"Home lat0={lat0:.7f}, lon0={lon0:.7f}, alt0={alt0:.2f}m")
    la0 = 41.358389
    lo0 = 2.185278
    print(f"Difference from home: dlat={lat0 - la0:.7f}, dlon={lon0 - lo0:.7f}")"""
    """lat0 = la0
    lon0 = lo0"""
    # Arm & mode
    # Bounds (ENU, z up)
    bounds = Bounds3D(-2, -2, -10, 20, 20, 1)
    # Start / goal (ENU, z up)
    pos_msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
    
    """depth = (depth_current) - 49.35

    lat_current = pos_msg.lat / 1e7
    lon_current = pos_msg.lon / 1e7
    alt_current = pos_msg.relative_alt / 1000.0
    enu = geodetic_to_enu(lat0, lon0, alt0, lat_current, lon_current, alt_current)
    print(depth)"""
    if pos_msg:
        x_current = pos_msg.y  # meters North
        y_current = pos_msg.x  # meters East
        up_current = -pos_msg.z  # meters Down (positive down)
    else:
        raise ValueError("No LOCAL_POSITION_NED message received")

    q_start = State(x_current, y_current, up_current, 0.0)
    print(f"Start ENU: x={q_start.x:.2f}, y={q_start.y:.2f}, z={q_start.z:.2f}")
    #q_start = State(11.35433787, 13.38846827, -5.0, 0.0)
    q_goal  = State(11.25, 13.5, -5, 0.0)
    #q_goal = State(11.835702, 13.253233, -5, 0.0)
    
    arm_vehicle_and_set_mode(True, "GUIDED")



    # Plan in local ENU
    path = generate_rrt_path(q_start, q_goal, step_size=0.30, bounds=bounds)
    if not path:
        print("No path to follow; exiting.")
        return

    # Recreate obstacles/bounds for plotting
    bounds = Bounds3D(-2, -2, -10, 20, 20, 0)
    aabbs: List[AABB] = []
    aabbs.append(AABB(2 - 0.25, -20, -12, 2 + 0.25, 20, -4.5))
    aabbs.append(AABB(2 - 0.25, -20, -2.5, 2 + 0.25, 20, 2))
    aabbs.append(AABB(2 - 0.25, -20, -12, 2 + 0.25, -1.5, 1))
    aabbs.append(AABB(2 - 0.25, 1.5, -12, 2 + 0.25, 20, 1))
    obbs = [
        OrientedBox(11.57346922, 12.78198424, -4.25, 1.04922147, 0.25, 1.5, -1.844678),
        OrientedBox(10.722990,    3.746699,   -5.0,  50.0,       3.25,  16.0, -1.844678),
    ]
  

    plot_rrt_path_3d(path, bounds, aabbs, obbs, q_start, q_goal)

    # Follow each waypoint
    THRESH = 0.1   # meters for reached_goal
    length = len(path)
    print(f"Length {len(path)}")
    print(f"First waypoint x={path[0].x}, y={path[0].y}, z={path[0].z}")
    print(path)
    for i, wp in enumerate(path):
        # Convert (x east, y north, z up) -> (lat, lon, depth)
        #lat, lon, alt = enu_to_geodetic(lat0, lon0, alt0, east=wp.x, north=wp.y, up=wp.z)
        
        if i > length - 3:
            THRESH = 0.05  # tighter threshold for last two waypoints
            yaw_deg = 105.6923
        else:
            yaw_deg = 0.0   # FORCE yaw to 0 always

        goto_position(wp.x, wp.y, wp.z, yaw_deg=yaw_deg)
        # Wait until reached

        print(f"[{i+1}/{len(path)}] ENU → x={wp.x:.2f}m, y={wp.y:.2f}m, z={wp.z:.2f}m, yaw={yaw_deg:.1f}°")
        
        # Wait until reachedS
        start_t = time.time()
        while True:
            
            ok, dist = reached_goal(wp.x, wp.y, wp.z, yaw_deg, threshold=THRESH)
            if ok:
                print(f"  reached (err≈{dist:.2f} m)")
                break
            if time.time() - start_t > 20.0:  # per-waypoint timeout
                print(f"  timeout at this waypoint (last err≈{dist:.2f} m), moving on")
                break
            time.sleep(0.3)

    print("RRT path complete. No further commands sent.")

if __name__ == "__main__":
    main()
