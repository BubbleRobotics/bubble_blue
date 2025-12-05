"""Luis Blunschi 27.10.2025
Mockup planner script to command ROV in a snake pattern underwater using MAVLink and ROS2."""

from pymavlink import mavutil
import math
import time
from mavros_msgs.srv import CommandBool, SetMode
import rclpy

# -------------------------
# Connect to ROV
# -------------------------
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print("Connected to ROV")

time.sleep(1)

# -------------------------
# Utility functions
# -------------------------
def set_mode(mode="GUIDED"):
    rclpy.init()
    node = rclpy.create_node('temp_set_mode_node')

    client = node.create_client(SetMode, '/mavros/set_mode')

    if not client.wait_for_service(timeout_sec=2.0):
        print("Set mode service not available!")
        node.destroy_node()
        rclpy.shutdown()
        return None

    req = SetMode.Request()
    req.custom_mode = mode

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    result = future.result()
    node.destroy_node()
    rclpy.shutdown()

    if result and result.mode_sent:
        print(f"Flight mode set to: {mode}")
        return True
    else:
        print("Failed to set flight mode")
        return False
    
def arm_vehicle_and_set_mode(arm=False,mode="MANUAL"):
    print(f"Setting mavros mode to {mode}")
    rclpy.init()
    node = rclpy.create_node('temp_arming_node')

    set_mode_client = node.create_client(SetMode, '/mavros/set_mode')

    if not set_mode_client.wait_for_service(timeout_sec=5.0):
        print("Set mode service not available!")
        node.destroy_node()
        rclpy.shutdown()
        return None

    req = SetMode.Request()
    req.custom_mode = mode

    future = set_mode_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    result = future.result()
 

    if result and result.mode_sent:
        print(f"Flight mode set to: {mode}")
    else:
        print("Failed to set flight mode")
        return False
    
    print("Arming vehicle" if arm else "Disarming vehicle")
    arming_client = node.create_client(CommandBool, '/mavros/cmd/arming')

    if not arming_client.wait_for_service(timeout_sec=5.0):
        print('Arming service not available!')
        node.destroy_node()
        rclpy.shutdown()
        return None

    req = CommandBool.Request()
    req.value = arm

    future = arming_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    result = future.result()
    node.destroy_node()
    rclpy.shutdown()
    print('Arming successful' if result.success else 'Arming failed')
    return result.success if result else None

def set_yaw(yaw_deg, speed_deg_per_s=10, relative=False):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        yaw_deg,
        speed_deg_per_s,
        0,
        1 if relative else 0,
        0, 0, 0
    )
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack:
        print(f"Yaw ACK: {ack.result}")
    else:
        print("No ACK for yaw")

def goto_position(x_east, y_north, z_down, yaw_deg=None):
    x_north = y_north
    y_east = x_east

    master.mav.set_position_target_local_ned_send(
        0,
        int(master.target_system),
        int(master.target_component),
        int(mavutil.mavlink.MAV_FRAME_LOCAL_NED),
        int(0b110111111000),  # use position only
        x_north,
        y_east,
        z_down,
        0, 0, 0,
        0, 0, 0,
        math.radians(yaw_deg) if yaw_deg is not None else 0,
        0
    )
    if yaw_deg is not None:
        set_yaw(yaw_deg)
    print(f"Sending local position: X (Norht): {x_north}, Y (East): {y_east}, Z (Down): {z_down}")
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack:
        print(f"Position ACK: {ack.result}")
    else:
        print("No ACK for position")

def reached_goal(x_east_goal, y_north_goal, z_down_goal, yaw_goal, threshold=0.05):
    """
    Check if current position is within threshold of target
    threshold in meters
    """
    pos_msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
    yaw_msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=1)
    #print(yaw_msg)
    if not pos_msg:
        return False, None
    
    x_east_current = pos_msg.y
    y_north_current = pos_msg.x
    z_down_current = pos_msg.z

    # Approximate distance in meters using simple latitude/longitude differences
    x_dist = (x_east_goal - x_east_current)
    y_dist = (y_north_goal - y_north_current)
    depth_dist = z_down_goal - z_down_current
    if yaw_msg:
        #print(yaw_msg)
        yaw_dist = yaw_goal - yaw_msg.yaw*360/2/math.pi 
    else:
        yaw_dist = 0
    total_dist = math.sqrt(x_dist**2 + y_dist**2 + depth_dist**2 + yaw_dist**2/100)

    # Get yaw for logging
    att_msg = master.recv_match(type='ATTITUDE', blocking=False)
    yaw_deg = math.degrees(att_msg.yaw) % 360 if att_msg else None

    # Print safely even if yaw is None
    yaw_str = f"{yaw_deg:.1f}°" if yaw_deg is not None else "N/A"
    print(f"Current: dist_x={x_dist:.7f}, dist_y={y_dist:.7f}, dist_depth={depth_dist:.2f} m, dist_yaw={yaw_dist}, total_dist={total_dist:.2f} m")

    return total_dist < threshold, total_dist


# -------------------------
# Snake path parameters
# -------------------------
home = master.recv_match(type='HOME_POSITION', blocking=True, timeout=2)


top_left = {"lat": 47.376824, "lon": 8.5417325, "depth": 3.75, "yaw": 180}
top_right = {"lat": 47.376824, "lon": 8.54172,   "depth": 3.75, "yaw": 180}
bottom_left = {"lat": 47.376824, "lon": 8.5417325, "depth": 5.15,  "yaw": 180}
bottom_right = {"lat": 47.376824, "lon": 8.54172,   "depth": 5.15,  "yaw": 180}

top_left = {"lat":41.358508333, "lon": 2.185419444, "altitude: 0.0depth": 3.1, "yaw": 105.6923}
top_right = {"lat": 41.3585, "lon": 2.185416667,    "depth": 3.75, "yaw": 105.6923}
bottom_left = {"lat": 41.358508333, "lon": 2.185419444, "depth": 5.15,  "yaw": 105.6923}
bottom_right = {"lat": 41.3585, "lon": 2.185416667,   "depth": 5.15,  "yaw": 105.6923}

top_left = {"x":10.92, "y": 13.55, "depth": 3.75, "yaw": 105.6923}
top_right = {"x": 10.7, "y": 12.63,    "depth": 3.75, "yaw": 105.6923}
bottom_left = {"x": 10.92, "y": 13.56, "depth": 5.15,  "yaw": 105.6923}
bottom_right = {"x": 10.7, "y": 12.63,   "depth": 5.15,  "yaw": 105.6923}

depth_step = 0.2
current_depth = top_left["depth"]
max_depth = bottom_left["depth"]
going_right = True
yaw = top_left["yaw"]
checked_apriltags = False


# -------------------------
# Arm system
# -------------------------
arm_vehicle_and_set_mode(True,"GUIDED")
# -------------------------
# Execute snake path
# -------------------------
done = False
while not done:
    if current_depth + depth_step > max_depth:
        done = True
        current_depth = max_depth

    home = master.recv_match(type='HOME_POSITION', blocking=True, timeout=5)
    if not checked_apriltags:
        print("Checking for AprilTags before starting snake path...")
        x_end = bottom_left["x"]
        y_end = bottom_left["y"]
        depth_end = bottom_left["depth"]
        goto_position(x_end, y_end, depth_end, yaw_deg=yaw)
        while not checked_apriltags:
            checked_apriltags, dist = reached_goal(x_end, y_end, depth_end, yaw, threshold=0.05)
            time.sleep(0.5)

    if going_right:
        x_start = top_left["x"]
        x_end = top_right["x"]
        y_start = top_left["y"]
        y_end = top_right["y"]
    else:
        x_start = top_right["x"]
        x_end = top_left["x"]
        y_start = top_right["y"]
        y_end = top_left["y"]

    # Move to start of line
    goto_position(x_start, y_start, current_depth, yaw_deg=yaw)
    reached = False
    while not reached:
        reached, dist = reached_goal(x_start, y_start, current_depth, yaw, threshold=0.05)
        time.sleep(0.5)

    # Move to end of line
    goto_position(x_end, y_end, current_depth, yaw_deg=yaw)
    reached = False
    while not reached:
        reached, dist = reached_goal(x_end, y_end, current_depth, yaw, threshold=0.05)
        time.sleep(0.5)

    # Step down
    current_depth += depth_step
    if current_depth > max_depth:
        break

    going_right = not going_right
    print(f"Next pass at depth {current_depth:.2f} m, direction: {'right' if going_right else 'left'}")

print("Snake path complete. No further commands sent.")
