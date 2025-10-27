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

def goto_position(lat, lon, depth, yaw_deg=None):
    master.mav.set_position_target_global_int_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        int(0b110111111000),  # use position only
        int(lat*1e7),
        int(lon*1e7),
        -depth,
        0, 0, 0,
        0, 0, 0,
        math.radians(yaw_deg) if yaw_deg is not None else 0,
        0
    )
    if yaw_deg is not None:
        set_yaw(yaw_deg)

    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack:
        print(f"Position ACK: {ack.result}")
    else:
        print("No ACK for position")

def reached_goal(lat_goal, lon_goal, depth_goal, threshold=0.05):
    """
    Check if current position is within threshold of target
    threshold in meters
    """
    pos_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if not pos_msg:
        return False, None

    lat_current = pos_msg.lat / 1e7
    lon_current = pos_msg.lon / 1e7
    depth_current = -pos_msg.relative_alt / 1000 - 47.2 # convert mm to m

    # Approximate distance in meters using simple latitude/longitude differences
    lat_dist = (lat_goal - lat_current) * 111000
    lon_dist = (lon_goal - lon_current) * 111000 * math.cos(math.radians(lat_goal))
    depth_dist = depth_goal - depth_current

    total_dist = math.sqrt(lat_dist**2 + lon_dist**2 + depth_dist**2)

    # Get yaw for logging
    att_msg = master.recv_match(type='ATTITUDE', blocking=False)
    yaw_deg = math.degrees(att_msg.yaw) % 360 if att_msg else None

    # Print safely even if yaw is None
    yaw_str = f"{yaw_deg:.1f}°" if yaw_deg is not None else "N/A"
    print(f"Current: lat={lat_current:.7f}, lon={lon_current:.7f}, depth={depth_current:.2f} m, yaw={yaw_str}, dist={total_dist:.2f} m")

    return total_dist < threshold, total_dist


# -------------------------
# Snake path parameters
# -------------------------
top_left = {"lat": 47.376824, "lon": 8.5417325, "depth": 3.75, "yaw": 180}
top_right = {"lat": 47.376824, "lon": 8.54172,   "depth": 3.75, "yaw": 180}
bottom_left = {"lat": 47.376824, "lon": 8.5417325, "depth": 5.15,  "yaw": 180}
bottom_right = {"lat": 47.376824, "lon": 8.54172,   "depth": 5.15,  "yaw": 180}

depth_step = 0.1
current_depth = top_left["depth"]
max_depth = bottom_left["depth"]
going_right = True
lat = top_left["lat"]


# -------------------------
# Arm system
# -------------------------
arm_vehicle_and_set_mode(True,"GUIDED")
# -------------------------
# Execute snake path
# -------------------------
while current_depth <= max_depth:
    if going_right:
        lon_start = top_left["lon"]
        lon_end = top_right["lon"]
    else:
        lon_start = top_right["lon"]
        lon_end = top_left["lon"]

    # Move to start of line
    goto_position(lat, lon_start, current_depth, yaw_deg=180)
    reached = False
    while not reached:
        reached, dist = reached_goal(lat, lon_start, current_depth, threshold=0.05)
        time.sleep(0.5)

    # Move to end of line
    goto_position(lat, lon_end, current_depth, yaw_deg=180)
    reached = False
    while not reached:
        reached, dist = reached_goal(lat, lon_end, current_depth, threshold=0.05)
        time.sleep(0.5)

    # Step down
    current_depth += depth_step
    if current_depth > max_depth:
        break

    going_right = not going_right
    print(f"Next pass at depth {current_depth:.2f} m, direction: {'right' if going_right else 'left'}")

print("Snake path complete. No further commands sent.")
