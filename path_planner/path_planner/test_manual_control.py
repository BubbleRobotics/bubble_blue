"""
Example of how to send MANUAL_CONTROL messages to the autopilot using
pymavlink.
This message is able to fully replace the joystick inputs.
"""

# Import mavutil
from pymavlink import mavutil
import time
# Create the connection
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Send a positive x value, negative y, negative z,
# positive rotation and no button.
# https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
# Warning: Because of some legacy workaround, z will work between [0-1000]
# where 0 is full reverse, 500 is no output and 1000 is full throttle.
# x,y and r will be between [-1000 and 1000].

while True:
    # To active button 0 (first button), 3 (fourth button) and 7 (eighth button)
    # It's possible to check and configure this buttons in the Joystick menu of QGC
    print("Sending")
    buttons = 0

    master.mav.manual_control_send(
        1,
        500,
        0,
        400,
        0,
        0)
    time.sleep(0.01)
 