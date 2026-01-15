"""
Example of how to send MANUAL_CONTROL messages to the autopilot using
pymavlink.
This message is able to fully replace the joystick inputs.
"""


# Import mavutil
from pymavlink import mavutil
import time

# Create the connection
#master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
#master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Create a function to send RC values
# More information about Joystick channels
# here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs

""" Set RC channel pwm value
Args:
    channel_id (TYPE): Channel ID
    pwm (int, optional): Channel pwm value 1100-1900
"""

# Mavlink 2 supports up to 18 channels:
# https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
rc_channel_values = [65535 for _ in range(18)]
# Initialize all channels to 65535 (ignore)
# Then set the channels you want to override
# 1500 is neutral for these channels
# Min and Max values are 1100 and 1900
rc_channel_values[0] = 1500 # Roll
rc_channel_values[1] = 1500 # Pitch
rc_channel_values[2] = 1500 # Throttle (Up, Down)
rc_channel_values[3] = 1500 # Yaw
rc_channel_values[4] = 1400 # Forward, Backward
rc_channel_values[5] = 1500 # Left, Right


def set_rc_channel_pwm(rc_channel_values):
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.


while True:    # Send RC channel values every second
    set_rc_channel_pwm(rc_channel_values)
    print("Sent RC channel override")
    time.sleep(0.001)
