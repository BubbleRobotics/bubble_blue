import argparse
from pymavlink import mavutil

def main():
    # use the argparse syntax
    parser = argparse.ArgumentParser(description="Monitor specific MAVLink " \
    "messages from a MAVLink device." \
    "\nType the \nOptional Arguments:" \
    "\n\t--device <device-address>")

    parser.add_argument(
        '--device',
        type=str,
        default='udp:127.0.0.1:14550',
        help='MAVLink connection string (default: udp:127.0.0.1:14550)'
    )
    parser.add_argument(
        'messages',
        nargs='+',
        help='MAVLink message types to monitor (e.g. HEARTBEAT ATTITUDE BATTERY_STATUS)'
    )
    args = parser.parse_args()

    print(f"Connecting to MAVLink on {args.device}...")
    master = mavutil.mavlink_connection(args.device)

    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print(f"Heartbeat received from system {master.target_system}," \
           f"component {master.target_component}\n")

    print(f"Monitoring MAVLink messages: {', '.join(args.messages)}\n")
    try:
        while True:
            msg = master.recv_match(blocking=True)
            if msg and msg.get_type() in args.messages:
                print(f"[{msg.get_type()}] {msg.to_dict()}")
    except KeyboardInterrupt:
        print("\nMonitoring stopped.")

if __name__ == '__main__':
    main()
