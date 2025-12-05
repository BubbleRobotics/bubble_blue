#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from path_planner_interfaces.srv import InitiatePath
from std_srvs.srv import Trigger

def main():
    rclpy.init()
    node = Node('stopper_path_client')
    cli = node.create_client(Trigger, '/bluerov2/path_planner/stop')
    cli.wait_for_service()
    req = Trigger.Request()
    
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut)
    print('response:', fut.result())
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
