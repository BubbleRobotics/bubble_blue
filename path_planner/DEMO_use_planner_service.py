#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from path_planner_interfaces.srv import InitiatePath
def main():
    
    rclpy.init()


    node = Node('initiate_path_client')
    cli = node.create_client(InitiatePath, '/bluerov2/path_planner/follow_rrt_path')
    cli.wait_for_service()
    req = InitiatePath.Request()
    req.pose = Pose()
    

    req.pose.position.x = 0.0#11.0
    req.pose.position.y = 13.45
    req.pose.position.z = -5.2
    req.pose.orientation.w = 1.0
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut)
    print('response:', fut.result())
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
