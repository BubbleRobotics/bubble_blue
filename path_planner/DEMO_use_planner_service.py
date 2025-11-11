#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from path_planner_interfaces.srv import InitiatePath
import time
def main():

    start_t = time.time()
    rclpy.init()
    x_1 = 11.0
    x_2 = 11.0
    for i in range(6):
        print(f"Starting call {i}")
        node = Node('initiate_path_client')
        cli = node.create_client(InitiatePath, '/path_planner/follow_rrt_path')
        cli.wait_for_service()
        req = InitiatePath.Request()
        req.pose = Pose()
        
        if i % 2 == 0:
            x_take = x_1
        else:
            x_take = x_2
        req.pose.position.x = x_take
        req.pose.position.y = 13.45
        req.pose.position.z = -5.2
        req.pose.orientation.w = 1.0
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut)
        print('response:', fut.result())
        node.destroy_node()
        time.sleep(10)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
