import rclpy
import time
from rclpy.node import Node
from crazyflie_interfaces.srv import Land, Takeoff


def call_takeoff_service():
    rclpy.init()
    node = Node('takeoff_client')
    
    client = node.create_client(Takeoff, '/cf231/takeoff')
    
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting...')
    
    request = Takeoff.Request()
    request.height = 0.5
    request.duration = rclpy.duration.Duration(seconds=2.5).to_msg()

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        node.get_logger().info('Takeoff service called successfully')
    else:
        node.get_logger().error('Failed to call takeoff service')
    
    node.destroy_node()
    rclpy.shutdown()


def call_land_service():
    rclpy.init()
    node = Node('land_client')
    
    client = node.create_client(Land, '/cf231/land')
    
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting...')
    
    request = Land.Request()
    request.height = 0.04
    request.duration = rclpy.duration.Duration(seconds=2.5).to_msg()

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        node.get_logger().info('Land service called successfully')
    else:
        node.get_logger().error('Failed to call land service')
    
    node.destroy_node()
    rclpy.shutdown()

call_takeoff_service()
time.sleep(10)
call_land_service()

