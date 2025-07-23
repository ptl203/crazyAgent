import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Point
from crazyflie_interfaces.srv import Land, Takeoff, GoTo


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

def arrayToGeometryPoint(a):
    result = Point()
    result.x = a[0]
    result.y = a[1]
    result.z = a[2]
    return result

def call_goto_service(goal, yaw):
    rclpy.init()
    node = Node('goto_client')
    
    client = node.create_client(GoTo, '/cf231/go_to')
    
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting...')
    
    request = GoTo.Request()
    request.relative = True
    request.goal = arrayToGeometryPoint(goal) #Meters
    request.yaw = float(yaw)
    # duration = calculate total distance to traverse and find a good duration
    request.duration = rclpy.duration.Duration(seconds=2.5).to_msg()

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        node.get_logger().info('GoTo service called successfully')
    else:
        node.get_logger().error('Failed to call GoTo service')
    
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
time.sleep(5)
call_goto_service([1.0,0.0,0.0], 0)
time.sleep(5)
call_land_service()

