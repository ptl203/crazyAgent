from langchain_google_community import GoogleSearchAPIWrapper
from langchain.tools import Tool
import os
import rclpy
import time
from rclpy.node import Node
from crazyflie_interfaces.srv import Land, Takeoff

#Load Environment
from dotenv import load_dotenv

load_dotenv()

# Initialize Google Search API Wrapper
search = GoogleSearchAPIWrapper()

# Create Google Search Tool
google_search_tool = Tool(
    name="google_search",
    description="Search Google for information on a specific topic or query. Useful for finding current information, news, facts, or general knowledge.",
    func=search.run
)

# Drone control functions
def drone_takeoff():
    """Launch/takeoff the Crazyflie drone to 1.0m height"""
    try:
        rclpy.init()
        node = Node('takeoff_client')
        
        client = node.create_client(Takeoff, '/cf231/takeoff')
        
        while not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('Service not available, waiting...')
        
        request = Takeoff.Request()
        request.height = 1.0
        request.duration = rclpy.duration.Duration(seconds=2.5).to_msg()
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        
        if future.result() is not None:
            node.get_logger().info('Takeoff service called successfully')
            result = "Drone takeoff successful - drone launched to 1.0m height"
        else:
            node.get_logger().error('Failed to call takeoff service')
            result = "Failed to launch drone - takeoff service error"
        
        node.destroy_node()
        rclpy.shutdown()
        return result
        
    except Exception as e:
        return f"Error during drone takeoff: {str(e)}"

def drone_land():
    """Land the Crazyflie drone safely"""
    try:
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
            result = "Drone landing successful - drone landed safely"
        else:
            node.get_logger().error('Failed to call land service')
            result = "Failed to land drone - landing service error"
        
        node.destroy_node()
        rclpy.shutdown()
        return result
        
    except Exception as e:
        return f"Error during drone landing: {str(e)}"

# Create Drone Control Tools
drone_takeoff_tool = Tool(
    name="drone_takeoff",
    description="Launch/takeoff the Crazyflie drone. The drone will take off to a height of 1.0 meters. Use this tool when asked to launch, takeoff, or start flying the drone.",
    func=drone_takeoff
)

drone_land_tool = Tool(
    name="drone_land",
    description="Land the Crazyflie drone safely. The drone will descend and land at the specified landing height. Use this tool when asked to land, stop, or bring down the drone.",
    func=drone_land
)