from langchain_google_community import GoogleSearchAPIWrapper
from langchain.tools import Tool
import os
import logging
import rclpy
import time
from rclpy.node import Node
from crazyflie_interfaces.srv import Land, Takeoff, GoTo

#Load Environment
from dotenv import load_dotenv

load_dotenv()

# Configure logging for tools
tools_logger = logging.getLogger('tools')
tools_logger.setLevel(logging.INFO)

# Create handler if it doesn't exist
if not tools_logger.handlers:
    handler = logging.StreamHandler()
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    tools_logger.addHandler(handler)
    
    # Also log to file
    file_handler = logging.FileHandler('tools.log')
    file_handler.setFormatter(formatter)
    tools_logger.addHandler(file_handler)

# Initialize Google Search API Wrapper
search = GoogleSearchAPIWrapper()

# Create Google Search Tool
google_search_tool = Tool(
    name="google_search",
    description="Search Google for information on a specific topic or query. Useful for finding current information, news, facts, or general knowledge.",
    func=search.run
)

# Drone control functions
def drone_takeoff(*args, **kwargs):
    """Launch/takeoff the Crazyflie drone"""
    tools_logger.info("=== DRONE TAKEOFF TOOL CALLED ===")
    tools_logger.info(f"Args received: {args}")
    tools_logger.info(f"Kwargs received: {kwargs}")
    
    try:
        tools_logger.info("Initializing ROS2...")
        rclpy.init()
        node = Node('takeoff_client')
        
        tools_logger.info("Creating takeoff service client...")
        client = node.create_client(Takeoff, '/cf231/takeoff')
        
        tools_logger.info("Waiting for takeoff service...")
        while not client.wait_for_service(timeout_sec=1.0):
            tools_logger.warning('Takeoff service not available, waiting...')
            node.get_logger().info('Service not available, waiting...')
        
        tools_logger.info("Service available, creating request...")
        request = Takeoff.Request()
        request.height = 0.5
        request.duration = rclpy.duration.Duration(seconds=2.5).to_msg()

        tools_logger.info(f"Request created - height: {request.height}, duration: {request.duration}")
        
        tools_logger.info("Calling takeoff service asynchronously...")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        
        tools_logger.info("Service call completed, checking result...")
        if future.result() is not None:
            tools_logger.info(f"Takeoff service result: {future.result()}")
            node.get_logger().info('Takeoff service called successfully')
            result = "Drone takeoff successful - drone launched to 1.0m height"
        else:
            tools_logger.error("Takeoff service returned None result")
            node.get_logger().error('Failed to call takeoff service')
            result = "Failed to launch drone - takeoff service error"
        
        tools_logger.info("Cleaning up ROS2 resources...")
        node.destroy_node()
        rclpy.shutdown()
        
        tools_logger.info(f"Takeoff tool returning: {result}")
        return result
        
    except Exception as e:
        error_msg = f"Error during drone takeoff: {str(e)}"
        tools_logger.error(error_msg, exc_info=True)
        return error_msg

def drone_land(*args, **kwargs):
    """Land the Crazyflie drone safely"""
    tools_logger.info("=== DRONE LAND TOOL CALLED ===")
    tools_logger.info(f"Args received: {args}")
    tools_logger.info(f"Kwargs received: {kwargs}")
    
    try:
        tools_logger.info("Initializing ROS2...")
        rclpy.init()
        node = Node('land_client')
        
        tools_logger.info("Creating land service client...")
        client = node.create_client(Land, '/cf231/land')
        
        tools_logger.info("Waiting for land service...")
        while not client.wait_for_service(timeout_sec=1.0):
            tools_logger.warning('Land service not available, waiting...')
            node.get_logger().info('Service not available, waiting...')
        
        tools_logger.info("Service available, creating request...")
        request = Land.Request()
        request.height = 0.04
        request.duration = rclpy.duration.Duration(seconds=2.5).to_msg()

        tools_logger.info(f"Request created - height: {request.height}, duration: {request.duration}")
        
        tools_logger.info("Calling land service asynchronously...")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        
        tools_logger.info("Service call completed, checking result...")
        if future.result() is not None:
            tools_logger.info(f"Land service result: {future.result()}")
            node.get_logger().info('Land service called successfully')
            result = "Drone landing successful - drone landed safely"
        else:
            tools_logger.error("Land service returned None result")
            node.get_logger().error('Failed to call land service')
            result = "Failed to land drone - landing service error"
        
        tools_logger.info("Cleaning up ROS2 resources...")
        node.destroy_node()
        rclpy.shutdown()
        
        tools_logger.info(f"Land tool returning: {result}")
        return result
        
    except Exception as e:
        error_msg = f"Error during drone landing: {str(e)}"
        tools_logger.error(error_msg, exc_info=True)
        return error_msg

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