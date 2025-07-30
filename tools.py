from langchain_google_community import GoogleSearchAPIWrapper
from langchain.tools import Tool
import os
import logging
import math
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Point
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

# Drone control functions
def drone_takeoff(*args, **kwargs):
    """Launch/takeoff the Crazyflie drone safely"""
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
            result = "Drone takeoff successful - drone launched to 0.5m height"
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

def arrayToGeometryPoint(a):
    """Convert array [x, y, z] to geometry_msgs Point"""
    result = Point()
    result.x = a[0]
    result.y = a[1]
    result.z = a[2]
    return result

def objective_to_coordinates(objective):
    """Convert string objective to coordinate array [x, y, z]"""
    tools_logger.info(f"Converting objective '{objective}' to coordinates")
    
    coordinate_map = {
        "A": [0.25, 0.25, 1.0],
        "B": [0.25, -0.25, 1.0],
        "C": [-0.25, -0.25, 1.0],
        "D": [-0.25, 0.25, 1.0]
    }
    
    objective_upper = objective.upper().strip()
    
    if objective_upper in coordinate_map:
        coordinates = coordinate_map[objective_upper]
        tools_logger.info(f"Mapped '{objective}' to coordinates: {coordinates}")
        return coordinates
    else:
        tools_logger.warning(f"Unknown objective '{objective}', defaulting to point A")
        return [0.25, 0.25, 1.0]

def drone_goto(objective: str):
    """Move the Crazyflie drone to a specified objective location"""
    tools_logger.info("=== DRONE GOTO TOOL CALLED ===")
    tools_logger.info(f"Objective received: {objective}")
    
    try:
        # Convert objective to coordinates
        goal_coordinates = objective_to_coordinates(objective)
        yaw = 0.0  # Default yaw angle
        
        tools_logger.info(f"Goal coordinates: {goal_coordinates}, yaw: {yaw}")
        
        tools_logger.info("Initializing ROS2...")
        rclpy.init()
        node = Node('goto_client')
        
        tools_logger.info("Creating GoTo service client...")
        client = node.create_client(GoTo, '/cf231/go_to')
        
        tools_logger.info("Waiting for GoTo service...")
        while not client.wait_for_service(timeout_sec=1.0):
            tools_logger.warning('GoTo service not available, waiting...')
            node.get_logger().info('Service not available, waiting...')
        
        tools_logger.info("Service available, creating request...")
        request = GoTo.Request()
        request.relative = False
        request.goal = arrayToGeometryPoint(goal_coordinates)
        request.yaw = float(yaw)
        request.duration = rclpy.duration.Duration(seconds=2.5).to_msg()

        tools_logger.info(f"Request created - goal: {request.goal}, yaw: {request.yaw}, relative: {request.relative}")
        
        tools_logger.info("Calling GoTo service asynchronously...")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        
        tools_logger.info("Service call completed, checking result...")
        if future.result() is not None:
            tools_logger.info(f"GoTo service result: {future.result()}")
            node.get_logger().info('GoTo service called successfully')
            result = f"Drone moved to {objective} successfully - coordinates: {goal_coordinates}"
        else:
            tools_logger.error("GoTo service returned None result")
            node.get_logger().error('Failed to call GoTo service')
            result = f"Failed to move drone to {objective} - GoTo service error"
        
        tools_logger.info("Cleaning up ROS2 resources...")
        node.destroy_node()
        rclpy.shutdown()
        
        tools_logger.info(f"GoTo tool returning: {result}")
        return result
        
    except Exception as e:
        error_msg = f"Error during drone GoTo operation: {str(e)}"
        tools_logger.error(error_msg, exc_info=True)
        return error_msg

def drone_turn(*args, **kwargs):
    """Turn the Crazyflie drone by a specified angle in degrees"""
    tools_logger.info("=== DRONE TURN TOOL CALLED ===")
    tools_logger.info(f"Args received: {args}")
    tools_logger.info(f"Kwargs received: {kwargs}")
    
    try:
        # Extract turn_angle from args or kwargs
        if args:
            turn_angle_str = args[0]
        elif 'turn_angle' in kwargs:
            turn_angle_str = kwargs['turn_angle']
        else:
            raise ValueError("No turn_angle parameter provided")
        
        # Convert string to float
        turn_angle = float(turn_angle_str)
        tools_logger.info(f"Turn angle converted to: {turn_angle} degrees")
        
        # Convert degrees to radians
        yaw_radians = math.radians(turn_angle)
        
        tools_logger.info(f"Turn angle in radians: {yaw_radians}")
        
        tools_logger.info("Initializing ROS2...")
        rclpy.init()
        node = Node('turn_client')
        
        tools_logger.info("Creating GoTo service client...")
        client = node.create_client(GoTo, '/cf231/go_to')
        
        tools_logger.info("Waiting for GoTo service...")
        while not client.wait_for_service(timeout_sec=1.0):
            tools_logger.warning('GoTo service not available, waiting...')
            node.get_logger().info('Service not available, waiting...')
        
        tools_logger.info("Service available, creating request...")
        request = GoTo.Request()
        request.relative = True
        request.goal = arrayToGeometryPoint([0.0, 0.0, 0.0])
        request.yaw = float(yaw_radians)
        request.duration = rclpy.duration.Duration(seconds=2.5).to_msg()

        tools_logger.info(f"Request created - goal: {request.goal}, yaw: {request.yaw}, relative: {request.relative}")
        
        tools_logger.info("Calling GoTo service asynchronously...")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        
        tools_logger.info("Service call completed, checking result...")
        if future.result() is not None:
            tools_logger.info(f"GoTo service result: {future.result()}")
            node.get_logger().info('GoTo service called successfully')
            result = f"Drone turned {turn_angle} degrees successfully"
        else:
            tools_logger.error("GoTo service returned None result")
            node.get_logger().error('Failed to call GoTo service')
            result = f"Failed to turn drone {turn_angle} degrees - GoTo service error"
        
        tools_logger.info("Cleaning up ROS2 resources...")
        node.destroy_node()
        rclpy.shutdown()
        
        tools_logger.info(f"Turn tool returning: {result}")
        return result
        
    except Exception as e:
        error_msg = f"Error during drone turn operation: {str(e)}"
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

drone_goto_tool = Tool(
    name="drone_goto",
    description="Move the Crazyflie drone to a specified objective location. Takes an 'Objective' parameter (A, B, C, or D) corresponding to corners of a 0.5x0.5 square at 1.0m height. Use this tool when asked to move, go to, or navigate to a specific position.",
    func=drone_goto
)

drone_turn_tool = Tool(
    name="drone_turn",
    description="Turn the Crazyflie drone by a specified angle in degrees. Takes a 'turn_angle' parameter in degrees (positive for clockwise, negative for counter-clockwise). Use this tool when asked to turn, rotate, or change the drone's orientation.",
    func=drone_turn
)