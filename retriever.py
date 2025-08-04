import json
import logging
from langchain_core.tools import tool

# Configure logging
retriever_logger = logging.getLogger(__name__)

@tool
def get_objective_coordinates(objective: str) -> list:
    """
    Retrieve coordinates for a given objective from the objective map.
    
    Args:
        objective (str): The objective identifier (e.g., 'A', 'B', 'C', 'D')
        
    Returns:
        list: Coordinates as [x, y, z] or empty list if objective not found
    """
    retriever_logger.info(f"Retrieving coordinates for objective '{objective}'")
    
    try:
        with open('objective_map.json', 'r') as f:
            coordinate_map = json.load(f)
        
        objective_upper = objective.upper().strip()
        
        if objective_upper in coordinate_map:
            coordinates = coordinate_map[objective_upper]
            retriever_logger.info(f"Found coordinates for '{objective_upper}': {coordinates}")
            return coordinates
        else:
            retriever_logger.warning(f"Objective '{objective_upper}' not found in coordinate map")
            return []
    
    except FileNotFoundError:
        retriever_logger.error("objective_map.json file not found")
        return []
    except json.JSONDecodeError:
        retriever_logger.error("Error decoding objective_map.json")
        return []
    except Exception as e:
        retriever_logger.error(f"Unexpected error retrieving objective coordinates: {e}")
        return []