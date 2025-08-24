#!/usr/bin/env python3
"""
Generate a stategraph diagram for the crazyAgent architecture
"""
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyBboxPatch
import numpy as np

def create_agent_stategraph():
    fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    
    # Define colors
    agent_color = '#4CAF50'  # Green
    tool_color = '#FF9800'   # Orange
    arrow_color = '#2196F3'  # Blue
    text_color = '#333333'   # Dark gray
    
    # Define positions
    agent_pos = (2, 6)
    tools_pos = (6, 6)
    
    # Draw Agent node
    agent_box = FancyBboxPatch(
        (agent_pos[0]-0.8, agent_pos[1]-0.4),
        1.6, 0.8,
        boxstyle="round,pad=0.1",
        facecolor=agent_color,
        edgecolor='black',
        linewidth=2
    )
    ax.add_patch(agent_box)
    ax.text(agent_pos[0], agent_pos[1], 'Agent\n(LLM)', ha='center', va='center', 
            fontsize=12, fontweight='bold', color='white')
    
    # Draw Tools node
    tools_box = FancyBboxPatch(
        (tools_pos[0]-0.8, tools_pos[1]-0.4),
        1.6, 0.8,
        boxstyle="round,pad=0.1",
        facecolor=tool_color,
        edgecolor='black',
        linewidth=2
    )
    ax.add_patch(tools_box)
    ax.text(tools_pos[0], tools_pos[1], 'Tools', ha='center', va='center', 
            fontsize=12, fontweight='bold', color='white')
    
    # Draw arrows
    # Agent to Tools (conditional)
    ax.annotate('', xy=(tools_pos[0]-0.8, tools_pos[1]), 
                xytext=(agent_pos[0]+0.8, agent_pos[1]),
                arrowprops=dict(arrowstyle='->', lw=2, color=arrow_color))
    ax.text(4, 6.3, 'tools_condition', ha='center', va='bottom', 
            fontsize=10, color=text_color)
    
    # Tools back to Agent
    ax.annotate('', xy=(agent_pos[0]+0.8, agent_pos[1]-0.2), 
                xytext=(tools_pos[0]-0.8, tools_pos[1]-0.2),
                arrowprops=dict(arrowstyle='->', lw=2, color=arrow_color))
    ax.text(4, 5.5, 'always', ha='center', va='top', 
            fontsize=10, color=text_color)
    
    # Entry point arrow
    ax.annotate('', xy=(agent_pos[0]-0.8, agent_pos[1]), 
                xytext=(0.5, agent_pos[1]),
                arrowprops=dict(arrowstyle='->', lw=3, color='green'))
    ax.text(0.3, agent_pos[1]+0.3, 'START', ha='center', va='bottom', 
            fontsize=10, fontweight='bold', color='green')
    
    # Tool details box
    tools_detail_y = 4
    detail_box = FancyBboxPatch(
        (3.5, tools_detail_y-1.2),
        5, 2.4,
        boxstyle="round,pad=0.15",
        facecolor='#f5f5f5',
        edgecolor='gray',
        linewidth=1
    )
    ax.add_patch(detail_box)
    
    tool_list = [
        "• drone_takeoff_tool",
        "• drone_land_tool", 
        "• drone_goto_tool",
        "• drone_turn_tool",
        "• get_objective_coordinates"
    ]
    
    ax.text(6, tools_detail_y+0.6, 'Available Tools:', ha='center', va='center',
            fontsize=11, fontweight='bold', color=text_color)
    
    for i, tool in enumerate(tool_list):
        ax.text(4, tools_detail_y+0.2-i*0.3, tool, ha='left', va='center',
                fontsize=9, color=text_color)
    
    # Add title
    ax.text(4, 7.5, 'CrazyAgent State Graph Architecture', 
            ha='center', va='center', fontsize=16, fontweight='bold')
    
    # Add framework info
    ax.text(4, 0.5, 'Built with: LangGraph + Google Gemini 2.5 Flash + Crazyswarm2 + ROS2', 
            ha='center', va='center', fontsize=10, style='italic', color='gray')
    
    # Set limits and remove axes
    ax.set_xlim(0, 8)
    ax.set_ylim(0, 8)
    ax.set_aspect('equal')
    ax.axis('off')
    
    # Save the diagram
    plt.tight_layout()
    plt.savefig('/home/paul/git/crazyAgent/readme_files/agent_stategraph.png', 
                dpi=300, bbox_inches='tight', facecolor='white')
    plt.close()
    
    print("Agent stategraph diagram saved to readme_files/agent_stategraph.png")

if __name__ == "__main__":
    create_agent_stategraph()