#!/bin/bash

# Source the virtual environment
source /home/paul/git/crazyAgent/ca_venv/bin/activate

# Source ROS2 setup
source /home/paul/ros2_ws/install/setup.bash

# Launch crazyflie
ros2 launch crazyflie launch.py &

# Run the python script
python /home/paul/git/crazyAgent/crazyAgent.py