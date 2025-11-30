#!/bin/bash

# build workspace + limo control and sim packages
cd /root/workspace
colcon build --packages-select limo_control limo_simulation

#source setup
source /root/workspace/install/setup.bash

#launch both the sim and the control nodes
ros2 launch limo_control limo_control.launch.py 