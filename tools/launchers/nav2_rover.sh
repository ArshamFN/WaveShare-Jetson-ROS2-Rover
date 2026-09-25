#!/bin/bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false params_file:=$HOME/ros2_ws/src/robot_description/config/nav2_params.yaml
echo ""
echo "--- Nav2 exited. Press Enter to close. ---"
read
