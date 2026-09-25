#!/bin/bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch robot_description bringup.launch.py use_rviz:=true
echo ""
echo "--- Launch exited. Press Enter to close. ---"
read
