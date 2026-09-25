#!/bin/bash
cd ~/ros2_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
echo ""
echo "--- Teleop exited. Press Enter to close. ---"
read
