#!/usr/bin/env bash

# Check if the user provided the robot IP address when running the script.
if [ -z "$1" ]; then
    echo "Usage: ./start_teleop.sh <ROBOT_IP>"
    exit 1
fi

ROBOT_IP=$1
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Load the ROS 2 Humble environment and the custom workspace.
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

echo "=== Starting Teleop for UR3e on IP: ${ROBOT_IP} ==="

# Create a safety trap: if the user presses Ctrl+C, this function ensures 
# all background processes are cleanly killed before closing the terminal.
cleanup() {
    echo -e "\nShutting down teleop pipeline..."
    kill $(jobs -p) 2>/dev/null
    wait $(jobs -p) 2>/dev/null
    echo "All nodes safely terminated."
}
trap cleanup SIGINT SIGTERM EXIT

# Start all the nodes. The '&' symbol at the end of the lines means the process 
# runs in the background, allowing the script to continue to the next step.

# 1. Start the official UR Robot Driver (Background)
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=${ROBOT_IP} launch_rviz:=false initial_joint_controller:=forward_velocity_controller &
sleep 5 # Wait 5 seconds for the hardware to connect properly.

# 2. Start the Robotiq Gripper Socket connection (Background)
ros2 run ur3e_teleop_control robotiq_bridge --ros-args -p robot_ip:=${ROBOT_IP} &
sleep 2

# 3. Start the custom Kinematics Controller (Background)
ros2 run ur3e_teleop_control task_space_controller &
sleep 1

# 4. Start the 1000 Hz OpenHaptics Bridge (Background)
ros2 run touch_haptics touch_haptic_bridge &
sleep 2

# 5. Start the Interactive Teleop Node (Foreground)
# This one runs in the foreground without an '&' because it needs to read the keyboard inputs.
ros2 run ur3e_teleop_control touch_publisher


