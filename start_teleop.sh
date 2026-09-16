#!/usr/bin/env bash

if [ -z "$1" ]; then
    echo "Gebruik: ./start_teleop.sh <ROBOT_IP>"
    exit 1
fi

ROBOT_IP=$1
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS 2 en workspace
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

echo "=== Teleop starten voor UR3e op IP: ${ROBOT_IP} ==="

# Trap om alle achtergrondprocessen netjes af te sluiten bij Ctrl+C
cleanup() {
    echo -e "\nAfsluiten teleop pipeline..."
    kill $(jobs -p) 2>/dev/null
    wait $(jobs -p) 2>/dev/null
    echo "Alles veilig beëindigd."
}
trap cleanup SIGINT SIGTERM EXIT

# 1. UR Driver (achtergrond)
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=${ROBOT_IP} launch_rviz:=false initial_joint_controller:=forward_velocity_controller &
sleep 5

# 2. Robotiq Gripper Socket Bridge (achtergrond)
ros2 run ur3e_teleop_control robotiq_bridge --ros-args -p robot_ip:=${ROBOT_IP} &
sleep 2

# 3. Kinematics Controller (achtergrond)
ros2 run ur3e_teleop_control task_space_controller &
sleep 1

# 4. OpenHaptics Bridge (achtergrond)
ros2 run touch_haptics touch_haptic_bridge &
sleep 2

# 5. Interactive Teleop Node (voorgrond - toetsenbordinvoer)
ros2 run ur3e_teleop_control touch_publisher
