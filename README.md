# UR3e Haptic Teleoperation with 3D Systems Touch (ROS 2)

Complete bilateral teleoperation pipeline for a Universal Robots UR3e with force feedback via a 3D Systems Touch haptic device in ROS 2 Humble.

---

## System Architecture

1. **Hardware Bridge (`touch_haptics/touch_haptic_bridge` - C++)**:
   - Runs an asynchronous 1000 Hz OpenHaptics scheduler loop for stable haptic force rendering.
   - Includes a startup hardware interlock to prevent streaming zero-vectors before device initialisation.
   - Topics: `/touch/raw_pose` (20 Hz), `/touch/buttons` (20 Hz), `/touch/cmd_force` (in).

2. **Teleop Publisher (`ur3e_teleop_control/touch_publisher` - Python)**:
   - Handles stylus offsets, clutching, deadbands, filtering, and workspace scaling.
   - Transforms tool-flange F/T sensor wrenches into the robot's `base_link` frame using forward kinematics (DH parameters).
   - Haptic force mapping features a 2.2 N deadband, EWMA smoothing, and counter-force rendering along the Touch axes.
   - Topics: `/target_pose` (out), `/gripper/cmd` (out), `/touch/cmd_force` (out).

3. **Task-Space Controller (`ur3e_teleop_control/task_space_controller` - Python)**:
   - Closed-loop DLS (Damped Least Squares) Jacobian inverse kinematics.
   - Startup interlock: commands zero velocity until a valid target pose is received.
   - Postural null-space projection towards the home configuration.
   - Topic: `/forward_velocity_controller/commands`.

---

## Startup Sequence (Step-by-Step)

Open 5 separate terminals and run `source ~/ros2_ws/install/setup.bash` in each:

### Terminal 1: Robot Driver & Velocity Controller
Launch the official UR driver with the forward velocity controller configured:
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=<ROBOT_IP> launch_rviz:=false initial_joint_controller:=forward_velocity_controller

### Terminal 2: Robotiq 2F Gripper Driver (Optional)
ros2 run robotiq_2f_driver gripper_node

### Terminal 3: Task Space Controller
Start the closed-loop kinematics controller (remains stationary until commands arrive):
source ~/ros2_ws/install/setup.bash
ros2 run ur3e_teleop_control task_space_controller

### Terminal 4: C++ Haptic Bridge (1000 Hz)
Initialise the Touch hardware interface:
source ~/ros2_ws/install/setup.bash
ros2 run touch_haptics touch_haptic_bridge

### Terminal 5: Touch Publisher & Input Manager
Start teleoperation mapping, clutching, and force-feedback processing:
source ~/ros2_ws/install/setup.bash
ros2 run ur3e_teleop_control touch_publisher

---

## Controls & Keybindings (Active in Terminal 5)

- [e]       : Engage & Tare (Locks origin, activates wrist orientation tracking, and tares F/T sensor)
- [SPACE]   : Clutch toggle (Freezes robot pose; allows repositioning stylus within workspace)
- [c]       : Close gripper (or Stylus Button 1)
- [o]       : Open gripper (or Stylus Button 2)
- [r]       : Reset Home (Returns target reference to home position)
- [Ctrl+C]  : Safe shutdown
