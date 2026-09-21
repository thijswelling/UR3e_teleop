# UR3e 6-DoF Haptic Teleoperation System

A high-performance ROS 2 Humble teleoperation pipeline connecting a **3D Systems Touch (Phantom Omni)** haptic device to a **Universal Robots UR3e** robotic arm equipped with a **Robotiq Hand-E** gripper and a Force/Torque sensor.

---

## Features
* **6-DoF Task-Space Control:** Full spatial position and absolute orientation tracking using a geometric Jacobian with Damped Least Squares (DLS).
* **PI-Control with Anti-Windup:** Guarantees zero steady-state tracking error and eliminates static offsets.
* **Haptic Force Feedback:** Translates real-time external forces measured by the robot's F/T sensor back into force feedback for the operator's haptic pen.
* **Gripper Integration:** Seamless socket communication bridge for the Robotiq Hand-E gripper.
* **Performance Evaluation Node:** Built-in telemetry logger (`TeleopEvaluator`) to track trajectory accuracy and compute Root Mean Square Error (RMSE) for academic reporting.

---

## System Architecture & Nodes
1. **`touch_haptic_bridge` (C++):** Interfacing directly with the OpenHaptics library at high frequency to read device positioning and apply force feedback.
2. **`touch_publisher` (Python):** Handles coordinate mapping, scaling, workspace limits, button inputs, and keyboard engagement commands (`[e]` to engage, `[SPACE]` for clutch, `[r]` to reset).
3. **`task_space_controller` (Python):** Computes forward kinematics, Jacobians, and closed-loop PI velocity commands (`/forward_velocity_controller/commands`).
4. **`robotiq_bridge` (Python):** Manages TCP socket communication with the Robotiq gripper controller.
5. **`teleop_evaluator` (Python):** Records performance metrics and generates comprehensive spatial, temporal, and force plots upon shutdown.

---

## Getting Started

### Prerequisites
* ROS 2 Humble
* Ubuntu 22.04 LTS
* OpenHaptics SDK & 3D Systems Touch drivers
* Universal Robots ROS 2 Driver (`ur_robot_driver`)

### Installation & Build
Clone the repository into your ROS 2 workspace source folder and build it:
```bash
cd ~/ros2_ws/src
git clone [https://github.com/thijswelling/UR3e_teleop.git](https://github.com/thijswelling/UR3e_teleop.git)
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

Running the Teleoperation Pipeline

Launch all nodes in order using the startup script by passing the IP address of your UR3e robot:
Bash

./start_teleop.sh 192.168.0.134

Operational Instructions

    Watch your terminal for the startup prompt:
    Plaintext

    ✅ ALLES GELADEN! Druk nu op [PLAY/RUN] op de UR Teach Pendant
       en druk op [e] in deze terminal om te koppelen (Engage)!

    Press [PLAY/RUN] on the physical UR Teach Pendant to activate external control.

    Press [e] in your computer terminal to lock the zero-point and engage teleoperation.

Optional Teleoperation Evaluator (teleop_evaluator.py)

For tracking position accuracy, force feedback synchronization, and generating performance graphs, you can use the optional telemetry evaluator node.

How to Use:

    Start the main teleop launch file and press [PLAY/RUN] on the UR Teach Pendant.

    Open a new terminal and run the evaluator node:
    Bash

    ros2 run ur3e_teleop_control teleop_evaluator

    Press [e] in your primary teleop terminal to engage the system and automatically start logging data.

    Press Ctrl+C in the evaluator terminal to stop the session, process the telemetry, and display the performance graphs.
