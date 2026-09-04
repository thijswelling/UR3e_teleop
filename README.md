================================================================================
  UR3e 6-DOF TELEOPERATION WITH 3D SYSTEMS TOUCH (GEOMAGIC TOUCH / OMNI)
  Complete DIY Installation, Configuration & Run Guide (Real Robot Pipeline)
================================================================================


1. SYSTEM PREREQUISITES & OS DEPENDENCIES
--------------------------------------------------------------------------------
Supported Environment:
- OS: Ubuntu 22.04 LTS (Jammy Jellyfish)
- ROS 2: Humble Hawksbill

Install base system build tools, controller packages and Python libraries:
$ sudo apt update
$ sudo apt install -y git build-essential python3-pip python3-colcon-common-extensions \
                      ros-humble-ros2-control ros-humble-ros2-controllers
$ pip3 install pynput numpy scipy


2. 3D SYSTEMS TOUCH / OPENHAPTICS DRIVER SETUP
--------------------------------------------------------------------------------
1. Download & install OpenHaptics Developer Edition (v3.4-0) and Touch Device Driver.
2. Link the shared C-library to system path:
   $ sudo ln -s /opt/OpenHaptics/Developer/3.4-0/lib/libHD.so /usr/lib/libHD.so
   $ sudo ldconfig

3. Configure USB / Serial permissions (udev rules):
   $ sudo usermod -aG dialout $USER
   $ echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0b62", MODE="0666"' | sudo tee /etc/udev/rules.d/99-geomagic.rules
   $ sudo udevadm control --reload-rules && sudo udevadm trigger

   (Note: Log out and log back in for dialout group permissions to apply)

4. Test hardware connection:
   $ Geomagic_Touch_Diagnostic


3. ROS 2 UNIVERSAL ROBOTS HARDWARE DRIVER & NETWORK SETUP
--------------------------------------------------------------------------------
1. Install the official Universal Robots ROS 2 driver:
   $ sudo apt install -y ros-humble-ur ros-humble-ur-robot-driver

2. Configure PC Static IP (Ethernet Settings):
   - IP Address: 192.168.0.100
   - Netmask:    255.255.255.0
   - Gateway:    192.168.0.1

3. Verify connection to UR3e robot:
   $ ping 192.168.0.134

4. On UR Teach Pendant:
   - Go to: Installation -> URCaps -> External Control
   - Set Host IP to: 192.168.0.100 (your laptop's IP)
   - Load 'external_control.urp' and ensure safety mode is NORMAL


4. WORKSPACE SETUP & BUILD
--------------------------------------------------------------------------------
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ git clone https://github.com/thijswelling/UR3e_teleop.git ur3e_teleop_control
$ cd ~/ros2_ws
$ colcon build --packages-select ur3e_teleop_control --symlink-install
$ source ~/ros2_ws/install/setup.bash


5. RUNNING THE SYSTEM (3 TERMINALS)
--------------------------------------------------------------------------------
Open three separate terminals and source the workspace in each:

TERMINAL 1 (UR3e Real-Time Hardware Driver & Controller Manager):
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3e \
    robot_ip:=192.168.0.134 \
    initial_joint_controller:=forward_velocity_controller \
    launch_rviz:=true

(Action on Teach Pendant: Press the Play button [▶] to start external control)

* Note: If scaled_joint_trajectory_controller stays active, switch it once via:
$ ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller --activate forward_velocity_controller

TERMINAL 2 (Real-Time 6-DOF Task-Space Controller):
$ source ~/ros2_ws/install/setup.bash
$ ros2 run ur3e_teleop_control task_space_controller

TERMINAL 3 (Touch Haptic Driver & Setpoint Publisher):
Place stylus in the physical dock before running:
$ source ~/ros2_ws/install/setup.bash
$ ros2 run ur3e_teleop_control touch_publisher


6. CONTROLS & SHORTCUTS (Inside Terminal 3)
--------------------------------------------------------------------------------
- Stylus linear movement (X, Y, Z)  -> UR3e Cartesian translation (DLS Jacobian)
- Stylus rotation (Pitch, Yaw, Roll)-> Flange orientation tracking (Lie Logarithm)
- [SPACEBAR]                        -> Clutch toggle (pause/resume mapping to prevent jumps)
- [e] key                           -> Engage orientation (aligns stylus with robot pose)
- [c] key                           -> Close gripper (/gripper/cmd)
- [o] key                           -> Open gripper (/gripper/cmd)
- [r] key                           -> Reset / Re-zero setpoint relative to dock position
- [Ctrl+C]                          -> Clean stop
================================================================================
