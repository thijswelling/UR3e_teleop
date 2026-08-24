================================================================================
  UR3e 6-DOF TELEOPERATION WITH 3D SYSTEMS TOUCH (GEOMAGIC TOUCH / OMNI)
  Complete DIY Installation, Configuration & Run Guide
================================================================================

1. SYSTEM PREREQUISITES & OS DEPENDENCIES
--------------------------------------------------------------------------------
Supported Environment:
- OS: Ubuntu 22.04 LTS (Jammy Jellyfish)
- ROS 2: Humble Hawksbill

Install base system build tools and Python libraries:
$ sudo apt update
$ sudo apt install -y git build-essential python3-pip python3-colcon-common-extensions
$ pip3 install pynput numpy


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


3. ROS 2 UNIVERSAL ROBOTS PACKAGES
--------------------------------------------------------------------------------
Install the official Universal Robots ROS 2 Humble packages:
$ sudo apt install -y ros-humble-ur ros-humble-ur-moveit-config ros-humble-moveit


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

TERMINAL 1 (MoveIt / RViz Mock Simulation):
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e use_mock_hardware:=true

TERMINAL 2 (Real-Time 6-DOF Task-Space Controller):
$ source ~/ros2_ws/install/setup.bash
$ ros2 run ur3e_teleop_control task_space_controller

TERMINAL 3 (Touch Haptic Driver):
$ source ~/ros2_ws/install/setup.bash
$ ros2 run ur3e_teleop_control touch_publisher


6. CONTROLS & SHORTCUTS
--------------------------------------------------------------------------------
- Stylus linear movement (X, Y, Z)     -> UR3e Arm translation (DLS Jacobian)
- Stylus tilt / pitch (wrist 1)        -> UR3e Wrist 1 joint (pitch up/down)
- Stylus swivel / yaw (wrist 2)        -> UR3e Wrist 2 joint (yaw left/right)
- Stylus axial roll (wrist 3)          -> UR3e Wrist 3 joint (tool roll)
- [SPACEBAR] (Hold)                    -> Clutch mode (disengages robot to reposition hand)
- [r] key or Inkwell Docking           -> Automatic / manual drift-free reset to home pose
================================================================================
