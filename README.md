# UR3e Haptic Teleoperation met 3D Systems Touch (ROS 2)

Complete bilaterale teleoperatie-pipeline voor een Universal Robots UR3e met force feedback via een 3D Systems Touch haptic device in ROS 2 Humble.

---

## Systeemarchitectuur

1. **Hardware Bridge (`touch_haptics/touch_haptic_bridge` - C++)**:
   - Draait op een asynchrone 1000 Hz OpenHaptics scheduler loop voor stabiele krachtsturing.
   - Bevat een startup hardware-interlock om te voorkomen dat nullen worden gepubliceerd voor de driver actief is.
   - Topics: `/touch/raw_pose` (20 Hz), `/touch/buttons` (20 Hz), `/touch/cmd_force` (in).

2. **Teleop Publisher (`ur3e_teleop_control/touch_publisher` - Python)**:
   - Verwerkt hardware-offsets, clutching, filtering en scaling.
   - Roteert F/T-metingen via voorwaartse kinematica (DH-parameters) van het gripper-frame naar het `base_link` robotframe.
   - Haptische mapping met dode band (2.2 N), EWMA-smoothing en actieve tegendruk op de Touch assen.
   - Topics: `/target_pose` (uit), `/gripper/cmd` (uit), `/touch/cmd_force` (uit).

3. **Task-Space Controller (`ur3e_teleop_control/task_space_controller` - Python)**:
   - Closed-loop DLS (Damped Least Squares) Jacobiaan inversiekinematica.
   - Bevat opstartvergrendeling: stuurt nulsnelheden totdat de eerste geldige target pose binnen is.
   - Null-space postural projectie naar de ruststand.
   - Topic: `/forward_velocity_controller/commands`.

---

## Opstartvolgorde (Stappenplan)

Gebruik 5 afzonderlijke terminals met telkens `source ~/ros2_ws/install/setup.bash`:

### Terminal 1: Robot Driver & Forward Velocity Controller
Start de UR-driver en zorg dat de forward velocity controller actief is:
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=<ROBOT_IP> launch_rviz:=false initial_joint_controller:=forward_velocity_controller

### Terminal 2: Robotiq 2F Gripper Driver (optioneel)
ros2 run robotiq_2f_driver gripper_node

### Terminal 3: Task Space Controller
Start de gesloten-lus kinematica controller (deze blijft stilstaan tot doelen binnenkomen):
source ~/ros2_ws/install/setup.bash
ros2 run ur3e_teleop_control task_space_controller

### Terminal 4: C++ Haptic Bridge (1000 Hz)
Initialiseer de Touch interface:
source ~/ros2_ws/install/setup.bash
ros2 run touch_haptics touch_haptic_bridge

### Terminal 5: Touch Publisher & Input Manager
Start de invoerbehandeling en force mapping:
source ~/ros2_ws/install/setup.bash
ros2 run ur3e_teleop_control touch_publisher

---

## Besturing & Toetsen (in Terminal 5)

- [e]      : Engage & Tare (Vergrendelt nulstand, activeert pols-oriëntatiesturing en tareert F/T sensor)
- [SPATIE] : Clutch (Pauzeert robotbeweging; verplaats de stylus zonder de robot mee te nemen)
- [c]      : Sluit gripper (of Stylus Knop 1)
- [o]      : Open gripper (of Stylus Knop 2)
- [r]      : Reset Home (Brengt doelreferentie terug naar de fysieke startpositie)
- [Ctrl+C] : Stop teleoperatie veilig
