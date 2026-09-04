import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import ctypes
import numpy as np
import sys
import select
import termios
import tty
import threading

try:
    libhd = ctypes.CDLL("libHD.so")
except Exception:
    libhd = ctypes.CDLL("/opt/OpenHaptics/Developer/3.4-0/lib/libHD.so")

HD_CURRENT_POSITION = 0x2050
HD_CURRENT_GIMBAL_ANGLES = 0x2150
HD_CURRENT_BUTTONS = 0x2000

class TouchPosePublisher(Node):
    def __init__(self):
        super().__init__('touch_publisher')
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.reset_pub = self.create_publisher(Bool, '/reset_home', 10)
        self.engage_pub = self.create_publisher(Bool, '/engage_orientation', 10)
        self.gripper_pub = self.create_publisher(Bool, '/gripper/cmd', 10)

        self.hHD = libhd.hdInitDevice(None)
        if self.hHD == 0xFFFFFFFF:
            self.get_logger().error("Kon Touch haptic device niet initialiseren!")
            return

        libhd.hdEnable(0x2200)
        libhd.hdStartScheduler()

        self.clutched = False
        self.dock_pos = None
        self.dock_ang = None
        
        self.clutch_offset_pos = np.zeros(3)
        self.clutch_offset_ang = np.zeros(3)
        self.clutch_start_pos = None
        self.clutch_start_ang = None

        self.scale_pos = 1.3
        self.prev_btn_state = 0
        self.running = True
        self.engaged = False

        # Terminal keyboard thread
        self.orig_settings = termios.tcgetattr(sys.stdin)
        self.key_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.key_thread.start()

        self.timer = self.create_timer(0.02, self.publish_pose) # 50 Hz
        
        print("\n=======================================================")
        print("  TOUCH TELEOP BEDIENING MET ALIGNMENT")
        print("  [e]      : ENGAGE ORIENTATION (Lijn pen en gripper uit)")
        print("  [c]      : Sluit gripper (CLOSE)")
        print("  [o]      : Open gripper (OPEN)")
        print("  [r]      : Reset robot naar 90-graden rustpositie")
        print("  [SPATIE] : Clutch (pauzeer sturing om pen te verplaatsen)")
        print("  [Ctrl+C] : Stoppen")
        print("=======================================================\n")

    def keyboard_loop(self):
        tty.setcbreak(sys.stdin.fileno())
        try:
            while self.running and rclpy.ok():
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist:
                    key = sys.stdin.read(1)
                    if key == 'e':
                        self.engage_alignment()
                    elif key == 'c':
                        msg = Bool()
                        msg.data = True
                        self.gripper_pub.publish(msg)
                        self.get_logger().info(">>> Knop 'c': GRIPPER SLUITEN <<<")
                    elif key == 'o':
                        msg = Bool()
                        msg.data = False
                        self.gripper_pub.publish(msg)
                        self.get_logger().info(">>> Knop 'o': GRIPPER OPENEN <<<")
                    elif key == 'r':
                        self.reset_reference()
                    elif key == ' ':
                        self.toggle_clutch()
                    elif key == '\x03': # Ctrl+C
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_settings)

    def engage_alignment(self):
        libhd.hdBeginFrame(self.hHD)
        gimbal = (ctypes.c_double * 3)()
        libhd.hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal)
        libhd.hdEndFrame(self.hHD)

        curr_ang = np.array([gimbal[1], gimbal[0], gimbal[2]])
        self.dock_ang = curr_ang.copy()
        self.clutch_offset_ang = np.zeros(3)
        self.engaged = True

        msg = Bool()
        msg.data = True
        self.engage_pub.publish(msg)
        self.get_logger().info(">>> [ENGAGED] Pen & Gripper oriëntatie nu 1-op-1 uitgelijnd! <<<")

    def reset_reference(self):
        libhd.hdBeginFrame(self.hHD)
        pos = (ctypes.c_double * 3)()
        libhd.hdGetDoublev(HD_CURRENT_POSITION, pos)
        gimbal = (ctypes.c_double * 3)()
        libhd.hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal)
        libhd.hdEndFrame(self.hHD)

        self.dock_pos = np.array([pos[2] / 1000.0, pos[0] / 1000.0, pos[1] / 1000.0])
        self.dock_ang = np.array([gimbal[1], gimbal[0], gimbal[2]])
        self.clutch_offset_pos = np.zeros(3)
        self.clutch_offset_ang = np.zeros(3)
        self.engaged = False

        msg = Bool()
        msg.data = True
        self.reset_pub.publish(msg)
        self.get_logger().info(">>> Knop 'r': Robot terug naar 90-graden rustpositie. <<<")

    def toggle_clutch(self):
        self.clutched = not self.clutched
        if self.clutched:
            libhd.hdBeginFrame(self.hHD)
            pos = (ctypes.c_double * 3)()
            libhd.hdGetDoublev(HD_CURRENT_POSITION, pos)
            gimbal = (ctypes.c_double * 3)()
            libhd.hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal)
            libhd.hdEndFrame(self.hHD)
            self.clutch_start_pos = np.array([pos[2] / 1000.0, pos[0] / 1000.0, pos[1] / 1000.0])
            self.clutch_start_ang = np.array([gimbal[1], gimbal[0], gimbal[2]])
            self.get_logger().info(">>> CLUTCH ACTIEF (Robot gepauzeerd) <<<")
        else:
            if self.clutch_start_pos is not None:
                libhd.hdBeginFrame(self.hHD)
                pos = (ctypes.c_double * 3)()
                libhd.hdGetDoublev(HD_CURRENT_POSITION, pos)
                gimbal = (ctypes.c_double * 3)()
                libhd.hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal)
                libhd.hdEndFrame(self.hHD)
                curr_pos = np.array([pos[2] / 1000.0, pos[0] / 1000.0, pos[1] / 1000.0])
                curr_ang = np.array([gimbal[1], gimbal[0], gimbal[2]])
                self.clutch_offset_pos += (curr_pos - self.clutch_start_pos)
                self.clutch_offset_ang += (curr_ang - self.clutch_start_ang)
            self.get_logger().info(">>> CLUTCH VRIJGEGEVEN (Sturing hervat) <<<")

    def publish_pose(self):
        libhd.hdBeginFrame(self.hHD)
        pos = (ctypes.c_double * 3)()
        libhd.hdGetDoublev(HD_CURRENT_POSITION, pos)
        gimbal = (ctypes.c_double * 3)()
        libhd.hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal)
        buttons = ctypes.c_int()
        libhd.hdGetIntegerv(HD_CURRENT_BUTTONS, ctypes.byref(buttons))
        libhd.hdEndFrame(self.hHD)

        # Knoppen op Touch pen: Knop 1 = Sluit, Knop 2 = Open
        btn_val = buttons.value
        if (btn_val & 1) and not (self.prev_btn_state & 1):
            msg = Bool()
            msg.data = True
            self.gripper_pub.publish(msg)
            self.get_logger().info(">>> Stylus Knop 1: GRIPPER SLUITEN <<<")
        elif (btn_val & 2) and not (self.prev_btn_state & 2):
            msg = Bool()
            msg.data = False
            self.gripper_pub.publish(msg)
            self.get_logger().info(">>> Stylus Knop 2: GRIPPER OPENEN <<<")
        self.prev_btn_state = btn_val

        curr_pos = np.array([pos[2] / 1000.0, pos[0] / 1000.0, pos[1] / 1000.0])
        curr_ang = np.array([gimbal[1], gimbal[0], gimbal[2]])

        if self.dock_pos is None:
            self.dock_pos = curr_pos.copy()
            self.dock_ang = curr_ang.copy()

        if self.clutched:
            return

        effective_pos = curr_pos - self.clutch_offset_pos
        effective_ang = curr_ang - self.clutch_offset_ang

        delta_pos = (effective_pos - self.dock_pos) * self.scale_pos
        
        # Alleen delta_ang doorgeven als we engaged zijn, anders blijft pols netjes in rust
        if self.engaged:
            delta_ang = effective_ang - self.dock_ang
        else:
            delta_ang = np.zeros(3)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.pose.position.x = float(delta_pos[0])
        msg.pose.position.y = float(delta_pos[1])
        msg.pose.position.z = float(delta_pos[2])

        msg.pose.orientation.x = float(delta_ang[0])
        msg.pose.orientation.y = float(delta_ang[1])
        msg.pose.orientation.z = float(delta_ang[2])
        msg.pose.orientation.w = 1.0

        self.pose_pub.publish(msg)

    def destroy_node(self):
        self.running = False
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_settings)
        libhd.hdStopScheduler()
        libhd.hdDisableDevice(self.hHD)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TouchPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
