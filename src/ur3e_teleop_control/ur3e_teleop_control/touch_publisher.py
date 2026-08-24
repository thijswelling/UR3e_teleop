import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
import ctypes
import numpy as np
from pynput import keyboard

try:
    libhd = ctypes.CDLL("libHD.so")
except Exception:
    libhd = ctypes.CDLL("/opt/OpenHaptics/Developer/3.4-0/lib/libHD.so")

HD_CURRENT_POSITION = 0x2050
HD_CURRENT_GIMBAL_ANGLES = 0x2150

class Touch6DOFPublisher(Node):
    def __init__(self):
        super().__init__('touch_publisher')
        self.publisher_ = self.create_publisher(TwistStamped, '/target_twist', 10)
        self.reset_pub_ = self.create_publisher(Bool, '/reset_home', 10)
        
        self.hHD = libhd.hdInitDevice(None)
        if self.hHD == 0xFFFFFFFF:
            self.get_logger().error("Kon Touch niet initialiseren!")
            return

        libhd.hdEnable(0x2200)
        libhd.hdStartScheduler()

        self.clutched = False
        self.prev_pos = None
        self.prev_angles = None
        self.prev_time = self.get_clock().now()

        # Kalibratie
        self.scale_lin = 1.5
        self.scale_ang = 1.8

        self.dock_pos = None
        self.dock_ang = None

        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()

        self.timer = self.create_timer(0.02, self.publish_twist) # 50 Hz
        self.get_logger().info("Touch 6-DOF Driver actief! [Spatie = Clutch | 'r' = Reset]")

    def on_press(self, key):
        if key == keyboard.Key.space:
            self.clutched = True
        try:
            if key.char == 'r':
                msg = Bool()
                msg.data = True
                self.reset_pub_.publish(msg)
                self.get_logger().info("Handmatige reset naar beginstand verstuurd!")
        except AttributeError:
            pass

    def on_release(self, key):
        if key == keyboard.Key.space:
            self.clutched = False
            self.prev_pos = None
            self.prev_angles = None

    def publish_twist(self):
        libhd.hdBeginFrame(self.hHD)
        
        pos = (ctypes.c_double * 3)()
        libhd.hdGetDoublev(HD_CURRENT_POSITION, pos)
        
        gimbal = (ctypes.c_double * 3)()
        libhd.hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal)
        
        libhd.hdEndFrame(self.hHD)

        curr_time = self.get_clock().now()
        dt = (curr_time - self.prev_time).nanoseconds / 1e9

        curr_pos = np.array([-pos[2] / 1000.0, -pos[0] / 1000.0, pos[1] / 1000.0])
        curr_ang = np.array([gimbal[1], gimbal[0], gimbal[2]])

        # Inktpot nulpunt vastleggen bij start
        if self.dock_pos is None:
            self.dock_pos = curr_pos.copy()
            self.dock_ang = curr_ang.copy()

        # Automatische detectie: pen terug in inktpot (< 5 mm en < 0.05 rad)
        dist_to_dock = np.linalg.norm(curr_pos - self.dock_pos)
        ang_to_dock = np.linalg.norm(curr_ang - self.dock_ang)
        if dist_to_dock < 0.005 and ang_to_dock < 0.05:
            reset_msg = Bool()
            reset_msg.data = True
            self.reset_pub_.publish(reset_msg)

        msg = TwistStamped()
        msg.header.stamp = curr_time.to_msg()
        msg.header.frame_id = "base_link"

        if not self.clutched and self.prev_pos is not None and self.prev_angles is not None and dt > 0.001:
            v_lin = (curr_pos - self.prev_pos) / dt * self.scale_lin
            v_lin = np.clip(v_lin, -0.35, 0.35)

            w_ang = (curr_ang - self.prev_angles) / dt * self.scale_ang
            w_ang = np.clip(w_ang, -1.5, 1.5)

            msg.twist.linear.x = float(v_lin[0])
            msg.twist.linear.y = float(v_lin[1])
            msg.twist.linear.z = float(v_lin[2])

            msg.twist.angular.x = float(w_ang[0])
            msg.twist.angular.y = float(w_ang[1])
            msg.twist.angular.z = float(w_ang[2])

        self.publisher_.publish(msg)
        self.prev_pos = curr_pos
        self.prev_angles = curr_ang
        self.prev_time = curr_time

    def destroy_node(self):
        self.listener.stop()
        libhd.hdStopScheduler()
        libhd.hdDisableDevice(self.hHD)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Touch6DOFPublisher()
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
