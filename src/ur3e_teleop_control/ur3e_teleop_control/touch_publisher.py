#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32
import numpy as np
from scipy.spatial.transform import Rotation as R
import sys, select, termios, tty, threading

class TouchPosePublisher(Node):
    def __init__(self):
        super().__init__('touch_publisher')
        
        # ROS 2 Publishers for sending target poses, reset signals, engage states, gripper commands, and haptic force feedback
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.reset_pub = self.create_publisher(Bool, '/reset_home', 10)
        self.engage_pub = self.create_publisher(Bool, '/engage_orientation', 10)
        self.gripper_pub = self.create_publisher(Bool, '/gripper/cmd', 10)
        self.cmd_force_pub = self.create_publisher(WrenchStamped, '/touch/cmd_force', 10)
        
        # Subscriptions to receive raw haptic data, button states, robot joint states, and force/torque sensor data
        self.raw_pose_sub = self.create_subscription(PoseStamped, '/touch/raw_pose', self.raw_pose_cb, 10)
        self.btn_sub = self.create_subscription(Int32, '/touch/buttons', self.button_cb, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        self.wrench_sub = self.create_subscription(WrenchStamped, '/force_torque_sensor_broadcaster/wrench', self.wrench_cb, qos_profile_sensor_data)

        # UR3e Denavit-Hartenberg parameters
        self.d = np.array([0.15185, 0.0, 0.0, 0.13105, 0.08535, 0.0921])
        self.a = np.array([0.0, -0.24355, -0.2132, 0.0, 0.0, 0.0])
        self.alpha = np.array([np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0])
        self.R_base_tool = np.eye(3)
        
        # Force/Torque sensor zero-calibration variables
        self.tare_external_force = np.zeros(3)
        self.is_tared = False
        self.tare_samples = []
        self.net_external_force = np.zeros(3)

        # Teleoperation state management variables
        self.clutched = False
        self.dock_pos = None
        self.dock_rot = None
        
        # Clutch offsets to prevent accumulation and drift during engagement
        self.clutch_offset_pos = np.zeros(3)
        self.clutch_offset_rot = R.identity()
        self.clutch_start_pos = None
        self.clutch_start_rot = None
        
        self.scale_pos = 1.8
        self.prev_btn_state = 0
        self.running = True
        self.engaged = False
        self.raw_pos = None
        self.raw_rot = None
        
        self.startup_samples = []
        self.is_initialized = False
        self.raw_wrench_force = np.zeros(3)
        self.filtered_force = np.zeros(3)
        
        # Setup non-blocking terminal keyboard listener for operator shortcuts
        self.orig_settings = termios.tcgetattr(sys.stdin)
        self.key_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.key_thread.start()

    def get_dh_matrix(self, theta, d, a, alpha):
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([[ct, -st*ca, st*sa, a*ct], [st, ct*ca, -ct*sa, a*st], [0, sa, ca, d], [0, 0, 0, 1.0]])

    def joint_cb(self, msg):
        try:
            names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            q = [msg.position[msg.name.index(n)] for n in names]
            T = np.eye(4)
            for i in range(6): T = T @ self.get_dh_matrix(q[i], self.d[i], self.a[i], self.alpha[i])
            self.R_base_tool = T[0:3, 0:3]
        except (ValueError, IndexError): pass

    def raw_pose_cb(self, msg):
        pos_raw = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q_raw = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        rot_raw = R.from_quat(q_raw)
        
        # --- OPLOSSING AS-MAPPING ---
        # 1. Positie Mapping (Naar UR3e Base Frame)
        R_pos = np.array([
            [0,  0, 1],
            [1,  0, 0],
            [0,  1, 0]
        ])
        curr_pos = R_pos @ (pos_raw / 1000.0)

        # 2. Rotatie Mapping (Naar UR3e TCP Frame)
        # We zetten assen rechtdoor (X=X, Y=Y, Z=Z). De -1 waarden spiegelen de input om 
        # aan te sluiten op jouw perspectief. (Als een as nog steeds verkeerd om draait, 
        # verander de 1 of -1 op die specifieke as!)
        R_rot = np.array([
            [-1,  0,  0],  # X-as (Roll)
            [ 0,  1,  0],  # Y-as (Pitch)
            [ 0,  0, -1]   # Z-as (Yaw)
        ])
        curr_rot = R.from_matrix(R_rot @ rot_raw.as_matrix() @ R_rot.T)
        # ----------------------------

        self.raw_pos = curr_pos
        self.raw_rot = curr_rot
        
        # Initialize zero-point docking position on startup
        if not self.is_initialized:
            self.startup_samples.append(curr_pos)
            if len(self.startup_samples) >= 25:
                self.dock_pos = np.mean(self.startup_samples, axis=0)
                self.dock_rot = curr_rot
                self.is_initialized = True
                self.get_logger().info('>>> Haptic pen zero-point locked <<<')
            return
            
        self.publish_haptic_force()
        
        if not self.engaged:
            out_msg = PoseStamped()
            out_msg.header.stamp = self.get_clock().now().to_msg()
            out_msg.header.frame_id = 'base_link'
            out_msg.pose.orientation.w = 1.0
            self.pose_pub.publish(out_msg)
            return

        if self.clutched: return
        
        effective_pos = curr_pos - self.clutch_offset_pos
        effective_rot = self.clutch_offset_rot.inv() * curr_rot

        delta_pos = (effective_pos - self.dock_pos) * self.scale_pos
        delta_rot = self.dock_rot.inv() * effective_rot
        delta_q = delta_rot.as_quat()
        
        out_msg = PoseStamped()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = 'base_link'
        out_msg.pose.position.x = float(delta_pos[0])
        out_msg.pose.position.y = float(delta_pos[1])
        out_msg.pose.position.z = float(delta_pos[2])
        out_msg.pose.orientation.x = float(delta_q[0])
        out_msg.pose.orientation.y = float(delta_q[1])
        out_msg.pose.orientation.z = float(delta_q[2])
        out_msg.pose.orientation.w = float(delta_q[3])
        self.pose_pub.publish(out_msg)

    def button_cb(self, msg):
        btn_val = msg.data
        if (btn_val & 1) and not (self.prev_btn_state & 1): self.gripper_pub.publish(Bool(data=True))
        elif (btn_val & 2) and not (self.prev_btn_state & 2): self.gripper_pub.publish(Bool(data=False))
        self.prev_btn_state = btn_val

    def wrench_cb(self, msg):
        sample = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        
        if not hasattr(self, 'is_first_wrench'):
            self.raw_wrench_force = sample
            self.is_first_wrench = False
        else:
            self.raw_wrench_force = 0.02 * sample + 0.98 * self.raw_wrench_force
        
        if not self.is_tared:
            self.tare_samples.append(self.raw_wrench_force)
            if len(self.tare_samples) >= 100: 
                self.tare_external_force = np.mean(self.tare_samples, axis=0)
                self.is_tared = True
                self.get_logger().info(f'>>> Sensor zero-measurement completed <<<')
                self.get_logger().info('==============================================================')
                self.get_logger().info('ALLES GELADEN! Druk nu op [PLAY/RUN] op de UR Teach Pendant')
                self.get_logger().info('==============================================================')

        self.net_external_force = self.raw_wrench_force - self.tare_external_force

    def publish_haptic_force(self):
        f_msg = WrenchStamped()
        f_msg.header.stamp = self.get_clock().now().to_msg()
        f_msg.header.frame_id = 'touch'
        target_force = np.zeros(3)
        
        if self.is_tared and self.engaged:
            net_force_base = self.R_base_tool @ self.net_external_force
            mag = np.linalg.norm(net_force_base)
            deadband = 1.7 
            if mag > deadband:
                eff = mag - deadband
                f_cmd = -(net_force_base / mag) * (eff * 0.2) 
                f_mag = np.linalg.norm(f_cmd)
                if f_mag > 2.5: f_cmd = (f_cmd / f_mag) * 2.5 
                target_force = f_cmd
                
        self.filtered_force = 0.3 * target_force + 0.7 * self.filtered_force
        f_msg.wrench.force.x = -float(self.filtered_force[1])
        f_msg.wrench.force.y = -float(self.filtered_force[2])
        f_msg.wrench.force.z = -float(self.filtered_force[0])
        self.cmd_force_pub.publish(f_msg)

    def keyboard_loop(self):
        tty.setcbreak(sys.stdin.fileno())
        try:
            while self.running and rclpy.ok():
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist:
                    key = sys.stdin.read(1)
                    if key == 'e':
                        if self.raw_pos is not None and self.raw_rot is not None:
                            self.dock_pos = self.raw_pos.copy()
                            self.dock_rot = self.raw_rot
                            self.clutched = False
                            self.clutch_offset_pos = np.zeros(3)
                            self.clutch_offset_rot = R.identity()
                            self.engaged = True
                            self.is_tared = False
                            self.tare_samples = []
                            self.filtered_force = np.zeros(3)
                            self.engage_pub.publish(Bool(data=True))
                            self.get_logger().info('>>> [ENGAGED] Clean reset executed! <<<')
                    elif key == 'c': self.gripper_pub.publish(Bool(data=True))
                    elif key == 'o': self.gripper_pub.publish(Bool(data=False))
                    elif key == 'r':
                        if self.raw_pos is not None and self.engaged:
                            self.dock_pos = self.raw_pos.copy()
                            self.dock_rot = self.raw_rot
                            self.clutch_offset_pos = np.zeros(3)
                            self.clutch_offset_rot = R.identity()
                            if self.clutched:
                                self.clutch_start_pos = self.raw_pos.copy()
                                self.clutch_start_rot = self.raw_rot
                            self.filtered_force = np.zeros(3)
                            self.reset_pub.publish(Bool(data=True))
                    elif key == ' ':
                        self.clutched = not self.clutched
                        if self.clutched and self.raw_pos is not None:
                            self.clutch_start_pos = self.raw_pos.copy()
                            self.clutch_start_rot = self.raw_rot
                            self.filtered_force = np.zeros(3)
                            self.get_logger().info('>>> [CLUTCH ON] Beweeg pen vrij rond <<<')
                        elif not self.clutched and self.clutch_start_pos is not None and self.raw_pos is not None:
                            self.clutch_offset_pos += (self.raw_pos - self.clutch_start_pos)
                            self.clutch_offset_rot = self.raw_rot * self.clutch_start_rot.inv() * self.clutch_offset_rot
                            self.get_logger().info('>>> [CLUTCH OFF] Koppeling hervat <<<')                    
                    elif key == '\x03': break

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_settings)

    def destroy_node(self):
        self.running = False
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_settings)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TouchPosePublisher()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()