#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Bool

class TeleopEvaluator(Node):
    def __init__(self):
        super().__init__('teleop_evaluator')
        
        # UR3e DH parameters voor Forward Kinematics
        self.d = np.array([0.15185, 0.0, 0.0, 0.13105, 0.08535, 0.0921])
        self.a = np.array([0.0, -0.24355, -0.2132, 0.0, 0.0, 0.0])
        self.alpha = np.array([np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0])

        # Data opslag arrays
        self.is_engaged = False
        self.home_pos = None
        
        self.target_data = {'t': [], 'x': [], 'y': [], 'z': []}
        self.robot_data = {'t': [], 'x': [], 'y': [], 'z': []}
        
        self.robot_force = {'t': [], 'fx': [], 'fy': [], 'fz': []}
        self.haptic_force = {'t': [], 'fx': [], 'fy': [], 'fz': []}

        # Subscribers
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(PoseStamped, '/target_pose', self.target_cb, 10)
        self.create_subscription(Bool, '/engage_orientation', self.engage_cb, 10)
        self.create_subscription(WrenchStamped, '/force_torque_sensor_broadcaster/wrench', self.robot_force_cb, 10)
        self.create_subscription(WrenchStamped, '/touch/cmd_force', self.haptic_force_cb, 10)

        self.start_time = None
        self.get_logger().info("Evaluator gestart. Druk op 'e' in je teleop terminal om het loggen te starten. Druk op Ctrl+C om de grafieken te genereren.")

    def get_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def get_dh_matrix(self, theta, d, a, alpha):
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([[ct, -st*ca, st*sa, a*ct], 
                         [st, ct*ca, -ct*sa, a*st], 
                         [0, sa, ca, d], 
                         [0, 0, 0, 1.0]])

    def forward_kinematics(self, q):
        T = np.eye(4)
        for i in range(3):
            T = T @ self.get_dh_matrix(q[i], self.d[i], self.a[i], self.alpha[i])
        return T[0:3, 3]

    def engage_cb(self, msg):
        if msg.data:
            self.is_engaged = True
            self.start_time = self.get_time()
            
            self.target_data = {'t': [], 'x': [], 'y': [], 'z': []}
            self.robot_data = {'t': [], 'x': [], 'y': [], 'z': []}
            self.robot_force = {'t': [], 'fx': [], 'fy': [], 'fz': []}
            self.haptic_force = {'t': [], 'fx': [], 'fy': [], 'fz': []}
            
            self.get_logger().info("Engaged! Data wordt nu opgeslagen voor evaluatie...")

    def joint_cb(self, msg):
        if not self.is_engaged: return
        try:
            names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint']
            q = [msg.position[msg.name.index(n)] for n in names]
            pos = self.forward_kinematics(q)
            
            if self.home_pos is None:
                self.home_pos = pos.copy()

            t = self.get_time() - self.start_time
            self.robot_data['t'].append(t)
            self.robot_data['x'].append(pos[0])
            self.robot_data['y'].append(pos[1])
            self.robot_data['z'].append(pos[2])
        except (ValueError, IndexError):
            pass

    def target_cb(self, msg):
        if not self.is_engaged or self.home_pos is None: return
        target_abs_x = self.home_pos[0] + msg.pose.position.x
        target_abs_y = self.home_pos[1] + msg.pose.position.y
        target_abs_z = self.home_pos[2] + msg.pose.position.z
        
        t = self.get_time() - self.start_time
        self.target_data['t'].append(t)
        self.target_data['x'].append(target_abs_x)
        self.target_data['y'].append(target_abs_y)
        self.target_data['z'].append(target_abs_z)

    def robot_force_cb(self, msg):
        if not self.is_engaged: return
        t = self.get_time() - self.start_time
        self.robot_force['t'].append(t)
        self.robot_force['fx'].append(msg.wrench.force.x)
        self.robot_force['fy'].append(msg.wrench.force.y)
        self.robot_force['fz'].append(msg.wrench.force.z)

    def haptic_force_cb(self, msg):
        if not self.is_engaged: return
        t = self.get_time() - self.start_time
        self.haptic_force['t'].append(t)
        self.haptic_force['fx'].append(msg.wrench.force.x)
        self.haptic_force['fy'].append(msg.wrench.force.y)
        self.haptic_force['fz'].append(msg.wrench.force.z)

    def plot_results(self):
        if len(self.target_data['t']) < 10 or len(self.robot_data['t']) < 10:
            self.get_logger().info("Niet genoeg data verzameld om te plotten.")
            return

        self.get_logger().info("Genereren van grafieken... Dit kan een moment duren.")

        t_tar = np.array(self.target_data['t'])
        pos_tar = np.array([self.target_data['x'], self.target_data['y'], self.target_data['z']])
        
        t_rob = np.array(self.robot_data['t'])
        pos_rob = np.array([self.robot_data['x'], self.robot_data['y'], self.robot_data['z']])

        # --- Interpolatie voor Foutberekening (Tijdlijnen synchroniseren) ---
        rob_x_interp = np.interp(t_tar, t_rob, pos_rob[0])
        rob_y_interp = np.interp(t_tar, t_rob, pos_rob[1])
        rob_z_interp = np.interp(t_tar, t_rob, pos_rob[2])
        
        # Euclidische afstandsfout per meetpunt
        error = np.sqrt((pos_tar[0] - rob_x_interp)**2 + 
                        (pos_tar[1] - rob_y_interp)**2 + 
                        (pos_tar[2] - rob_z_interp)**2)

        # Bereken de Cumulatieve RMSE over de tijd
        squared_error = error**2
        cum_mean_sq_error = np.cumsum(squared_error) / np.arange(1, len(squared_error) + 1)
        rmse_over_time = np.sqrt(cum_mean_sq_error) * 1000  # Omgezet naar millimeters

        # ==========================================
        # FIGUUR 1: POSITIE EN KINEMATISCHE ACCURAATHEID (Ruimtelijk)
        # ==========================================
        fig1 = plt.figure(figsize=(16, 10))
        fig1.canvas.manager.set_window_title('Ruimtelijk: 2D & 3D Trajecten')

        ax1 = fig1.add_subplot(2, 2, 1)
        ax1.plot(pos_tar[0], pos_tar[1], 'r--', label='Omni Target', linewidth=2)
        ax1.plot(pos_rob[0], pos_rob[1], 'b-', label='UR3e', alpha=0.7)
        ax1.set_title("Bovenaanzicht (X-Y Vlak)")
        ax1.set_xlabel("X (m)"); ax1.set_ylabel("Y (m)")
        ax1.legend(); ax1.grid(True)

        ax2 = fig1.add_subplot(2, 2, 2)
        ax2.plot(pos_tar[0], pos_tar[2], 'r--', label='Omni Target', linewidth=2)
        ax2.plot(pos_rob[0], pos_rob[2], 'b-', label='UR3e', alpha=0.7)
        ax2.set_title("Vooraanzicht (X-Z Vlak)")
        ax2.set_xlabel("X (m)"); ax2.set_ylabel("Z (m)")
        ax2.legend(); ax2.grid(True)

        ax4 = fig1.add_subplot(2, 1, 2, projection='3d')
        ax4.plot(pos_tar[0], pos_tar[1], pos_tar[2], 'r--', label='Target')
        ax4.plot(pos_rob[0], pos_rob[1], pos_rob[2], 'b-', label='UR3e', alpha=0.7)
        ax4.set_title("3D Traject Overlay")
        ax4.set_xlabel("X"); ax4.set_ylabel("Y"); ax4.set_zlabel("Z")
        ax4.legend()

        # ==========================================
        # FIGUUR 2: KRACHTEN FEEDBACK (HAPTICS)
        # ==========================================
        fig2 = plt.figure(figsize=(16, 6))
        fig2.canvas.manager.set_window_title('Krachten: F/T Sensor vs Haptic Output')

        t_rf = np.array(self.robot_force['t'])
        t_hf = np.array(self.haptic_force['t'])

        ax_fx = fig2.add_subplot(1, 3, 1)
        ax_fx.plot(t_rf, self.robot_force['fx'], label='Robot Sensor (X)', color='gray', alpha=0.5)
        ax_fx.plot(t_hf, self.haptic_force['fx'], label='Haptic Output (X)', color='red')
        ax_fx.set_title("Krachten X-As"); ax_fx.set_xlabel("Tijd (s)"); ax_fx.set_ylabel("Kracht (N)")
        ax_fx.legend(); ax_fx.grid(True)

        ax_fy = fig2.add_subplot(1, 3, 2)
        ax_fy.plot(t_rf, self.robot_force['fy'], label='Robot Sensor (Y)', color='gray', alpha=0.5)
        ax_fy.plot(t_hf, self.haptic_force['fy'], label='Haptic Output (Y)', color='green')
        ax_fy.set_title("Krachten Y-As"); ax_fy.set_xlabel("Tijd (s)")
        ax_fy.legend(); ax_fy.grid(True)

        ax_fz = fig2.add_subplot(1, 3, 3)
        ax_fz.plot(t_rf, self.robot_force['fz'], label='Robot Sensor (Z)', color='gray', alpha=0.5)
        ax_fz.plot(t_hf, self.haptic_force['fz'], label='Haptic Output (Z)', color='blue')
        ax_fz.set_title("Krachten Z-As"); ax_fz.set_xlabel("Tijd (s)")
        ax_fz.legend(); ax_fz.grid(True)

        # ==========================================
        # FIGUUR 3: TIJDSDOMEIN (X, Y, Z OVER TIJD) & RMSE
        # ==========================================
        fig3 = plt.figure(figsize=(16, 10))
        fig3.canvas.manager.set_window_title('Tijdsdomein: Positie per as & RMSE Foutmarge')

        ax_x = fig3.add_subplot(2, 2, 1)
        ax_x.plot(t_tar, pos_tar[0], 'r--', label='Omni Target X')
        ax_x.plot(t_rob, pos_rob[0], 'b-', label='UR3e X', alpha=0.7)
        ax_x.set_title("X Positie over Tijd (Check hier op vertraging/lag)")
        ax_x.set_xlabel("Tijd (s)"); ax_x.set_ylabel("Positie X (m)")
        ax_x.legend(); ax_x.grid(True)

        ax_y = fig3.add_subplot(2, 2, 2)
        ax_y.plot(t_tar, pos_tar[1], 'r--', label='Omni Target Y')
        ax_y.plot(t_rob, pos_rob[1], 'b-', label='UR3e Y', alpha=0.7)
        ax_y.set_title("Y Positie over Tijd")
        ax_y.set_xlabel("Tijd (s)"); ax_y.set_ylabel("Positie Y (m)")
        ax_y.legend(); ax_y.grid(True)

        ax_z = fig3.add_subplot(2, 2, 3)
        ax_z.plot(t_tar, pos_tar[2], 'r--', label='Omni Target Z')
        ax_z.plot(t_rob, pos_rob[2], 'b-', label='UR3e Z', alpha=0.7)
        ax_z.set_title("Z Positie over Tijd")
        ax_z.set_xlabel("Tijd (s)"); ax_z.set_ylabel("Positie Z (m)")
        ax_z.legend(); ax_z.grid(True)

        # RMSE en Absolute fout plot
        ax_rmse = fig3.add_subplot(2, 2, 4)
        ax_rmse.plot(t_tar, error * 1000, color='gray', alpha=0.4, label='Huidige Fout (Instantaan)')
        ax_rmse.plot(t_tar, rmse_over_time, 'm-', linewidth=2.5, label='Cumulatieve RMSE')
        
        final_rmse = rmse_over_time[-1]
        ax_rmse.set_title(f"Systeem Foutmarge (Eind RMSE: {final_rmse:.2f} mm)")
        ax_rmse.set_xlabel("Tijd (s)"); ax_rmse.set_ylabel("Fout (mm)")
        ax_rmse.legend(); ax_rmse.grid(True)

        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.plot_results()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()