import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
from collections import deque

class LivePlotterNode(Node):
    def __init__(self):
        super().__init__('live_dashboard')
        
        self.create_subscription(PoseStamped, '/target_pose', self.target_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(WrenchStamped, '/force_torque_sensor_broadcaster/wrench', self.robot_force_cb, qos_profile_sensor_data)
        self.create_subscription(WrenchStamped, '/touch/cmd_force', self.pen_force_cb, 10)

        self.d = np.array([0.15185, 0.0, 0.0, 0.13105, 0.08535, 0.0921])
        self.a = np.array([0.0, -0.24355, -0.2132, 0.0, 0.0, 0.0])
        self.alpha = np.array([np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0])
        self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

        # Variabelen voor berekening
        self.target_delta = np.zeros(3)
        self.current_q = None
        self.home_pos = None
        self.robot_fz = 0.0
        self.pen_fy = 0.0

        # Data opslag voor de live grafiek (bewaart maximaal ~100 metingen = 10 seconden live view)
        self.max_pts = 100
        self.t_data = deque(maxlen=self.max_pts)
        self.t_target_x = deque(maxlen=self.max_pts); self.t_target_y = deque(maxlen=self.max_pts); self.t_target_z = deque(maxlen=self.max_pts)
        self.t_actual_x = deque(maxlen=self.max_pts); self.t_actual_y = deque(maxlen=self.max_pts); self.t_actual_z = deque(maxlen=self.max_pts)
        self.t_error = deque(maxlen=self.max_pts)
        self.t_rob_fz = deque(maxlen=self.max_pts)
        self.t_pen_fy = deque(maxlen=self.max_pts)
        
        self.start_time = None

    def get_dh_matrix(self, theta, d, a, alpha):
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([[ct, -st*ca, st*sa, a*ct], [st, ct*ca, -ct*sa, a*st], [0, sa, ca, d], [0, 0, 0, 1.0]])

    def forward_kinematics_wrist(self, q):
        T = np.eye(4)
        for i in range(3):
            T = T @ self.get_dh_matrix(q[i], self.d[i], self.a[i], self.alpha[i])
        return T[0:3, 3]

    def joint_cb(self, msg):
        try:
            q = [0.0] * 6
            for idx, name in enumerate(self.joint_names):
                q[idx] = msg.position[msg.name.index(name)]
            self.current_q = np.array(q)
            if self.home_pos is None:
                self.home_pos = self.forward_kinematics_wrist(self.current_q[0:3])
        except (ValueError, IndexError):
            pass

    def target_cb(self, msg):
        self.target_delta = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def robot_force_cb(self, msg):
        self.robot_fz = msg.wrench.force.z

    def pen_force_cb(self, msg):
        self.pen_fy = msg.wrench.force.y

    def collect_data(self):
        if self.current_q is None or self.home_pos is None:
            return

        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9

        current_pos = self.forward_kinematics_wrist(self.current_q[0:3])
        current_delta = current_pos - self.home_pos
        error_mag_mm = np.linalg.norm(self.target_delta - current_delta) * 1000.0

        t = (self.get_clock().now().nanoseconds / 1e9) - self.start_time

        # Voeg live data toe aan de lijsten
        self.t_data.append(t)
        self.t_target_x.append(self.target_delta[0]); self.t_target_y.append(self.target_delta[1]); self.t_target_z.append(self.target_delta[2])
        self.t_actual_x.append(current_delta[0]); self.t_actual_y.append(current_delta[1]); self.t_actual_z.append(current_delta[2])
        self.t_error.append(error_mag_mm)
        self.t_rob_fz.append(self.robot_fz)
        self.t_pen_fy.append(self.pen_fy)


# --- MATPLOTLIB GUI & ANIMATIE ---
def main(args=None):
    rclpy.init(args=args)
    node = LivePlotterNode()

    # Draai ROS 2 op de achtergrond
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # Maak de grafiek eenmalig op
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(10, 9))
    fig.canvas.manager.set_window_title('UR3e Live Teleop Dashboard')

    # Teken lege lijnen (dit is véél sneller dan de grafiek steeds wissen)
    line_tx, = ax1.plot([], [], 'r--', label='Target X')
    line_ty, = ax1.plot([], [], 'g--', label='Target Y')
    line_tz, = ax1.plot([], [], 'b--', label='Target Z')
    line_ax, = ax1.plot([], [], 'r-', label='Actual X')
    line_ay, = ax1.plot([], [], 'g-', label='Actual Y')
    line_az, = ax1.plot([], [], 'b-', label='Actual Z')
    ax1.set_title('Cartesian Tracking (Live)')
    ax1.set_ylabel('Positie (m)')
    ax1.grid(True)
    ax1.legend(loc='upper right', ncol=3)

    line_err, = ax2.plot([], [], 'k-', label='Tracking Error (mm)')
    err_text = ax2.text(0.02, 0.85, '', transform=ax2.transAxes, fontsize=11, color='orange', weight='bold')
    ax2.set_ylabel('Fout (mm)')
    ax2.grid(True)
    ax2.legend(loc='upper right')

    line_fz, = ax3.plot([], [], 'b-', label='Sensor Fz (Robot)')
    line_fy, = ax3.plot([], [], 'm-', label='Touch Fy (Haptic)')
    ax3.set_ylabel('Kracht (N)')
    ax3.set_xlabel('Tijd (s)')
    ax3.grid(True)
    ax3.legend(loc='upper right')

    def update_plot(frame):
        node.collect_data()
        if len(node.t_data) == 0:
            return
        
        t = list(node.t_data)

        # Update alleen de data van de lijnen
        line_tx.set_data(t, node.t_target_x); line_ty.set_data(t, node.t_target_y); line_tz.set_data(t, node.t_target_z)
        line_ax.set_data(t, node.t_actual_x); line_ay.set_data(t, node.t_actual_y); line_az.set_data(t, node.t_actual_z)
        line_err.set_data(t, node.t_error)
        line_fz.set_data(t, node.t_rob_fz); line_fy.set_data(t, node.t_pen_fy)

        # Toon de actuele foutwaarde in tekst
        huidige_fout = node.t_error[-1]
        err_text.set_text(f'Huidige fout: {huidige_fout:.1f} mm')

        # X-as mee laten schuiven als een hartslagmonitor
        x_max = t[-1]
        x_min = max(0, x_max - 10.0)
        ax1.set_xlim(x_min, max(10.0, x_max))

        # Y-assen dynamisch schalen op basis van de lijnen
        ax1.relim(); ax1.autoscale_view(scalex=False, scaley=True)
        ax2.relim(); ax2.autoscale_view(scalex=False, scaley=True)
        ax3.relim(); ax3.autoscale_view(scalex=False, scaley=True)

    # Ververs interval op 150ms voor boterzachte performance zonder overbelasting
    ani = FuncAnimation(fig, update_plot, interval=150, blit=False)
    plt.tight_layout()
    plt.show()

    # Netjes afsluiten
    rclpy.shutdown()
    ros_thread.join()

if __name__ == '__main__':
    main()