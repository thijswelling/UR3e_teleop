import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped

class UR3eTaskSpaceController(Node):
    def __init__(self):
        super().__init__('ur3e_task_space_controller')

        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # UR3e DH-parameters
        self.d = np.array([0.15185, 0.0, 0.0, 0.13105, 0.08535, 0.0921])
        self.a = np.array([0.0, -0.24355, -0.2132, 0.0, 0.0, 0.0])
        self.alpha = np.array([np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0])

        # Fysieke limieten UR3e
        self.q_min = np.array([-2*np.pi] * 6)
        self.q_max = np.array([ 2*np.pi] * 6)
        self.joint_buffer = np.radians(5.0)  # 5 graden veiligheidsmarge

        # Cartesische werkruimte-grenzen (t.o.v. base_link in meters)
        self.z_min = 0.02   # Voorkom botsen met tafel/grondvlak
        self.r_max = 0.50   # Max bereik UR3e radius

        # Beginpositie
        self.current_q = np.array([0.0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0.0])
        self.target_twist = np.zeros(6)
        self.max_joint_vel = 1.0  # rad/s

        self.create_subscription(TwistStamped, '/target_twist', self.twist_cb, 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("UR3e Controller met Veiligheids- & Workspace-limieten actief!")

    def twist_cb(self, msg):
        self.target_twist = np.array([
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
            msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z
        ])

    def forward_kinematics(self, q):
        T = np.eye(4)
        z = [np.array([0, 0, 1])]
        p = [np.array([0, 0, 0])]

        for i in range(6):
            theta = q[i]
            Ti = np.array([
                [np.cos(theta), -np.sin(theta)*np.cos(self.alpha[i]),  np.sin(theta)*np.sin(self.alpha[i]), self.a[i]*np.cos(theta)],
                [np.sin(theta),  np.cos(theta)*np.cos(self.alpha[i]), -np.cos(theta)*np.sin(self.alpha[i]), self.a[i]*np.sin(theta)],
                [0,              np.sin(self.alpha[i]),                np.cos(self.alpha[i]),               self.d[i]],
                [0,              0,                                    0,                                   1]
            ])
            T = T @ Ti
            z.append(T[0:3, 2])
            p.append(T[0:3, 3])

        p_end = p[-1]
        J = np.zeros((6, 6))
        for i in range(6):
            J[0:3, i] = np.cross(z[i], (p_end - p[i]))
            J[3:6, i] = z[i]
        return p_end, J

    def control_loop(self):
        if not np.allclose(self.target_twist, 0, atol=1e-4):
            p_curr, J = self.forward_kinematics(self.current_q)

            # Workspace Box check (Z-min limit)
            twist_cmd = self.target_twist.copy()
            if p_curr[2] <= self.z_min and twist_cmd[2] < 0:
                twist_cmd[2] = 0.0  # Blokkeer verdere neerwaartse beweging

            # Adaptieve demping gebaseerd op manipuleerbaarheid w = sqrt(det(J*J^T))
            w = np.sqrt(np.maximum(0.0, np.linalg.det(J @ J.T)))
            damping = 0.02 if w > 0.05 else 0.02 + 0.1 * (1.0 - w / 0.05)

            # Damped Least Squares
            A = J @ J.T + (damping ** 2) * np.eye(6)
            q_dot = J.T @ np.linalg.solve(A, twist_cmd)

            # Proportionele snelheidsbegrenzing
            max_val = np.max(np.abs(q_dot))
            if max_val > self.max_joint_vel:
                q_dot = q_dot * (self.max_joint_vel / max_val)

            # Joint limit stop (voorkom doordraaien buiten [-2pi, 2pi])
            for i in range(6):
                if self.current_q[i] <= (self.q_min[i] + self.joint_buffer) and q_dot[i] < 0:
                    q_dot[i] = 0.0
                elif self.current_q[i] >= (self.q_max[i] - self.joint_buffer) and q_dot[i] > 0:
                    q_dot[i] = 0.0

            self.current_q += q_dot * self.dt

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_q.tolist()
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UR3eTaskSpaceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()