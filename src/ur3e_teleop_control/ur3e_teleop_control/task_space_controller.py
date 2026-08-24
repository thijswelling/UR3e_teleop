import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool

class UR3eDecoupled6DOFController(Node):
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

        self.d = np.array([0.15185, 0.0, 0.0, 0.13105, 0.08535, 0.0921])
        self.a = np.array([0.0, -0.24355, -0.2132, 0.0, 0.0, 0.0])
        self.alpha = np.array([np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0])

        # Vaste beginstand
        self.home_q = np.array([0.0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0.0])
        self.current_q = self.home_q.copy()
        self.target_twist = np.zeros(6)

        self.dt = 0.02
        self.max_joint_vel = 1.5

        self.create_subscription(TwistStamped, '/target_twist', self.twist_cb, 10)
        self.create_subscription(Bool, '/reset_home', self.reset_cb, 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("UR3e 6-DOF Velocity Controller met Auto-Home actief!")

    def twist_cb(self, msg):
        self.target_twist = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z
        ])

    def reset_cb(self, msg):
        if msg.data:
            # Zachte overgang terug naar exacte homepositie
            self.current_q += 0.15 * (self.home_q - self.current_q)

    def get_dh_matrix(self, theta, d, a, alpha):
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,   sa,       ca,      d],
            [0,   0,        0,       1.0]
        ])

    def compute_arm_jacobian(self, q):
        T = np.eye(4)
        origins = [T[0:3, 3]]
        z_axes = [T[0:3, 2]]

        for i in range(3):
            Ti = self.get_dh_matrix(q[i], self.d[i], self.a[i], self.alpha[i])
            T = T @ Ti
            origins.append(T[0:3, 3])
            z_axes.append(T[0:3, 2])

        p_wrist = origins[-1]
        J_arm = np.zeros((3, 3))
        for i in range(3):
            J_arm[:, i] = np.cross(z_axes[i], (p_wrist - origins[i]))

        return J_arm

    def control_loop(self):
        v_lin = self.target_twist[0:3]
        w_ang = self.target_twist[3:6]

        # 1. Arm translatie
        J_arm = self.compute_arm_jacobian(self.current_q[0:3])
        damping = 0.02
        A = J_arm @ J_arm.T + (damping ** 2) * np.eye(3)
        q_dot_arm = J_arm.T @ np.linalg.solve(A, v_lin)

        # 2. Polsgewrichten met de geverifieerde assenrichting
        q_dot_wrist = np.array([-w_ang[0], -w_ang[1], w_ang[2]])

        q_dot = np.hstack([q_dot_arm, q_dot_wrist])

        # Begrens snelheden
        max_val = np.max(np.abs(q_dot))
        if max_val > self.max_joint_vel:
            q_dot = q_dot * (self.max_joint_vel / max_val)

        # Update gewrichten
        self.current_q += q_dot * self.dt

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_q.tolist()
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UR3eDecoupled6DOFController()
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
