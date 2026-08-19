import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class UR3eTaskSpaceController(Node):
    def __init__(self):
        super().__init__('ur3e_task_space_controller')

        # Joint volgorde voor de UR3e
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # UR3e DH-parameters (a, d, alpha in meters/radialen)
        self.d = np.array([0.15185, 0.0, 0.0, 0.13105, 0.08535, 0.0921])
        self.a = np.array([0.0, -0.24355, -0.2132, 0.0, 0.0, 0.0])
        self.alpha = np.array([np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0])

        self.current_q = None
        self.target_twist = np.zeros(6)
        self.damping = 0.05  # Damping factor lambda tegen singulariteiten

        # Subscribers
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.create_subscription(TwistStamped, '/target_twist', self.twist_cb, 10)

        # Publisher naar de Joint Trajectory Controller
        self.cmd_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        # Control loop op 50 Hz
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("UR3e Task-Space Controller node gestart!")

    def joint_state_cb(self, msg):
        if all(name in msg.name for name in self.joint_names):
            indices = [msg.name.index(name) for name in self.joint_names]
            self.current_q = np.array([msg.position[i] for i in indices])

    def twist_cb(self, msg):
        self.target_twist = np.array([
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
            msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z
        ])

    def compute_jacobian(self, q):
        T = np.eye(4)
        z = [np.array([0, 0, 1])]
        p = [np.array([0, 0, 0])]

        for i in range(6):
            theta = q[i]
            d = self.d[i]
            a = self.a[i]
            alpha = self.alpha[i]

            Ti = np.array([
                [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                [0,              np.sin(alpha),                np.cos(alpha),               d],
                [0,              0,                            0,                           1]
            ])
            T = T @ Ti
            z.append(T[0:3, 2])
            p.append(T[0:3, 3])

        p_end = p[-1]
        J = np.zeros((6, 6))
        for i in range(6):
            J[0:3, i] = np.cross(z[i], (p_end - p[i]))
            J[3:6, i] = z[i]
        return J

    def control_loop(self):
        if self.current_q is None:
            return

        if np.allclose(self.target_twist, 0, atol=1e-4):
            return

        J = self.compute_jacobian(self.current_q)

        # Damped Least Squares: J_dls = J^T * (J * J^T + lambda^2 * I)^(-1)
        lambda_sq = self.damping ** 2
        J_dls = J.T @ np.linalg.inv(J @ J.T + lambda_sq * np.eye(6))

        q_dot = J_dls @ self.target_twist
        q_next = self.current_q + q_dot * self.dt

        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = q_next.tolist()
        point.velocities = q_dot.tolist()
        point.time_from_start = Duration(sec=0, nanosec=int(self.dt * 1e9))
        traj.points.append(point)

        self.cmd_pub.publish(traj)

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
