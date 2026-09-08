import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64MultiArray

class UR3eClosedLoopPoseController(Node):
    def __init__(self):
        super().__init__("ur3e_task_space_controller")

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        self.d = np.array([0.15185, 0.0, 0.0, 0.13105, 0.08535, 0.0921])
        self.a = np.array([0.0, -0.24355, -0.2132, 0.0, 0.0, 0.0])
        self.alpha = np.array([np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0])

        self.current_q = None
        self.home_q = None
        self.home_pos = None
        self.engaged_wrist_q = None

        self.target_delta_pos = np.zeros(3)
        self.target_delta_rot = np.zeros(3)
        self.filtered_delta_pos = np.zeros(3)
        self.filtered_delta_rot = np.zeros(3)

        self.has_received_target = False

        self.dt = 0.02
        self.Kp_pos = 3.0
        self.Kp_wrist = 3.5
        self.K_posture = 1.0
        self.max_joint_vel = 0.55

        self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)
        self.create_subscription(PoseStamped, "/target_pose", self.pose_cb, 10)
        self.create_subscription(Bool, "/reset_home", self.reset_cb, 10)
        self.create_subscription(Bool, "/engage_orientation", self.engage_cb, 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, "/forward_velocity_controller/commands", 10)

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("UR3e Pose Tracking Controller actief met Startup Interlock!")

    def joint_state_cb(self, msg):
        try:
            q = [0.0] * 6
            for idx, name in enumerate(self.joint_names):
                i = msg.name.index(name)
                q[idx] = msg.position[i]
            self.current_q = np.array(q)

            if self.home_q is None:
                self.home_q = self.current_q.copy()
                self.home_pos = self.forward_kinematics_wrist(self.home_q[0:3])
                self.engaged_wrist_q = self.home_q[3:6].copy()
                self.get_logger().info(f"Vaste rustpositie vastgelegd: {np.round(self.home_q, 2)}")
        except (ValueError, IndexError):
            pass

    def pose_cb(self, msg):
        raw_pos = np.clip(np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]), -0.20, 0.20)

        raw_rot = np.clip(np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z
        ]), -1.5, 1.5)

        if not self.has_received_target:
            self.filtered_delta_pos = raw_pos.copy()
            self.filtered_delta_rot = raw_rot.copy()
            self.has_received_target = True

        self.target_delta_pos = raw_pos
        self.target_delta_rot = raw_rot

    def engage_cb(self, msg):
        if msg.data and self.current_q is not None:
            self.engaged_wrist_q = self.current_q[3:6].copy()
            self.get_logger().info("Pols-uitlijning vastgezet op huidige stand!")

    def reset_cb(self, msg):
        if msg.data:
            self.target_delta_pos = np.zeros(3)
            self.target_delta_rot = np.zeros(3)
            self.filtered_delta_pos = np.zeros(3)
            self.filtered_delta_rot = np.zeros(3)
            if self.home_q is not None:
                self.engaged_wrist_q = self.home_q[3:6].copy()
            self.get_logger().info("Doelpositie teruggezet naar Home.")

    def get_dh_matrix(self, theta, d, a, alpha):
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,   sa,       ca,      d],
            [0,   0,        0,       1.0]
        ])

    def forward_kinematics_wrist(self, q):
        T = np.eye(4)
        for i in range(3):
            Ti = self.get_dh_matrix(q[i], self.d[i], self.a[i], self.alpha[i])
            T = T @ Ti
        return T[0:3, 3]

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
        if self.current_q is None or self.home_pos is None:
            return

        if not self.has_received_target:
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [0.0] * 6
            self.cmd_pub.publish(cmd_msg)
            return

        self.filtered_delta_pos = 0.2 * self.target_delta_pos + 0.8 * self.filtered_delta_pos
        self.filtered_delta_rot = 0.2 * self.target_delta_rot + 0.8 * self.filtered_delta_rot

        curr_pos = self.forward_kinematics_wrist(self.current_q[0:3])
        target_pos = self.home_pos + self.filtered_delta_pos
        pos_error = target_pos - curr_pos
        v_lin = self.Kp_pos * pos_error

        J_arm = self.compute_arm_jacobian(self.current_q[0:3])
        damping = 0.04
        A = J_arm @ J_arm.T + (damping ** 2) * np.eye(3)
        J_dls = J_arm.T @ np.linalg.inv(A)

        q_null = self.K_posture * (self.home_q[0:3] - self.current_q[0:3])
        N = np.eye(3) - (J_dls @ J_arm)
        q_dot_arm = (J_dls @ v_lin) + (N @ q_null)

        ref_wrist = self.engaged_wrist_q if self.engaged_wrist_q is not None else self.home_q[3:6]
        target_wrist_q = ref_wrist + np.array([
            -self.filtered_delta_rot[0],
            -self.filtered_delta_rot[1],
             self.filtered_delta_rot[2]
        ])
        wrist_error = target_wrist_q - self.current_q[3:6]
        q_dot_wrist = self.Kp_wrist * wrist_error

        q_dot = np.hstack([q_dot_arm, q_dot_wrist])

        max_val = np.max(np.abs(q_dot))
        if max_val > self.max_joint_vel:
            q_dot = q_dot * (self.max_joint_vel / max_val)

        if np.linalg.norm(pos_error) < 0.002 and np.linalg.norm(wrist_error) < 0.01:
            q_dot = np.zeros(6)

        cmd_msg = Float64MultiArray()
        cmd_msg.data = q_dot.tolist()
        self.cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = UR3eClosedLoopPoseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
