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
        self.prev_target_pos = None

        self.has_received_target = False
        self.is_engaged = False # HOUDT BIJ OF WE GEKOPPELD ZIJN

        self.dt = 0.02
        
        # MAXIMALE GAINS
        self.Kp_pos_max = 6.5
        self.Kp_wrist_max = 10.0
        
        # HUIDIGE GAINS (Beginnen op 0 voor de soft-start)
        self.Kp_pos = 0.0
        self.Kp_wrist = 0.0
        
        self.K_posture = 0.0
        self.max_joint_vel = 2.2

        self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)
        self.create_subscription(PoseStamped, "/target_pose", self.pose_cb, 10)
        self.create_subscription(Bool, "/reset_home", self.reset_cb, 10)
        self.create_subscription(Bool, "/engage_orientation", self.engage_cb, 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, "/forward_velocity_controller/commands", 10)

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("UR3e Pose Tracking Controller actief (Met Soft-Start)!")

    def joint_state_cb(self, msg):
        try:
            q = [0.0] * 6
            for idx, name in enumerate(self.joint_names):
                i = msg.name.index(name)
                q[idx] = msg.position[i]
            self.current_q = np.array(q)

            # Blijf het nulpunt updaten totdat de gebruiker op [e] drukt
            if not self.is_engaged:
                self.home_q = self.current_q.copy()
                self.home_pos = self.forward_kinematics_wrist(self.home_q[0:3])
                self.engaged_wrist_q = self.home_q[3:6].copy()
        except (ValueError, IndexError):
            pass

    def pose_cb(self, msg):
        raw_pos = np.clip(np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]), -0.25, 0.25)

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
            self.is_engaged = True
            
            # Vang EXACT de stand op het moment dat [e] wordt ingedrukt
            self.home_q = self.current_q.copy()
            self.home_pos = self.forward_kinematics_wrist(self.home_q[0:3])
            self.engaged_wrist_q = self.current_q[3:6].copy()
            
            # Zet gains op 0 voor de soft-start ramp
            self.Kp_pos = 0.0
            self.Kp_wrist = 0.0
            
            self.get_logger().info("Nulpunt vastgezet op HUIDIGE stand! (Soft-start actief)")

    def reset_cb(self, msg):
        if msg.data and self.is_engaged:
            # We verbreken de engage NIET, maar we dwingen de doelen naar 0
            self.target_delta_pos = np.zeros(3)
            self.target_delta_rot = np.zeros(3)
            
            # Door de gefilterde delta niet meteen op 0 te zetten, maar de target wel, 
            # zal de 0.85/0.15 filter in de loop() zorgen dat de robot zachtjes naar Home terugzweeft.
            self.get_logger().info("Return to Home geactiveerd: Zachtjes terug naar nulpunt.")

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

        # ZACHTE OPBOUW VAN DE GAINS (over 1.0 seconde)
        if self.is_engaged:
            ramp_speed_pos = self.Kp_pos_max / (1.0 / self.dt)
            ramp_speed_wrist = self.Kp_wrist_max / (1.0 / self.dt)
            
            if self.Kp_pos < self.Kp_pos_max:
                self.Kp_pos += ramp_speed_pos
            if self.Kp_wrist < self.Kp_wrist_max:
                self.Kp_wrist += ramp_speed_wrist
        else:
            self.Kp_pos = 0.0
            self.Kp_wrist = 0.0

        if not self.has_received_target or not self.is_engaged:
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [0.0] * 6
            self.cmd_pub.publish(cmd_msg)
            return

        #tril dempingen
        # Oude code was 85% oud / 15% nieuw. We maken hem nu trager tegen hand-trillingen (92% oud / 8% nieuw)
        self.filtered_delta_pos = 0.92 * self.filtered_delta_pos + 0.08 * self.target_delta_pos
        # De rotatie had eerst helemáál geen filter! We voegen nu een lichte demping toe voor de pols:
        self.filtered_delta_rot = 0.80 * self.filtered_delta_rot + 0.20 * self.target_delta_rot

        curr_pos = self.forward_kinematics_wrist(self.current_q[0:3])
        target_pos = self.home_pos + self.filtered_delta_pos

        shoulder_pos = np.array([0.0, 0.0, self.d[0]])
        vec_from_shoulder = target_pos - shoulder_pos
        dist_from_shoulder = np.linalg.norm(vec_from_shoulder)

        # --- 1. SLIMME GRENSBEWAKING (Voorkomt pols-kantelen) ---
        max_reach = 0.415
        if dist_from_shoulder > max_reach:
            scale = max_reach / dist_from_shoulder
            target_pos = shoulder_pos + vec_from_shoulder * scale
            
            # FIX: Als de positie botst tegen de grens, schalen we de rotatiewens mee!
            # Dit voelt als een virtueel elastiek en voorkomt dat de pols extreem scheef trekt.
            self.filtered_delta_rot = self.filtered_delta_rot * (scale ** 2)

        pos_error = target_pos - curr_pos
        
        # --- VIRTUEL VEER STURING (Geen feedforward) ---
        # Feedforward veroorzaakt haptische resonantie (trillen). 
        # Een pure P-controller werkt als een stabiele veer.
        self.prev_target_pos = target_pos.copy()
        v_lin = self.Kp_pos * pos_error

        # --- 2. DYNAMISCHE DEMPING (Voorkomt singulariteit-explosies) ---
        J_arm = self.compute_arm_jacobian(self.current_q[0:3])
        det_J = abs(np.linalg.det(J_arm))
        
        # SR-DLS: Verhoog de demping exponentieel als we de grens (gestrekte arm) naderen
        base_damping = 0.015
        if det_J < 0.02:
            damping = base_damping + 0.15 * ((0.02 - det_J) / 0.02)
        else:
            damping = base_damping

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

        # --- 3. HARD ANTI-DOORKNIK SLOT (Veiligheid voor de elleboog) ---
        # De elleboog is q[2]. 0.0 radialen is volledig kaarsrecht.
        # We voorkomen dat de motor het gewricht door de 0-lijn probeert te drukken.
        elbow_angle = self.current_q[2]
        elbow_vel = q_dot[2]
        buffer = 0.12  # Ongeveer 7 graden veiligheidsmarge
        
        if elbow_angle > 0.0 and elbow_angle < buffer and elbow_vel < 0.0:
            q_dot[2] = 0.0  # Blokkeer verdere strekking
        elif elbow_angle < 0.0 and elbow_angle > -buffer and elbow_vel > 0.0:
            q_dot[2] = 0.0  # Blokkeer verdere strekking

        # Maximale snelheidslimieten
        max_arm = np.max(np.abs(q_dot[0:3]))
        if max_arm > 2.8:
            q_dot[0:3] = q_dot[0:3] * (2.8 / max_arm)

        max_wrist = np.max(np.abs(q_dot[3:6]))
        if max_wrist > 3.0:
            q_dot[3:6] = q_dot[3:6] * (3.0 / max_wrist)

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