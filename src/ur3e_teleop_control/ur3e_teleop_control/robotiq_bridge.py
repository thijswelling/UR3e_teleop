#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import socket
import time

class RobotiqBridge(Node):
    def __init__(self):
        super().__init__('robotiq_bridge')
        self.declare_parameter('robot_ip', '192.168.1.102')
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.port = 63352
        self.sock = None

        self.connect_gripper()

        self.sub = self.create_subscription(
            Bool,
            '/gripper/cmd',
            self.cmd_callback,
            10
        )
        self.get_logger().info(f"Robotiq Bridge gereed op {self.robot_ip}:{self.port} luisterend naar /gripper/cmd")

    def connect_gripper(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect((self.robot_ip, self.port))
            
            # Initialiseer en activeer de Hand-E
            self.sock.sendall(b"SET ACT 1\n")
            time.sleep(0.1)
            self.sock.sendall(b"SET GTO 1\n")
            self.sock.sendall(b"SET SPE 255\n")
            self.sock.sendall(b"SET FOR 15\n")
            self.get_logger().info("Hand-E succesvol geactiveerd en gekoppeld!")
        except Exception as e:
            self.get_logger().error(f"Fout bij verbinden met Robotiq gripper: {e}")

    def cmd_callback(self, msg: Bool):
        if not self.sock:
            return
        try:
            if msg.data:
                # Sluiten (pos 255)
                self.sock.sendall(b"SET POS 255\n")
                self.get_logger().info("Gripper sluit...")
            else:
                # Openen (pos 0)
                self.sock.sendall(b"SET POS 0\n")
                self.get_logger().info("Gripper opent...")
        except Exception as e:
            self.get_logger().error(f"Fout bij versturen grippercommando: {e}")

    def destroy_node(self):
        if self.sock:
            self.sock.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RobotiqBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
