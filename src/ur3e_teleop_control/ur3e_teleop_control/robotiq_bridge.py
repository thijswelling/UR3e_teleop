#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import socket
import time

class RobotiqBridge(Node):
    def __init__(self):
        super().__init__('robotiq_bridge')
        
        # Define the IP address and the specific socket port to communicate with the gripper.
        self.declare_parameter('robot_ip', '192.168.1.102')
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.port = 63352
        self.sock = None

        # Establish connection with the hardware gripper
        self.connect_gripper()

        # Listen for open/close boolean commands from ROS 2.
        self.sub = self.create_subscription(
            Bool,
            '/gripper/cmd',
            self.cmd_callback,
            10
        )
        self.get_logger().info(f"Robotiq Bridge ready on {self.robot_ip}:{self.port}")

    def connect_gripper(self):
        # Attempt to create a TCP socket connection directly to the robot controller interface.
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect((self.robot_ip, self.port))
            
            # Send initialization commands to activate and configure the gripper.
            self.sock.sendall(b"SET ACT 1\n") # Activate gripper
            time.sleep(0.1)
            self.sock.sendall(b"SET GTO 1\n") # Go to position mode
            self.sock.sendall(b"SET SPE 255\n") # Set movement speed to maximum (255)
            self.sock.sendall(b"SET FOR 15\n")  # Set grasping force to a safe, low value
            self.get_logger().info("Hand-E successfully activated and connected!")
        except Exception as e:
            self.get_logger().error(f"Error connecting to Robotiq gripper: {e}")

    def cmd_callback(self, msg: Bool):
        if not self.sock:
            return
        
        # Send physical movement commands based on the received message data.
        try:
            if msg.data:
                # Value 255 commands the gripper to fully close.
                self.sock.sendall(b"SET POS 255\n")
                self.get_logger().info("Closing gripper...")
            else:
                # Value 0 commands the gripper to fully open.
                self.sock.sendall(b"SET POS 0\n")
                self.get_logger().info("Opening gripper...")
        except Exception as e:
            self.get_logger().error(f"Error sending gripper command: {e}")

    def destroy_node(self):
        # Safely close the network socket connection when shutting down the node.
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