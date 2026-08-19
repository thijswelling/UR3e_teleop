import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

HELP_MSG = """
--------------------------------------------------
UR3e Keyboard Teleop Controller
--------------------------------------------------
Translatie (Lineair):
   W / S : Vooruit / Achteruit (+X / -X)
   A / D : Links / Rechts       (+Y / -Y)
   R / F : Omhoog / Omlaag      (+Z / -Z)

Rotatie (Angulair):
   U / O : Roll  (+Rx / -Rx)
   I / K : Pitch (+Ry / -Ry)
   J / L : Yaw   (+Rz / -Rz)

Spatiebalk : Stop alle beweging (0 snelheid)
Q          : Afsluiten
--------------------------------------------------
"""

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('ur3e_keyboard_teleop')
        self.pub = self.create_publisher(TwistStamped, '/target_twist', 10)

        self.lin_vel = 0.05   # m/s
        self.ang_vel = 0.2    # rad/s

        self.target_twist = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.timer = self.create_timer(0.02, self.publish_twist)  # 50 Hz loop

    def publish_twist(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.twist.linear.x = self.target_twist[0]
        msg.twist.linear.y = self.target_twist[1]
        msg.twist.linear.z = self.target_twist[2]

        msg.twist.angular.x = self.target_twist[3]
        msg.twist.angular.y = self.target_twist[4]
        msg.twist.angular.z = self.target_twist[5]

        self.pub.publish(msg)

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = KeyboardTeleop()

    print(HELP_MSG)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = get_key(settings).lower()

            if key == 'w':
                node.target_twist = [node.lin_vel, 0.0, 0.0, 0.0, 0.0, 0.0]
            elif key == 's':
                node.target_twist = [-node.lin_vel, 0.0, 0.0, 0.0, 0.0, 0.0]
            elif key == 'a':
                node.target_twist = [0.0, node.lin_vel, 0.0, 0.0, 0.0, 0.0]
            elif key == 'd':
                node.target_twist = [0.0, -node.lin_vel, 0.0, 0.0, 0.0, 0.0]
            elif key == 'r':
                node.target_twist = [0.0, 0.0, node.lin_vel, 0.0, 0.0, 0.0]
            elif key == 'f':
                node.target_twist = [0.0, 0.0, -node.lin_vel, 0.0, 0.0, 0.0]
            elif key == 'u':
                node.target_twist = [0.0, 0.0, 0.0, node.ang_vel, 0.0, 0.0]
            elif key == 'o':
                node.target_twist = [0.0, 0.0, 0.0, -node.ang_vel, 0.0, 0.0]
            elif key == 'i':
                node.target_twist = [0.0, 0.0, 0.0, 0.0, node.ang_vel, 0.0]
            elif key == 'k':
                node.target_twist = [0.0, 0.0, 0.0, 0.0, -node.ang_vel, 0.0]
            elif key == 'j':
                node.target_twist = [0.0, 0.0, 0.0, 0.0, 0.0, node.ang_vel]
            elif key == 'l':
                node.target_twist = [0.0, 0.0, 0.0, 0.0, 0.0, -node.ang_vel]
            elif key == ' ':
                node.target_twist = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            elif key == 'q':
                node.target_twist = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                node.publish_twist()
                break

    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
