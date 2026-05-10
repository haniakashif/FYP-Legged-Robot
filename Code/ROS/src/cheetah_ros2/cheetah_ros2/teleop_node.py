#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

# --- SETTINGS ---
MAX_LIN_VEL = 1.0  # m/s
MAX_ANG_VEL = 1.0  # rad/s
MIN_HEIGHT = 0.04  # m
MAX_HEIGHT = 0.11  # m

MSG = """
---------------------------
Reading from the keyboard
---------------------------
Controls:
   q    w    e       r (Raise Height)
   a         d       f (Lower Height)
   z    s    c

W to increase linear velocity
S to decrease linear velocity
A to increase angular velocity (left)
D to decrease angular velocity (right)

CTRL-C to quit
"""

moveBindings = {
    'w': (0, 1, 0, 0),  # forward
    's': (0, -1, 0, 0),  # backward
    'q': (0, 1, 0, 1),  # turn forward right
    'e': (0, 1, 0, -1),  # turn forward left
    'z': (0, -1, 0, -1),  # turn backward right
    'c': (0, -1, 0, 1),  # turn backward left
    'a': (-1, 0, 0, 0),  # left strafe
    'd': (1, 0, 0, 0),  # right
}

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        # Publish teleop commands for the gait controllers
        self.pub_teleop = self.create_publisher(Twist, '/teleop', 1)
        
        self.speed = MAX_LIN_VEL / 2
        self.turn = MAX_ANG_VEL / 2
        self.target_height = 0.075
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        # Save terminal settings to restore them on exit
        self.settings = termios.tcgetattr(sys.stdin)
        
        print(MSG)

        self.create_timer(0.1, self.timer_callback)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def timer_callback(self):
        key = self.get_key()

        updated = False

        if key:
            if key in moveBindings.keys():
                self.x = moveBindings[key][0]
                self.y = moveBindings[key][1]
                self.th = moveBindings[key][3]
                updated = True
            elif key == 'W':
                self.speed = min(self.speed * 1.1, MAX_LIN_VEL)
                self.get_logger().info(f"Linear speed: {self.speed:.2f}")
            elif key == 'S':
                self.speed = max(self.speed * 0.9, 0.0)
                self.get_logger().info(f"Linear speed: {self.speed:.2f}")
            elif key == 'A':
                self.turn = min(self.turn * 1.1, MAX_ANG_VEL)
                self.get_logger().info(f"Angular speed: {self.turn:.2f}")
            elif key == 'D':
                self.turn = max(self.turn * 0.9, 0.0)
                self.get_logger().info(f"Angular speed: {self.turn:.2f}")
            elif key == 'r':
                self.target_height = min(self.target_height + 0.01, MAX_HEIGHT)
                self.get_logger().info(f"Target Height: {self.target_height:.2f} m")
            elif key == 'f':
                self.target_height = max(self.target_height - 0.01, MIN_HEIGHT)
                self.get_logger().info(f"Target Height: {self.target_height:.2f} m")
            elif key == '\x03':
                raise KeyboardInterrupt

        if not key in moveBindings.keys() and key not in ('W', 'S', 'A', 'D', 'r', 'f'):
            self.x = 0.0
            self.y = 0.0
            self.th = 0.0

        if updated:
            sys.stdout.write(f"\rVx: {self.x * self.speed:.1f} m/s | Vy: {self.y * self.speed:.1f} m/s | Wz: {self.th * self.turn:.1f} rad/s    ")
            sys.stdout.flush()

        twist = Twist()
        twist.linear.x = float(self.x * self.speed)
        twist.linear.y = float(self.y * self.speed)
        twist.linear.z = float(self.target_height)
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(self.th * self.turn)

        self.pub_teleop.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        # Restore terminal settings gracefully
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()