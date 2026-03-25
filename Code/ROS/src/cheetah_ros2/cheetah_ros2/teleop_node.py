#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

# Instructions to display in the terminal
MSG = """
Control Your THex Quadruped!
---------------------------
Moving around:
        UP
   LEFT DOWN RIGHT

Turning:
   'q' : Turn Left (CCW)
   'e' : Turn Right (CW)

'ctrl+c' : Quit

Current Speeds:
"""

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        # Publish to the standard ROS velocity topic
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Safe maximum speeds for a quadruped crawl/trot
        self.max_lin_vel = 0.5  # m/s
        self.max_ang_vel = 1.0  # rad/s
        
        # Step size for each key press
        self.lin_step = 0.1
        self.ang_step = 0.2
        
        # Current commanded velocities
        self.cmd_xvel = 0.5
        self.cmd_yvel = 0.5
        self.cmd_yaw_rate = 0.0
        
        # Save terminal settings to restore them on exit
        self.settings = termios.tcgetattr(sys.stdin)
        
        print(MSG)
        
        # Run the input loop at 20Hz
        self.timer = self.create_timer(0.05, self.keyboard_loop)

    def get_key(self):
        """Captures a single keypress from the terminal without requiring 'Enter'."""
        tty.setraw(sys.stdin.fileno())
        # Non-blocking read
        rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
        if rlist:
            key = sys.stdin.read(1)
            # Handle arrow keys (which send 3 characters starting with '\x1b')
            if key == '\x1b':
                key += sys.stdin.read(2)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def keyboard_loop(self):
        key = self.get_key()
        
        updated = False
        
        if key:
            if key == '\x1b[A':    # Up Arrow
                self.cmd_xvel += self.lin_step
                updated = True
            elif key == '\x1b[B':  # Down Arrow
                self.cmd_xvel -= self.lin_step
                updated = True
            elif key == '\x1b[C':  # Right Arrow
                self.cmd_yvel -= self.lin_step
                updated = True
            elif key == '\x1b[D':  # Left Arrow
                self.cmd_yvel += self.lin_step
                updated = True
            elif key == 'q' or key == 'Q':
                self.cmd_yaw_rate += self.ang_step
                updated = True
            elif key == 'e' or key == 'E':
                self.cmd_yaw_rate -= self.ang_step
                updated = True
            elif key == ' ' or key == 's' or key == 'S':
                self.cmd_xvel = 0.0
                self.cmd_yvel = 0.0
                self.cmd_yaw_rate = 0.0
                updated = True
            elif key == '\x03':    # Ctrl+C
                raise KeyboardInterrupt
                
            # Clamp velocities to safe limits
            self.cmd_xvel = max(min(self.cmd_xvel, self.max_lin_vel), -self.max_lin_vel)
            self.cmd_yvel = max(min(self.cmd_yvel, self.max_lin_vel), -self.max_lin_vel)
            self.cmd_yaw_rate = max(min(self.cmd_yaw_rate, self.max_ang_vel), -self.max_ang_vel)

        if updated:
            # Print current state so you know what you are commanding
            sys.stdout.write(f"\rVx: {self.cmd_xvel:.1f} m/s | Vy: {self.cmd_yvel:.1f} m/s | Wz: {self.cmd_yaw_rate:.1f} rad/s    ")
            sys.stdout.flush()

        # Always publish the Twist message
        msg = Twist()
        msg.linear.x = float(self.cmd_xvel)
        msg.linear.y = float(self.cmd_yvel)
        msg.angular.z = float(self.cmd_yaw_rate)
        self.pub_cmd.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Restore terminal settings gracefully
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()