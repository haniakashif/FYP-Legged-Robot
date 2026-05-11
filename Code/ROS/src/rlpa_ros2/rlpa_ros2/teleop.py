import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64  # NEW: For sending the lookahead parameter
import sys
import termios
import tty
import select

# --- SETTINGS ---
MAX_LIN_VEL = 1.0  # m/s
MAX_ANG_VEL = 1.0  # rad/s

msg = """
---------------------------
Reading from the keyboard
---------------------------
Controls:
   q    w    e
   a         d
   z    s    c

W/S to increase/decrease linear velocity
A/D to increase/decrease angular velocity
T/G to increase/decrease Lookahead Distance (Debugging)

CTRL-C to quit
"""

moveBindings = {
    'w' : (0, 1, 0, 0), # forward
    's' : (0, -1, 0, 0), # backward
    'q' : (0, 1, 0, 1), # turn forward right
    'e' : (0, 1, 0, -1), # turn forward left
    'z' : (0, -1, 0, -1), # turn backward right
    'c' : (0, -1, 0, 1), # turn backward left
    'a' : (-1, 0, 0, 0), # left strafe
    'd' : (1, 0, 0, 0), # right
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')
        self.pub_teleop = self.create_publisher(Twist, '/teleop', 1) 
        
        # NEW: Publisher for the lookahead max
        self.pub_lookahead = self.create_publisher(Float64, '/perception/lookahead_max', 1)
        self.lookahead_max = 0.30 # Starting default

        self.speed = MAX_LIN_VEL/2
        self.turn = MAX_ANG_VEL/2
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.settings = termios.tcgetattr(sys.stdin)
        print(msg)
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        key = getKey(self.settings)
        
        if key in moveBindings.keys():
            self.x = moveBindings[key][0]
            self.y = moveBindings[key][1]
            self.th = moveBindings[key][3]
        elif key == "W":
            self.speed = min(self.speed*1.1, MAX_LIN_VEL)
            self.get_logger().info(f"Linear speed: {self.speed:.2f}")
        elif key == "S":
            self.speed = max(self.speed*0.9, 0.0)
            self.get_logger().info(f"Linear speed: {self.speed:.2f}")
        elif key == "A":
            self.turn = min(self.turn*1.1, MAX_ANG_VEL)
            self.get_logger().info(f"Angular speed: {self.turn:.2f}")
        elif key == "D":
            self.turn = max(self.turn*0.9, 0.0)
            self.get_logger().info(f"Angular speed: {self.turn:.2f}")
        elif key == "T":
            self.lookahead_max += 0.05
            self.get_logger().info(f"Lookahead Max increased to: {self.lookahead_max:.2f}m")
            lh_msg = Float64()
            lh_msg.data = float(self.lookahead_max)
            self.pub_lookahead.publish(lh_msg)
        elif key == "G":
            # Don't let it go below the min bound!
            self.lookahead_max = max(0.06, self.lookahead_max - 0.05)
            self.get_logger().info(f"Lookahead Max decreased to: {self.lookahead_max:.2f}m")
            lh_msg = Float64()
            lh_msg.data = float(self.lookahead_max)
            self.pub_lookahead.publish(lh_msg)
            
        elif key == '\x03':
            self.destroy_node()
            rclpy.shutdown()
        else:
            self.x = 0.0
            self.y = 0.0
            self.th = 0.0

        twist = Twist()
        twist.linear.x = float(self.x * self.speed)
        twist.linear.y = float(self.y * self.speed)
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(self.th * self.turn)
        
        self.pub_teleop.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = Teleop()
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