import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

# --- SETTINGS ---
MAX_LIN_VEL = 0.5  # m/s
MAX_ANG_VEL = 1.0  # rad/s
LIN_STEP = 0.05
ANG_STEP = 0.1

msg = """
---------------------------
Reading from the keyboard!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

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
        
        self.speed = 0.2
        self.turn = 0.5
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.status = 0

        self.settings = termios.tcgetattr(sys.stdin)
        
        print(msg)
        
        # Timer to repeatedly publish the last command (Safety)
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        key = getKey(self.settings)
        
        if key in moveBindings.keys():
            self.x = moveBindings[key][0]
            self.y = moveBindings[key][1]
            self.z = moveBindings[key][2]
            self.th = moveBindings[key][3]
        elif key == '\x03': # Ctrl+C
            self.destroy_node()
            rclpy.shutdown()
        else:
            # If no key, DECELERATE (Deadman switch)
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.th = 0.0

        twist = Twist()
        twist.linear.x = self.x * self.speed
        twist.linear.y = self.y * self.speed
        twist.linear.z = self.z * self.speed
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.th * self.turn
        
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