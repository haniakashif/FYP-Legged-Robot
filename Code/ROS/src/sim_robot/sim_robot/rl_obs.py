import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist

def R_from_q(q): 
    x, y, z, w = q
    s = 1/(x*x + y*y + z*z + w*w)
    R = [[0 for _ in range(3)] for _ in range(3)]
    R[0][0] = 1 - 2*s*(y*y + z*z)
    R[0][1] = 2*s*(x*y - z*w)
    R[0][2] = 2*s*(x*z + y*w)
    R[1][0] = 2*s*(x*y + z*w)
    R[1][1] = 1 - 2*s*(x*x + z*z)
    R[1][2] = 2*s*(y*z - x*w)
    R[2][0] = 2*s*(x*z - y*w)
    R[2][1] = 2*s*(y*z + x*w)
    R[2][2] = 1 - 2*s*(x*x + y*y)
    return R

class RLObs(Node):
    def __init__(self):
        super().__init__('rl_obs')

        self.target_freq = 50 # freq of the policy
        self.dt = 1.0/self.target_freq

        self.joints = ["bl_hip", "br_hip", "fl_hip", "fr_hip",
                        "bl_knee", "br_knee", "fl_knee", "fr_knee",
                        "bl_foot", "br_foot", "fl_foot", "fr_foot"] 

        # observation variables
        self.joint_states = {} # for storing latest joint readings
        for joint in self.joints:
            self.joint_states[joint] = None 
        self.ang_vel = None # NOTE: TBD
        self.projected_gravity = None # NOTE: TBD
        self.last_action = [0.0 for _ in range(12)] # NOTE: TBD
        self.command = None # NOTE: TBD

        # subscribers
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 1)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 1)
        self.create_subscription(Float64MultiArray, '/rl/actions', self.actions_cb, 1) # NOTE: revisit after rl/actions defined
        self.create_subscription(Twist, '/teleop', self.teleop_cb, 1) # NOTE: revisit when teleop node defined

        # publisher
        self.obs_pub = self.create_publisher(Float64MultiArray, "/rl/observations", 33) # NOTE: verify size of this

        self.timer = self.create_timer(self.dt, self.timer_callback)

    def joint_state_cb(self, msg): 
        
        for i, full_name in enumerate(msg.name):
            parts = full_name.split('_')
            base_name = full_name.replace('_joint', '')
            
            if len(parts) < 2:
                print("Incorrect joint name format:", full_name)
                continue 

            self.joint_states[base_name] = msg.position[i]
        
    def imu_cb(self, msg):
        self.ang_vel = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        
        orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

        Rq = R_from_q(orientation)

        # multiplication with R_t of [0, 0, -1]
        self.projected_gravity = [-1*Rq[2][0], -1*Rq[2][1], -1*Rq[2][2]]

    def actions_cb(self, msg):
        self.last_action = msg.data

    def teleop_cb(self, msg):
        self.command = [msg.linear.x, msg.linear.y, msg.angular.z]
        
        # for testing
        # self.command = [0.3, 0.3, 0.0]

    def timer_callback(self):
        
        if any(v is None for v in [self.ang_vel, self.projected_gravity, self.command]):
            return

        if any(v is None for v in self.joint_states.values()):
            return

        self.get_logger().info("Publishing observation...")
    
        obs_list = []
        # observation format [ang_vel, projected_gravity, command, joint_pos, last_action]

        scaled_ang_vel = [x * 0.25 for x in self.ang_vel] # scale down angular velocity for better learning
        obs_list.extend(scaled_ang_vel)

        # obs_list.extend(self.ang_vel)

        obs_list.extend(self.projected_gravity)

        scaled_command = [x * 2.0 for x in self.command] # scale up command for better learning
        obs_list.extend(scaled_command)

        # obs_list.extend(self.command)

        default_pos = {
            "hip": 0.0,
            "knee": 0.0,
            "foot": 0.0 
        }

        for joint in self.joints:
            joint_type = joint.split('_')[1] # extract joint type (hip/knee/foot) from joint name like "bl_hip"
            val = self.joint_states[joint]
            offset = val - default_pos[joint_type] # extract joint type (hip/knee/foot) from joint name like "bl_hip"
            obs_list.append(offset)

        obs_list.extend(self.last_action)
        obs_list = [float(x) for x in obs_list]

        msg = Float64MultiArray()
        msg.data = obs_list
        self.obs_pub.publish(msg) 

def main(args=None):
    rclpy.init(args=args)
    node = RLObs()
    
    try:
        # Blocks here until Ctrl+C
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()