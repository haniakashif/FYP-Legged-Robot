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
        self.joint_velocities = {}
        for joint in self.joints:
            self.joint_states[joint] = None 
            self.joint_velocities[joint] = None
        self.ang_vel = None 
        self.projected_gravity = None 
        self.last_action = [0.0 for _ in range(12)] 
        self.command = None 

        # subscribers
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 1)
        self.create_subscription(Imu, '/imu_sensor_broadcaster/imu', self.imu_cb, 1)
        self.create_subscription(Float64MultiArray, '/rl/actions', self.actions_cb, 1) 
        self.create_subscription(Twist, '/teleop', self.teleop_cb, 1) 

        # publisher
        self.obs_pub = self.create_publisher(Float64MultiArray, "/rl/observations", 33) # the 33 here is just an artifact from when we mistook the buffer size for the array size - still, its fine as is

        self.timer = self.create_timer(self.dt, self.timer_callback)

    def joint_state_cb(self, msg): 
        
        alpha = 0.5

        for i, full_name in enumerate(msg.name):
            parts = full_name.split('_')
            base_name = full_name.replace('_joint', '')
            
            if len(parts) < 2:
                print("Incorrect joint name format:", full_name)
                continue 
            
            new_pos = msg.position[i]
            old_pos = self.joint_states[base_name]

            if old_pos is not None:
                raw_vel = (new_pos - old_pos) / self.dt
                if self.joint_velocities[base_name] is not None:
                    self.joint_velocities[base_name] = alpha * raw_vel + (1 - alpha) * self.joint_velocities[base_name]
                else:
                    self.joint_velocities[base_name] = raw_vel
            else:
                self.joint_velocities[base_name] = 0.0
            
            self.joint_states[base_name] = new_pos
        
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

    def timer_callback(self):
        
        if any(v is None for v in [self.ang_vel, self.projected_gravity, self.command]):
            return

        if any(v is None for v in self.joint_states.values()):
            return

        # self.get_logger().info("Publishing observation...")
    
        obs_list = []
        # observation format [ang_vel, projected_gravity, command, joint_pos, joint_vel, last_action]

        scaled_ang_vel = [x * 0.25 for x in self.ang_vel] # scale down angular velocity
        obs_list.extend(scaled_ang_vel)

        # obs_list.extend(self.ang_vel)

        obs_list.extend(self.projected_gravity)

        scaled_command = [x * 2 for x in self.command]
        obs_list.extend(scaled_command)

        # obs_list.extend(self.command)

        default_pos = {
            "hip": 0.0,
            "knee": 0.0,
            "foot": 0.0 
        }

        for joint in self.joints:
            joint_type = joint.split('_')[1] 
            val = self.joint_states[joint]
            offset = val - default_pos[joint_type] 
            obs_list.append(offset)

        for joint in self.joints:
            joint_type = joint.split('_')[1] 
            val = self.joint_velocities[joint]
            scaled_val = val * 0.1
            obs_list.append(scaled_val)

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