import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from ros_gz_interfaces.msg import Contacts  # NEW: Import Gazebo Contact messages
import csv
import time
import os

class FlightRecorder(Node):
    def __init__(self):
        super().__init__('flight_recorder')

        self.history = []
        self.start_time = time.time()
        
        self.create_subscription(Float64MultiArray, '/rl/observations', self.obs_cb, 1)
        self.create_subscription(Float64MultiArray, '/rl/actions', self.act_cb, 1)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 1)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 1)
        
        # --- NEW: Subscribe to the Contact Sensor ---
        self.create_subscription(Contacts, '/environment_collisions', self.contact_cb, 10)
        self.is_colliding = 0.0 # Binary flag for the RL reward

        self.latest_obs = None
        self.latest_act = None
        self.latest_cmd = None
        self.latest_joints = None

        self.create_timer(0.02, self.logging_loop)
        self.get_logger().info("Recorder Armed. Monitoring Collisions.")

    # --- NEW: Collision Filter Logic ---
    def contact_cb(self, msg):
        current_collision_state = False
        
        for contact in msg.contact:
            col1 = contact.collision1.name
            col2 = contact.collision2.name
            
            # If the robot's leg hits the ground_plane, ignore it completely!
            if "ground_plane" in col1 or "ground_plane" in col2:
                continue
                
            # If the robot hits a wall or ceiling module ("cave" or "cap"), flag it!
            if "cave" in col1 or "cave" in col2 or "cap" in col1 or "cap" in col2:
                current_collision_state = True
                
                # Print a warning to the terminal (throttled to 1 message per second so it doesn't spam)
                self.get_logger().warn(f"Collision detected: {col1.split('::')[-1]} hit {col2.split('::')[0]}", throttle_duration_sec=1.0)
                
        self.is_colliding = 1.0 if current_collision_state else 0.0

    def obs_cb(self, msg): self.latest_obs = msg.data
    def act_cb(self, msg): self.latest_act = msg.data
    def cmd_cb(self, msg): self.latest_cmd = [msg.linear.x, msg.linear.y, msg.angular.z]
    def joint_cb(self, msg): self.latest_joints = msg.position

    def logging_loop(self):
        if self.latest_obs is None:
            return

        row = {'time': time.time() - self.start_time}
        
        # --- NEW: Log the collision status to the CSV ---
        row['wall_collision'] = self.is_colliding
        
        for i, val in enumerate(self.latest_obs):
            row[f'obs_{i}'] = val
            
        if self.latest_act:
            for i, val in enumerate(self.latest_act):
                row[f'act_{i}'] = val
                
        if self.latest_cmd:
            row['cmd_x'] = self.latest_cmd[0]
            row['cmd_y'] = self.latest_cmd[1]
            row['cmd_yaw'] = self.latest_cmd[2]

        self.history.append(row)

    def save_to_disk(self):
        if not self.history:
            return

        save_dir = "../logs"
        os.makedirs(save_dir, exist_ok=True)
        filepath = os.path.join(save_dir, f"log_{int(time.time())}.csv")
        
        fieldnames = []
        for row in self.history:
            for key in row.keys():
                if key not in fieldnames:
                    fieldnames.append(key)
                    
        if 'time' in fieldnames:
            fieldnames.remove('time')
            fieldnames.insert(0, 'time')

        with open(filepath, mode='w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.history)
            
        self.get_logger().info(f"Saved {len(self.history)} frames to {filepath}")

def main(args=None):
    rclpy.init(args=args)
    node = FlightRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_to_disk()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()