import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
import csv
import time
import os
import numpy as np

class FlightRecorder(Node):
    def __init__(self):
        super().__init__('flight_recorder')

        # --- DATA STORAGE ---
        # We use a list of dictionaries for fast appending
        self.history = []
        self.start_time = time.time()
        
        # --- SUBSCRIBERS ---
        # We subscribe to EVERYTHING relevant
        self.create_subscription(Float64MultiArray, '/rl/observations', self.obs_cb, 1)
        self.create_subscription(Float64MultiArray, '/rl/actions', self.act_cb, 1)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 1)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 1)
        
        # Buffers for latest data (to synchronize loosely)
        self.latest_obs = None
        self.latest_act = None
        self.latest_cmd = None
        self.latest_joints = None

        # Timer: 50Hz Logging (Matches Policy)
        self.create_timer(0.02, self.logging_loop)
        
        self.get_logger().info("Recorder Armed. Will save on Ctrl+C.")

    def obs_cb(self, msg): self.latest_obs = msg.data
    def act_cb(self, msg): self.latest_act = msg.data
    def cmd_cb(self, msg): self.latest_cmd = [msg.linear.x, msg.linear.y, msg.angular.z]
    def joint_cb(self, msg): self.latest_joints = msg.position

    def logging_loop(self):
        # Only log if we have received at least one observation
        if self.latest_obs is None:
            return

        timestamp = time.time() - self.start_time
        
        # Create a single row
        row = {'time': timestamp}
        
        # Add Observations (Obs_0, Obs_1...)
        for i, val in enumerate(self.latest_obs):
            row[f'obs_{i}'] = val
            
        # Add Actions (Act_0... Act_11)
        if self.latest_act:
            for i, val in enumerate(self.latest_act):
                row[f'act_{i}'] = val
                
        # Add Commands (Cmd_X, Cmd_Y, Cmd_Yaw)
        if self.latest_cmd:
            row['cmd_x'] = self.latest_cmd[0]
            row['cmd_y'] = self.latest_cmd[1]
            row['cmd_yaw'] = self.latest_cmd[2]

        self.history.append(row)

    def save_to_disk(self):
        if not self.history:
            self.get_logger().warn("No data recorded!")
            return

        # Create directory
        save_dir = "/workspaces/FYP-Legged-Robot/Code/logs"
        os.makedirs(save_dir, exist_ok=True)
        
        # Filename with timestamp
        filename = f"log_{int(time.time())}.csv"
        filepath = os.path.join(save_dir, filename)
        
        # Dynamically extract all unique columns (in case early rows missed some like cmd)
        fieldnames = []
        for row in self.history:
            for key in row.keys():
                if key not in fieldnames:
                    fieldnames.append(key)
                    
        # Guarantee 'time' is the very first column
        if 'time' in fieldnames:
            fieldnames.remove('time')
            fieldnames.insert(0, 'time')

        # Write to CSV
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
        # HERE is where we save
        node.save_to_disk()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()