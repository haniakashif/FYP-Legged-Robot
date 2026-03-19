#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time
import numpy as np

class EffortController(Node):
    def __init__(self):
        super().__init__('effort_controller')
        
        self.sub_modified = self.create_subscription(Float64MultiArray, '/modified_contacts', self.modified_cb, 10)
        
        
        # we are nesting this topic due to the specific structure of ros2_control library. for that, I have configured JointGroupEffortController as forward_efort_controller, that basically makes a folder of the controller running (we might have multiple controllers so the library has a standrad structure)
        
        # for that controller, Gazebo reads the .yaml file and opens up a listener topic at /<controller_name>/commands
        self.pub_torques = self.create_publisher(Float64MultiArray, '/forward_effort_controller/commands', 10)
        
        self.start_time = time.time()
        # Default all legs to stance (1.0) until the FSM connects
        self.modified_contacts = np.array([1.0, 1.0, 1.0, 1.0])
        
        self.cmd_xvel = 0.5       
        self.cmd_yvel = 0.0       
        self.cmd_yaw_rate = 0.0
        
        # Publish at 100Hz for a smooth sine wave (testing for controller)
        self.timer = self.create_timer(0.01, self.control_loop)
        self.get_logger().info('Effort Controller Active: Translating modified_contacts to joint torques.')


    def modified_cb(self, msg):
        self.modified_contacts = np.array(msg.data)
        
        
    def control_loop(self):
        
        # NOTE: this is a dummy function to move legs
        msg_torques = Float64MultiArray()
        elapsed_time = time.time() - self.start_time
        
        # Generate an oscillating wiggle for legs that are supposed to be swinging
        swing_torque = 0.6 * math.sin(2.0 * math.pi * 2.0 * elapsed_time)
        
        # Initialize an array for all 12 joints (4 legs * 3 joints each: Hip, Knee, Foot)
        torques = [0.0] * 12
        
        for leg_idx in range(4):
            # Calculate the starting index for this specific leg's joints (0, 3, 6, 9)
            joint_offset = leg_idx * 3 
            
            # Check the FSM state for this leg
            if self.modified_contacts[leg_idx] == 0.0:
                # STATE: SWING. Apply the wiggle torque to the Knee joint.
                torques[joint_offset + 1] = swing_torque
                
            else:
                # STATE: STANCE. Lock the leg (0.0 torque).
                torques[joint_offset + 1] = 0.0 
                
        msg_torques.data = torques
        self.pub_torques.publish(msg_torques)


def main(args=None):
    rclpy.init(args=args)
    node = EffortController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()