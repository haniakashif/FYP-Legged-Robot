#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math, time
import numpy as np
from cheetah_ros2.linear_mpc_configs import LinearMpcConfig

class EffortController(Node):
    def __init__(self):
        super().__init__('effort_controller')
        
        self.modified_contacts = np.array([1.0, 1.0, 1.0, 1.0])
        self.stance_torques = np.zeros(12)
        self.swing_torques = np.zeros(12)
        self.dt_control = LinearMpcConfig.dt_control
        
        self.sub_modified = self.create_subscription(Float64MultiArray, '/fsm_state', self.fsm_state_cb, 1)
        self.sub_stance = self.create_subscription(Float64MultiArray, '/stance_torques', self.stance_cb, 1)
        self.sub_swing = self.create_subscription(Float64MultiArray, '/swing_torques', self.swing_cb, 1)

        # for that controller, Gazebo reads the .yaml file and opens up a listener topic at /<controller_name>/commands
        self.pub_torques = self.create_publisher(Float64MultiArray, '/forward_effort_controller/commands', 1)
        
        self.start_time = time.time()
        
        # Publish at 100Hz for a smooth sine wave (testing for controller)
        self.timer = self.create_timer(self.dt_control, self.control_loop)
        self.get_logger().info('Effort Controller Active: Translating modified_contacts to joint torques.')


    def fsm_state_cb(self, msg): 
        # self.get_logger().info(f'FSM State Callback: Received FSM state data. {msg}')
        self.modified_contacts = np.array(msg.data)
    def stance_cb(self, msg): 
        # self.get_logger().info(f'Stance Torques Callback: Received stance torques data. {msg}')
        self.stance_torques = np.array(msg.data)      
    def swing_cb(self, msg): 
        # self.get_logger().info(f'Swing Torques Callback: Received swing torques data. {msg}')
        self.swing_torques = np.array(msg.data)
        
        
    def control_loop(self):
        # self.get_logger().info('Control Loop: Computing swing leg torques.')
        final_torques = np.zeros(12)
        
        for i in range(4):
            idx = slice(i*3, i*3+3)
            if self.modified_contacts[i] == 1.0:
                final_torques[idx] = self.stance_torques[idx]
            else:
                final_torques[idx] = self.swing_torques[idx]
                
        msg_torques = Float64MultiArray()
        msg_torques.data = final_torques.tolist()
        self.pub_torques.publish(msg_torques)
        
        
        # NOTE: this is a dummy function to move legs
        # msg_torques = Float64MultiArray()
        # elapsed_time = time.time() - self.start_time
        
        # # Generate an oscillating wiggle for legs that are supposed to be swinging
        # swing_torque = 0.6 * math.sin(2.0 * math.pi * 2.0 * elapsed_time)
        
        # # Initialize an array for all 12 joints (4 legs * 3 joints each: Hip, Knee, Foot)
        # torques = [0.0] * 12
        
        # for leg_idx in range(4):
        #     # Calculate the starting index for this specific leg's joints (0, 3, 6, 9)
        #     joint_offset = leg_idx * 3
        #     joint_offset_hip = joint_offset + 0
        #     joint_offset_knee = joint_offset + 1
        #     joint_offset_foot = joint_offset + 2 
            
        #     # Check the FSM state for this leg
        #     if self.modified_contacts[leg_idx] == 0.0:
        #         # STATE: SWING. Apply the wiggle torque to the Knee joint.
        #         torques[joint_offset_foot] = swing_torque
                
        #     else:
        #         # STATE: STANCE. Lock the leg (0.0 torque).
        #         torques[joint_offset_foot] = 0.8
                
        # msg_torques.data = torques
        # self.pub_torques.publish(msg_torques)


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