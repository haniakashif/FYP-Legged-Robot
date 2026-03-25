#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class FSMNode(Node):
    def __init__(self):
        super().__init__('fsm_node')
        
        # self.declare_parameter('use_sim_time', True)
        
        self.physical_contacts = np.zeros(4)
        self.nominal_schedule = np.zeros(4)
        self.swing_phases = np.zeros(4)
        
        self.pub_fsm_state = self.create_publisher(Float64MultiArray, '/fsm_state', 1)
        
        self.sub_contacts = self.create_subscription(Float64MultiArray, '/estimated_contacts', self.contact_cb, 1)
        self.sub_nominal = self.create_subscription(Float64MultiArray, '/nominal_schedule', self.nominal_cb, 1)
        self.sub_swing_phase = self.create_subscription(Float64MultiArray, '/swing_phases', self.swing_phase_cb, 1)
        
        self.get_logger().info("Contact-Conditioned FSM Active: Running Shuo Yang logic.")
        
        self.timer = self.create_timer(0.001, self.fsm_loop) 

    def contact_cb(self, msg):
        # self.get_logger().info(f'Contact Callback: Received contact data. {msg}')
        self.physical_contacts = np.array(msg.data)

    def nominal_cb(self, msg):
        # self.get_logger().info(f'Nominal Schedule Callback: Received nominal schedule data. {msg}')
        self.nominal_schedule = np.array(msg.data)
        
    def swing_phase_cb(self, msg):
        # self.get_logger().info(f'Swing Phase Callback: Received swing phase data. {msg}')
        self.swing_phases = np.array(msg.data)

    def fsm_loop(self):
        # self.get_logger().info('FSM Loop: Evaluating FSM logic.')
        fsm_output = np.zeros(4)
        
        for i in range(4):
            plan_contact = bool(self.nominal_schedule[i] == 1.0)
            physical_contact = bool(self.physical_contacts[i] == 1.0)
            swing_phase = self.swing_phases[i] 
            
            # Early Contact Detection (shuoyang repo)
            # Only valid if we are NOT scheduled for contact, AND we are in the second half of swing
            early_contact = False
            # if no planned swing in gait table, and other conditions, so early contact
            if not plan_contact and (swing_phase > 0.5) and physical_contact:
                # NOTE: before setting true, we should add a counter to ensure that its triggered when reached a certain count
                early_contact = True

            # Late Contact Detection
            # Valid if we ARE scheduled for contact, but the sensor reads air
            late_contact = False
            if plan_contact and not physical_contact:
                late_contact = True 

            # Final State Resolution
            if plan_contact or early_contact:
                if late_contact:
                    fsm_output[i] = 0.0 # Force swing to extend leg into the hole
                else:
                    fsm_output[i] = 1.0 # Standard Stance or Early Rock Strike
            else:
                fsm_output[i] = 0.0 # Standard Swing
                    
        msg_out = Float64MultiArray()
        msg_out.data = fsm_output.tolist()
        self.pub_fsm_state.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()
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