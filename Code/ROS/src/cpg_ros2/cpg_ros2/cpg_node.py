import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from .cpg import CPGController

class cpgNode(Node):
    def __init__(self):
        super().__init__('cpg_node')

        self.target_freq = 50  # Hz
        self.dt = 1.0 / self.target_freq
        self.cpg = CPGController()
        
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, 
            '/joint_group_position_controller/commands', 
            10
        )
        
        self.get_logger().info(f"Initializing CPG with omega={self.cpg.omega} Hz")
        self.get_logger().info(f"Parameters: gamma={self.cpg.gamma:.4f}, duty_cycle={self.cpg.duty_cycle:.4f}")
        
        self.c_a0 = np.array([self.cpg.mu_r0] * self.cpg.n_legs)
        self.c_o0 = np.array([self.cpg.mu_o0] * self.cpg.n_legs)
        self.c_a1 = np.array([self.cpg.mu_r1] * self.cpg.n_legs)
        self.c_o1 = np.array([self.cpg.mu_o1] * self.cpg.n_legs)
        self.c_a2_1 = np.array([self.cpg.mu_r2_1] * self.cpg.n_legs)
        self.c_a2_2 = np.array([self.cpg.mu_r2_2] * self.cpg.n_legs)
        self.c_o2 = np.array([self.cpg.mu_o2] * self.cpg.n_legs)
        
        self.t_a0 = np.array([self.cpg.mu_r0] * self.cpg.n_legs)
        self.t_o0 = np.array([self.cpg.mu_o0] * self.cpg.n_legs)
        self.t_a1 = np.array([self.cpg.mu_r1] * self.cpg.n_legs)
        self.t_o1 = np.array([self.cpg.mu_o1] * self.cpg.n_legs)
        self.t_a2_1 = np.array([self.cpg.mu_r2_1] * self.cpg.n_legs)
        self.t_a2_2 = np.array([self.cpg.mu_r2_2] * self.cpg.n_legs)
        self.t_o2 = np.array([self.cpg.mu_o2] * self.cpg.n_legs)
        
        self.c_phi_0 = self.cpg.target_offsets.copy()

        self.get_logger().info("CPG initialized. Starting control loop.")
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def timer_callback(self):
        self.c_a0 = self.cpg.update_state_variables(self.c_a0, self.t_a0, self.dt)
        self.c_o0 = self.cpg.update_state_variables(self.c_o0, self.t_o0, self.dt)
        self.c_a1 = self.cpg.update_state_variables(self.c_a1, self.t_a1, self.dt)
        self.c_o1 = self.cpg.update_state_variables(self.c_o1, self.t_o1, self.dt)
        self.c_a2_1 = self.cpg.update_state_variables(self.c_a2_1, self.t_a2_1, self.dt)
        self.c_a2_2 = self.cpg.update_state_variables(self.c_a2_2, self.t_a2_2, self.dt)
        self.c_o2 = self.cpg.update_state_variables(self.c_o2, self.t_o2, self.dt)

        self.c_phi_0 = self.cpg.update_global_phases(self.c_phi_0, self.dt)
        
        phi_1, phi_2 = self.cpg.compute_intra_leg_phases(self.c_phi_0)

        phi_0_w = self.cpg.apply_duty_cycle_filter(self.c_phi_0)
        phi_1_w = self.cpg.apply_duty_cycle_filter(phi_1)
        phi_2_w = self.cpg.apply_duty_cycle_filter(phi_2)
        
        phi_2_2pi = np.mod(phi_2_w, 2 * np.pi)
        c_a2 = np.where(phi_2_2pi < np.pi, self.c_a2_1, self.c_a2_2) 
        
        phi_2_spline = self.cpg.apply_spline_filter(phi_2_w)

        theta_0 = self.cpg.compute_target_angles(self.c_a0, self.c_o0, phi_0_w, False)
        theta_1 = self.cpg.compute_target_angles(self.c_a1, self.c_o1, phi_1_w, False)
        theta_2 = self.cpg.compute_target_angles(c_a2, self.c_o2, phi_2_spline, True)

        raw_angles = np.zeros(12)
        # Hips (Indices 0-3)
        raw_angles[0] = -theta_0[0]  # bl_hip
        raw_angles[1] =  theta_0[1]  # br_hip
        raw_angles[2] = -theta_0[2]  # fl_hip
        raw_angles[3] =  theta_0[3]  # fr_hip
        
        # Knees (Indices 4-7)
        raw_angles[4] = -theta_1[0]  # bl_knee
        raw_angles[5] =  theta_1[1]  # br_knee
        raw_angles[6] = -theta_1[2]  # fl_knee
        raw_angles[7] =  theta_1[3]  # fr_knee
        
        # Feet (Indices 8-11)
        raw_angles[8]  = -theta_2[0] # bl_foot
        raw_angles[9]  =  theta_2[1] # br_foot
        raw_angles[10] = -theta_2[2] # fl_foot
        raw_angles[11] =  theta_2[3] # fr_foot
        
        clamped_angles = self.cpg.clamp_to_joint_limits(raw_angles)

        msg = Float64MultiArray()
        msg.data = [float(angle) for angle in clamped_angles]
        self.cmd_pub.publish(msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = cpgNode()
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