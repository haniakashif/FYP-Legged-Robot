#!/usr/bin/env python3
import rclpy
import numpy as np
import math

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from cheetah_ros2.linear_mpc_configs import LinearMpcConfig
from cheetah_ros2.robot_configs import THexConfig

class SwingController(Node):

    def __init__(self):
        super().__init__('swing_controller')
        
        self.robot_state = np.zeros(40)
        self.foot_positions = np.zeros(12)
        self.swing_phases = np.zeros(4)
        self.p0 = np.zeros((4, 3)) # To store the liftoff position of each foot
        self.foot_jacobians = np.zeros((4, 3, 12))
        self.swing_time = 0.001
        
        self.cmd_xvel = LinearMpcConfig.cmd_xvel   # m/s
        self.cmd_yvel = LinearMpcConfig.cmd_yvel   # m/s
        self.cmd_yaw_turn_rate = LinearMpcConfig.cmd_yaw_turn_rate # rad/s
        self.target_z = THexConfig.base_height_des # desired ride height 
        self.g = LinearMpcConfig.gravity
        self.kp_Cartesian = THexConfig.kp_Cartesian
        self.kd_Cartesian = THexConfig.kd_Cartesian
        
        self.first_swing = np.array([True, True, True, True])
        
        # Hip offsets [FL, FR, BL, BR]
        self.hip_offsets_local = np.array([
            [-0.059512,  0.078854,  self.target_z],  # (fl_hip_joint)
            [ 0.059512,  0.078854,  self.target_z],  # (fr_hip_joint)
            [-0.053162, -0.127110,  self.target_z],  # (bl_hip_joint)
            [ 0.053162, -0.127110,  self.target_z]   # (br_hip_joint)
        ])

        self.sprawl_offsets_local = np.array([
            [-0.08,  0.00,  0.0],  # FL: Forward and Left
            [0.08,  0.00,  0.0],  # FR: Forward and Right
            [-0.08, -0.00,  0.0],  # BL: Backward and Left
            [0.08, -0.00,  0.0]   # BR: Backward and Right
        ])
        
        self.pub_torques = self.create_publisher(Float64MultiArray, '/swing_torques', 1)
        
        self.sub_state = self.create_subscription(Float64MultiArray, '/estimated_robot_state', self.state_cb, 1)
        self.sub_stance_time = self.create_subscription(Float64, '/gait/stance_time', self.stance_time_cb, 1)
        self.sub_swing_time = self.create_subscription(Float64, '/gait/swing_time', self.swing_time_cb, 1)
        self.sub_foot_pos = self.create_subscription(Float64MultiArray, '/foot_positions', self.foot_pos_cb, 1)
        self.sub_swing_phase = self.create_subscription(Float64MultiArray, '/swing_phases', self.swing_phase_cb, 1)
        self.sub_jacobians = self.create_subscription(Float64MultiArray, '/foot_jacobians', self.jacobian_cb, 1)
        
        self.dt_control = LinearMpcConfig.dt_control
        self.timer = self.create_timer(self.dt_control, self.control_loop)
        

    def state_cb(self, msg): 
        # self.get_logger().info(f'State Callback: Received robot state data. {msg}')
        self.robot_state = np.array(msg.data)
    
    def stance_time_cb(self, msg): 
        # self.get_logger().info(f'Stance Time Callback: Received stance time data. {msg}')
        self.stance_time = msg.data
    
    def swing_time_cb(self, msg): 
        # self.get_logger().info(f'Swing Time Callback: Received swing time data. {msg}')
        self.swing_time = msg.data
    
    def foot_pos_cb(self, msg): 
        # self.get_logger().info(f'Foot Position Callback: Received foot position data. {msg}')
        self.foot_positions = np.array(msg.data)
    
    def jacobian_cb(self, msg): 
        # self.get_logger().info(f'Jacobian Callback: Received Jacobian data. {msg}')
        self.foot_jacobians = np.array(msg.data).reshape(4, 3, 12)
        
    def swing_phase_cb(self, msg):
        # self.get_logger().info(f'Swing Phase Callback: Received swing phase data. {msg}')
        self.swing_phases = np.array(msg.data)
        # This logic captures the exact liftoff position the moment the phase > 0
        for i in range(4):
            if self.swing_phases[i] > 0.0:
                if self.first_swing[i]:
                    self.first_swing[i] = False
                    # store x y z pos of each leg
                    self.p0[i] = self.foot_positions[i*3 : i*3+3]
            else:
                # If phase is 0, the foot is in stance. Reset the trigger for the next step.
                self.first_swing[i] = True
    
    def compute_foot_placement(self, leg_idx: int, stance_time: float) -> np.ndarray:
        """
        Calculates the target global stepping location for a swing leg using 
        Equation 6 from the MIT Cheetah 3 architecture (Raibert + Capture Point).
        """
        # [p(3), v(3), q(12), qdot(12), quat(4), omega(3), accel(3)]
        p_com = self.robot_state[0:3]
        v_com = self.robot_state[3:6]
        w, x, y, z = self.robot_state[30:34]
        # our robot frame is defined to move in +y direction
        v_des = np.array([self.cmd_yvel, self.cmd_xvel, 0.0])
        
        # --- Calculate Global Hip Position (p_h_i) ---
        # Convert quaternion to Rotation Matrix (R_mat)
        R_mat = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),       1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w),       2*(y*z + x*w),       1 - 2*(x**2 + y**2)]
        ])
        
        hip_local = self.hip_offsets_local[leg_idx] 
        sprawl_local = self.sprawl_offsets_local[leg_idx]
        
        p_resting_local = hip_local + sprawl_local
        p_h_global = p_com + (R_mat @ p_resting_local)
        # self.get_logger().info(f'leg_idx={leg_idx}, robot_com={p_com}, hip_computed_global={p_h_global}')
        
        # --- Raibert Heuristic Term ---
        raibert_term = (stance_time / 2.0) * v_des[0:2]
        z_0 = self.target_z
        # if z_0 < 0.01:
        #     z_0 = 0.01 
            
        time_constant = math.sqrt(z_0 / self.g)
        capture_term = time_constant * (v_com[0:2] - v_des[0:2])

        # --- Combine and Clamp (Bounding Box) ---
        p_step_rel = raibert_term + capture_term
        # self.get_logger().info(f'raibert_term={raibert_term}, capture_term={capture_term}, p_step_rel(before clamp)={p_step_rel}')
        
        # NOTE: i need to set the bounding box experimentally
        p_rel_max = 0.06
        p_step_rel[0] = np.clip(p_step_rel[0], -p_rel_max, p_rel_max)
        p_step_rel[1] = np.clip(p_step_rel[1], -p_rel_max, p_rel_max)
        # self.get_logger().info(f'Foot Placement Debug: p_step_rel(after clamp)={p_step_rel}')
        
        p_step_global = np.zeros(3)
        p_step_global[0] = p_h_global[0] + p_step_rel[0]
        p_step_global[1] = p_h_global[1] + p_step_rel[1]
        p_step_global[2] = -0.001  # Penetrate the floor slightly to trigger the contact sensor to flip the FSM to stance mode.
        
        return p_step_global
    
    
    def _bezier_curve(self, s: float, y0: float, yf: float) -> tuple:
        """
        Computes the 1D position, phase-velocity, and phase-acceleration 
        of a simplified cubic Bezier curve.
        
        s: Normalized phase (0.0 to 1.0)
        y0: Starting coordinate (liftoff)
        yf: Ending coordinate (touchdown)
        """
        p = y0 + (yf - y0) * (3 * s**2 - 2 * s**3)
        v = (yf - y0) * (6 * s - 6 * s**2)
        a = (yf - y0) * (6 - 12 * s)
        
        return p, v, a
    
    
    def swing_traj_gen(self, leg_idx: int, phase: float, stance_time: float, swing_time: float) -> tuple:
        """
        Generates the 3D Cartesian reference trajectory for a swinging leg using 
        piecewise Bezier curves.
        
        Returns:
            p_des (3,): Desired 3D position
            v_des (3,): Desired 3D velocity
            a_des (3,): Desired 3D acceleration
        """
        # --- Liftoff and Touchdown Coordinates ---
        p0 = self.p0[leg_idx]
        pf = self.compute_foot_placement(leg_idx, stance_time)
        
        h_clearance = THexConfig.swing_height
        
        p_des = np.zeros(3)
        v_des = np.zeros(3)
        a_des = np.zeros(3)
        
        for i in range(2): # 0 is X, 1 is Y
            p, v_phase, a_phase = self._bezier_curve(phase, p0[i], pf[i])
            p_des[i] = p
            
            # Convert phase-velocity to real time (meters/second)
            v_des[i] = v_phase / swing_time
            a_des[i] = a_phase / (swing_time**2)
            
        # --- Z Trajectory (Piecewise) ---
        if phase < 0.5:
            # First half: Moving UP from p0 to (p0 + clearance)
            # this maps first half between 0 and 1, and the next block does this for second half because we split trajectory into 2 pieces
            s_z = phase * 2.0
            p, v_phase, a_phase = self._bezier_curve(s_z, p0[2], p0[2] + h_clearance)
            p_des[2] = p
            
            # Chain Rule for 2x phase speed
            v_des[2] = v_phase * 2.0 / swing_time
            a_des[2] = a_phase * 4.0 / (swing_time**2)
            
        else:
            # Second half: Moving DOWN from (p0 + clearance) to pf
            s_z = (phase * 2.0) - 1.0
            p, v_phase, a_phase = self._bezier_curve(s_z, p0[2] + h_clearance, pf[2])
            p_des[2] = p
            
            # Chain Rule for 2x phase speed
            v_des[2] = v_phase * 2.0 / swing_time
            a_des[2] = a_phase * 4.0 / (swing_time**2)
            
        return p_des, v_des, a_des
    
    
    def control_loop(self):
        # self.get_logger().info('Control Loop: Computing swing leg torques.')
        tau_swing = np.zeros(12)        
        p_com = self.robot_state[0:3]
        v_com = self.robot_state[3:6]
        qdot = self.robot_state[18:30]
        omega = self.robot_state[34:37]
        swing_time = self.swing_time 
        
        for i in range(4):
            # Only calculate trajectory and torques if the leg is in the air
            if self.swing_phases[i] > 0.0:
                
                p_des, v_des, a_des = self.swing_traj_gen(i, self.swing_phases[i], self.stance_time, swing_time)
                
                p_act = self.foot_positions[i*3 : i*3+3]
                J_i = self.foot_jacobians[i]
                
                # absolute foot velocity: v_base + (omega x r) + J*qdot
                r_foot = p_act - p_com
                v_act = v_com + np.cross(omega, r_foot) + (J_i @ qdot)
                
                # F = Kp(p_des - p_act) + Kd(v_des - v_act)
                F_pd = self.kp_Cartesian @ (p_des - p_act) + self.kd_Cartesian @ (v_des - v_act)
                
                # tau = J^T * F
                tau_i = J_i.T @ F_pd
                
                # accumulate the torques for this specific leg
                tau_swing += tau_i
                
        msg_tau = Float64MultiArray()
        msg_tau.data = tau_swing.tolist()
        self.pub_torques.publish(msg_tau)
    
    
    
def main(args=None):
    rclpy.init(args=args)
    node = SwingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()   