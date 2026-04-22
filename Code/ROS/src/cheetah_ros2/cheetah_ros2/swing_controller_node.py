#!/usr/bin/env python3
import rclpy
import numpy as np
import math
import pinocchio as pin

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from cheetah_ros2.linear_mpc_configs import LinearMpcConfig
from cheetah_ros2.robot_configs import THexConfig
from cheetah_ros2.kinematics import inv_kin

# for debugging
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class SwingController(Node):

    def __init__(self):
        super().__init__('swing_controller')
        
        self.robot_state = np.zeros(40)
        self.foot_positions = np.zeros(12)
        self.swing_phases = np.zeros(4)
        self.p0 = np.zeros((4, 3)) # To store the liftoff position of each foot
        self.foot_jacobians = np.zeros((4, 3, 12))
        self.stance_time = 0.001
        self.swing_time = 0.001
        self.fsm_state = np.zeros(4)
        
        self.cmd_xvel = LinearMpcConfig.cmd_xvel   # m/s
        self.cmd_yvel = LinearMpcConfig.cmd_yvel   # m/s
        self.cmd_yaw_turn_rate = LinearMpcConfig.cmd_yaw_turn_rate # rad/s
        self.target_z = THexConfig.base_height_des # desired ride height 
        self.g = LinearMpcConfig.gravity
        # self.kp_Cartesian = THexConfig.kp_Cartesian
        # self.kd_Cartesian = THexConfig.kd_Cartesian
        self.Kp_joint = THexConfig.Kp_joint
        self.Kd_joint = THexConfig.Kd_joint

        self.ros_to_ik_map = [3, 0, 2, 1]
        
        # To calculate qdot_des via finite difference
        self.q_des_prev = np.zeros((4, 3))

        urdf_path = "/workspaces/FYP-Legged-Robot/Code/ROS/src/cheetah_ros2/models/THex_Quadruped/model.urdf"
        self.pin_model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
        self.pin_data = self.pin_model.createData()

        self.controller_joint_names = [
            'fl_hip_joint','fl_knee_joint','fl_foot_joint',
            'fr_hip_joint','fr_knee_joint','fr_foot_joint',
            'bl_hip_joint','bl_knee_joint','bl_foot_joint',
            'br_hip_joint','br_knee_joint','br_foot_joint'
        ]

        self.pin_joint_names = []
        for jid, jname in enumerate(self.pin_model.names):
            jmodel = self.pin_model.joints[jid]
            if jmodel.nq == 1 and jname in self.controller_joint_names:
                self.pin_joint_names.append(jname)

        self.ctrl_to_pin = np.array(
            [self.pin_joint_names.index(n) for n in self.controller_joint_names],
            dtype=int
        )
        self.pin_to_ctrl = np.argsort(self.ctrl_to_pin)
        
        self.first_swing = np.array([True, True, True, True])
        
        # Hip offsets [FL, FR, BL, BR]
        self.hip_offsets_local = np.array([
            [-0.059512,  0.078854,  0],  # (fl_hip_joint)
            [ 0.059512,  0.078854,  0],  # (fr_hip_joint)
            [-0.053162, -0.127110,  0],  # (bl_hip_joint)
            [ 0.053162, -0.127110,  0]   # (br_hip_joint)
        ])

        self.hip_yaws_local = np.array([
             2.3666,   # fl_hip_joint
             0.76931,  # fr_hip_joint
            -2.372,    # bl_hip_joint
            -0.77529   # br_hip_joint
        ])

        self.sprawl_offsets_local = np.array([
            [-THexConfig.x_sprawl,  THexConfig.y_sprawl,  0.0],  # FL: Forward and Left
            [THexConfig.x_sprawl,  THexConfig.y_sprawl,  0.0],  # FR: Forward and Right
            [-THexConfig.x_sprawl, -THexConfig.y_sprawl,  0.0],  # BL: Backward and Left
            [THexConfig.x_sprawl, -THexConfig.y_sprawl,  0.0]   # BR: Backward and Right
        ])

        self.max_reach_cm = THexConfig.max_reach_cm
        
        self.pub_torques = self.create_publisher(Float64MultiArray, '/swing_torques', 1)

        # for debugging
        self.pub_markers = self.create_publisher(Marker, '/swing_foot_markers', 1)
        
        self.sub_state = self.create_subscription(Float64MultiArray, '/estimated_robot_state', self.state_cb, 1)
        self.sub_stance_time = self.create_subscription(Float64, '/gait/stance_time', self.stance_time_cb, 1)
        self.sub_swing_time = self.create_subscription(Float64, '/gait/swing_time', self.swing_time_cb, 1)
        self.sub_foot_pos = self.create_subscription(Float64MultiArray, '/foot_positions', self.foot_pos_cb, 1)
        self.sub_swing_phase = self.create_subscription(Float64MultiArray, '/swing_phases', self.swing_phase_cb, 1)
        self.sub_jacobians = self.create_subscription(Float64MultiArray, '/foot_jacobians', self.jacobian_cb, 1)
        self.sub_fsm = self.create_subscription(Float64MultiArray, '/fsm_state', self.fsm_state_cb, 1)
        
        self.dt_control = LinearMpcConfig.dt_control
        self.timer = self.create_timer(self.dt_control, self.control_loop)
    
    # for debugging
    def publish_trajectory_marker(self, leg_idx: int, stance_time: float, num_samples: int = 25):
        marker = Marker()
        # 'odom' is your global fixed frame from the Gazebo bridge
        marker.header.frame_id = 'odom' 
        marker.header.stamp = self.get_clock().now().to_msg()
        
        # Give each leg its own namespace and ID so they don't overwrite each other
        marker.ns = f'foot_target_{leg_idx}'
        marker.id = leg_idx
        
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        
        # Color: Bright Blue (RGBA)
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 0.8  # Slightly transparent

        p0 = self.p0[leg_idx]
        pf = self.compute_foot_placement(leg_idx, stance_time)
        h_clearance = THexConfig.swing_height

        for phase in np.linspace(0.0, 1.0, num_samples):
            point = Point()
            point.x = float(self._bezier_curve(phase, p0[0], pf[0])[0])
            point.y = float(self._bezier_curve(phase, p0[1], pf[1])[0])

            if phase < 0.5:
                s_z = phase * 2.0
                point.z = float(self._bezier_curve(s_z, p0[2], p0[2] + h_clearance)[0])
            else:
                s_z = (phase * 2.0) - 1.0
                point.z = float(self._bezier_curve(s_z, p0[2] + h_clearance, pf[2])[0])

            # for cube debugging, add offset in z
            point.z += 0.0

            marker.points.append(point)
        
        self.pub_markers.publish(marker)

    def clear_trajectory_marker(self, leg_idx: int):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f'foot_target_{leg_idx}'
        marker.id = leg_idx
        marker.action = Marker.DELETE
        self.pub_markers.publish(marker)

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
        
    def fsm_state_cb(self, msg):
        self.fsm_state = np.array(msg.data)


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
        v_des_global = R_mat @ v_des
        
        hip_local = self.hip_offsets_local[leg_idx] 
        sprawl_local = self.sprawl_offsets_local[leg_idx]
        
        p_resting_local = hip_local + sprawl_local
        p_h_global = p_com + (R_mat @ p_resting_local)
        # self.get_logger().info(f'leg_idx={leg_idx}, robot_com={p_com}, hip_computed_global={p_h_global}')
        
        raibert_term = (stance_time / 2.0) * v_des_global[0:2]
        
        z_0 = self.target_z
        time_constant = math.sqrt(abs(z_0) / self.g)
        capture_term = time_constant * (v_com[0:2] - v_des_global[0:2])

        p_step_rel = raibert_term + capture_term
        p_rel_max = 0.06
        p_step_rel[0] = np.clip(p_step_rel[0], -p_rel_max, p_rel_max)
        p_step_rel[1] = np.clip(p_step_rel[1], -p_rel_max, p_rel_max)
        
        p_step_global = np.zeros(3)
        p_step_global[0] = p_h_global[0] + p_step_rel[0]
        p_step_global[1] = p_h_global[1] + p_step_rel[1]

        p_step_global[2] = self.p0[leg_idx][2] - 0.001  # Penetrate the floor slightly to trigger the contact sensor to flip the FSM to stance mode.
        
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

        # for cube debugging, add offset in z
        p_des[2] += 0.0
            
        return p_des, v_des, a_des
    
    
    def control_loop(self):
        # self.get_logger().info('Control Loop: Computing swing leg torques.')
        tau_swing = np.zeros(12)        
        p_com = self.robot_state[0:3]
        v_com = self.robot_state[3:6]
        q_joints = self.robot_state[6:18]
        qdot = self.robot_state[18:30]
        w, x, y, z = self.robot_state[30:34]
        R_mat = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),       1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w),       2*(y*z + x*w),       1 - 2*(x**2 + y**2)]
        ])
        omega = self.robot_state[34:37]
        swing_time = self.swing_time 

        # Pinocchio uses [x, y, z, w] for quaternions
        pin_quat = np.array([x, y, z, w])
        
        # Reorder joints from Controller order to Pinocchio order
        q_pin = q_joints[self.pin_to_ctrl]
        
        # Construct the 19-element configuration vector for the FreeFlyer model
        q_gen = np.concatenate((p_com, pin_quat, q_pin))
        
        # Compute G(q)
        pin.computeGeneralizedGravity(self.pin_model, self.pin_data, q_gen)
        
        # The result is an 18-element vector. Indices 6-17 are the 12 motors.
        g_vector_pin = self.pin_data.g[6:18] 
        
        # Reorder back to your controller's joint order
        g_vector_ctrl = g_vector_pin[self.ctrl_to_pin]
        # ----------------------------------------
        
        for i in range(4):

            # if self.swing_phases[i] > 0.0:
            #     # Operational Space Control
            #     p_des, v_des, a_des = self.swing_traj_gen(i, self.swing_phases[i], self.stance_time, swing_time)

            #     self.publish_trajectory_marker(i, self.stance_time)

            #     p_act = self.foot_positions[i*3 : i*3+3]
            #     J_i = self.foot_jacobians[i]
                
            #     # absolute foot velocity: v_base + (omega x r) + J*qdot
            #     r_foot = p_act - p_com
            #     v_act = v_com + np.cross(omega, r_foot) + (J_i @ qdot)
                
            #     # F = Kp(p_des - p_act) + Kd(v_des - v_act)
            #     F_pd = self.kp_Cartesian @ (p_des - p_act) + self.kd_Cartesian @ (v_des - v_act)
                
            #     idx = slice(i*3, i*3+3)
                
            #     # Extract the 3x3 Jacobian for just this leg
            #     J_leg = J_i[:, idx]
                
            #     # Extract the 3 gravity compensation torques for this leg
            #     g_leg = g_vector_ctrl[idx]
                
            #     # tau (3x1) = J_leg^T (3x3) * F_pd (3x1) + g_leg (3x1)
            #     tau_i = J_leg.T @ F_pd + g_leg
                
            #     # accumulate the torques for this specific leg
            #     tau_swing[idx] = tau_i
            # else:
            #     self.clear_trajectory_marker(i)

            # if self.fsm_state[i] == 0.0:
            if self.swing_phases[i] > 0.0:
                phase = self.swing_phases[i]
                if phase == 0.0:
                    phase = 1.0
            
                # Joint Space Control
                p_des_global, _, _ = self.swing_traj_gen(i, phase, self.stance_time, swing_time)
                self.publish_trajectory_marker(i, self.stance_time)
                
                p_des_base = R_mat.T @ (p_des_global - p_com)
                
                x_base_offset, y_base_offset, z_local = p_des_base - self.hip_offsets_local[i]
                
                gamma = self.hip_yaws_local[i]
                cos_g = math.cos(gamma)
                sin_g = math.sin(gamma)
                
                x_local = x_base_offset * cos_g + y_base_offset * sin_g
                y_local = -x_base_offset * sin_g + y_base_offset * cos_g

                x_local = x_local * 100.0
                y_local = y_local * 100.0
                z_local = z_local * 100.0

                target_dist = math.sqrt(x_local**2 + y_local**2 + z_local**2)
                
                if target_dist > self.max_reach_cm:
                    scale_factor = self.max_reach_cm / target_dist
                    x_local *= scale_factor
                    y_local *= scale_factor
                    z_local *= scale_factor
                
                ik_leg_idx = self.ros_to_ik_map[i]
                try:
                    q1, q2, q3 = inv_kin(x_local, y_local, z_local, ik_leg_idx)
                    q_des = np.array([q1, q2, q3])
                except Exception as e:
                    self.get_logger().error(str(e))
                    q_des = self.q_des_prev[i]

                if self.first_swing[i]:
                    self.q_des_prev[i] = q_des
                    
                qdot_des = (q_des - self.q_des_prev[i]) / self.dt_control
                self.q_des_prev[i] = q_des
                
                idx = slice(i*3, i*3+3)
                q_act = q_joints[idx]
                qdot_act = qdot[idx]
                
                tau_pd = self.Kp_joint @ (q_des - q_act) + self.Kd_joint @ (qdot_des - qdot_act)
                
                g_leg = g_vector_ctrl[idx]
                tau_i = tau_pd + g_leg
                
                tau_swing[idx] = tau_i
            else:
                self.clear_trajectory_marker(i)
                
        msg_tau = Float64MultiArray()
        tau_swing = np.clip(tau_swing, -0.9414, 0.9414) 
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