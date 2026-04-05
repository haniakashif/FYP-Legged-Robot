import rclpy
import numpy as np
import math
import osqp

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from scipy import sparse
from cheetah_ros2.linear_mpc_configs import LinearMpcConfig
from cheetah_ros2.robot_configs import THexConfig

class StanceController(Node):
    def __init__(self):
        super().__init__('stance_controller')

        self.dt_control = LinearMpcConfig.dt_control
        self.base_inertia = np.array(THexConfig.base_inertia)
        
        self.cmd_xvel = LinearMpcConfig.cmd_xvel # m/s, robot frame
        self.cmd_yvel = LinearMpcConfig.cmd_yvel # m/s, robot frame
        self.cmd_yaw_turn_rate = LinearMpcConfig.cmd_yaw_turn_rate # rad/s
        self.target_z = THexConfig.base_height_des # m, world frame, desired ride height    
        
        self.Kp_balance_COM = THexConfig.Kp_balance_COM
        self.Kd_balance_COM = THexConfig.Kd_balance_COM
        self.Kp_balance_ori = THexConfig.Kp_balance_ori
        self.Kd_balance_ori = THexConfig.Kd_balance_ori
        
        self.mass_robot = THexConfig.mass_robot # kg
        self.fz_max = THexConfig.fz_max # N
        self.fz_min = THexConfig.fz_min # N
        self.mu = LinearMpcConfig.friction_coef

        self.robot_state = np.zeros(40)
        self.foot_positions = np.zeros(12)
        self.stance_phases = np.zeros(4) 
        self.swing_phases = np.zeros(4)
        self.fsm_state = np.zeros(4)
        self.foot_jacobians = np.zeros((4, 3, 12))
        
        self.S_weight = THexConfig.S
        self.alpha_W = THexConfig.alpha
        self.beta_W = THexConfig.beta
        self.f_prev = np.zeros(12)

        self.pub_torques = self.create_publisher(Float64MultiArray, '/stance_torques', 1)
        
        self.sub_state = self.create_subscription(Float64MultiArray, '/estimated_robot_state', self.state_cb, 1)
        self.sub_foot = self.create_subscription(Float64MultiArray, '/foot_positions', self.foot_cb, 1)
        self.sub_stance = self.create_subscription(Float64MultiArray, '/stance_phases', self.stance_cb, 1)
        self.sub_swing = self.create_subscription(Float64MultiArray, '/swing_phases', self.swing_cb, 1)
        self.sub_nom = self.create_subscription(Float64MultiArray, '/fsm_state', self.nom_cb, 1)
        self.sub_jacobians = self.create_subscription(Float64MultiArray, '/foot_jacobians', self.jacobian_cb, 1)
        
        self.timer = self.create_timer(self.dt_control, self.control_loop)

    def state_cb(self, msg): 
        # self.get_logger().info(f'Robot State Callback: Received robot state data. {msg}')
        self.robot_state = np.array(msg.data)
    
    def foot_cb(self, msg): 
        # self.get_logger().info(f'Foot Position Callback: Received foot position data. {msg}')
        self.foot_positions = np.array(msg.data)
    
    def stance_cb(self, msg): 
        # self.get_logger().info(f'Stance Phase Callback: Received stance phase data. {msg}')
        self.stance_phases = np.array(msg.data)
    
    def swing_cb(self, msg): 
        # self.get_logger().info(f'Swing Phase Callback: Received swing phase data. {msg}')
        self.swing_phases = np.array(msg.data)
    
    def nom_cb(self, msg): 
        # self.get_logger().info(f'FSM State Callback: Received FSM state data. {msg}')
        self.fsm_state = np.array(msg.data)
    
    def jacobian_cb(self, msg): 
        # self.get_logger().info(f'Jacobian Callback: Received Jacobian data. {msg}')
        self.foot_jacobians = np.array(msg.data).reshape(4, 3, 12)

    def compute_desired_COM(self) -> np.ndarray:

        weights = np.zeros(4)
        sqrt_2 = math.sqrt(2.0)
        virtual_points = np.zeros((4, 3))
        
        # variances for stance (c) and swing (c_bar) transitions
        # these dictate how "steep" the erf curve is at touchdown and liftoff
        # NOTE: should they be arbitrarily set?
        sigma_c0 = 0.05
        sigma_c1 = 0.05
        sigma_cbar0 = 0.05
        sigma_cbar1 = 0.05
        
        for i in range(4):
            # check if leg is in stance (s_phi = 1.0) or swing (s_phi = 0.0)
            if self.fsm_state[i] == 1.0: 
                phi = self.stance_phases[i]
                weights[i] = 0.5 * (math.erf(phi / (sigma_c0 * sqrt_2)) + 
                                    math.erf((1.0 - phi) / (sigma_c1 * sqrt_2)))
            else: 
                phi = self.swing_phases[i]
                # during swing, weight drops to 0 in the middle, but rises to 0.5 at the edges
                weights[i] = 0.5 * (2.0 + 
                                    math.erf(-phi / (sigma_cbar0 * sqrt_2)) + 
                                    math.erf((phi - 1.0) / (sigma_cbar1 * sqrt_2)))
                    
        # rows: [FL, FR, BL, BR], cols: [x, y, z], in world frame
        p_foot = self.foot_positions.reshape(4, 3)  
        # print(p_foot)
        
        # (current foot position, adjacent clockwise foot, adjacent counter-clockwise foot)
        foot_pos = [
            (p_foot[0], p_foot[1], p_foot[2]),
            (p_foot[1], p_foot[3], p_foot[0]),
            (p_foot[2], p_foot[0], p_foot[3]),
            (p_foot[3], p_foot[2], p_foot[1])
        ]
        
        foot_weights = [
            (weights[0], weights[1], weights[2]),
            (weights[1], weights[3], weights[0]),
            (weights[2], weights[0], weights[3]),
            (weights[3], weights[2], weights[1])
        ]
        
        for i in range(4):
            neg_vp = foot_weights[i][0]*foot_pos[i][0] + (1-foot_weights[i][0])*foot_pos[i][2]
            pos_vp = foot_weights[i][0]*foot_pos[i][0] + (1-foot_weights[i][0])*foot_pos[i][1]
            
            numer = foot_weights[i][0]*foot_pos[i][0] + neg_vp*foot_weights[i][2] + pos_vp*foot_weights[i][1]
            denom = foot_weights[i][0] + foot_weights[i][2] + foot_weights[i][1]
            v_point = numer / denom
            virtual_points[i] = v_point
        
        # print("Virtual Points:\n", virtual_points)
        p_c_bar = np.mean(virtual_points, axis=0)
          
        # NOTE: this won't entirely work for sloped terrain
        p_c_d = np.array([p_c_bar[0], p_c_bar[1], self.target_z])
        
        return p_c_d
    
        
        
    def balance_controller(self, p_c_d: np.ndarray) -> np.ndarray:
        
        # [p_com(3), v_com(3), q(12), qdot(12), quat(4), omega(3), accel(3)]
        p_com = self.robot_state[0:3] # world frame
        v_com = self.robot_state[3:6] # world frame
        w, x, y, z = self.robot_state[30:34]
        omega = self.robot_state[34:37] # world frame
        
        v_des = np.array([self.cmd_yvel, self.cmd_xvel, 0.0]) # robot frame, z velocity 0
        omega_des = np.array([0.0, 0.0, self.cmd_yaw_turn_rate]) # robot frame, roll and pitch velocity 0
        
        Kp_p = np.array(self.Kp_balance_COM) 
        Kd_p = np.array(self.Kd_balance_COM)
        Kp_w = np.array(self.Kp_balance_ori)
        Kd_w = np.array(self.Kd_balance_ori)
        
        I_body_local = self.base_inertia
        S_weight = self.S_weight # weight for dynamics tracking
        alpha_W = self.alpha_W # weight for force magnitude
        beta_W = self.beta_W # weight for command deviation

        # convert current quaternion [w, x, y, z] to rotation matrix
        # https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
        R_mat = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),       1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w),       2*(y*z + x*w),       1 - 2*(x**2 + y**2)]
        ])
        
        # define desired rotation
        # calculate current yaw from quaternion
        # https://discuss.luxonis.com/d/5453-how-to-convert-quaternions-to-pitchrollyaw
        # https://robotics.stackexchange.com/questions/16471/get-yaw-from-quaternion
        current_yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
        R_yaw = np.array([
            [math.cos(current_yaw), -math.sin(current_yaw), 0.0],
            [math.sin(current_yaw),  math.cos(current_yaw), 0.0],
            [0.0,                    0.0,                   1.0]
        ])
        
        # target yaw is current yaw + (commanded rate * timestep)
        yaw_des = current_yaw + (self.cmd_yaw_turn_rate * self.dt_control)
        R_des = np.array([
            [math.cos(yaw_des), -math.sin(yaw_des), 0.0],
            [math.sin(yaw_des),  math.cos(yaw_des), 0.0],
            [0.0,                0.0,               1.0]
        ]) # roll and pitch 0
        
        p_c_d_world = p_c_d # if the pcd is already in the world frame
               
        # rotating the error from world frame to yaw-aligned frame
        error_p_yaw = R_yaw.T @ (p_c_d_world - p_com)
        error_v_yaw = R_yaw.T @ (v_des - v_com)
        error_omega_yaw = R_yaw.T @ (omega_des - omega)
        
        # orientation error log map (R_yaw^T * R_des * R_mat^T * R_yaw)
        # R_d @ R_mat.T is based on the repo, where both are in the world frame, so we're going from world to yaw when given the rotation mat of yaw wrt world
        R_err_yaw = R_yaw.T @ R_des @ R_mat.T @ R_yaw
        
        # NOTE: the approach below is used by official MIT repo, trace-based calculation. The general approach for finding log of any diagonalizable aquare matrix gives us complex numbers and is not optimal for real-time, so we make use of trace.
        # https://en.wikipedia.org/wiki/Rotation_matrix  (section on Conversion from rotation matrix to axis–angle)
        trace = np.trace(R_err_yaw) # trace is the sum of diagonal elements
        tmp = (trace - 1.0) / 2.0  # arg for arccos in formula
        
        # bound the input to math.acos to prevent domain errors
        # this handles the edge case of 0 and pi so that small inaccuracies dont make it out of bounds
        if tmp >= 1.0:
            theta = 0.0
        elif tmp <= -1.0:
            theta = math.pi
        else:
            theta = math.acos(tmp)
            
        # extract the unscaled skew-symmetric off-diagonal elements
        error_ori_yaw = np.array([
            R_err_yaw[2, 1] - R_err_yaw[1, 2],
            R_err_yaw[0, 2] - R_err_yaw[2, 0],
            R_err_yaw[1, 0] - R_err_yaw[0, 1]
        ])
        
        # apply the scaling factor, with a small-angle approximation
        # theta/2sin(theta) = 1/2 as lim theta -> 0
        if theta > 1e-5:
            error_ori_yaw *= theta / (2.0 * math.sin(theta))
        else:
            error_ori_yaw /= 2.0

        # calculating desired accelerations in the yaw-aligned frame using PD control
        p_des_acc_yaw = Kp_p @ error_p_yaw + Kd_p @ error_v_yaw
        omega_des_acc_yaw = Kp_w @ error_ori_yaw + Kd_w @ error_omega_yaw

        # NOTE: why is this always 9.81, this will not work for sloped terrain
        gravity_yaw = np.array([0.0, 0.0, 9.81])
        F_des_yaw = self.mass_robot * (p_des_acc_yaw + gravity_yaw)
        
        # Rank-2 Tensor Transformation: I_yaw = R_yaw^T * R_mat * I_local * R_mat^T * R_yaw
        # inertia tensor is in robot frame, its rotated to world frame and then to yaw frame (where we have R_yaw as the rotation from yaw to world, so we do R_yaw^T to go from world to yaw)
        I_yaw = R_yaw.T @ R_mat @ I_body_local @ R_mat.T @ R_yaw
        tau_des_yaw = I_yaw @ omega_des_acc_yaw 
        
        W_des_yaw = np.concatenate((F_des_yaw, tau_des_yaw)) # desired wrench in yaw-aligned frame

        # A matrix from forward dynamics
        A_mat = np.zeros((6, 12))
        for i in range(4):
            A_mat[0:3, i*3 : i*3+3] = np.eye(3)
            
            # r_i_world is the vector pointing from the CoM to the specific foot in the world frame
            r_i_world = self.foot_positions[i*3 : i*3+3] - p_com
            # rotation into the yaw frame for the cross product
            r_i_yaw = R_yaw.T @ r_i_world
            
            r_skew = np.array([
                [ 0.0,        -r_i_yaw[2],  r_i_yaw[1]],
                [ r_i_yaw[2],  0.0,        -r_i_yaw[0]],
                [-r_i_yaw[1],  r_i_yaw[0],  0.0       ]
            ])
            A_mat[3:6, i*3 : i*3+3] = r_skew
            
        if not hasattr(self, 'f_prev'):
            self.f_prev = np.zeros(12)
        
        # NOTE: calculation in notebook
        P_mat = 2.0 * (A_mat.T @ S_weight @ A_mat + alpha_W + beta_W)
        q_vec = -2.0 * (A_mat.T @ S_weight @ W_des_yaw + beta_W @ self.f_prev)
        
        # --- Constraints Setup ---
        # C_mat has 12 cols, which rep the x, y, z of all 4 legs
        # C_mat has 5 rows of constraints:
        # row 1: z constraints (between fz_max and fz_min)
        # ros 2/3: friction cone constraints in x (+ve and -ve)
        # row 4/5: friction cone constraints in y (+ve and -ve)
        C_mat = np.zeros((20, 12))
        l_bound = np.zeros(20)
        u_bound = np.zeros(20)
        
        for i in range(4):
            row = i * 5  # 5 constraints for each leg
            col = i * 3  # x, y, z of each leg
            contact_state = self.fsm_state[i] 
            
            # NOTE: friction cone constraints understood from modern robotics video lecture
            # Z constraints
            C_mat[row, col+2] = 1.0
            l_bound[row] = 0.0 if contact_state == 0.0 else self.fz_min
            u_bound[row] = 0.0 if contact_state == 0.0 else self.fz_max
            
            # X/Y Friction constraints (rotationally symmetric around Z, so valid in yaw frame)
            # C_mat @ F represents (1.0 * Fx) + (0.0 * Fy) + (-mu * Fz)
            C_mat[row+1, col] = 1.0;  C_mat[row+1, col+2] = -self.mu
            # C_mat @ F represents (-1.0 * Fx) + (0.0 * Fy) + (-mu * Fz)
            C_mat[row+2, col] = -1.0; C_mat[row+2, col+2] = -self.mu
            C_mat[row+3, col+1] = 1.0;  C_mat[row+3, col+2] = -self.mu
            C_mat[row+4, col+1] = -1.0; C_mat[row+4, col+2] = -self.mu
            
            # The bounds enforce that these linear combinations must be <= 0
            l_bound[row+1:row+5] = -np.inf
            u_bound[row+1:row+5] = 0.0
            
            # Zero out XY if swinging (forces are zeroed out when swinging)
            if contact_state == 0.0: 
                # here we're only clamping row1 and row3, and the solver evalutes the constraints to be 0 clapmed for row2 and row4 so we dont explicity need to clamp it
                C_mat[row+1, col] = 1.0; C_mat[row+1, col+2] = 0.0
                C_mat[row+3, col+1] = 1.0; C_mat[row+3, col+2] = 0.0
                l_bound[row+1] = 0.0; u_bound[row+1] = 0.0
                l_bound[row+3] = 0.0; u_bound[row+3] = 0.0

        # Convert P and C to Scipy Sparse matrices as required by OSQP
        P_sparse = sparse.csc_matrix(P_mat)
        C_sparse = sparse.csc_matrix(C_mat)
        
        # Force 1D arrays for OSQP
        q_vec = q_vec.flatten()
        l_bound = l_bound.flatten()
        u_bound = u_bound.flatten()
        
        prob = osqp.OSQP()
        prob.setup(P_sparse, q_vec, A=C_sparse, l=l_bound, u=u_bound, verbose=False)
        res = prob.solve()
        
        # The solver returns forces in the Yaw-Aligned frame (res.x means solve for the variable x)
        f_yaw = res.x 
        self.f_prev = f_yaw
        
        # --- Rotate Forces Back to World Frame ---
        # Pinocchio's LOCAL_WORLD_ALIGNED Jacobian expects forces mapped to the absolute global axes.
        f_world = np.zeros(12)
        for i in range(4):
            f_world[i*3 : i*3+3] = R_yaw @ f_yaw[i*3 : i*3+3]

        # --- Map Forces to Joint Torques ---
        tau = np.zeros(12)
        
        for i in range(4):
            f_i = f_world[i*3 : i*3+3]
            J_i = self.foot_jacobians[i]
            
            # Accumulate the torques
            # we are taking -ve because the solver calculates ground reaction forces, we apply opposite for the robot to move
            tau += -J_i.T @ f_i

        msg_tau = Float64MultiArray()
        msg_tau.data = tau.tolist()
        self.pub_torques.publish(msg_tau)
        
        
    
    def control_loop(self):
        # 1. Compute the desired Center of Mass position (Predictive Raibert step)
        p_c_d = self.compute_desired_COM()
        
        # 2. Pass it into the Balance Controller to solve the OSQP forces
        # self.get_logger().info(f'Beginning balance control loop. Desired CoM: {p_c_d}')
        self.balance_controller(p_c_d)
        # self.get_logger().info(f'Finished balance control loop. Published torques to /stance_torques.')


    
def main(args=None):
    rclpy.init(args=args)
    node = StanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()