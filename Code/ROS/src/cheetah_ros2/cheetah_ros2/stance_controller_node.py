import rclpy
import numpy as np
import math
import osqp
import pinocchio as pin

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from scipy import sparse
from cheetah_ros2.linear_mpc_configs import LinearMpcConfig
from cheetah_ros2.robot_configs import THexConfig

# for debugging
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

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
        
        # Variances for stance (c) and swing (c_bar) transitions
        # These dictate how "steep" the erf curve is at touchdown and liftoff
        self.sigma_c0 = THexConfig.sigma_c0
        self.sigma_c1 = THexConfig.sigma_c1
        self.sigma_cbar0 = THexConfig.sigma_cbar0
        self.sigma_cbar1 = THexConfig.sigma_cbar1

        # Pinocchio stuff
        urdf_path = "/workspaces/FYP-Legged-Robot/Code/ROS/src/cheetah_ros2/models/THex_Quadruped/model.urdf"
        self.pin_model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
        self.pin_data = self.pin_model.createData()

        self.controller_joint_names = [
            'fl_hip_joint','fl_knee_joint','fl_foot_joint',
            'fr_hip_joint','fr_knee_joint','fr_foot_joint',
            'bl_hip_joint','bl_knee_joint','bl_foot_joint',
            'br_hip_joint','br_knee_joint','br_foot_joint'
        ]
        
        # --- THE FIX: Using the working loop from your estimator ---
        self.pin_joint_names = []
        for jid, jname in enumerate(self.pin_model.names):
            jmodel = self.pin_model.joints[jid]
            if jmodel.nq == 1 and jname in self.controller_joint_names:
                self.pin_joint_names.append(jname)

        self.ctrl_to_pin = np.array([self.pin_joint_names.index(n) for n in self.controller_joint_names], dtype=int)
        self.pin_to_ctrl = np.argsort(self.ctrl_to_pin)

        self.pub_torques = self.create_publisher(Float64MultiArray, '/stance_torques', 1)
        self.pub_markers = self.create_publisher(Marker, '/com_target_markers', 1) # for debugging
        self.pub_support_polygon = self.create_publisher(Marker, '/support_polygon_markers', 1) # for debugging
        
        self.sub_state = self.create_subscription(Float64MultiArray, '/estimated_robot_state', self.state_cb, 1)
        self.sub_foot = self.create_subscription(Float64MultiArray, '/foot_positions', self.foot_cb, 1)
        self.sub_stance = self.create_subscription(Float64MultiArray, '/stance_phases', self.stance_cb, 1)
        self.sub_swing = self.create_subscription(Float64MultiArray, '/swing_phases', self.swing_cb, 1)
        self.sub_nom = self.create_subscription(Float64MultiArray, '/fsm_state', self.nom_cb, 1)
        self.sub_jacobians = self.create_subscription(Float64MultiArray, '/foot_jacobians', self.jacobian_cb, 1)
        self.sub_teleop = self.create_subscription(Twist, '/teleop', self.teleop_cb, 1)
        
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

    def teleop_cb(self, msg):
        self.cmd_xvel = float(msg.linear.x)
        self.cmd_yvel = float(msg.linear.y)
        self.cmd_yaw_turn_rate = float(msg.angular.z)

    # for debugging
    def publish_target_marker(self, p_target: np.ndarray):

        marker = Marker()
        # 'odom' is your global fixed frame from the Gazebo bridge
        marker.header.frame_id = 'odom' 
        marker.header.stamp = self.get_clock().now().to_msg()
        
        # Give each leg its own namespace and ID so they don't overwrite each other
        marker.ns = 'desired_com'
        marker.id = 4
        
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Set the position
        marker.pose.position.x = float(p_target[0])
        marker.pose.position.y = float(p_target[1])
        marker.pose.position.z = float(p_target[2])
        
        # Set orientation to default
        marker.pose.orientation.w = 1.0
        
        # Scale: A 4cm diameter sphere
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        
        # Color: Bright Blue (RGBA)
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.8  # Slightly transparent
        
        self.pub_markers.publish(marker)

    def publish_support_polygon_marker(self, virtual_points: np.ndarray):

        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'predictive_support_polygon'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.012

        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.2
        marker.color.a = 0.9

        center_xy = np.mean(virtual_points[:, :2], axis=0)
        angles = np.arctan2(virtual_points[:, 1] - center_xy[1], virtual_points[:, 0] - center_xy[0])
        ordered_indices = np.argsort(angles)
        ordered_points = virtual_points[ordered_indices]

        for point_xyz in ordered_points:
            point = Point()
            point.x = float(point_xyz[0])
            point.y = float(point_xyz[1])
            point.z = float(point_xyz[2])
            marker.points.append(point)

        if len(marker.points) > 0:
            marker.points.append(marker.points[0])

        self.pub_support_polygon.publish(marker)

    def publish_command_velocity_marker(self, p_com: np.ndarray, v_des_world: np.ndarray):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'commanded_velocity'
        marker.id = 0
        marker.type = Marker.ARROW

        speed = np.linalg.norm(v_des_world)
        if speed < 1e-6:
            marker.action = Marker.DELETE
            self.pub_markers.publish(marker)
            return

        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        # Arrow length scales with speed while keeping a visible minimum.
        arrow_length = max(0.08, 0.6 * speed)
        direction = v_des_world / speed

        start = Point()
        start.x = float(p_com[0])
        start.y = float(p_com[1])
        start.z = float(p_com[2] + 0.05)

        end = Point()
        end.x = float(start.x + arrow_length * direction[0])
        end.y = float(start.y + arrow_length * direction[1])
        end.z = float(start.z)

        marker.points = [start, end]

        marker.scale.x = 0.01  # shaft diameter
        marker.scale.y = 0.02  # head diameter
        marker.scale.z = 0.03  # head length

        marker.color.r = 0.95
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 0.95

        self.pub_markers.publish(marker)

    def compute_desired_COM(self) -> np.ndarray:

        weights = np.zeros(4)
        sqrt_2 = math.sqrt(2.0)
        virtual_points = np.zeros((4, 3))
        
        # Variances for stance (c) and swing (c_bar) transitions
        # These dictate how "steep" the erf curve is at touchdown and liftoff
        sigma_c0 = self.sigma_c0
        sigma_c1 = self.sigma_c1
        sigma_cbar0 = self.sigma_cbar0
        sigma_cbar1 = self.sigma_cbar1

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
        
        self.publish_support_polygon_marker(virtual_points)
        p_c_bar = np.mean(virtual_points, axis=0)
          
        # calculate avg_foot_z froms stance feet
        avg_foot_z = 0.0
        count = 0
        for i in range(4):
            if self.fsm_state[i] == 1.0:
                avg_foot_z += p_foot[i][2]
                count += 1
        avg_foot_z = avg_foot_z / count if count > 0 else p_c_bar[2]

        desired_z = avg_foot_z + self.target_z

        # NOTE: this won't entirely work for sloped terrain
        p_c_d = np.array([p_c_bar[0], p_c_bar[1], desired_z])
        
        # self.get_logger().info(f'DEBUG: Computed desired CoM position: {p_c_d}')
        self.publish_target_marker(p_c_d) # for debugging

        return p_c_d

    # # for testing, just compute the centroid of the stance foot positions for x and y
    # def compute_desired_COM(self) -> np.ndarray:

    #     p_foot = self.foot_positions.reshape(4, 3)

    #     stance_foot_positions = []
    #     for i in range(4):
    #         if self.fsm_state[i] == 1.0: 
    #             stance_foot_positions.append(p_foot[i])

    #     if len(stance_foot_positions) > 0:
    #         desired_x = np.mean([pos[0] for pos in stance_foot_positions])
    #         desired_y = np.mean([pos[1] for pos in stance_foot_positions])
    #     else:
    #         desired_x = self.robot_state[0]
    #         desired_y = self.robot_state[1]

    #     avg_foot_z = 0.0
    #     count = 0
    #     for i in range(4):
    #         if self.fsm_state[i] == 1.0:
    #             avg_foot_z += p_foot[i][2]
    #             count += 1
    #     avg_foot_z = avg_foot_z / count if count > 0 else -8
    #     desired_z = avg_foot_z + self.target_z

    #     # NOTE: this won't entirely work for sloped terrain
    #     # p_c_d = np.array([desired_x, desired_y, desired_z])
    #     p_c_d = np.array([self.robot_state[0], self.robot_state[1], desired_z]) 
        
    #     # self.get_logger().info(f'DEBUG: Computed desired CoM position: {p_c_d}'))
    #     self.publish_target_marker(p_c_d) # for debugging

    #     return p_c_d
        
    def balance_controller(self, p_c_d: np.ndarray) -> np.ndarray:
        
        # [p_com(3), v_com(3), q(12), qdot(12), quat(4), omega(3), accel(3)]
        p_com = self.robot_state[0:3] # world frame
        v_com = self.robot_state[3:6] # world frame
        q_joints = self.robot_state[6:18]
        w, x, y, z = self.robot_state[30:34]
        omega = self.robot_state[34:37] # world frame

        pin_quat = np.array([x, y, z, w])
        q_pin = q_joints[self.pin_to_ctrl]
        q_gen = np.concatenate((p_com, pin_quat, q_pin))

        pin.centerOfMass(self.pin_model, self.pin_data, q_gen)
        true_com = self.pin_data.com[0] 
        p_com = true_com
        
        # # True Inertia Matrix from PInochio
        # v_gen = np.zeros(self.pin_model.nv)
        # pin.ccrba(self.pin_model, self.pin_data, q_gen, v_gen)
        # I_com_world = self.pin_data.Ig.inertia

        pin.computeGeneralizedGravity(self.pin_model, self.pin_data, q_gen)
        g_vector_ctrl = self.pin_data.g[6:18][self.ctrl_to_pin]
        
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

        v_des_world = R_yaw @ v_des
        self.publish_command_velocity_marker(p_com, v_des_world)
        
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

        # # Use the total inertia in the yaw frame directly
        # I_yaw = R_yaw.T @ I_com_world @ R_yaw
        # tau_des_yaw = I_yaw @ omega_des_acc_yaw
        
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
        
        # The solver returns forces in the Yaw-Aligned frame
        f_yaw = res.x 
        self.f_prev = f_yaw
        
        # rotate forces back to World Frame as Pinocchio's LOCAL_WORLD_ALIGNED Jacobian expects forces mapped to the absolute global axes.
        f_world = np.zeros(12)
        for i in range(4):
            f_world[i*3 : i*3+3] = R_yaw @ f_yaw[i*3 : i*3+3]

        tau = np.zeros(12)
        
        for i in range(4):
            f_i = f_world[i*3 : i*3+3]
            J_i = self.foot_jacobians[i]

            idx = slice(i*3, i*3+3)
            J_leg = J_i[:, idx]
            
            g_leg = g_vector_ctrl[idx]
            # tau (3x1) = -J_leg^T (3x3) * F_grf (3x1) + G(q) (3x1)
            tau[idx] = -J_leg.T @ f_i + g_leg

        msg_tau = Float64MultiArray()
        tau = np.clip(tau, -0.9414, 0.9414) 
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