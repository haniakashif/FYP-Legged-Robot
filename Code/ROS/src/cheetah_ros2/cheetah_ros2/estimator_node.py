#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu
from ros_gz_interfaces.msg import Contacts
import numpy as np
import math
import pinocchio as pin
from cheetah_ros2.linear_mpc_configs import LinearMpcConfig
from nav_msgs.msg import Odometry

# for debugging
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

PI = math.pi

'''
NOTE: STATE ESTIMATION SHOULD BE DONE HERE, RIGHT NOW IM READING JOINT STATES AND USING CONTACT SENSORS, THEY SHOULD BE REMOVED AFTER STATE ESTMATION
'''

class EstimatorNode(Node):
    def __init__(self):
        super().__init__('state_estimator_node')
        
        urdf_path = "/workspaces/FYP-Legged-Robot/Code/ROS/src/cheetah_ros2/models/THex_Quadruped/model.urdf"
        self.pin_model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
        self.pin_data = self.pin_model.createData()
        
        # Extract the internal frame IDs for your feet
        self.foot_frames = ['fl_foot_tip', 'fr_foot_tip', 'bl_foot_tip', 'br_foot_tip']
        self.foot_ids = [self.pin_model.getFrameId(frame) for frame in self.foot_frames]
        
        
        self.dt_control = LinearMpcConfig.dt_control
        self.q = np.zeros(12)
        self.qdot = np.zeros(12)
        self.quat = np.array([1.0, 0.0, 0.0, 0.0]) # w, x, y, z
        self.omega = np.zeros(3)
        self.accel = np.zeros(3)
        
        # NOTE: contact sensors dont have standard topics with ros2_control, which is why we need to add a bridge in the yaml file, remove clock from there as ros2_control will handle time itself and also add the contact sensor to the world file
        
        self.physical_contacts = np.zeros(4)
        self.last_contact_time = [0.0, 0.0, 0.0, 0.0]
        self.contact_timeout = 0.05  # 50 milliseconds timeout
        self.foot_positions = np.zeros(12)
        self.foot_jacobians = np.zeros((4, 3, 12))
        
        # Ground Truth Tracking Variables
        self.p_com = np.zeros(3)
        self.v_com = np.zeros(3)
        
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

        if len(self.pin_joint_names) != 12:
            raise RuntimeError(f"Expected 12 actuated joints in pin model, got {len(self.pin_joint_names)}: {self.pin_joint_names}")

        missing = [n for n in self.controller_joint_names if n not in self.pin_joint_names]
        if missing:
            raise RuntimeError(f"Missing controller joints in pin model: {missing}")

        self.get_logger().info(f"Pinocchio Joint Order: {self.pin_joint_names}")
        self.get_logger().info(f"Controller to Pinocchio Joint Mapping: {self.ctrl_to_pin}")
        
        self.pub_estimated_contacts = self.create_publisher(Float64MultiArray, '/estimated_contacts', 1)
        self.pub_robot_state = self.create_publisher(Float64MultiArray, '/estimated_robot_state', 1)
        self.pub_foot_positions = self.create_publisher(Float64MultiArray, '/foot_positions', 1)
        # Publisher for the flattened 4x3x12 Jacobians (144 elements)
        self.pub_jacobians = self.create_publisher(Float64MultiArray, '/foot_jacobians', 1)
        
        self.tf_broadcaster = TransformBroadcaster(self)

        '''
        order of getting joint_states:
        fl_hip_joint
        fl_knee_joint
        fl_foot_joint
        fr_hip_joint
        fr_knee_joint
        fr_foot_joint
        bl_hip_joint
        bl_knee_joint
        bl_foot_joint
        br_hip_joint
        br_knee_joint
        br_foot_joint
        '''
        
        self.sub_joints = self.create_subscription(JointState, '/joint_states', self.joint_cb, 1)
        # NOTE: the IMU readings should be integrated to find COM pos and vel but I'm reading odom rn (so I know absolute pos and vel of COM)
        # self.sub_imu = self.create_subscription(Imu, '/imu_broadcaster/imu', self.imu_cb, 1)
        
        self.sub_odom = self.create_subscription(Odometry, '/model/THex_Quadruped/odometry', self.odom_cb, 1)
        
        self.sub_fl = self.create_subscription(Contacts, '/contact/fl', self.fl_cb, 1)
        self.sub_fr = self.create_subscription(Contacts, '/contact/fr', self.fr_cb, 1)
        self.sub_bl = self.create_subscription(Contacts, '/contact/bl', self.bl_cb, 1)
        self.sub_br = self.create_subscription(Contacts, '/contact/br', self.br_cb, 1)
        
        self.get_logger().info("State Estimator Node active: Using Watchdog Timers for Contacts And Reading clean IMU/Joints (Perfect State Knowledge)")
        # timer is created for calling back with some freq
        # NOTE: assuming all sensors are running at same dt_control freq
        self.timer = self.create_timer(self.dt_control, self.timer_cb)
         


    # --- Contact Callbacks (Update Timestamp on Event) ---
    def fl_cb(self, msg):
        # self.get_logger().info(f'FL Contact Callback: Detected {len(msg.contacts)} contacts.')
        if len(msg.contacts) > 0:
            self.physical_contacts[0] = 1.0
            self.last_contact_time[0] = self.get_clock().now().nanoseconds / 1e9

    def fr_cb(self, msg):
        # self.get_logger().info(f'FR Contact Callback: Detected {len(msg.contacts)} contacts.')
        if len(msg.contacts) > 0:
            self.physical_contacts[1] = 1.0
            self.last_contact_time[1] = self.get_clock().now().nanoseconds / 1e9

    def bl_cb(self, msg):
        # self.get_logger().info(f'BL Contact Callback: Detected {len(msg.contacts)} contacts.')
        if len(msg.contacts) > 0:
            self.physical_contacts[2] = 1.0
            self.last_contact_time[2] = self.get_clock().now().nanoseconds / 1e9

    def br_cb(self, msg):
        # self.get_logger().info(f'BR Contact Callback: Detected {len(msg.contacts)} contacts.')
        if len(msg.contacts) > 0:
            self.physical_contacts[3] = 1.0
            self.last_contact_time[3] = self.get_clock().now().nanoseconds / 1e9
    
    def joint_cb(self, msg):
        # self.get_logger().info(f'Joint State Callback: Received positions for joints: {msg.name}') 
        
        # necessary for pinnochio
        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        
        for i, name in enumerate(self.controller_joint_names):
            if name in name_to_idx:
                idx = name_to_idx[name]
                self.q[i] = msg.position[idx]
                self.qdot[i] = msg.velocity[idx]

    def imu_cb(self, msg):
        # self.get_logger().info(f'IMU Callback: Received IMU data. {msg}')
        self.quat = np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])
        self.omega = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self.accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        
    def odom_cb(self, msg):
        # self.get_logger().info(f'Odometry Callback: Received odometry data. {msg}')

        # world frame
        self.p_com[0] = msg.pose.pose.position.x
        self.p_com[1] = msg.pose.pose.position.y
        self.p_com[2] = msg.pose.pose.position.z
        
        # world frame
        self.quat[0] = msg.pose.pose.orientation.w
        self.quat[1] = msg.pose.pose.orientation.x
        self.quat[2] = msg.pose.pose.orientation.y
        self.quat[3] = msg.pose.pose.orientation.z

        # body frame
        v_local = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        
        # Convert Quaternion to Rotation Matrix
        w, x, y, z = self.quat
        R_mat = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),       1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w),       2*(y*z + x*w),       1 - 2*(x**2 + y**2)]
        ])

        # world frame
        self.v_com = R_mat @ v_local

        # body frame
        omega_local = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])

        # world frame
        self.omega = R_mat @ omega_local

        # for debugging
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.p_com[0]
        t.transform.translation.y = self.p_com[1]
        t.transform.translation.z = self.p_com[2]
        
        t.transform.rotation.w = self.quat[0]
        t.transform.rotation.x = self.quat[1]
        t.transform.rotation.y = self.quat[2]
        t.transform.rotation.z = self.quat[3]
        
        self.tf_broadcaster.sendTransform(t)

    def timer_cb(self):
        # self.get_logger().info('Timer Callback: Evaluating estimator.')
        # NOTE: estimated_contacts is active, the entries for quat, omega and accel are all 0        
        
        # Evaluate Watchdog Timers
        current_time = self.get_clock().now().nanoseconds / 1e9
        for i in range(4):
            # If the sensor has been silent for longer than our timeout, the foot is swinging
            if (current_time - self.last_contact_time[i]) > self.contact_timeout:
                self.physical_contacts[i] = 0.0
        
        # NOTE: 
        # Pinocchio uses [x, y, z, w] for quaternions, unlike ROS which uses [w, x, y, z]
        pin_quat = np.array([self.quat[1], self.quat[2], self.quat[3], self.quat[0]])
        q_pin = self.q[self.pin_to_ctrl]  # Reorder q to match Pinocchio's expected joint order
        q_gen = np.concatenate((self.p_com, pin_quat, q_pin))
        
        pin.forwardKinematics(self.pin_model, self.pin_data, q_gen)
        pin.updateFramePlacements(self.pin_model, self.pin_data)
        
        for i, foot_id in enumerate(self.foot_ids):
            # reading the cached 3D position of the foot.
            self.foot_positions[i*3 : i*3+3] = self.pin_data.oMf[foot_id].translation
            
            # Extract the Jacobian (LOCAL_WORLD_ALIGNED aligns the forces to the global XYZ grid)
            J_full = pin.computeFrameJacobian(self.pin_model, self.pin_data, q_gen, foot_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            
            # self.foot_jacobians[i] = J_full[0:3, 6:18]
            
            J_pin = J_full[0:3, 6:18]  
            J_ctrl = J_pin[:, self.ctrl_to_pin]  
            self.foot_jacobians[i] = J_ctrl
            
            # 3 for linear velocity ($v_x, v_y, v_z$) and 3 for angular velocity ($\omega_x, \omega_y, \omega_z$) Since your robot's feet are point contacts (they can't exert rotational torque on the ground, only linear pushing forces), we discard the bottom 3 rows.
            # Because it is a floating-base robot, the system has 18 Degrees of Freedom (DoF) for velocities (6 for the unactuated base + 12 for the motors). Columns 0 through 5 represent how moving the base moves the foot. We don't have motors on the base, so we discard them. Columns 6 through 17 represent the 12 actual actuated joints.
        
        # Flatten the list of four 3x12 matrices into a single 1D array
        # flat_jacobians = np.concatenate([J.flatten() for J in self.foot_jacobians])
        flat_jacobians = self.foot_jacobians.flatten()
        msg_jac = Float64MultiArray()
        msg_jac.data = flat_jacobians.tolist()
        self.pub_jacobians.publish(msg_jac)
        
        # robot_state = np.concatenate((self.q, self.qdot, self.quat, self.omega, self.accel)) # 34 x 1
        robot_state = np.concatenate((self.p_com, self.v_com, self.q, self.qdot, self.quat, self.omega, self.accel)) # 40 x 1
        msg_state = Float64MultiArray()
        msg_state.data = robot_state.tolist()
        self.pub_robot_state.publish(msg_state)
        
        msg_contacts = Float64MultiArray()
        msg_contacts.data = self.physical_contacts.tolist()
        self.pub_estimated_contacts.publish(msg_contacts)
        
        # The publisher will publish regardless, the line below needs to be uncomented when using my own FK implementation
        # self.foot_positions = self.compute_forward_kinematics(self.q)
        msg_foot_pos = Float64MultiArray()
        msg_foot_pos.data = self.foot_positions.tolist()
        self.pub_foot_positions.publish(msg_foot_pos)
        
    # NOTE: using pinnochio for FK is another approach which gives the kinematics in the world frame. Using my own implementation gives the FK in the robot frame
    def compute_forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        # q is a 12-element array: [FL_hip, FL_knee, FL_foot, FR_hip...]
        L1 = 0.02845  # in m
        L2 = 0.05439
        L3 = 0.02637
        L4 = 0.09265
        
        flt1 = q[0]
        flt2 = q[1]
        flt3 = q[2]
        frt1 = q[3]
        frt2 = q[4]
        frt3 = q[5]
        blt1 = q[6]
        blt2 = q[7]
        blt3 = q[8]
        brt1 = q[9]
        brt2 = q[10]
        brt3 = q[11]
        
        rtheta = PI/4
        ltheta = -PI/4
        
        ''' 
        Right side calculations
        cos(theta1)*(l1 + l4*cos(rtheta - theta2 + theta3) + l2*cos(theta2) + l3*cos(theta2 - rtheta))
        sin(theta1)*(l1 + l4*cos(rtheta - theta2 + theta3) + l2*cos(theta2) + l3*cos(theta2 - rtheta))
        l2*sin(theta2) - l4*sin(rtheta - theta2 + theta3) + l3*sin(theta2 - rtheta)
        
        Left side calculations
        cos(theta1)*(l1 + l4*cos(ltheta - theta2 + theta3) + l2*cos(theta2) + l3*cos(theta2 - ltheta))
        sin(theta1)*(l1 + l4*cos(ltheta - theta2 + theta3) + l2*cos(theta2) + l3*cos(theta2 - ltheta))
        l4*sin(ltheta - theta2 + theta3) - l2*sin(theta2) - l3*sin(theta2 - ltheta)
        '''                    
        
        fl_x = math.cos(flt1)*(L1 + L4*math.cos(ltheta - flt2 + flt3) + L2*math.cos(flt2) + L3*math.cos(flt2 - ltheta))
        fl_y = math.sin(flt1)*(L1 + L4*math.cos(ltheta - flt2 + flt3) + L2*math.cos(flt2) + L3*math.cos(flt2 - ltheta))
        fl_z = L4*math.sin(ltheta - flt2 + flt3) - L2*math.sin(flt2) - L3*math.sin(flt2 - ltheta)
        
        fr_x = math.cos(frt1)*(L1 + L4*math.cos(rtheta - frt2 + frt3) + L2*math.cos(frt2) + L3*math.cos(frt2 - rtheta))
        fr_y = math.sin(frt1)*(L1 + L4*math.cos(rtheta - frt2 + frt3) + L2*math.cos(frt2) + L3*math.cos(frt2 - rtheta))
        fr_z = L2*math.sin(frt2) - L4*math.sin(rtheta - frt2 + frt3) + L3*math.sin(frt2 - rtheta)
        
        bl_x = math.cos(blt1)*(L1 + L4*math.cos(ltheta - blt2 + blt3) + L2*math.cos(blt2) + L3*math.cos(blt2 - ltheta))
        bl_y = math.sin(blt1)*(L1 + L4*math.cos(ltheta - blt2 + blt3) + L2*math.cos(blt2) + L3*math.cos(blt2 - ltheta))
        bl_z = L4*math.sin(ltheta - blt2 + blt3) - L2*math.sin(blt2) - L3*math.sin(blt2 - ltheta)
        
        br_x = math.cos(brt1)*(L1 + L4*math.cos(rtheta - brt2 + brt3) + L2*math.cos(brt2) + L3*math.cos(brt2 - rtheta))
        br_y = math.sin(brt1)*(L1 + L4*math.cos(rtheta - brt2 + brt3) + L2*math.cos(brt2) + L3*math.cos(brt2 - rtheta))
        br_z = L2*math.sin(brt2) - L4*math.sin(rtheta - brt2 + brt3) + L3*math.sin(brt2 - rtheta)
                   
        return np.array([fl_x, fl_y, fl_z, fr_x, fr_y, fr_z, bl_x, bl_y, bl_z, br_x, br_y, br_z])

        
def main(args=None):
    rclpy.init(args=args)
    node = EstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()