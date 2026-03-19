import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

from . import kinematics as kin

class KinNode(Node):
    def __init__(self):
        super().__init__('kin_node')

        self.target_freq = 10
        self.dt = 1.0 / self.target_freq

        self.cmd_pub = self.create_publisher(
            Float64MultiArray, 
            '/joint_group_position_controller/commands', 
            10
        )

        self.get_logger().info("Pre-computing Trajectory...")
        xyz = kin.generate_trajectory()
        
        xyz0 = kin.shift_trajectory(0, kin.rotate_trajectory(0, xyz)) # FR (Leg 0)
        xyz1 = kin.shift_trajectory(1, kin.rotate_trajectory(1, xyz)) # BR (Leg 1)
        xyz2 = kin.shift_trajectory(2, kin.rotate_trajectory(2, xyz)) # BL (Leg 2)
        xyz3 = kin.shift_trajectory(3, kin.rotate_trajectory(3, xyz)) # FL (Leg 3)

        self.theta_targets = [
            kin.inv_kin_array(xyz0, 0), # FR (Index 0)
            kin.inv_kin_array(xyz1, 1), # BR (Index 1)
            kin.inv_kin_array(xyz2, 2), # BL (Index 2)
            kin.inv_kin_array(xyz3, 3)  # FL (Index 3)
        ]
        
        self.steps_len = len(self.theta_targets[0][0])
        self.current_step = 0
        self.get_logger().info(f"Generated {self.steps_len} steps per cycle. Starting loop.")

        self.timer = self.create_timer(self.dt, self.timer_callback)

    def timer_callback(self):
        msg = Float64MultiArray()
        
        step = self.current_step
        
        # theta_targets[LEG][JOINT][STEP] 
        # JOINT: 0=Hip, 1=Knee, 2=Foot
        
        # FR (Leg 0)
        fr_hip, fr_knee, fr_foot = self.theta_targets[0][0][step], self.theta_targets[0][1][step], self.theta_targets[0][2][step]
        # BR (Leg 1)
        br_hip, br_knee, br_foot = self.theta_targets[1][0][step], self.theta_targets[1][1][step], self.theta_targets[1][2][step]
        # BL (Leg 2)
        bl_hip, bl_knee, bl_foot = self.theta_targets[2][0][step], self.theta_targets[2][1][step], self.theta_targets[2][2][step]
        # FL (Leg 3)
        fl_hip, fl_knee, fl_foot = self.theta_targets[3][0][step], self.theta_targets[3][1][step], self.theta_targets[3][2][step]

        # MAP TO CONTROLLERS.YAML ORDER
        # [bl_hip, br_hip, fl_hip, fr_hip, bl_knee, br_knee, fl_knee, fr_knee, bl_foot, br_foot, fl_foot, fr_foot]
        msg.data = [
            float(bl_hip), float(br_hip), float(fl_hip), float(fr_hip),
            float(bl_knee), float(br_knee), float(fl_knee), float(fr_knee),
            float(bl_foot), float(br_foot), float(fl_foot), float(fr_foot)
        ]

        self.cmd_pub.publish(msg)

        self.current_step += 1
        if self.current_step >= self.steps_len:
            self.current_step = 0

def main(args=None):
    rclpy.init(args=args)
    node = KinNode()
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