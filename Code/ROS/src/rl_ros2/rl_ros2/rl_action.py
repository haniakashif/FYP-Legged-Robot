import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class RLAction(Node):
    def __init__(self):
        super().__init__('rl_action')

        self.default_pos = 0.0
        self.action_scale = 0.15

        topic_name = '/joint_group_position_controller/commands'
        self.cmd_pub = self.create_publisher(Float64MultiArray, topic_name, 1)
        
        self.get_logger().info(f"Mapped output -> {topic_name}")

        self.create_subscription(Float64MultiArray, '/rl/actions', self.action_cb, 1)

        self.get_logger().info("RL Action Node Ready! Waiting for policy...")

    def action_cb(self, msg):
        actions = msg.data
        
        if len(actions) != 12:
            self.get_logger().error(f"Received {len(actions)} actions, expected 12")
            return

        # Calculate Targets
        target_positions = [(a * self.action_scale) + self.default_pos for a in actions]

        # can possibly add a clipping here to limit target_pos within joint limits

        # Publish the single array to ros2_control
        msg_out = Float64MultiArray()
        msg_out.data = target_positions
        self.cmd_pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = RLAction()
    
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

