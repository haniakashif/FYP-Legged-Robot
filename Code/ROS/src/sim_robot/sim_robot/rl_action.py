import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
import numpy as np

class RLAction(Node):
    def __init__(self):
        super().__init__('rl_action')

        self.joints = ["bl_hip", "br_hip", "fl_hip", "fr_hip",
                        "bl_knee", "br_knee", "fl_knee", "fr_knee",
                        "bl_foot", "br_foot", "fl_foot", "fr_foot"] 
        
        self.default_pos = 0.0

        self.action_scale = 0.15

        self.pubs = {} 
        
        self.index_to_pub = []

        for joint in self.joints:
                topic_name = f"/{joint}/command"
                
                pub = self.create_publisher(Float64, topic_name, 1)
                
                self.pubs[joint] = pub
                self.index_to_pub.append(pub)
                
                self.get_logger().info(f"mapped index {len(self.index_to_pub)-1} -> {topic_name}")

        self.create_subscription(Float64MultiArray, '/rl/actions', self.action_cb, 1)

        self.get_logger().info("RL Action Node Ready! Waiting for policy...")

    def action_cb(self, msg):
        actions = msg.data
        
        if len(actions) != 12:
            self.get_logger().error(f"Received {len(actions)} actions, expected 12")
            return

        for i, action_delta in enumerate(actions):
            
            target_pos = self.default_pos + (action_delta * self.action_scale)

            # can possibly add a clipping here to limit target_pos within joint limits

            msg_out = Float64()
            msg_out.data = float(target_pos)
            self.index_to_pub[i].publish(msg_out)

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