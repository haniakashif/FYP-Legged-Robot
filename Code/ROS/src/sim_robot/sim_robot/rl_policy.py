import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import onnxruntime as ort
import os

class RLPolicy(Node):
    def __init__(self):
        super().__init__('rl_policy')

        self.onnx_path = "/workspaces/FYP-Legged-Robot/Code/Policies/2026-02-13_12-26-48_v1.onnx"

        self.obs_dim = 33 # hardcoded in training

        self.get_logger().info(f"Loading ONNX model from {self.onnx_path}...")
        try:
            self.ort_session = ort.InferenceSession(self.onnx_path)
            self.input_name = self.ort_session.get_inputs()[0].name
        except Exception as e:
            self.get_logger().error(f"Failed to load ONNX: {e}")
            raise

        self.action_pub = self.create_publisher(Float64MultiArray, '/rl/actions', 1)
        
        # event-driven instead of internal timer
        self.create_subscription(Float64MultiArray, '/rl/observations', self.obs_cb, 1)

        self.get_logger().info("RL Policy Node Ready!")

    def obs_cb(self, msg):
        
        obs = np.array(msg.data, dtype=np.float32)
        
        if len(obs) != self.obs_dim:
            self.get_logger().warn(f"Obs dimension mismatch! Expected {self.obs_dim}, got {len(obs)}")
            return

        self.get_logger().info("Received observation, running policy...")

        # ONNX expects a batch dimension: (1, 33)
        input_tensor = obs.reshape(1, -1)
        
        # Run inference
        outputs = self.ort_session.run(None, {self.input_name: input_tensor})
        raw_actions = outputs[0][0] # Remove batch dim

        self.get_logger().info(f"Actions:")
        for i, action in enumerate(raw_actions):
            self.get_logger().info(f"  Action {i}: {action:.4f}")

        # 4. Publish
        out_msg = Float64MultiArray()
        out_msg.data = raw_actions.tolist()
        self.action_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RLPolicy()
    
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