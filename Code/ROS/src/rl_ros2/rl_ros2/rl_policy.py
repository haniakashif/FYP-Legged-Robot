import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import onnxruntime as ort
import os

class RLPolicy(Node):
    def __init__(self):
        super().__init__('rl_policy')

        self.onnx_path = "../Policies/pa_2026-04-28_01-01-12_v1.onnx"

        self.get_logger().info(f"Loading ONNX model from {self.onnx_path}...")
        try:
            self.ort_session = ort.InferenceSession(self.onnx_path)
            self.input_name = self.ort_session.get_inputs()[0].name
        except Exception as e:
            self.get_logger().error(f"Failed to load ONNX: {e}")
            raise

        self.expected_input_dim = self.ort_session.get_inputs()[0].shape[1]

        if self.expected_input_dim not in [33, 45, 48]:
            self.get_logger().error(f"Unexpected input dimension in ONNX model: {self.expected_input_dim}")
            raise ValueError(f"ONNX model must have input dimension of either 33, 45, or 48. Received: {self.expected_input_dim}")

        if self.expected_input_dim == 33:
            self.get_logger().info("ONNX model expects 33 inputs, will exclude joint velocities and height command from observations.")
        elif self.expected_input_dim == 45:
            self.get_logger().info("ONNX model expects 45 inputs, will include joint velocities and exclude height command from observations.")
        elif self.expected_input_dim == 48:
            self.get_logger().info("ONNX model expects 48 inputs, will include joint velocities and height command in observations.")
        

        self.action_pub = self.create_publisher(Float64MultiArray, '/rl/actions', 1)
        
        # event-driven instead of internal timer
        self.create_subscription(Float64MultiArray, '/rl/observations', self.obs_cb, 1)

        self.get_logger().info("RL Policy Node Ready!")

    def obs_cb(self, msg):
        
        obs = np.array(msg.data, dtype=np.float32)

        # self.get_logger().info("Received observation, running policy...")

        if (obs[6:9] == 0.0).all(): # if user command is zero, publish zero actions to avoid unintended movement
            # self.get_logger().info("Zero command received, publishing zero actions.")
            self.action_pub.publish(Float64MultiArray(data=[0.0 for _ in range(12)]))
            return

        # ONNX expects a batch dimension: (1, 36)
        input_tensor = obs.reshape(1, -1)

        # can I add a test here to see what is the input dimension in the ONNX file and exclude joint_velocities (indices 24 to 35)and height command (indices 9 to 11) if it is 33 and exclude just height command if it is 45
        if self.expected_input_dim != 48:
            if self.expected_input_dim == 33:
                input_tensor = np.delete(input_tensor, np.s_[24:36], axis=1)
            input_tensor = np.delete(input_tensor, np.s_[9:12], axis=1)

        if input_tensor.shape[1] != self.expected_input_dim:
            self.get_logger().error(f"Input tensor has wrong shape: {input_tensor.shape}, expected (1, {self.expected_input_dim})")
            return

        # Run inference
        outputs = self.ort_session.run(None, {self.input_name: input_tensor})
        raw_actions = outputs[0][0] # Remove batch dim

        # self.get_logger().info(f"Actions:")
        # for i, action in enumerate(raw_actions):
        #     self.get_logger().info(f"  Action {i}: {action:.4f}")

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