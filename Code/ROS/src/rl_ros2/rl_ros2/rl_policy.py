import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import onnxruntime as ort
import os
 
class RLPolicy(Node):
    def __init__(self):
        super().__init__('rl_policy')

        self.interp_steps = 200 # number of steps to interpolate the command to zero when user command goes to zero, to avoid abrupt stops
        self.interp_counter = 0
        self.last_command = np.zeros(12, dtype=np.float32)

        self.onnx_path = "../Policies/2026-04-20_09-49-36_v1.onnx" # best sim no PA
        # self.onnx_path = "../Policies/2026-05-09_15-50-31_v1.onnx" # best hardware
        # self.onnx_path = "../Policies/pa_2026-05-11_10-16-55_v1.onnx" # height PA
        self.height_cmd_in_obs = True # if the network is 48D, then this decides whether to include height or sprawl
        # self.onnx_path = "../Policies/pa_2026-05-11_09-17-58_v1.onnx" # sprawl PA
        # self.height_cmd_in_obs = False # if the network is 48D, then this decides whether to include height or sprawl

        self.get_logger().info(f"Loading ONNX model from {self.onnx_path}...")
        try:
            self.ort_session = ort.InferenceSession(self.onnx_path)
            self.input_name = self.ort_session.get_inputs()[0].name
        except Exception as e:
            self.get_logger().error(f"Failed to load ONNX: {e}")
            raise

        self.expected_input_dim = self.ort_session.get_inputs()[0].shape[1]

        if self.expected_input_dim not in [33, 45, 48, 51]:
            self.get_logger().error(f"Unexpected input dimension in ONNX model: {self.expected_input_dim}")
            raise ValueError(f"ONNX model must have input dimension of either 33, 45, 48, or 51. Received: {self.expected_input_dim}")

        if self.expected_input_dim == 33:
            self.get_logger().info("ONNX model expects 33 inputs, will exclude joint velocities, height command, and sprawl command from observations.")
        elif self.expected_input_dim == 45:
            self.get_logger().info("ONNX model expects 45 inputs, will include joint velocities, exclude height command and sprawl command from observations.")
        elif self.expected_input_dim == 48 and self.height_cmd_in_obs:
            self.get_logger().info("ONNX model expects 48 inputs, will include joint velocities and height command, exclude sprawl command from observations.")
        elif self.expected_input_dim == 48 and not self.height_cmd_in_obs:
            self.get_logger().info("ONNX model expects 48 inputs, will include joint velocities and sprawl command, exclude height command from observations.")
        elif self.expected_input_dim == 51:
            self.get_logger().info("ONNX model expects 51 inputs, will include joint velocities, height command, and sprawl command in observations.")
        

        self.action_pub = self.create_publisher(Float64MultiArray, '/rl/actions', 1)
        
        # event-driven instead of internal timer
        self.create_subscription(Float64MultiArray, '/rl/observations', self.obs_cb, 1)

        self.get_logger().info("RL Policy Node Ready!")

    def obs_cb(self, msg):
        
        obs = np.array(msg.data, dtype=np.float32)

        # self.get_logger().info("Received observation, running policy...")

        if (obs[6:9] == 0.0).all(): # if user command is zero, interpolate in joint space to zero actions to avoid abrupt stops
            # self.get_logger().info("Zero command received, publishing zero actions.")
            self.interp_counter = min(self.interp_counter + 1, self.interp_steps)
            zero_command = np.zeros(12, dtype=np.float32)
            interp_command = Float64MultiArray()
            alpha = self.interp_counter / self.interp_steps
            interp_command.data = (alpha * zero_command + (1 - alpha) * self.last_command).tolist()
            self.action_pub.publish(interp_command)
            return

        self.interp_counter = 0 # reset interpolation counter when non-zero command is received
        self.last_command = obs[-12:] # save last command for interpolation

        # ONNX expects a batch dimension: (1, 36)
        input_tensor = obs.reshape(1, -1)

        # For 51D: keep all (ang_vel + gravity + command + height + sprawl + joint_pos + joint_vel + last_action)
        # For 48D: delete sprawl [12:15]
        # For 45D: delete height [9:12] and sprawl [15:18] (after joint_vel shift)
        # For 33D: delete joint_velocities [24:36] and height [9:12] and sprawl
        if self.expected_input_dim != 51:
            if self.expected_input_dim == 33:
                input_tensor = np.delete(input_tensor, np.s_[27:39], axis=1)  # delete joint_velocities
                input_tensor = np.delete(input_tensor, np.s_[12:15], axis=1)  # delete sprawl after removing joint_vel
                input_tensor = np.delete(input_tensor, np.s_[9:12], axis=1)   # delete height
            elif self.expected_input_dim == 45:
                input_tensor = np.delete(input_tensor, np.s_[12:15], axis=1)  # delete sprawl
                input_tensor = np.delete(input_tensor, np.s_[9:12], axis=1)   # delete height
            elif self.expected_input_dim == 48:
                if self.height_cmd_in_obs:
                    input_tensor = np.delete(input_tensor, np.s_[12:15], axis=1)  # delete sprawl
                else:
                    input_tensor = np.delete(input_tensor, np.s_[9:12], axis=1)   # delete height

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