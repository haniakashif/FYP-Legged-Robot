import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import onnxruntime as ort

class RLPolicy(Node):
    def __init__(self):
        super().__init__('rl_policy')

        self.interp_steps = 30 # number of steps to interpolate the command to zero when user command goes to zero, to avoid abrupt stops
        self.interp_counter = 0
        self.last_command = np.zeros(12, dtype=np.float32)

        path_regular = "../Policies/2026-04-20_09-49-36_v1.onnx"
        path_crawl   = "../Policies/pa_2026-04-28_01-01-12_v1.onnx"
        path_squeeze = "../Policies/pa_2026-05-11_09-17-58_v1.onnx"

        self.crawl_threshold = 0.08 # height command threshold to switch to crawl policy
        self.squeeze_threshold = 0.35 # sprawl command threshold to switch to squeeze policy

        self.get_logger().info("Loading ONNX models into memory...")
        
        self.sess_regular, self.name_regular, self.dim_regular = self.load_model(path_regular, "Regular")
        self.sess_crawl, self.name_crawl, self.dim_crawl = self.load_model(path_crawl, "Crawl")
        self.sess_squeeze, self.name_squeeze, self.dim_squeeze = self.load_model(path_squeeze, "Squeeze")

        self.action_pub = self.create_publisher(Float64MultiArray, '/rl/actions', 1)
        self.create_subscription(Float64MultiArray, '/rl/observations', self.obs_cb, 1)

        self.current_state = "regular" # Used to track and log state changes

        self.get_logger().info("Policy node ready!")

    def load_model(self, path, label):
        try:
            session = ort.InferenceSession(path)
            name = session.get_inputs()[0].name
            dim = session.get_inputs()[0].shape[1]
            self.get_logger().info(f"[{label} Policy] Loaded. Expected input dim: {dim}")
            return session, name, dim
        except Exception as e:
            self.get_logger().error(f"Failed to load {label} ONNX: {e}")
            raise

    def slice_observation(self, obs, expected_dim, policy_type):
        input_tensor = obs.reshape(1, -1) 
        
        if expected_dim == 51:
            return input_tensor
            
        elif expected_dim == 48:
            if policy_type == "crawl":
                # delete sprawl
                return np.delete(input_tensor, np.s_[12:15], axis=1)
            elif policy_type == "squeeze":
                # delete height
                return np.delete(input_tensor, np.s_[9:12], axis=1)
            else:
                self.get_logger().error(f"Unexpected policy type for 48D model: {policy_type}")
                return input_tensor
                
        elif expected_dim == 45:
            input_tensor = np.delete(input_tensor, np.s_[12:15], axis=1)
            input_tensor = np.delete(input_tensor, np.s_[9:12], axis=1)
            return input_tensor
            
        elif expected_dim == 33:
            input_tensor = np.delete(input_tensor, np.s_[27:39], axis=1)
            input_tensor = np.delete(input_tensor, np.s_[12:15], axis=1)
            input_tensor = np.delete(input_tensor, np.s_[9:12], axis=1)
            return input_tensor

        return input_tensor

    def obs_cb(self, msg):
        obs = np.array(msg.data, dtype=np.float32)

        if (obs[6:9] == 0.0).all():
            self.interp_counter = min(self.interp_counter + 1, self.interp_steps)
            zero_command = np.zeros(12, dtype=np.float32)
            interp_command = Float64MultiArray()
            alpha = self.interp_counter / self.interp_steps
            interp_command.data = (alpha * zero_command + (1 - alpha) * self.last_command).tolist()
            self.action_pub.publish(interp_command)
            return

        self.interp_counter = 0
        self.last_command = obs[-12:]

        # obtain actual height values from the scaled commands
        current_target_height = obs[9] / 10.0
        current_target_sprawl = obs[12] / 10.0

        if current_target_height < self.crawl_threshold:
            active_sess = self.sess_crawl
            input_name = self.name_crawl
            expected_dim = self.dim_crawl
            policy_type = "crawl"
            
        elif current_target_sprawl < self.squeeze_threshold:
            active_sess = self.sess_squeeze
            input_name = self.name_squeeze
            expected_dim = self.dim_squeeze
            policy_type = "squeeze"
            
        else:
            active_sess = self.sess_regular
            input_name = self.name_regular
            expected_dim = self.dim_regular
            policy_type = "regular"

        if policy_type != self.current_state:
            self.get_logger().info(f"Switching policy to -> {policy_type.upper()}")
            self.current_state = policy_type

        input_tensor = self.slice_observation(obs, expected_dim, policy_type)

        if input_tensor.shape[1] != expected_dim:
            self.get_logger().error(f"Shape mismatch! Expected {expected_dim}, got {input_tensor.shape[1]}")
            return

        outputs = active_sess.run(None, {input_name: input_tensor})
        raw_actions = outputs[0][0] 

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