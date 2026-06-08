import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import onnxruntime as ort

class RLPolicy(Node):
    def __init__(self):
        super().__init__('rl_policy')

        self.interp_steps = 30 
        self.interp_counter = 0
        self.last_command = np.zeros(12, dtype=np.float32)

        # --- 1. DEFINE MODEL PATHS ---
        path_regular = "../Policies/2026-04-20_09-49-36_v1.onnx"
        path_crawl   = "../Policies/pa_2026-04-28_01-01-12_v1.onnx"
        path_squeeze = "../Policies/pa_2026-05-11_05-16-41_v1.onnx"

        self.get_logger().info("Loading Multi-Policy ONNX models into memory...")
        
        # --- 2. LOAD SESSIONS ---
        self.sess_regular, self.name_regular, self.dim_regular = self.load_model(path_regular, "Regular")
        self.sess_crawl, self.name_crawl, self.dim_crawl = self.load_model(path_crawl, "Crawl")
        self.sess_squeeze, self.name_squeeze, self.dim_squeeze = self.load_model(path_squeeze, "Squeeze")

        self.action_pub = self.create_publisher(Float64MultiArray, '/rl/actions', 1)
        self.create_subscription(Float64MultiArray, '/rl/observations', self.obs_cb, 1)

        # --- STATE MACHINE DEBOUNCING ---
        self.current_state = "regular" 
        self.pending_state = "regular"
        self.debounce_counter = 0
        self.debounce_threshold = 10  # Must request the same state for 10 consecutive frames

        self.get_logger().info("Multi-Policy Switcher Node Ready!")

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
        if expected_dim == 51: return input_tensor
        elif expected_dim == 48:
            if policy_type == "crawl": return np.delete(input_tensor, np.s_[12:15], axis=1)
            elif policy_type == "squeeze": return np.delete(input_tensor, np.s_[9:12], axis=1)
            else: return np.delete(input_tensor, np.s_[12:15], axis=1)
        elif expected_dim == 45:
            input_tensor = np.delete(input_tensor, np.s_[12:15], axis=1)
            return np.delete(input_tensor, np.s_[9:12], axis=1)
        elif expected_dim == 33:
            input_tensor = np.delete(input_tensor, np.s_[27:39], axis=1)
            input_tensor = np.delete(input_tensor, np.s_[12:15], axis=1)
            return np.delete(input_tensor, np.s_[9:12], axis=1)
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

        # Decode spatial constraints
        current_target_height = obs[9] / 10.0
        current_target_sprawl = obs[12] / 10.0

        # --- HYSTERESIS / DEBOUNCE LOGIC ---
        requested_state = "regular"
        if current_target_height < 0.08:
            requested_state = "crawl"
        elif current_target_sprawl < 0.35:
            requested_state = "squeeze"

        # If the requested state matches the pending state, increment counter
        if requested_state == self.pending_state:
            if requested_state != self.current_state:
                self.debounce_counter += 1
                
                # If we've held this request long enough, execute the switch!
                if self.debounce_counter >= self.debounce_threshold:
                    self.current_state = requested_state
                    self.get_logger().info(f"Confirmed Obstacle: Hot-swapping to -> {self.current_state.upper()} policy")
        else:
            # Reset counter if the signal bounces
            self.pending_state = requested_state
            self.debounce_counter = 0

        # Assign active session based on confirmed current_state
        if self.current_state == "crawl":
            active_sess, input_name, expected_dim = self.sess_crawl, self.name_crawl, self.dim_crawl
        elif self.current_state == "squeeze":
            active_sess, input_name, expected_dim = self.sess_squeeze, self.name_squeeze, self.dim_squeeze
        else:
            active_sess, input_name, expected_dim = self.sess_regular, self.name_regular, self.dim_regular

        input_tensor = self.slice_observation(obs, expected_dim, self.current_state)

        if input_tensor.shape[1] != expected_dim:
            return

        outputs = active_sess.run(None, {input_name: input_tensor})
        
        out_msg = Float64MultiArray()
        out_msg.data = outputs[0][0].tolist()
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
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()