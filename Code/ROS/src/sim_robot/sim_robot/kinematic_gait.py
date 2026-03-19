import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
import math
import matplotlib.pyplot as plt
import csv

# Import your library
from . import kinematics as kin

class KinematicGait(Node):
    def __init__(self):
        super().__init__('kinematic_gait')
        
        # --- 1. SETUP PARAMETERS ---
        self.target_freq = 10
        self.dt = 1.0 / self.target_freq
        
        # --- 2. DATA STORAGE ---
        # Structure matches your original lists but accessible via self
        # Index 0=FR, 1=BR, 2=BL, 3=FL
        self.theta_states = [
            {"hip": [], "knee": [], "foot": []}, 
            {"hip": [], "knee": [], "foot": []}, 
            {"hip": [], "knee": [], "foot": []}, 
            {"hip": [], "knee": [], "foot": []}
        ]
        
        self.theta_commands = [
            {"hip": [], "knee": [], "foot": []}, 
            {"hip": [], "knee": [], "foot": []}, 
            {"hip": [], "knee": [], "foot": []}, 
            {"hip": [], "knee": [], "foot": []}
        ]
        
        self.torques = [
            {"hip": [], "knee": [], "foot": []}, 
            {"hip": [], "knee": [], "foot": []}, 
            {"hip": [], "knee": [], "foot": []}, 
            {"hip": [], "knee": [], "foot": []}
        ]

        self.legs = ["FR", "BR", "BL", "FL"]
        self.joint_types = ["hip", "knee", "foot"]

        # --- 3. CREATE PUBLISHERS & SUBSCRIBERS ---
        self.pubs = {}
        
        # Create Publishers for Commands
        for leg in self.legs:
            for joint in self.joint_types:
                topic = f'/{leg.lower()}_{joint}/command'
                self.pubs[f'{leg}_{joint}'] = self.create_publisher(Float64, topic, 1)
        
        # Subscribe to Joint States
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 1)

        # Subscribe to Force/Torque Sensors
        for i, leg in enumerate(self.legs):
            for joint in self.joint_types:
                topic = f'/{leg.lower()}_{joint}/force_torque'
                self.create_subscription(
                    WrenchStamped, 
                    topic, 
                    lambda msg, l=i, j=joint: self.joint_torque_cb(msg, l, j), 
                    1
                )

        # --- 4. PRE-COMPUTE TRAJECTORY ---
        self.get_logger().info("Pre-computing Trajectory...")
        xyz = kin.generate_trajectory()
        
        xyz0 = kin.shift_trajectory(0, kin.rotate_trajectory(0, xyz))
        xyz1 = kin.shift_trajectory(1, kin.rotate_trajectory(1, xyz))
        xyz2 = kin.shift_trajectory(2, kin.rotate_trajectory(2, xyz))
        xyz3 = kin.shift_trajectory(3, kin.rotate_trajectory(3, xyz))

        # Store targets in the same structure as your theta arrays
        self.theta_targets = [
            kin.inv_kin_array(xyz0, 0), # FR
            kin.inv_kin_array(xyz1, 1), # BR
            kin.inv_kin_array(xyz2, 2), # BL
            kin.inv_kin_array(xyz3, 3)  # FL
        ]
        
        self.steps_len = len(self.theta_targets[0][0])
        self.current_step = 0
        self.cycle_count = 0
        self.get_logger().info(f"Generated {self.steps_len} steps per cycle.")

        # --- 5. START LOOP ---
        self.timer = self.create_timer(self.dt, self.timer_callback)

    # --- CALLBACKS ---

    def timer_callback(self):

        # Log cycle start
        if self.current_step == 0:
            self.get_logger().info(f"\n\n=== Gait Cycle {self.cycle_count} Started ===")

        # Publish commands for all 4 legs
        for leg_idx, leg_name in enumerate(self.legs):
            # Get targets for this leg's 3 joints (A, B, C)
            t_hip = self.theta_targets[leg_idx][0][self.current_step]
            t_knee = self.theta_targets[leg_idx][1][self.current_step]
            t_foot = self.theta_targets[leg_idx][2][self.current_step]

            # Publish
            self.publish_command(leg_name, 'hip', t_hip)
            self.publish_command(leg_name, 'knee', t_knee)
            self.publish_command(leg_name, 'foot', t_foot)
            
            # Log commands
            self.get_logger().info(
                f"\nLeg {leg_name} Step {self.current_step}: "
                f"Hip={math.degrees(t_hip):.2f}°, "
                f"Knee={math.degrees(t_knee):.2f}°, "
                f"Foot={math.degrees(t_foot):.2f}°"
            )

            # Store Command Data (For plotting/CSV)
            self.theta_commands[leg_idx]['hip'].append(t_hip)
            self.theta_commands[leg_idx]['knee'].append(t_knee)
            self.theta_commands[leg_idx]['foot'].append(t_foot)

        # Increment Step
        self.current_step += 1
        if self.current_step >= self.steps_len:
            self.current_step = 0
            self.cycle_count += 1

    def publish_command(self, leg, joint, value):
        msg = Float64()
        msg.data = float(value)
        self.pubs[f'{leg}_{joint}'].publish(msg)

    def joint_state_cb(self, msg):
        for i, full_name in enumerate(msg.name):
            # Parse name "fr_hip_joint" -> leg="FR", joint="hip"
            parts = full_name.split('_') 
            if len(parts) < 2: 
                print("Unexpected joint name format:", full_name)
                continue
            
            leg_code = parts[0].upper() # FR
            joint_type = parts[1]       # hip

            if leg_code in self.legs:
                leg_idx = self.legs.index(leg_code)
                # Check consistency
                if len(self.theta_states[leg_idx][joint_type]) < len(self.theta_commands[leg_idx][joint_type]):
                    self.theta_states[leg_idx][joint_type].append(msg.position[i])

    def joint_torque_cb(self, msg, leg_idx, joint_type):
        # Calculate torque magnitude (Z-axis only)
        torque_mag = abs(msg.wrench.torque.z)

        if len(self.torques[leg_idx][joint_type]) < len(self.theta_commands[leg_idx][joint_type]):
            self.torques[leg_idx][joint_type].append(torque_mag)

    # --- PLOTTING & CSV LOGIC (Replicated from Original) ---
    def save_data(self):
        print("\nProcessing data...")
        self.plot_graphs()
        self.export_csvs()

    def plot_graphs(self):
        print("Plotting control curves...")
        
        # 1. Plot Commands vs States
        fig, axes = plt.subplots(4, 3, figsize=(15, 12))
        fig.suptitle("All Joints: Command vs State", fontsize=16)

        for leg_ind, leg in enumerate(self.legs):
            for joint_ind, joint_type in enumerate(self.joint_types):
                ax = axes[leg_ind, joint_ind]
                
                # Convert to degrees for plotting
                cmds = [math.degrees(x) for x in self.theta_commands[leg_ind][joint_type]]
                states = [math.degrees(x) for x in self.theta_states[leg_ind][joint_type]]
                
                ax.plot(cmds, label="Command", linestyle='--', linewidth=2)
                ax.plot(states[:len(cmds)], label="State", linestyle='-', linewidth=1)
                
                ax.set_title(f"{leg} {joint_type}")
                ax.set_xlabel("Time Step")
                ax.set_ylabel("Angle (deg)")
                ax.legend()
                ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig("joint_commands_vs_states.png")
        print("Saved joint_commands_vs_states.png")

        # 2. Plot Torques
        fig2, axes2 = plt.subplots(4, 3, figsize=(15, 12))
        fig2.suptitle("All Joints: Torque Magnitude", fontsize=16)

        # Calculate Y limits
        all_vals = []
        for leg_data in self.torques:
            for j_type in self.joint_types:
                all_vals.extend(leg_data[j_type])
        
        if all_vals:
            y_min, y_max = min(all_vals), max(all_vals)
            pad = (y_max - y_min) * 0.1
            y_min -= pad
            y_max += pad
        else:
            y_min, y_max = 0, 1

        for leg_ind, leg in enumerate(self.legs):
            for joint_ind, joint_type in enumerate(self.joint_types):
                ax = axes2[leg_ind, joint_ind]
                data = self.torques[leg_ind][joint_type]
                
                if data:
                    ax.plot(data, label="Torque", linestyle='-', linewidth=1.5, color='r')
                
                ax.axhline(y=0.3*0.9414, color='g', linestyle=':', linewidth=2, label="30% Stall Torque")
                ax.set_title(f"{leg} {joint_type}")
                ax.set_ylim(y_min, y_max)
                ax.set_xlabel("Time Step")
                ax.set_ylabel("Torque Magnitude (N⋅m)")
                ax.legend()
                ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig("joint_torques.png")
        print("Saved joint_torques.png")

    def export_csvs(self):
        print("Saving data to CSV...")

        # --- CSV 1: Joint Commands vs States ---
        with open('joint_commands_vs_states.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # Header
            header = ['Time_Step']
            for leg in self.legs:
                for joint_type in self.joint_types:
                    header.extend([f'{leg}_{joint_type}_command', f'{leg}_{joint_type}_state'])
            writer.writerow(header)
            
            # Data Rows
            # Use the length of commands as reference
            max_len = len(self.theta_commands[0]['hip'])
            
            for i in range(max_len):
                row = [i]
                for leg_ind, leg in enumerate(self.legs):
                    for joint_type in self.joint_types:
                        # Command (Degrees)
                        try: 
                            cmd_val = math.degrees(self.theta_commands[leg_ind][joint_type][i])
                        except IndexError: 
                            cmd_val = ''
                        
                        # State (Degrees)
                        try: 
                            state_val = math.degrees(self.theta_states[leg_ind][joint_type][i])
                        except IndexError: 
                            state_val = ''
                            
                        row.extend([cmd_val, state_val])
                writer.writerow(row)
        
        print("Joint data saved to joint_commands_vs_states.csv")

        # --- CSV 2: Torques ---
        with open('joint_torques.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # Header
            header = ['Time_Step']
            for leg in self.legs:
                for joint_type in self.joint_types:
                    header.append(f'{leg}_{joint_type}_torque')
            writer.writerow(header)
            
            # Data Rows
            max_len_torque = len(self.torques[0]['hip']) # Might differ slightly from commands
            # We use max_len from commands to keep time steps aligned, filling empty if needed
            
            for i in range(max_len):
                row = [i]
                for leg_ind, leg in enumerate(self.legs):
                    for joint_type in self.joint_types:
                        try:
                            torque_val = self.torques[leg_ind][joint_type][i]
                        except IndexError:
                            torque_val = ''
                        row.append(torque_val)
                writer.writerow(row)

        print("Torque data saved to joint_torques.csv")

def main(args=None):
    rclpy.init(args=args)
    node = KinematicGait()
    
    try:
        # Blocks here until Ctrl+C
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Run saving logic before shutdown
        node.save_data()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()