import kinematics as kin
import time
import math
from gz.transport13 import Node
from gz.msgs10 import double_pb2, model_pb2, wrench_pb2

# commands frequency
TARGET_FREQ = 10.0  # Hz
DT = 1.0 / TARGET_FREQ

# topics created by joint_position_controller plugins in sdf
CMD_TOPICS = {
    "FR": {
        "hip": "/model/THex_Quadruped/joint/fr_hip_joint/0/cmd_pos",
        "knee": "/model/THex_Quadruped/joint/fr_knee_joint/0/cmd_pos",
        "foot": "/model/THex_Quadruped/joint/fr_foot_joint/0/cmd_pos",
    },
    "BR": {
        "hip": "/model/THex_Quadruped/joint/br_hip_joint/0/cmd_pos",
        "knee": "/model/THex_Quadruped/joint/br_knee_joint/0/cmd_pos",
        "foot": "/model/THex_Quadruped/joint/br_foot_joint/0/cmd_pos",
    },
    "BL": {
        "hip": "/model/THex_Quadruped/joint/bl_hip_joint/0/cmd_pos",
        "knee": "/model/THex_Quadruped/joint/bl_knee_joint/0/cmd_pos",
        "foot": "/model/THex_Quadruped/joint/bl_foot_joint/0/cmd_pos",
    },
    "FL": {
        "hip": "/model/THex_Quadruped/joint/fl_hip_joint/0/cmd_pos",
        "knee": "/model/THex_Quadruped/joint/fl_knee_joint/0/cmd_pos",
        "foot": "/model/THex_Quadruped/joint/fl_foot_joint/0/cmd_pos",
    }
}

# topics created by the force_torque sensor in world and model sdf
FORCE_TORQUE_TOPICS = {
    "FR": {
        "hip": "/fr_hip_force_torque",
        "knee": "/fr_knee_force_torque",
        "foot": "/fr_foot_force_torque",
    },
    "BR": {
        "hip": "/br_hip_force_torque",
        "knee": "/br_knee_force_torque",
        "foot": "/br_foot_force_torque",
    },
    "BL": {
        "hip": "/bl_hip_force_torque",
        "knee": "/bl_knee_force_torque",
        "foot": "/bl_foot_force_torque",
    },
    "FL": {
        "hip": "/fl_hip_force_torque",
        "knee": "/fl_knee_force_torque",
        "foot": "/fl_foot_force_torque",
    }
}

# topic created by joint_state plugin
READ_TOPIC = "/world/friction_world/model/THex_Quadruped/joint_state"

# store joint states
theta0_states = {"hip": [], "knee": [], "foot": []}  
theta1_states = {"hip": [], "knee": [], "foot": []}
theta2_states = {"hip": [], "knee": [], "foot": []}
theta3_states = {"hip": [], "knee": [], "foot": []}

# store joint commands
theta0_commands = {"hip": [], "knee": [], "foot": []}
theta1_commands = {"hip": [], "knee": [], "foot": []}
theta2_commands = {"hip": [], "knee": [], "foot": []}
theta3_commands = {"hip": [], "knee": [], "foot": []}

# store joint torques
torque0 = {"hip": [], "knee": [], "foot": []}
torque1 = {"hip": [], "knee": [], "foot": []}
torque2 = {"hip": [], "knee": [], "foot": []}
torque3 = {"hip": [], "knee": [], "foot": []}

def joint_state_cb(msg):
    
    global theta0_states, theta1_states, theta2_states, theta3_states

    joint = msg.joint[0].name
    position = msg.joint[0].axis1.position

    if joint == "fr_hip_joint" and len(theta0_states["hip"]) < len(theta0_commands["hip"]):
        theta0_states["hip"].append(position)
    elif joint == "fr_knee_joint" and len(theta0_states["knee"]) < len(theta0_commands["knee"]):
        theta0_states["knee"].append(position)
    elif joint == "fr_foot_joint" and len(theta0_states["foot"]) < len(theta0_commands["foot"]):
        theta0_states["foot"].append(position)
    elif joint == "br_hip_joint" and len(theta1_states["hip"]) < len(theta1_commands["hip"]):
        theta1_states["hip"].append(position)
    elif joint == "br_knee_joint" and len(theta1_states["knee"]) < len(theta1_commands["knee"]):
        theta1_states["knee"].append(position)
    elif joint == "br_foot_joint" and len(theta1_states["foot"]) < len(theta1_commands["foot"]):
        theta1_states["foot"].append(position)
    elif joint == "bl_hip_joint" and len(theta2_states["hip"]) < len(theta2_commands["hip"]):
        theta2_states["hip"].append(position)
    elif joint == "bl_knee_joint" and len(theta2_states["knee"]) < len(theta2_commands["knee"]):
        theta2_states["knee"].append(position)
    elif joint == "bl_foot_joint" and len(theta2_states["foot"]) < len(theta2_commands["foot"]):
        theta2_states["foot"].append(position)
    elif joint == "fl_hip_joint" and len(theta3_states["hip"]) < len(theta3_commands["hip"]):
        theta3_states["hip"].append(position)
    elif joint == "fl_knee_joint" and len(theta3_states["knee"]) < len(theta3_commands["knee"]):
        theta3_states["knee"].append(position)
    elif joint == "fl_foot_joint" and len(theta3_states["foot"]) < len(theta3_commands["foot"]):
        theta3_states["foot"].append(position)

# NOTE: This computes the magnitude of the torque about every axis and the resultant measured torques are much higher than expected from the torque analysis. Furthermore, they also exceed the joint torque limits set in the SDF, so need to figure out why that is.
def joint_torque_cb(msg, leg, joint_type): 

    global torque0, torque1, torque2, torque3

    # torque_mag = math.sqrt(msg.torque.x**2 + msg.torque.y**2 + msg.torque.z**2)
    torque_mag = abs(msg.torque.z) # only compute torque about the z-axis

    if leg == "FR" and len(torque0[joint_type]) < len(theta0_commands[joint_type]):
        torque0[joint_type].append(torque_mag)
    elif leg == "BR" and len(torque1[joint_type]) < len(theta1_commands[joint_type]):
        torque1[joint_type].append(torque_mag)
    elif leg == "BL" and len(torque2[joint_type]) < len(theta2_commands[joint_type]):
        torque2[joint_type].append(torque_mag)
    elif leg == "FL" and len(torque3[joint_type]) < len(theta3_commands[joint_type]):
        torque3[joint_type].append(torque_mag)

def main():
    # setup Gazebo node
    node = Node()
    publishers = {}

    print("Initializing Publishers...")
    
    # create a publisher for every joint
    for leg, joints in CMD_TOPICS.items():
        publishers[leg] = {}
        for joint_type, topic in joints.items():
            publishers[leg][joint_type] = node.advertise(topic, double_pb2.Double)

    print(f"Subscribing to {READ_TOPIC}...")
    node.subscribe(model_pb2.Model, READ_TOPIC, joint_state_cb)

    print("Setting up Force/Torque Subscribers...")
    for leg, joints in FORCE_TORQUE_TOPICS.items():
        for joint_type, topic in joints.items():
            node.subscribe(wrench_pb2.Wrench, topic, lambda msg, l=leg, j=joint_type: joint_torque_cb(msg, l, j))
    
    # compute trajectory
    print("Pre-computing Trajectory...")
    xyz = kin.generate_trajectory()

    # rotate trajectory to be parallel with the body for each leg
    xyz0 = kin.rotate_trajectory(0, xyz)  # FR
    xyz1 = kin.rotate_trajectory(1, xyz)  # BR
    xyz2 = kin.rotate_trajectory(2, xyz)  # BL
    xyz3 = kin.rotate_trajectory(3, xyz)  # FL

    # shift trajectory to be in each leg's workspace
    xyz0 = kin.shift_trajectory(0, xyz0)
    xyz1 = kin.shift_trajectory(1, xyz1)
    xyz2 = kin.shift_trajectory(2, xyz2)
    xyz3 = kin.shift_trajectory(3, xyz3)

    # compute joint angle commands
    theta0 = kin.inv_kin_array(xyz0, 0)  # FR
    theta1 = kin.inv_kin_array(xyz1, 1)  # BR
    theta2 = kin.inv_kin_array(xyz2, 2)  # BL
    theta3 = kin.inv_kin_array(xyz3, 3)  # FL
    
    time.sleep(2)

    print(f"Generated {len(theta0)} steps. Starting Loop at {TARGET_FREQ}Hz.")
    
    cycle = 0
    msg = double_pb2.Double()

    while True:
        print(f"=== Gait Cycle {cycle} Started ===")
        
        for step_idx in range(len(theta0[0])):

            tA = theta0[0][step_idx] # FR: A, B, C
            tB = theta0[1][step_idx]
            tC = theta0[2][step_idx]

            tG = theta1[0][step_idx] # BR: G, H, I
            tH = theta1[1][step_idx]
            tI = theta1[2][step_idx]

            tP = theta2[0][step_idx] # BL: P, Q, R
            tQ = theta2[1][step_idx]
            tR = theta2[2][step_idx]

            tJ = theta3[0][step_idx] # FL: J, K, L
            tK = theta3[1][step_idx]
            tL = theta3[2][step_idx]

            print(f"Step {step_idx}: FR({math.degrees(tA):.2f}, {math.degrees(tB):.2f}, {math.degrees(tC):.2f}) | BR({math.degrees(tG):.2f}, {math.degrees(tH):.2f}, {math.degrees(tI):.2f}) | BL({math.degrees(tP):.2f}, {math.degrees(tQ):.2f}, {math.degrees(tR):.2f}) | FL({math.degrees(tJ):.2f}, {math.degrees(tK):.2f}, {math.degrees(tL):.2f})")

            start_time = time.time()
            
            # --- PUBLISH FR LEG ---
            msg.data = tA
            publishers["FR"]["hip"].publish(msg)

            msg.data = tB 
            publishers["FR"]["knee"].publish(msg)

            msg.data = tC
            publishers["FR"]["foot"].publish(msg)

            # --- PUBLISH BR LEG ---
            msg.data = tG
            publishers["BR"]["hip"].publish(msg)

            msg.data = tH 
            publishers["BR"]["knee"].publish(msg)

            msg.data = tI
            publishers["BR"]["foot"].publish(msg)

            # --- PUBLISH BL LEG ---
            msg.data = tP
            publishers["BL"]["hip"].publish(msg)
            
            msg.data = tQ
            publishers["BL"]["knee"].publish(msg)
            
            msg.data = tR
            publishers["BL"]["foot"].publish(msg)

            # --- PUBLISH FL LEG ---
            msg.data = tJ
            publishers["FL"]["hip"].publish(msg)

            msg.data = tK
            publishers["FL"]["knee"].publish(msg)

            msg.data = tL
            publishers["FL"]["foot"].publish(msg)

            theta0_commands["hip"].append(tA)
            theta0_commands["knee"].append(tB)
            theta0_commands["foot"].append(tC)

            theta1_commands["hip"].append(tG)
            theta1_commands["knee"].append(tH)
            theta1_commands["foot"].append(tI)

            theta2_commands["hip"].append(tP)
            theta2_commands["knee"].append(tQ)
            theta2_commands["foot"].append(tR)

            theta3_commands["hip"].append(tJ)
            theta3_commands["knee"].append(tK)
            theta3_commands["foot"].append(tL)

            # SLEEP AS PER RATE
            elapsed = time.time() - start_time
            sleep_time = DT - elapsed
            
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        cycle += 1
        time.sleep(0.5) 

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Plotting control curves...")

        if len(theta0_states["hip"]) > 0:
            # ================= PLOTTING =================
            import matplotlib.pyplot as plt

            # plot subplots comparing commands vs states for each joint of each leg
            legs = ["FR", "BR", "BL", "FL"]
            joint_types = ["hip", "knee", "foot"]

            theta_states = [theta0_states, theta1_states, theta2_states, theta3_states]
            theta_commands = [theta0_commands, theta1_commands, theta2_commands, theta3_commands]

            fig, axes = plt.subplots(4, 3, figsize=(15, 12))
            fig.suptitle("All Joints: Command vs State", fontsize=16)

            for leg_ind in range(len(legs)):
                leg = legs[leg_ind]
                states = theta_states[leg_ind]
                commands = theta_commands[leg_ind]

                for joint_ind, joint_type in enumerate(joint_types):
                    ax = axes[leg_ind, joint_ind]
                    ax.plot([math.degrees(x) for x in commands[joint_type]], label="Command", linestyle='--', linewidth=2)
                    ax.plot([math.degrees(x) for x in states[joint_type]], label="State", linestyle='-', linewidth=1)
                    ax.set_title(f"{leg} {joint_type}")
                    ax.set_xlabel("Time Step")
                    ax.set_ylabel("Angle (deg)")
                    ax.legend()
                    ax.grid(True, alpha=0.3)

            plt.tight_layout()
            plt.savefig("joint_commands_vs_states.png")

            # plot torques for all joints
            torques = [torque0, torque1, torque2, torque3]

            # Find global min and max torque values for consistent y-axis limits
            all_torque_values = []
            for joint_torques in torques:
                for joint_type in joint_types:
                    all_torque_values.extend(joint_torques[joint_type])
            
            if all_torque_values:
                y_min = min(all_torque_values)
                y_max = max(all_torque_values)
                # Add some padding to the bounds
                y_padding = (y_max - y_min) * 0.1
                y_min -= y_padding
                y_max += y_padding
            else:
                y_min, y_max = 0, 1

            fig2, axes2 = plt.subplots(4, 3, figsize=(15, 12))
            fig2.suptitle("All Joints: Torque Magnitude", fontsize=16)

            for leg_ind in range(len(legs)):
                leg = legs[leg_ind]
                joint_torques = torques[leg_ind]

                for joint_ind, joint_type in enumerate(joint_types):
                    ax = axes2[leg_ind, joint_ind]
                    if len(joint_torques[joint_type]) > 0:
                        ax.plot(joint_torques[joint_type], label="Torque", linestyle='-', linewidth=1.5, color='r')
                    ax.axhline(y=0.3*0.9414, color='g', linestyle=':', linewidth=2, label="30% Stall Torque")
                    ax.set_title(f"{leg} {joint_type}")
                    ax.set_xlabel("Time Step")
                    ax.set_ylabel("Torque Magnitude (N⋅m)")
                    ax.set_ylim(y_min, y_max)
                    ax.legend()
                    ax.grid(True, alpha=0.3)

            plt.tight_layout()
            plt.savefig("joint_torques.png")
            plt.show()
            
            # ================= CSV EXPORT =================
            import csv

            # Save data to CSV
            print("Saving data to CSV...")
            
            # Save joint commands and states
            with open('joint_commands_vs_states.csv', 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                
                # Write header
                header = ['Time_Step']
                for leg in legs:
                    for joint_type in joint_types:
                        header.extend([f'{leg}_{joint_type}_command', f'{leg}_{joint_type}_state'])
                writer.writerow(header)
                
                # Write data rows
                max_len = max(len(theta0_commands["hip"]), len(theta1_commands["hip"]), len(theta2_commands["hip"]), len(theta3_commands["hip"]))
                
                for i in range(max_len):
                    row = [i]
                    for leg_ind, leg in enumerate(legs):
                        commands = theta_commands[leg_ind]
                        states = theta_states[leg_ind]
                        for joint_type in joint_types:
                            cmd_val = math.degrees(commands[joint_type][i]) if i < len(commands[joint_type]) else ''
                            state_val = math.degrees(states[joint_type][i]) if i < len(states[joint_type]) else ''
                            row.extend([cmd_val, state_val])
                    writer.writerow(row)
            
            print("Joint data saved to joint_commands_vs_states.csv")
            
            # Save torque data
            with open('joint_torques.csv', 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                
                # Write header
                header = ['Time_Step']
                for leg in legs:
                    for joint_type in joint_types:
                        header.append(f'{leg}_{joint_type}_torque')
                writer.writerow(header)
                
                # Write data rows
                max_len = max(len(torque0["hip"]), len(torque1["hip"]), len(torque2["hip"]), len(torque3["hip"]))
                
                for i in range(max_len):
                    row = [i]
                    for joint_torques in torques:
                        for joint_type in joint_types:
                            torque_val = joint_torques[joint_type][i] if i < len(joint_torques[joint_type]) else ''
                            row.append(torque_val)
                    writer.writerow(row)
            
            print("Torque data saved to joint_torques.csv")