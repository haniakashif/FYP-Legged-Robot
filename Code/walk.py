import sys
import time
import math
import numpy as np
from gz.transport13 import Node
from gz.msgs10 import double_pb2, model_pb2

# ================================================================
# ---------------------- Configuration ---------------------------
# ================================================================

# Simulation Speed
TARGET_FREQ = 10.0  # Hz
DT = 1.0 / TARGET_FREQ

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

READ_TOPIC = "/world/friction_world/model/THex_Quadruped/joint_state"

# store joint states
theta0_states = {"hip": [], "knee": [], "foot": []}  
theta1_states = {"hip": [], "knee": [], "foot": []}
theta2_states = {"hip": [], "knee": [], "foot": []}
theta3_states = {"hip": [], "knee": [], "foot": []}

theta0_commands = {"hip": [], "knee": [], "foot": []}
theta1_commands = {"hip": [], "knee": [], "foot": []}
theta2_commands = {"hip": [], "knee": [], "foot": []}
theta3_commands = {"hip": [], "knee": [], "foot": []}

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

# Math Constants
PI = math.pi

# Link Lengths
L1 = 3.1
L2 = 4.5
L3 = 3.0
L4 = 9.0

# Trajectory Constants
T_STALL = 2
NUM_DATA_POINTS = 20
SWING_FACTOR = 1/4 # SWING_FACTOR of the points are for swing phase
STANCE_FACTOR = 1 - SWING_FACTOR

X = 8
S = -11
A = 3
T = 8

P1 = [-T/2, S]
P2 = [0, S + 2*A]
P3 = [T/2, S]

X_OFFSET = -2
Y_OFFSET = 5

# Helpers

def inv_kin(x, y, z, leg_ind, step=0):

    if leg_ind < 2:
        theta3 = PI/4

        theta1 = math.atan2(y, x)

        LHS = ((x * math.cos(theta1) + y * math.sin(theta1) - L1)**2 + z**2 - L2**2 - L3**2 - L4**2 - 2*L2*L3*math.cos(theta3)) / (2*L4)
        A_1 = L2*math.cos(theta3) + L3
        B_1 = L2*math.sin(theta3)
        phi1 = math.atan2(A_1, B_1)
        a1 = math.sqrt(A_1**2 + B_1**2)
        # a1 = A_1/math.sin(phi1)
        theta4 = phi1 - math.asin(LHS / a1)

        A_2 = L2 + L3*math.cos(theta3) + L4*math.cos(theta3 + theta4)
        B_2 = L4*math.sin(theta3 + theta4) + L3*math.sin(theta3)
        phi2 = math.atan2(B_2, A_2)
        a2 = math.sqrt(A_2**2 + B_2**2)
        # a2 = B_2/math.sin(phi2)
        theta2 = math.asin(z / a2) + phi2

    else:
        theta3 = -PI/4

        theta1 = math.atan2(y, x)

        LHS = ((x * math.cos(theta1) + y * math.sin(theta1) - L1)**2 + z**2 - L2**2 - L3**2 - L4**2 - 2*L2*L3*math.cos(theta3)) / (2*L4)
        A_1 = L2*math.cos(theta3) + L3
        B_1 = L2*math.sin(theta3)
        phi1 = math.atan2(B_1, A_1)
        a1 = math.sqrt(A_1**2 + B_1**2)
        # a1 = B_1/math.sin(phi1)
        theta4 = -1*math.acos(LHS / a1) - phi1

        A_2 = L2 + L3*math.cos(theta3) + L4*math.cos(theta3 + theta4)
        B_2 = L4*math.sin(theta3 + theta4) + L3*math.sin(theta3)
        phi2 = math.atan2(A_2, B_2)
        a2 = math.sqrt(A_2**2 + B_2**2)
        # a2 = A_2/math.sin(phi2)
        theta2 = math.acos(z/a2) - phi2

        
    # clamp angles between -180 to 180 degrees
    theta1 = (theta1 + PI) % (2*PI) - PI
    theta2 = (theta2 + PI) % (2*PI) - PI
    theta4 = (theta4 + PI) % (2*PI) - PI

    LEGS = {0: "FR", 1: "BR", 2: "BL", 3: "FL"}

    if theta1 < -PI/4 or theta1 > PI/4:
        print(f"Warning: For {LEGS[leg_ind]}, point {step}: theta1 out of bounds: {math.degrees(theta1)}") 
    if theta2 < -PI/2 or theta2 > PI/2:
        print(f"Warning: For {LEGS[leg_ind]}, point {step}: theta2 out of bounds: {math.degrees(theta2)}")
    if theta4 < -PI/2 or theta4 > PI/2:
        print(f"Warning: For {LEGS[leg_ind]}, point {step}: theta4 out of bounds: {math.degrees(theta4)}")
    return theta1, theta2, theta4

def inv_kin_array(xyz, leg_ind):
    theta1s = []
    theta2s = []
    theta4s = []

    step = 0
    for (x, y, z) in xyz:
        t1, t2, t4 = inv_kin(x, y, z, leg_ind, step)
        theta1s.append(t1)
        theta2s.append(t2)
        theta4s.append(t4)
        step += 1

    return theta1s, theta2s, theta4s

def generate_trajectory():
    points = []

    t = np.linspace(0, 1, int(NUM_DATA_POINTS*SWING_FACTOR), endpoint=True)

    for i in range(int(NUM_DATA_POINTS*SWING_FACTOR)):
        x = X 
        y = ((1 - t[i])**2)*P1[0] + 2*(1 - t[i])*t[i]*P2[0] + (t[i]**2)*P3[0]
        z = ((1 - t[i])**2)*P1[1] + 2*(1 - t[i])*t[i]*P2[1] + (t[i]**2)*P3[1]
        points.append((x, y, z)) 

    for i in range(T_STALL):
        x, y, z = points[-1]
        points.append((x, y, z))

    y_stance = np.linspace(T/2, -T/2, int(NUM_DATA_POINTS*STANCE_FACTOR), endpoint=True)

    for i in range(int(NUM_DATA_POINTS*STANCE_FACTOR)):
        x = X 
        y = y_stance[i]
        z = S
        points.append((x, y, z))

    for i in range(T_STALL):
        x, y, z = points[-1]
        points.append((x, y, z))
        
    return points

def rotate_trajectory(leg_ind, xyzK):
    
    beta = [-PI/4, PI/4, -PI/4, PI/4]  # FR, BR, BL, FL
    y_pos_signs = [1, 1, -1, -1] # left legs move in opposite y direction to right legs
    y_offset_signs = [1, -1, 1, -1]

    angle = beta[leg_ind]
    cosB = math.cos(angle)
    sinB = math.sin(angle)

    rotated_points = []

    for i in range(len(xyzK)):
        x_old, y_old, z_old = xyzK[i]
        y_old = y_old * y_pos_signs[leg_ind]

        x = (x_old + X_OFFSET)*cosB - (y_old + Y_OFFSET*y_offset_signs[leg_ind])*sinB
        y = (x_old + X_OFFSET)*sinB + (y_old + Y_OFFSET*y_offset_signs[leg_ind])*cosB
        z = z_old

        rotated_points.append((x, y, z))
    
    return rotated_points

def shift_trajectory(leg_ind, xyzK):

    schedule = [0, 2, 1, 3] # Order of swing: FR, BL, BR, FL
    # schedule = [3, 1, 2, 0] # Order of swing: FL, BR, BL, FR
    # schedule = [0, 1, 3, 2] # Order of swing: FR, BL, BR, FL

    for swing in range(len(schedule)):
        if leg_ind == schedule[swing]:
            xyzK_copy = xyzK.copy()
            xyzK_ind = 0
            for i in range(int(NUM_DATA_POINTS + 2*T_STALL - swing*(NUM_DATA_POINTS*SWING_FACTOR)), int(NUM_DATA_POINTS + 2*T_STALL)):
                x, y, z = xyzK_copy[i]
                xyzK[xyzK_ind] = (x, y, z)
                xyzK_ind += 1

            for i in range(0, int(NUM_DATA_POINTS + 2*T_STALL - swing*(NUM_DATA_POINTS*SWING_FACTOR))):
                x, y, z = xyzK_copy[i]
                xyzK[xyzK_ind] = (x, y, z)
                xyzK_ind += 1

            return xyzK

def main():
    # 1. Setup Gazebo Node
    node = Node()
    publishers = {}

    print("Initializing Publishers...")
    
    # Create a publisher for EVERY joint in the map
    for leg, joints in CMD_TOPICS.items():
        publishers[leg] = {}
        for joint_type, topic in joints.items():
            # advertise(topic, message_type)
            publishers[leg][joint_type] = node.advertise(topic, double_pb2.Double)
    
    print(f"Subscribing to {READ_TOPIC}...")
    node.subscribe(model_pb2.Model, READ_TOPIC, joint_state_cb)
    
    # 2. Pre-compute Trajectory
    print("Pre-computing Trajectory...")
    xyz = generate_trajectory()

    xyz0 = rotate_trajectory(0, xyz)  # FR
    xyz1 = rotate_trajectory(1, xyz)  # BR
    xyz2 = rotate_trajectory(2, xyz)  # BL
    xyz3 = rotate_trajectory(3, xyz)  # FL

    xyz0 = shift_trajectory(0, xyz0)
    xyz1 = shift_trajectory(1, xyz1)
    xyz2 = shift_trajectory(2, xyz2)
    xyz3 = shift_trajectory(3, xyz3)

    theta0 = inv_kin_array(xyz0, 0)  # FR
    theta1 = inv_kin_array(xyz1, 1)  # BR
    theta2 = inv_kin_array(xyz2, 2)  # BL
    theta3 = inv_kin_array(xyz3, 3)  # FL
    
    time.sleep(2)

    print(f"Generated {len(theta0)} steps. Starting Loop at {TARGET_FREQ}Hz.")
    
    # 3. Control Loop
    cycle = 0
    msg = double_pb2.Double() # Reusable message object

    while True:
        print(f"=== Gait Cycle {cycle} Started ===")
        
        for step_idx in range(len(theta0[0])):
            # Extract joint angles for this step
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
            import matplotlib.pyplot as plt

            # plot subplots comparing commands vs states for each joint of each leg
            legs = ["FR", "BR", "BL", "FL"]
            joint_types = ["hip", "knee", "foot"]
            theta_states = [theta0_states, theta1_states, theta2_states, theta3_states]
            theta_commands = [theta0_commands, theta1_commands, theta2_commands, theta3_commands]

            # Create a single figure with 4x3 subplots (4 legs x 3 joints)
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
            plt.show()
