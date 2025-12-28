import kinematics as kin
import numpy as np 
import math
import matplotlib.pyplot as plt

PI = math.pi

WEIGHT = 0 # temporary
L1 = kin.L1
L2 = kin.L2
L3 = kin.L3
L4 = kin.L4

def leg_to_body(leg_ind, xyz):
    x, y, z = xyz
    # need to measure and include the transforms from leg base frame to body frame
    pass

def leg_to_body_array(leg_ind, xyz_arr):
    new_xyz_arr = []
    for xyz in xyz_arr:
        new_xyz = leg_to_body(leg_ind, xyz)
        new_xyz_arr.append(new_xyz)
    return new_xyz_arr

def compute_forces(xy1, xy2, xy3): # based on Garcia et al. 
    W = np.array([[0], [0], [-WEIGHT]])    
    x1, y1 = xy1
    x2, y2 = xy2
    x3, y3 = xy3
    A = np.array([[x1, x2, x3],
                    [y1, y2, y3],
                    [1, 1, 1]])
    try:
        Ainv = np.linalg.inv(A)
    except: 
        Ainv = np.linalg.pinv(A) 
    F = Ainv @ W 
    return F 

def compute_J(side, thetas): 
    # thetas: [hip, knee, foot]
    # side: "L" or "R"
    theta1 = thetas[0]
    theta2 = thetas[1]
    theta3 = PI/4 if side == "R" else -PI/4
    theta4 = thetas[2]

    dxdt1 = -math.sin(theta1)*(L1 + L4*math.cos(theta3 - theta2 + theta4) + L2*math.cos(theta2) + L3*math.cos(theta2 - theta3))
    dxdt2 = L4*math.cos(theta1)*math.sin(theta3 - theta2 + theta4) - L2*math.cos(theta1)*math.sin(theta2) - L3*math.cos(theta1)*math.sin(theta2 - theta3)
    dxdt4 = -L4*math.cos(theta1)*math.sin(theta3 - theta2 + theta4)

    dydt1 = math.cos(theta1)*(L1 + L4*math.cos(theta3 - theta2 + theta4) + L2*math.cos(theta2) + L3*math.cos(theta2 - theta3))
    dydt2 = L4*math.sin(theta1)*math.sin(theta3 - theta2 + theta4) - L2*math.sin(theta1)*math.sin(theta2) - L3*math.sin(theta1)*math.sin(theta2 - theta3)
    dydt4 = -L4*math.sin(theta1)*math.sin(theta3 - theta2 + theta4)

    dzdt1 = 0
    dzdt2 = L2*math.cos(theta2) + L4*math.cos(theta3 - theta2 + theta4) + L3*math.cos(theta2 - theta3)
    dzdt4 = -L4*math.cos(theta3 - theta2 + theta4)

    J = np.array([[dxdt1, dxdt2, dxdt4], 
                    [dydt1, dydt2, dydt4], 
                    [dzdt1, dzdt2, dzdt4]])

    return J

def main():
    xyz = kin.generate_trajectory()

    xyz0 = kin.rotate_trajectory(0, xyz)  # FR
    xyz1 = kin.rotate_trajectory(1, xyz)  # BR
    xyz2 = kin.rotate_trajectory(2, xyz)  # BL
    xyz3 = kin.rotate_trajectory(3, xyz)  # FL

    xyz0 = kin.shift_trajectory(0, xyz0)
    xyz1 = kin.shift_trajectory(1, xyz1)
    xyz2 = kin.shift_trajectory(2, xyz2)
    xyz3 = kin.shift_trajectory(3, xyz3)

    xyz0 = leg_to_body_array(0, xyz0)
    xyz1 = leg_to_body_array(1, xyz1)
    xyz2 = leg_to_body_array(2, xyz2)
    xyz3 = leg_to_body_array(3, xyz3)

    theta0 = kin.inv_kin_array(xyz0, 0)  # FR
    theta1 = kin.inv_kin_array(xyz1, 1)  # BR
    theta2 = kin.inv_kin_array(xyz2, 2)  # BL
    theta3 = kin.inv_kin_array(xyz3, 3)  # FL

    torques0 = []
    torques1 = []
    torques2 = []
    torques3 = []

    ind = {0: {"xyz": xyz0, "theta": theta0, "side": "R"}, 
           1: {"xyz": xyz1, "theta": theta1, "side": "R"},
           2: {"xyz": xyz2, "theta": theta2, "side": "L"},
           3: {"xyz": xyz3, "theta": theta3, "side": "L"}}

    NUM_POINTS = len(xyz)

    for i in range(NUM_POINTS):
        xy_legs = []
        theta_legs = []
        J_legs = []
        for leg in range(4):
            xyz_leg = ind[leg]["xyz"][i]
            xyz_body = leg_to_body(leg, xyz_leg)
            xy_legs.append((xyz_body[0], xyz_body[1]))
            theta_leg = [ind[leg]["theta"][0][i], ind[leg]["theta"][1][i], ind[leg]["theta"][2][i]]
            theta_legs.append(theta_leg)
            J_leg = compute_J(ind[leg]["side"], theta_leg)
            J_legs.append(J_leg)
        
        F_legs = compute_forces(xy_legs[0], xy_legs[1], xy_legs[2])
        
        torque0 = J_legs[0].T @ F_legs
        torque1 = J_legs[1].T @ F_legs
        torque2 = J_legs[2].T @ F_legs
        torque3 = J_legs[3].T @ F_legs

        torques0.append(torque0.flatten().tolist())
        torques1.append(torque1.flatten().tolist())
        torques2.append(torque2.flatten().tolist())
        torques3.append(torque3.flatten().tolist())

    # Plot torques for each joint in a 4 x 3 grid
    legs = ["FR", "BR", "BL", "FL"]
    joint_types = ["hip", "knee", "foot"]
    all_torques = [torques0, torques1, torques2, torques3]

    # Create a single figure with 4x3 subplots (4 legs x 3 joints)
    fig, axes = plt.subplots(4, 3, figsize=(15, 12))
    fig.suptitle("Joint Torques for All Legs", fontsize=16)

    for leg_ind in range(len(legs)):
        leg = legs[leg_ind]
        torques = all_torques[leg_ind]
        
        for joint_ind, joint_type in enumerate(joint_types):
            ax = axes[leg_ind, joint_ind]
            # Extract torque values for this joint across all time steps
            torque_values = [torque[joint_ind] for torque in torques]
            ax.plot(torque_values, linewidth=2)
            ax.set_title(f"{leg} {joint_type}")
            ax.set_xlabel("Time Step")
            ax.set_ylabel("Torque (N⋅m)")
            ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("joint_torques.png")
    plt.show()


if __name__ == "__main__":
    main()