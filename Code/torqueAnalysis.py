import kinematics as kin
import numpy as np 
import math
import matplotlib.pyplot as plt

'''
This script works off of the torque calculation method discussed in Garcia et al., "Quadrupedal Locomotion: An introduction to the control of four-legged robots" (2006).

The approach uses the equation AF = W to obtain the vertical forces at each stance foot, where:
    - A is a 3 x N matrix (N = number of stance legs) with each column representing the (x, y, 1) coordinates of the foothold of a stance leg in the body frame (taken at the body's center of gravity)
    - F is a N x 1 matrix representing the vertical forces at each stance foot
    - W = [0, 0, -weight]^T is the weight vector of the robot

Once the forces are obtained, the torque at each joint is computed using the equation tau = J^T * F, where:
    - tau is the magnitude of torque at each joint
    - J is the Jacobian matrix for the leg in question
    - F is the force vector at the foot
'''

PI = math.pi

WEIGHT = (1.394 + 0.170) * 9.81 # mass of robot and camera in kg, acceleration in m/s^2

L1 = kin.L1/100  # convert cm to m
L2 = kin.L2/100
L3 = kin.L3/100
L4 = kin.L4/100

# NOTE: This transform is NOT taken at the body's center of gravity as per Garcia et al.; it is taken at the body frame origin as defined in the SolidWorks model. TO BE UPDATED LATER.
def leg_to_body(leg_ind, xyz): # convert positions (in cm) from leg base frame to body frame (in m)

    x, y, z = [xyz[0]/100, xyz[1]/100, xyz[2]/100]  # convert cm to m

    beta = [PI/4, -PI/4, -3*PI/4, 3*PI/4][leg_ind]
    x_displacement = [5.951/100, 5.316/100, -5.316/100, -5.951/100][leg_ind] # displacements in m
    y_displacement = [7.883/100, -12.709/100, -12.709/100, 7.883/100][leg_ind]
    z_displacement = 0 

    T = np.array([[math.cos(beta), -math.sin(beta), 0, x_displacement],
                    [math.sin(beta),  math.cos(beta), 0, y_displacement],
                    [0,               0,              1, z_displacement],
                    [0,               0,              0, 1]])

    leg_xyz_homogeneous = np.array([[x], [y], [z], [1]])

    body_xyz_homogeneous = T @ leg_xyz_homogeneous
    body_x = body_xyz_homogeneous[0, 0]
    body_y = body_xyz_homogeneous[1, 0]
    body_z = body_xyz_homogeneous[2, 0]

    return [body_x, body_y, body_z]

def leg_to_body_array(leg_ind, xyz_arr):
    new_xyz_arr = []
    for xyz in xyz_arr:
        new_xyz = leg_to_body(leg_ind, xyz)
        new_xyz_arr.append(new_xyz)
    return new_xyz_arr

def compute_forces(xy1, xy2, xy3, xy4 = None): # compute vertical forces on stance legs based on their footholds (based on Garcia et al.) 

    W = np.array([[0], [0], [WEIGHT]]) # NOTE: Taking weight as positive here, in contrast to Garcia et al. because the paper they cite, Klein and Cheng (1987), takes weight as positive in their formulation
    x1, y1 = xy1
    x2, y2 = xy2
    x3, y3 = xy3

    if xy4 is not None:
        x4, y4 = xy4
        A = np.array([[x1, x2, x3, x4],
                        [y1, y2, y3, y4],
                        [1, 1, 1, 1]])
    else: 
        A = np.array([[x1, x2, x3],
                        [y1, y2, y3],
                        [1, 1, 1]])

    try:
        # print("Computing inverse of A matrix for torque analysis.")
        Ainv = np.linalg.inv(A)
    except: 
        # print("Singular matrix encountered in torque analysis; using pseudo-inverse.")
        Ainv = np.linalg.pinv(A) 

    F = Ainv @ W # vector of vertical forces on each stance foot

    return F

def compute_J(side, thetas): # compute Jacobian matrix for a given leg side ("L" or "R") and joint angles
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

def compute_torque(xyz_legs, leg_indices = [0, 1, 2]): # footholds in leg frame in cm along with leg_indices
    
    thetas = []

    for ind in range(len(xyz_legs)):
        x, y, z = xyz_legs[ind]
        theta_leg = kin.inv_kin(x, y, z, leg_indices[ind])
        thetas.append(theta_leg)
    
    xyz_body = [leg_to_body(leg_indices[ind], xyz_legs[ind]) for ind in range(len(xyz_legs))]
    xy_body = [(xyz_body[ind][0], xyz_body[ind][1]) for ind in range(len(xyz_body))]
    
    if len(xyz_legs) == 3:
        F_legs = compute_forces(xy_body[0], xy_body[1], xy_body[2])
    else:
        F_legs = compute_forces(xy_body[0], xy_body[1], xy_body[2], xy_body[3])

    J_legs = []
    for ind in range(len(xyz_legs)):
        side = "R" if leg_indices[ind] in [0, 1] else "L"
        J_leg = compute_J(side, thetas[ind])
        J_legs.append(J_leg)

    torques = []
    for ind in range(len(J_legs)):
        force_leg = np.array([[0], [0], [F_legs[ind, 0]]]) # force vector at the foot, assuming only vertical forces as per Garcia et al.
        torque_leg = J_legs[ind].T @ force_leg # torque at each joint is given by J^T * F where F is the force vector at the foot
        torques.append(torque_leg.flatten().tolist())
    
    return torques

def main():
    # generate initial bezier curve trajectory (these points are generated in cm)
    xyz = kin.generate_trajectory()

    # rotate it to be parallel to body frame for each leg
    xyz0 = kin.rotate_trajectory(0, xyz)  # FR
    xyz1 = kin.rotate_trajectory(1, xyz)  # BR
    xyz2 = kin.rotate_trajectory(2, xyz)  # BL
    xyz3 = kin.rotate_trajectory(3, xyz)  # FL

    # shift it to be within each leg's workspace
    xyz0 = kin.shift_trajectory(0, xyz0)
    xyz1 = kin.shift_trajectory(1, xyz1)
    xyz2 = kin.shift_trajectory(2, xyz2)
    xyz3 = kin.shift_trajectory(3, xyz3)

    # duplicate the trajectory for more steps
    xyz0 = xyz0 * 3
    xyz1 = xyz1 * 3
    xyz2 = xyz2 * 3
    xyz3 = xyz3 * 3

    NUM_POINTS = len(xyz0)
    STANCE_IDENTIFIER = xyz0[0][2] # the first leg starts off on the ground so we can use its z value as the identifier for footholds

    torques0 = []
    torques1 = []
    torques2 = []
    torques3 = []

    for i in range(NUM_POINTS):
        xyz_legs = []
        leg_indices = []
        if xyz0[i][2] == STANCE_IDENTIFIER:
            xyz_legs.append(xyz0[i])
            leg_indices.append(0)

        if xyz1[i][2] == STANCE_IDENTIFIER:
            xyz_legs.append(xyz1[i])
            leg_indices.append(1)

        if xyz2[i][2] == STANCE_IDENTIFIER:
            xyz_legs.append(xyz2[i])
            leg_indices.append(2)

        if xyz3[i][2] == STANCE_IDENTIFIER:
            xyz_legs.append(xyz3[i])
            leg_indices.append(3)

        torques = compute_torque(xyz_legs, leg_indices)
        indices_stance = [False, False, False, False]

        for ind in range(len(torques)):
            if leg_indices[ind] == 0:
                torques0.append(torques[ind])
                indices_stance[0] = True
            elif leg_indices[ind] == 1:
                torques1.append(torques[ind])
                indices_stance[1] = True
            elif leg_indices[ind] == 2:
                torques2.append(torques[ind])
                indices_stance[2] = True
            elif leg_indices[ind] == 3:
                torques3.append(torques[ind])
                indices_stance[3] = True
        
        for ind in range(4):
            if not indices_stance[ind]:
                if ind == 0:
                    torques0.append([0, 0, 0])
                elif ind == 1:
                    torques1.append([0, 0, 0])
                elif ind == 2:
                    torques2.append([0, 0, 0])
                elif ind == 3:
                    torques3.append([0, 0, 0])

    # ================ PLOTTING ==================
    # Plot torques for each joint in a 4 x 3 grid
    legs = ["FR", "BR", "BL", "FL"]
    joint_types = ["hip", "knee", "foot"]
    all_torques = [torques0, torques1, torques2, torques3]

    # Find global min and max torque values for consistent torque-axis limits
    all_torque_values = []
    for torques in all_torques:
        for torque in torques:
            all_torque_values.extend(torque)
    
    y_min = min(all_torque_values)
    y_max = max(all_torque_values)
    # Add some padding to the bounds
    y_padding = (y_max - y_min) * 0.1
    y_min -= y_padding
    y_max += y_padding

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
            ax.axhline(y=0.3*0.9414, color='g', linestyle=':', linewidth=2, label="30% Stall Torque")
            ax.set_title(f"{leg} {joint_type}")
            ax.set_xlabel("Time Step")
            ax.set_ylabel("Torque (N⋅m)")
            ax.set_ylim(y_min, y_max)
            ax.legend()
            ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("required_torques.png")
    plt.show()

if __name__ == "__main__":
    main()