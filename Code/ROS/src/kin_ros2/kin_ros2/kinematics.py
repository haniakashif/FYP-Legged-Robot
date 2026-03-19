import math
import numpy as np

# Math Constants
PI = math.pi

# Link Lengths
L1 = 2.845  # in cm
L2 = 5.439
L3 = 2.637
L4 = 9.265

# Trajectory Constants
T_STALL = 4
NUM_DATA_POINTS = 32
SWING_FACTOR = 1/4 # SWING_FACTOR of the points are for swing phase
STANCE_FACTOR = 1 - SWING_FACTOR

X = 15
S = -7
A = 3
T = 6

P1 = [-T/2, S]
P2 = [0, S + 2*A]
P3 = [T/2, S]

X_OFFSET = -5
Y_OFFSET = 4

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
        raise Exception(f"ERROR: For {LEGS[leg_ind]}, point {step}: theta1 out of bounds: {math.degrees(theta1)}") 
    if theta2 < -PI/2 or theta2 > PI/2:
        raise Exception(f"ERROR: For {LEGS[leg_ind]}, point {step}: theta2 out of bounds: {math.degrees(theta2)}")
    if theta4 < -PI/2 or theta4 > PI/2:
        raise Exception(f"ERROR: For {LEGS[leg_ind]}, point {step}: theta4 out of bounds: {math.degrees(theta4)}")
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

    # schedule = [(0, 0), (2, 1), (1, 2), (3, 3)] # Order of swing: FR, BL, BR, FL
    # schedule = [(3, 0), (1, 1), (2, 2), (0, 3)] # Order of swing: FL, BR, BL, FR
    # schedule = [(0, 0), (1, 1), (3, 2), (2, 3)] # Order of swing: FR, BL, BR, FL
    schedule = [(1, 0), (2, 1), (0, 2), (3, 3)] # Order of swing: BR, BL, FR, FL
    # schedule = [(0, 0), (3, 1), (1, 2), (2, 3) ] # Order of swing: FR, FL, BR, BL
    # schedule = [(0, 0), (2, 0), (1, 1), (3, 1)] # Trot

    for swing_leg, swing_index in schedule:
        if leg_ind == swing_leg:
            xyzK_copy = xyzK.copy()
            xyzK_ind = 0
            for i in range(int(NUM_DATA_POINTS + 2*T_STALL - swing_index*(NUM_DATA_POINTS*SWING_FACTOR)), int(NUM_DATA_POINTS + 2*T_STALL)):
                x, y, z = xyzK_copy[i]
                xyzK[xyzK_ind] = (x, y, z)
                xyzK_ind += 1

            for i in range(0, int(NUM_DATA_POINTS + 2*T_STALL - swing_index*(NUM_DATA_POINTS*SWING_FACTOR))):
                x, y, z = xyzK_copy[i]
                xyzK[xyzK_ind] = (x, y, z)
                xyzK_ind += 1

            return xyzK