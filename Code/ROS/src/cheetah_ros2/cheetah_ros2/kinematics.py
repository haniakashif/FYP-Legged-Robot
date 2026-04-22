import math
import numpy as np

# Math Constants
PI = math.pi

# Link Lengths
L1 = 2.845  # in cm
L2 = 5.439
L3 = 2.637
L4 = 9.265

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
        theta1 = max(min(theta1, PI/4), -PI/4)
        # raise Exception(f"ERROR: For {LEGS[leg_ind]}, point {step}: theta1 out of bounds: {math.degrees(theta1)}") 
    if theta2 < -PI/2 or theta2 > PI/2:
        theta2 = max(min(theta2, PI/2), -PI/2)
        # raise Exception(f"ERROR: For {LEGS[leg_ind]}, point {step}: theta2 out of bounds: {math.degrees(theta2)}")
    if theta4 < -PI/2 or theta4 > PI/2:
        theta4 = max(min(theta4, PI/2), -PI/2)
        # raise Exception(f"ERROR: For {LEGS[leg_ind]}, point {step}: theta4 out of bounds: {math.degrees(theta4)}")
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