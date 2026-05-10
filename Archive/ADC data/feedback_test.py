import smbus
import serial
import time
import csv
import sys

# IMP PARAMS TO ADJUST 
HOLD_TIME_SEC = 4  # Time to hold each position in the arrays
MOVE_SPEED_MS = 1000 # Time to move between each command
KIN_GAIT = False # False if calibrating between limits, True if calibrating in kinematic gait

# CONFIGURATION
SERIAL_PORT = '/dev/serial0'
BAUD_RATE = 115200
I2C_BUS = 1
LOG_FILE = 'adc_calibration_log.csv'
POLL_RATE_HZ = 50

# ADC Addresses based on your physical wiring
ADC_ADDR = [0x48, 0x49, 0x4A]

# I2C Configuration Bytes (+/- 4.096V, 3300 SPS)
MUX_CONFIGS = [0xC3, 0xD3, 0xE3, 0xF3]
CONFIG_LSB = 0xE3

# JOINT MAPPING
JOINTS = [
    # Board 1: 0x48 (GND)
    {"name": "br_hip",  "pin": 4,  "min_pwm": 500, "max_pwm": 2700, "min_rad": -1.5707, "max_rad": 1.5707},
    {"name": "bl_hip",  "pin": 16, "min_pwm": 600, "max_pwm": 2440, "min_rad": -1.5707, "max_rad": 1.5707},
    {"name": "bl_knee", "pin": 17, "min_pwm": 600, "max_pwm": 2500, "min_rad": -1.5707, "max_rad": 1.5707},
    {"name": "bl_foot", "pin": 18, "min_pwm": 600, "max_pwm": 2500, "min_rad": -1.5707, "max_rad": 1.5707},

    # Board 2: 0x49 (3.3V)
    {"name": "br_knee", "pin": 5,  "min_pwm": 600, "max_pwm": 2400, "min_rad": -1.5707, "max_rad": 1.5707},
    {"name": "fl_foot", "pin": 30, "min_pwm": 600, "max_pwm": 2500, "min_rad": -1.5707, "max_rad": 1.5707},
    {"name": "fl_knee", "pin": 29, "min_pwm": 600, "max_pwm": 2440, "min_rad": -1.5707, "max_rad": 1.5707},
    {"name": "fl_hip",  "pin": 28, "min_pwm": 560, "max_pwm": 2440, "min_rad": -1.5707, "max_rad": 1.5707},

    # Board 3: 0x4A (SDA)
    {"name": "br_foot", "pin": 6,  "min_pwm": 600, "max_pwm": 2500, "min_rad": -1.5707, "max_rad": 1.5707},
    {"name": "fr_hip",  "pin": 0,  "min_pwm": 600, "max_pwm": 2400, "min_rad": -1.5707, "max_rad": 1.5707},
    {"name": "fr_knee", "pin": 1,  "min_pwm": 700, "max_pwm": 2560, "min_rad": -1.5707, "max_rad": 1.5707},
    {"name": "fr_foot", "pin": 2,  "min_pwm": 560, "max_pwm": 2500, "min_rad": -1.5707, "max_rad": 1.5707},
]

if not KIN_GAIT:
    TARGET_TRAJECTORIES = {"fr_hip":  [0.0, 0.7853, 0.0, -0.7853], "fr_knee": [0.0, 1.5707, 0.0, -1.5707], "fr_foot": [0.0, 1.5707, 0.0, -1.5707], "br_hip":  [0.0, 0.7853, 0.0, -0.7853], "br_knee": [0.0, 1.5707, 0.0, -1.5707], "br_foot": [0.0, 1.5707, 0.0, -1.5707], "fl_hip":  [0.0, 0.7853, 0.0, -0.7853], "fl_knee": [0.0, 1.5707, 0.0, -1.5707], "fl_foot": [0.0, 1.5707, 0.0, -1.5707], "bl_hip":  [0.0, 0.7853, 0.0, -0.7853], "bl_knee": [0.0, 1.5707, 0.0, -1.5707], "bl_foot": [0.0, 1.5707, 0.0, -1.5707]}
else:
    TARGET_TRAJECTORIES = { "fr_hip":  [-0.4162, -0.4391, -0.4623, -0.4860, -0.5100, -0.5343, -0.5589, -0.5838, -0.6090, -0.6344, -0.6600, -0.6857, -0.6857, -0.6857, -0.6857, -0.6857, -0.6857, -0.6018, -0.5204, -0.4424, -0.3685, -0.2992, -0.2345, -0.1747, -0.1747, -0.1747, -0.1747, -0.1747, -0.1747, -0.1924, -0.2106, -0.2291, -0.2482, -0.2677, -0.2876, -0.3079, -0.3287, -0.3500, -0.3716, -0.3937], "fr_knee": [0.5517, 0.5525, 0.5532, 0.5538, 0.5542, 0.5545, 0.5548, 0.5549, 0.5550, 0.5551, 0.5551, 0.5552, 0.5552, 0.5552, 0.5552, 0.5552, 0.5552, 0.7635, 0.9040, 0.9686, 0.9572, 0.8749, 0.7283, 0.5218, 0.5218, 0.5218, 0.5218, 0.5218, 0.5218, 0.5261, 0.5300, 0.5335, 0.5367, 0.5396, 0.5421, 0.5443, 0.5463, 0.5480, 0.5494, 0.5507], "fr_foot": [1.2636, 1.2741, 1.2839, 1.2930, 1.3013, 1.3089, 1.3157, 1.3218, 1.3272, 1.3318, 1.3357, 1.3389, 1.3389, 1.3389, 1.3389, 1.3389, 1.3389, 1.4667, 1.5230, 1.5254, 1.4848, 1.4036, 1.2740, 1.0785, 1.0785, 1.0785, 1.0785, 1.0785, 1.0785, 1.0982, 1.1170, 1.1351, 1.1524, 1.1690, 1.1848, 1.1998, 1.2140, 1.2276, 1.2403, 1.2523], "br_hip":  [0.1747, 0.2345, 0.2992, 0.3685, 0.4424, 0.5204, 0.6018, 0.6857, 0.6857, 0.6857, 0.6857, 0.6857, 0.6857, 0.6600, 0.6344, 0.6090, 0.5838, 0.5589, 0.5343, 0.5100, 0.4860, 0.4623, 0.4391, 0.4162, 0.3937, 0.3716, 0.3500, 0.3287, 0.3079, 0.2876, 0.2677, 0.2482, 0.2291, 0.2106, 0.1924, 0.1747, 0.1747, 0.1747, 0.1747, 0.1747], "br_knee": [0.5218, 0.7283, 0.8749, 0.9572, 0.9686, 0.9040, 0.7635, 0.5552, 0.5552, 0.5552, 0.5552, 0.5552, 0.5552, 0.5551, 0.5551, 0.5550, 0.5549, 0.5548, 0.5545, 0.5542, 0.5538, 0.5532, 0.5525, 0.5517, 0.5507, 0.5494, 0.5480, 0.5463, 0.5443, 0.5421, 0.5396, 0.5367, 0.5335, 0.5300, 0.5261, 0.5218, 0.5218, 0.5218, 0.5218, 0.5218], "br_foot": [1.0785, 1.2740, 1.4036, 1.4848, 1.5254, 1.5230, 1.4667, 1.3389, 1.3389, 1.3389, 1.3389, 1.3389, 1.3389, 1.3357, 1.3318, 1.3272, 1.3218, 1.3157, 1.3089, 1.3013, 1.2930, 1.2839, 1.2741, 1.2636, 1.2523, 1.2403, 1.2276, 1.2140, 1.1998, 1.1848, 1.1690, 1.1524, 1.1351, 1.1170, 1.0982, 1.0785, 1.0785, 1.0785, 1.0785, 1.0785], "fl_hip":  [0.2482, 0.2677, 0.2876, 0.3079, 0.3287, 0.3500, 0.3716, 0.3937, 0.4162, 0.4391, 0.4623, 0.4860, 0.5100, 0.5343, 0.5589, 0.5838, 0.6090, 0.6344, 0.6600, 0.6857, 0.6857, 0.6857, 0.6857, 0.6857, 0.6857, 0.6018, 0.5204, 0.4424, 0.3685, 0.2992, 0.2345, 0.1747, 0.1747, 0.1747, 0.1747, 0.1747, 0.1747, 0.1924, 0.2106, 0.2291], "fl_knee": [-0.5367, -0.5396, -0.5421, -0.5443, -0.5463, -0.5480, -0.5494, -0.5507, -0.5517, -0.5525, -0.5532, -0.5538, -0.5542, -0.5545, -0.5548, -0.5549, -0.5550, -0.5551, -0.5551, -0.5552, -0.5552, -0.5552, -0.5552, -0.5552, -0.5552, -0.7635, -0.9040, -0.9686, -0.9572, -0.8749, -0.7283, -0.5218, -0.5218, -0.5218, -0.5218, -0.5218, -0.5218, -0.5261, -0.5300, -0.5335], "fl_foot": [-1.1524, -1.1690, -1.1848, -1.1998, -1.2140, -1.2276, -1.2403, -1.2523, -1.2636, -1.2741, -1.2839, -1.2930, -1.3013, -1.3089, -1.3157, -1.3218, -1.3272, -1.3318, -1.3357, -1.3389, -1.3389, -1.3389, -1.3389, -1.3389, -1.3389, -1.4667, -1.5230, -1.5254, -1.4848, -1.4036, -1.2740, -1.0785, -1.0785, -1.0785, -1.0785, -1.0785, -1.0785, -1.0982, -1.1170, -1.1351], "bl_hip":  [-0.2291, -0.2106, -0.1924, -0.1747, -0.1747, -0.1747, -0.1747, -0.1747, -0.1747, -0.2345, -0.2992, -0.3685, -0.4424, -0.5204, -0.6018, -0.6857, -0.6857, -0.6857, -0.6857, -0.6857, -0.6857, -0.6600, -0.6344, -0.6090, -0.5838, -0.5589, -0.5343, -0.5100, -0.4860, -0.4623, -0.4391, -0.4162, -0.3937, -0.3716, -0.3500, -0.3287, -0.3079, -0.2876, -0.2677, -0.2482], "bl_knee": [-0.5335, -0.5300, -0.5261, -0.5218, -0.5218, -0.5218, -0.5218, -0.5218, -0.5218, -0.7283, -0.8749, -0.9572, -0.9686, -0.9040, -0.7635, -0.5552, -0.5552, -0.5552, -0.5552, -0.5552, -0.5552, -0.5551, -0.5551, -0.5550, -0.5549, -0.5548, -0.5545, -0.5542, -0.5538, -0.5532, -0.5525, -0.5517, -0.5507, -0.5494, -0.5480, -0.5463, -0.5443, -0.5421, -0.5396, -0.5367], "bl_foot": [-1.1351, -1.1170, -1.0982, -1.0785, -1.0785, -1.0785, -1.0785, -1.0785, -1.0785, -1.2740, -1.4036, -1.4848, -1.5254, -1.5230, -1.4667, -1.3389, -1.3389, -1.3389, -1.3389, -1.3389, -1.3389, -1.3357, -1.3318, -1.3272, -1.3218, -1.3157, -1.3089, -1.3013, -1.2930, -1.2839, -1.2741, -1.2636, -1.2523, -1.2403, -1.2276, -1.2140, -1.1998, -1.1848, -1.1690, -1.1524]}

def rad_to_pwm(target_rad, joint):
    """Exactly mirrors the C++ QuadHardwareInterface::write logic."""
    rad_range = joint["max_rad"] - joint["min_rad"]
    pwm_range = joint["max_pwm"] - joint["min_pwm"]
    
    pwm_val = int((joint["min_pwm"] + joint["max_pwm"])/2.0 + target_rad * pwm_range / rad_range)
    
    if pwm_val < joint["min_pwm"]: pwm_val = joint["min_pwm"]
    if pwm_val > joint["max_pwm"]: pwm_val = joint["max_pwm"]
    
    return pwm_val

def read_all_adcs_fast(bus):
    """Reads all 12 channels simultaneously across the 3 boards."""
    voltages = [0.0] * 12
    for ch in range(4):
        for addr in ADC_ADDR:
            bus.write_i2c_block_data(addr, 0x01, [MUX_CONFIGS[ch], CONFIG_LSB])
        
        time.sleep(0.002) # Wait for conversions
        
        for idx, addr in enumerate(ADC_ADDR):
            data = bus.read_i2c_block_data(addr, 0x00, 2)
            raw = (data[0] << 8) | data[1]
            raw >>= 4
            if raw > 2047:
                raw -= 4096
            voltages[idx * 4 + ch] = raw * 0.002
            
    return voltages

def main():
    print("Initializing Hardware...")
    try:
        ssc32 = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        bus = smbus.SMBus(I2C_BUS)
    except Exception as e:
        print(f"Hardware Error: {e}")
        sys.exit(1)

    # Determine the number of steps in the arrays
    num_steps = len(TARGET_TRAJECTORIES["fr_hip"])
    
    # Setup CSV Logging Headers
    header = ["timestamp"]
    for j in JOINTS: header.append(f"RAD_{j['name']}")
    for j in JOINTS: header.append(f"PWM_{j['name']}")
    for j in JOINTS: header.append(f"VOLT_{j['name']}")

    print(f"Starting test. Logging to {LOG_FILE} at {POLL_RATE_HZ}Hz.")
    print("Press Ctrl+C to stop.")
    time.sleep(5)

    try:
        with open(LOG_FILE, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(header)

            start_time = time.time()

            # Loop through 2 full cycles plus 1 extra step
            for seq_idx in range(4*num_steps):
                step = seq_idx % num_steps
                print(f"\n--- EXECUTING ARRAY STEP: {step}/{num_steps - 1} ---")
                
                cmd_rads = []
                cmd_pwms = []
                cmd_str = ""
                
                # 1. Pull the specific target radian for each joint from its dedicated array
                for j in JOINTS:
                    target_rad = TARGET_TRAJECTORIES[j["name"]][step]
                    pwm = rad_to_pwm(target_rad, j)
                    
                    cmd_rads.append(target_rad)
                    cmd_pwms.append(pwm)
                    cmd_str += f"#{j['pin']}P{pwm} "
                
                # Command execution time
                cmd_str += f"T{MOVE_SPEED_MS}\r"
                ssc32.write(cmd_str.encode('utf-8'))

                # 2. Fast Polling Loop
                state_start_time = time.time()
                loop_period = 1.0 / POLL_RATE_HZ
                
                print("Logging data... ", end="")
                sys.stdout.flush()

                while (time.time() - state_start_time) < HOLD_TIME_SEC:
                    loop_start = time.time()
                    
                    voltages = read_all_adcs_fast(bus)
                    
                    current_elapsed = time.time() - start_time
                    row = [f"{current_elapsed:.4f}"] + [f"{r:.4f}" for r in cmd_rads] + cmd_pwms + [f"{v:.4f}" for v in voltages]
                    writer.writerow(row)
                    
                    elapsed = time.time() - loop_start
                    if elapsed < loop_period:
                        time.sleep(loop_period - elapsed)

                print("Done.")

    except KeyboardInterrupt:
        print("\nTest stopped by user.")
        safe_cmd = ""
        for j in JOINTS:
            safe_pwm = rad_to_pwm(0.0, j)
            safe_cmd += f"#{j['pin']}P{safe_pwm} "
        safe_cmd += "T1000\r"
        ssc32.write(safe_cmd.encode('utf-8'))
        print("Robot returned to safe mid-positions. Exiting.")

if __name__ == '__main__':
    main()
