import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import yaml
from scipy.signal import lfilter

# Feature Flag
APPLY_FILTER = True 

# --- CONFIGURATION ---
CSV_FILE = 'adc_calibration_log_pcb_1.csv'
YAML_FILE = 'calibration_curves_pcb_1/joint_calibration.yaml'
CSV_SUFFIX = "pcb_" + CSV_FILE.split('.')[0][-1] if CSV_FILE.startswith('adc_calibration_log_pcb_') else CSV_FILE.split('.')[0][-1]
YAML_SUFFIX = "pcb_" + YAML_FILE.split('/')[0][-1] if YAML_FILE.startswith('calibration_curves_pcb_') else YAML_FILE.split('/')[0][-1]
OUTPUT_DIR = 'state_tracking_plots_' + CSV_SUFFIX  + '_' + YAML_SUFFIX  

OUTPUT_DIR = OUTPUT_DIR + ('_filtered' if APPLY_FILTER else '_raw')

# COEFFS = [0.035445281711098, -0.139943584023538, 0.609060988977394, 0.609060988977394, -0.139943584023538, 0.035445281711098]
# COEFFS = [-0.000285989470915967,	-0.000774971893404532,	-0.00168912900425785,	-0.00312184420500738,	-0.00511809225016721,	-0.00760968137531287,	-0.0103735180774238,	-0.0130085023399371,	-0.0149423203130024,	-0.0154765791888457,	-0.0138696203655593,	-0.00945131985257700,	-0.00175174646987127,	0.00937643730820774,	0.0236680560188620,	0.0404086001103375,	0.0584656014786480,	0.0763906435190855,	0.0925821587313740,	0.105486559300488,	0.113806105216014,	0.116679808754240,	0.113806105216014,	0.105486559300488,	0.0925821587313740,	0.0763906435190855,	0.0584656014786480,	0.0404086001103375,	0.0236680560188620,	0.00937643730820774,	-0.00175174646987127,	-0.00945131985257700,	-0.0138696203655593,	-0.0154765791888457,	-0.0149423203130024,	-0.0130085023399371,	-0.0103735180774238,	-0.00760968137531287,	-0.00511809225016721,	-0.00312184420500738,	-0.00168912900425785,	-0.000774971893404532,	-0.000285989470915967]
COEFFS = [0.018617559455263,-0.114628672778309,0.596290909881029,0.596290909881029,-0.114628672778309,0.018617559455263]

JOINTS = [
    "fr_hip", "fr_knee", "fr_foot",
    "br_hip", "br_knee", "br_foot",
    "fl_hip", "fl_knee", "fl_foot",
    "bl_hip", "bl_knee", "bl_foot"
]

def load_calibration_data(filepath):
    """Safely loads the YAML configuration file."""
    try:
        with open(filepath, 'r') as file:
            # safe_load mitigates arbitrary code execution vulnerabilities in YAML parsing
            return yaml.safe_load(file)
    except FileNotFoundError:
        print(f"Error: Calibration file '{filepath}' not found.")
        return None
    except yaml.YAMLError as exc:
        print(f"Error parsing YAML: {exc}")
        return None

def main():
    # 1. Load Data
    try:
        df = pd.read_csv(CSV_FILE)
    except FileNotFoundError:
        print(f"Error: Could not find '{CSV_FILE}'.")
        return

    calib_data = load_calibration_data(YAML_FILE)
    if not calib_data:
        return

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    time = df['timestamp'].values
    
    print(f"Generating tracking plots. FIR Filter applied: {APPLY_FILTER}")

    # 2. Process and Plot Each Joint
    for joint in JOINTS:
        rad_col = f"RAD_{joint}"
        volt_col = f"VOLT_{joint}"

        if rad_col not in df.columns or volt_col not in df.columns:
            continue

        if joint not in calib_data:
            print(f"Warning: No calibration parameters found for {joint} in YAML. Skipping.")
            continue

        # Extract mapping parameters
        slope = calib_data[joint]['slope']
        intercept = calib_data[joint]['intercept']

        # Extract raw data
        commanded_theta = df[rad_col].values
        raw_voltage = df[volt_col].values

        # Calculate actual (estimated) state from raw voltage
        actual_theta_raw = (raw_voltage * slope) + intercept

        # Setup plot
        fig, ax = plt.subplots(figsize=(10, 5))
        
        # Plot Commanded Angle
        ax.plot(time, commanded_theta, color='blue', linewidth=2, linestyle='--', label='Commanded Trajectory')
        
        if APPLY_FILTER:
            # Filter the raw voltage FIRST, then map to angle
            filtered_voltage = lfilter(COEFFS, 1.0, raw_voltage)
            actual_theta_filtered = (filtered_voltage * slope) + intercept
            
            # Plot raw state as background, filtered state as foreground
            ax.plot(time, actual_theta_raw, color='red', alpha=0.3, linewidth=1, label='Raw Estimated State')
            ax.plot(time, actual_theta_filtered, color='darkred', linewidth=2, label='Filtered Estimated State')
        else:
            # Plot only raw state
            ax.plot(time, actual_theta_raw, color='red', linewidth=1.5, label='Raw Estimated State')

        # Formatting
        ax.set_title(f"{joint.upper()} - Position Tracking Performance")
        ax.set_xlabel("Time (Seconds)")
        ax.set_ylabel("Angle (Radians)")
        ax.grid(True, linestyle='--', alpha=0.7)
        ax.legend(loc='upper right')
        
        plt.tight_layout()
        
        # Save
        filename = f"{joint}_tracking.png"
        filepath = os.path.join(OUTPUT_DIR, filename)
        plt.savefig(filepath, dpi=150)
        plt.close(fig)

    print(f"\nAll plots saved to '{OUTPUT_DIR}'.")

if __name__ == '__main__':
    main()