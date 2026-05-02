import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np
from scipy.signal import lfilter

# --- CONFIGURATION ---
CSV_FILE = 'adc_calibration_log_pcb_1.csv'
SUFFIX = "pcb_" + CSV_FILE.split('.')[0][-1] if CSV_FILE.startswith('adc_calibration_log_pcb_') else CSV_FILE.split('.')[0][-1]
OUTPUT_DIR = 'filtered_plots_' + SUFFIX 

# Equiripple filter coefficients (6-tap FIR)
# Designed for specific passband/stopband characteristics
# COEFFS = [0.035445281711098, -0.139943584023538, 0.609060988977394, 0.609060988977394, -0.139943584023538, 0.035445281711098]
COEFFS = [-0.000285989470915967,	-0.000774971893404532,	-0.00168912900425785,	-0.00312184420500738,	-0.00511809225016721,	-0.00760968137531287,	-0.0103735180774238,	-0.0130085023399371,	-0.0149423203130024,	-0.0154765791888457,	-0.0138696203655593,	-0.00945131985257700,	-0.00175174646987127,	0.00937643730820774,	0.0236680560188620,	0.0404086001103375,	0.0584656014786480,	0.0763906435190855,	0.0925821587313740,	0.105486559300488,	0.113806105216014,	0.116679808754240,	0.113806105216014,	0.105486559300488,	0.0925821587313740,	0.0763906435190855,	0.0584656014786480,	0.0404086001103375,	0.0236680560188620,	0.00937643730820774,	-0.00175174646987127,	-0.00945131985257700,	-0.0138696203655593,	-0.0154765791888457,	-0.0149423203130024,	-0.0130085023399371,	-0.0103735180774238,	-0.00760968137531287,	-0.00511809225016721,	-0.00312184420500738,	-0.00168912900425785,	-0.000774971893404532,	-0.000285989470915967]

# The exact names of your 12 joints
JOINTS = [
    "fr_hip", "fr_knee", "fr_foot",
    "br_hip", "br_knee", "br_foot",
    "fl_hip", "fl_knee", "fl_foot",
    "bl_hip", "bl_knee", "bl_foot"
]

def main():
    # 1. Load the data
    try:
        df = pd.read_csv(CSV_FILE)
    except FileNotFoundError:
        print(f"Error: Could not find '{CSV_FILE}'.")
        return

    # 2. Create the output directory if it doesn't exist
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    # Extract the common time axis
    time = df['timestamp']

    print(f"Loaded {len(df)} rows of data. Applying Equiripple filter and generating plots...")

    # 3. Generate a plot for each joint
    for joint in JOINTS:
        rad_col = f"RAD_{joint}"
        volt_col = f"VOLT_{joint}"

        if rad_col not in df.columns or volt_col not in df.columns:
            print(f"Warning: Missing data columns for {joint}. Skipping.")
            continue

        # --- Filter Application ---
        # lfilter(b, a, x) applies the filter coefficients 'b' to signal 'x'.
        # For FIR filters, the denominator 'a' is always 1.0.
        raw_voltage = df[volt_col].values
        filtered_voltage = lfilter(COEFFS, 1.0, raw_voltage)

        # Create a figure with 1 row and 2 columns
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
        
        # --- Left Subplot: Commanded Angle ---
        ax1.plot(time, df[rad_col], color='blue', linewidth=2, label='Target Radian')
        ax1.set_title(f"{joint.upper()} - Commanded Angle")
        ax1.set_xlabel("Time (Seconds)")
        ax1.set_ylabel("Angle (Radians)")
        ax1.grid(True, linestyle='--', alpha=0.7)
        ax1.legend()

        # --- Right Subplot: Raw vs Filtered Voltage ---
        ax2.plot(time, raw_voltage, color='red', alpha=0.4, linewidth=1, label='Raw ADC Readout')
        ax2.plot(time, filtered_voltage, color='darkred', linewidth=2, label='Filtered Voltage (Equiripple)')
        ax2.set_title(f"{joint.upper()} - Voltage Feedback")
        ax2.set_xlabel("Time (Seconds)")
        ax2.set_ylabel("Voltage (V)")
        ax2.grid(True, linestyle='--', alpha=0.7)
        ax2.legend()

        # Overall figure formatting
        plt.suptitle(f"Step Response & Signal Filtering: {joint}", fontsize=14, fontweight='bold')
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])

        # Save the figure
        filename = f"{joint}_response_filtered.png"
        filepath = os.path.join(OUTPUT_DIR, filename)
        plt.savefig(filepath, dpi=150)
        plt.close(fig)
        
        print(f"Saved: {filename}")

    print(f"\nSuccess! Filtered plots saved to '{OUTPUT_DIR}'.")

if __name__ == '__main__':
    main()