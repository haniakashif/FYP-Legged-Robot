import csv
import math
import matplotlib.pyplot as plt


def read_csv(filename):
    data = {}
    with open(filename, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            for key, value in row.items():
                if key not in data:
                    data[key] = []
                try:
                    data[key].append(float(value) if value != '' else None)
                except ValueError:
                    data[key].append(value if value != '' else None)
    return data


def calculate_percentage_errors(actual_data, simplified_data, eps: float = 1e-9):
    """Calculate percentage errors per key: |actual - simplified| / |actual| * 100.

    If |actual| is 0 or None, percentage error is reported as None for that timestep.
    """
    perc_errors = {}

    for key in actual_data.keys():
        if key == 'Time_Step':
            continue
        if key in simplified_data:
            a_vals = actual_data[key]
            s_vals = simplified_data[key]
            min_len = min(len(a_vals), len(s_vals))

            e_vals = []
            for i in range(min_len):
                a = a_vals[i]
                s = s_vals[i]
                if a is None or s is None:
                    e_vals.append(None)
                    continue
                denom = abs(a)
                if denom <= eps:
                    e_vals.append(None)
                else:
                    e_vals.append(abs(a - s) / denom * 100.0)
            perc_errors[key] = e_vals

    return perc_errors


def _plot_actual_vs_simplified(actual_data, simplified_data, title_prefix: str, ylabel: str, key_suffix: str, save_path: str):
    legs = ["FR", "BR", "BL", "FL"]
    joint_types = ["hip", "knee", "foot"]

    fig, axes = plt.subplots(4, 3, figsize=(15, 12))
    suptitle = f"{title_prefix}: Actual vs Simplified" + (" + Command" if key_suffix == "state" else "")
    fig.suptitle(suptitle, fontsize=16)

    for leg_ind, leg in enumerate(legs):
        for joint_ind, joint_type in enumerate(joint_types):
            ax = axes[leg_ind, joint_ind]
            key = f"{leg}_{joint_type}_{key_suffix}"

            if key in actual_data and key in simplified_data:
                a_vals = actual_data[key]
                s_vals = simplified_data[key]
                min_len = min(len(a_vals), len(s_vals))
                x = list(range(min_len))
                ax.plot(x, a_vals[:min_len], linestyle='-', linewidth=1.5, color='blue', label='Actual')
                ax.plot(x, s_vals[:min_len], linestyle='--', linewidth=1.5, color='orange', label='Simplified')
                # If plotting joint states, also overlay the commanded angle
                if key_suffix == "state":
                    cmd_key = f"{leg}_{joint_type}_command"
                    cmd_source = None
                    if cmd_key in actual_data:
                        cmd_source = actual_data[cmd_key]
                    elif cmd_key in simplified_data:
                        cmd_source = simplified_data[cmd_key]
                    if cmd_source is not None:
                        ax.plot(x, cmd_source[:min_len], linestyle=':', linewidth=1.5, color='green', label='Command')
                ax.set_title(f"{leg} {joint_type}")
                ax.set_xlabel("Time Step")
                ax.set_ylabel(ylabel)
                ax.grid(True, alpha=0.3)
                ax.legend(loc='upper right', fontsize=8)
            else:
                ax.set_title(f"{leg} {joint_type} - No Data")
                ax.set_xlabel("Time Step")
                ax.set_ylabel(ylabel)

    plt.tight_layout()
    plt.savefig(save_path)


def _plot_percentage_errors(errors, title_prefix: str, ylabel: str, save_path: str):
    legs = ["FR", "BR", "BL", "FL"]
    joint_types = ["hip", "knee", "foot"]

    fig, axes = plt.subplots(4, 3, figsize=(15, 12))
    fig.suptitle(f"{title_prefix}: Percentage Error |Actual - Simplified| / |Actual|", fontsize=16)

    for leg_ind, leg in enumerate(legs):
        for joint_ind, joint_type in enumerate(joint_types):
            ax = axes[leg_ind, joint_ind]
            key = f"{leg}_{joint_type}_{'state' if 'State' in title_prefix else 'torque'}"
            # The key_suffix detection above is a convenience, but we'll also try both possibilities.
            candidate_keys = [f"{leg}_{joint_type}_state", f"{leg}_{joint_type}_torque"]
            key_to_use = None
            for k in candidate_keys:
                if k in errors:
                    key_to_use = k
                    break

            if key_to_use:
                e_vals = errors[key_to_use]
                # Remove None while keeping timeline continuity by plotting as-is
                if e_vals:
                    x = list(range(len(e_vals)))
                    ax.plot(x, e_vals, linestyle='-', linewidth=1.5, color='purple')
                    ax.set_title(f"{leg} {joint_type}")
                    ax.set_xlabel("Time Step")
                    ax.set_ylabel(ylabel)
                    ax.grid(True, alpha=0.3)
            else:
                ax.set_title(f"{leg} {joint_type} - No Data")
                ax.set_xlabel("Time Step")
                ax.set_ylabel(ylabel)

    plt.tight_layout()
    plt.savefig(save_path)


def main():
    # Input files (same naming convention as v1 script)
    actual_torque_file = "joint_torques_walk_norm.csv"
    simplified_torque_file = "joint_torques_walk_newc.csv"
    actual_joint_state_file = "joint_commands_vs_states_walk_norm.csv"
    simplified_joint_state_file = "joint_commands_vs_states_walk_newc.csv"
    file_suffix = "_walk_newc"

    print(f"Reading {actual_torque_file}...")
    actual_torque_data = read_csv(actual_torque_file)
    print(f"Reading {simplified_torque_file}...")
    simplified_torque_data = read_csv(simplified_torque_file)

    print(f"Reading {actual_joint_state_file}...")
    actual_joint_state_data = read_csv(actual_joint_state_file)
    print(f"Reading {simplified_joint_state_file}...")
    simplified_joint_state_data = read_csv(simplified_joint_state_file)

    # Plot actual vs simplified for joint states and torques
    print("Plotting Actual vs Simplified for Joint States...")
    _plot_actual_vs_simplified(
        actual_joint_state_data,
        simplified_joint_state_data,
        title_prefix="Joint States",
        ylabel="Angle (deg)",
        key_suffix="state",
        save_path="joint_state_actual_vs_simplified" + file_suffix + ".png",
    )

    print("Plotting Actual vs Simplified for Torques...")
    _plot_actual_vs_simplified(
        actual_torque_data,
        simplified_torque_data,
        title_prefix="Joint Torques",
        ylabel="Torque (N⋅m)",
        key_suffix="torque",
        save_path="joint_torque_actual_vs_simplified" + file_suffix + ".png",
    )

    # Percentage errors
    print("Calculating percentage errors...")
    torque_perc_errors = calculate_percentage_errors(actual_torque_data, simplified_torque_data)
    joint_state_perc_errors = calculate_percentage_errors(actual_joint_state_data, simplified_joint_state_data)

    print("Plotting percentage errors for Joint States...")
    _plot_percentage_errors(
        joint_state_perc_errors,
        title_prefix="Joint State Percentage Errors",
        ylabel="Percent Error (%)",
        save_path="joint_state_percentage_errors" + file_suffix + ".png",
    )

    print("Plotting percentage errors for Torques...")
    _plot_percentage_errors(
        torque_perc_errors,
        title_prefix="Torque Percentage Errors",
        ylabel="Percent Error (%)",
        save_path="joint_torque_percentage_errors" + file_suffix + ".png",
    )

    # Statistics
    print("\n=== Percentage Error Statistics ===")

    print("\nJoint State Percentage Errors (%):")
    for key, errors in joint_state_perc_errors.items():
        valid = [e for e in errors if e is not None and not math.isnan(e)]
        if valid:
            avg_err = sum(valid) / len(valid)
            max_err = max(valid)
            print(f"  {key}: Avg={avg_err:.4f}%, Max={max_err:.4f}%")

    print("\nTorque Percentage Errors (%):")
    for key, errors in torque_perc_errors.items():
        valid = [e for e in errors if e is not None and not math.isnan(e)]
        if valid:
            avg_err = sum(valid) / len(valid)
            max_err = max(valid)
            print(f"  {key}: Avg={avg_err:.4f}%, Max={max_err:.4f}%")

    print("\nPlots saved as 'joint_state_actual_vs_simplified" + file_suffix + ".png', 'joint_torque_actual_vs_simplified" + file_suffix + ".png',\n'joint_state_percentage_errors" + file_suffix + ".png', and 'joint_torque_percentage_errors" + file_suffix + ".png'")
    plt.show()

if __name__ == "__main__":
    main()
