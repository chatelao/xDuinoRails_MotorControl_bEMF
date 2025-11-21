import os
import subprocess
import sys
import matplotlib
matplotlib.use('Agg') # Use non-interactive backend for CI
import matplotlib.pyplot as plt
import numpy as np

def run_simulation():
    # Change to simulation directory
    sim_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(sim_dir)

    # Run ngspice if result doesn't exist or forcing update
    # For CI, we always run it. locally, we might want to skip if exists.
    # But "run the simulation" implies running it.
    print("Running ngspice...")
    try:
        subprocess.run(["ngspice", "-b", "motor.net"], check=True)
    except FileNotFoundError:
        print("Error: ngspice not found. Please install ngspice.")
        sys.exit(1)
    except subprocess.CalledProcessError as e:
        print(f"Error running ngspice: {e}")
        sys.exit(1)

    if not os.path.exists("simulation_result.txt"):
        print("Error: simulation_result.txt not generated.")
        sys.exit(1)

    print("Parsing results...")
    data = parse_ngspice_ascii("simulation_result.txt")

    if not data:
        print("Error: No data parsed.")
        sys.exit(1)

    print(f"Available signals: {list(data.keys())}")

    print("Plotting results...")
    plot_results(data)
    print(f"Simulation complete. Graph saved to {os.path.join(sim_dir, 'simulation_result.png')}")

def parse_ngspice_ascii(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()

    values = []
    in_vars = False
    in_vals = False
    var_map = {} # index -> name
    current_point = []

    for line in lines:
        line = line.strip()
        if line.startswith("Variables:"):
            in_vars = True
            continue
        if line.startswith("Values:"):
            in_vars = False
            in_vals = True
            continue

        if in_vars:
            parts = line.split()
            if len(parts) >= 3:
                idx = int(parts[0])
                name = parts[1]
                var_map[idx] = name

        if in_vals:
            if not line: continue
            parts = line.split()

            if len(parts) == 2: # Start of new point: Index Value
                if current_point:
                    values.append(current_point)
                current_point = [float(parts[1])]
            elif len(parts) == 1:
                try:
                    current_point.append(float(parts[0]))
                except ValueError:
                    pass # Ignore non-float lines if any
            elif len(parts) > 2:
                # Handle case where multiple values are on one line (rare but possible)
                if current_point:
                     values.append(current_point)
                current_point = [float(x) for x in parts[1:]]

    if current_point:
        values.append(current_point)

    # Filter out incomplete points if any
    if values:
        expected_len = len(var_map)
        clean_values = [row for row in values if len(row) == expected_len]
        if len(clean_values) < len(values):
            print(f"Warning: Filtered out {len(values) - len(clean_values)} incomplete points.")
        data_arr = np.array(clean_values)
    else:
        data_arr = np.array([])

    result = {}
    for idx, name in var_map.items():
        if idx < data_arr.shape[1]:
            result[name] = data_arr[:, idx]

    return result

def plot_results(data):
    # Case insensitive lookup
    keys = {k.lower(): k for k in data.keys()}

    if 'time' not in keys:
        print("Error: 'time' variable not found in results.")
        return

    time = data[keys['time']]

    # Try to find voltage and current variables
    voltage_key = keys.get('v(1)', None)
    current_key = keys.get('i(vsense)', None)

    if not voltage_key or not current_key:
        print(f"Error: Could not find expected variables 'v(1)' or 'i(vsense)'. Found: {list(data.keys())}")
        return

    voltage = data[voltage_key]
    current = data[current_key]

    fig, ax1 = plt.subplots(figsize=(10, 6))

    color = 'tab:red'
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Voltage (V)', color=color)
    ax1.plot(time, voltage, color=color, label='PWM Voltage', alpha=0.6)
    ax1.tick_params(axis='y', labelcolor=color)
    ax1.grid(True)

    ax2 = ax1.twinx()
    color = 'tab:blue'
    ax2.set_ylabel('Current (A)', color=color)
    ax2.plot(time, current, color=color, label='Motor Current')
    ax2.tick_params(axis='y', labelcolor=color)

    plt.title('Motor Simulation: 50% PWM at 20V')
    fig.tight_layout()
    plt.savefig('simulation_result.png')

if __name__ == "__main__":
    run_simulation()
