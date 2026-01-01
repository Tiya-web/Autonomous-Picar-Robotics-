import numpy as np
import matplotlib.pyplot as plt
import os

# Ask for filename
file_name = input("Enter the data file name (e.g., data_kp_8.7.txt): ").strip()

# Read file contents
with open(file_name) as file:
    lines = file.read().splitlines()

# Prepare time and RPS arrays
time_arr = []
rps_arr = []

# Skip header and parse values
for line in lines[1:]:
    values = line.split()
    time_arr.append(float(values[0]))       # Time
    rps_arr.append(float(values[2]))        # RPS

# Convert to numpy arrays
time_arr = np.array(time_arr)
rps_arr = np.array(rps_arr)

# Plot
plt.figure(figsize=(8, 5))
plt.plot(time_arr, rps_arr, label='RPS (rev/sec)', color='blue')
plt.xlabel('Time (s)')
plt.ylabel('RPS')
plt.title('RPS vs Time')
plt.grid(True)
plt.legend()

# Generate output file name
base_name = os.path.splitext(file_name)[0]
plot_name = base_name.replace("data", "plot") + ".png"

# Save and show plot
plt.savefig(plot_name)
print(f"Plot saved as {plot_name}")
plt.show()

