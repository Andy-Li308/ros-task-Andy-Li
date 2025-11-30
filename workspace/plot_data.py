import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# Load csv file
file_path = "workspace/controller_log.csv"  
df = pd.read_csv(file_path)

# compute errors 
df['x_error'] = df['Target_X'] - df['Current_X']
df['y_error'] = df['Target_Y'] - df['Current_Y']

# Create a folder to save plots
save_folder = "plots"
os.makedirs(save_folder, exist_ok=True)

# plot x error as a function of time elapsed
plt.figure(figsize=(10, 5))
plt.plot(df['Time'], df['x_error'], label='X Error', color='r')
plt.xlabel('Time (s)')
plt.ylabel('X Error (m)')
plt.title('X Error as a Function of Time')
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(save_folder, "x_error.png"))
plt.close()

# plot y error as a function of time elapsed
plt.figure(figsize=(10, 5))
plt.plot(df['Time'], df['y_error'], label='Y Error', color='b')
plt.xlabel('Time (s)')
plt.ylabel('Y Error (m)')
plt.title('Y Error as a Function of Time')
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(save_folder, "y_error.png"))
plt.close()

# Calculate heading vector components for arrows
arrow_length = 0.2  # adjust for visibility
dx = arrow_length * np.cos(df['Current_Theta'])
dy = arrow_length * np.sin(df['Current_Theta'])

# plot trajectory with heading arrows
# Create plot
plt.figure(figsize=(10, 10))
plt.plot(df['Current_X'], df['Current_Y'], 'bo-', label='Robot Path')  # line with markers
plt.quiver(df['Current_X'], df['Current_Y'], dx, dy, angles='xy', scale_units='xy', scale=1, color='r', label='Heading')

plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Robot Path with Orientation')
plt.legend()
plt.grid(True)
plt.axis('equal')  # keeps x and y scales equal

# Save plot
plt.savefig(os.path.join(save_folder, "robot_path.png"))
plt.close()

print(f"plots saved in folder: {save_folder}")
print("Done plotting")
