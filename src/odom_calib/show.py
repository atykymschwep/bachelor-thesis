import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse

# COMMAND LINE ARGUMENTS SETUP
parser = argparse.ArgumentParser(description="Odometry calibration visualization.")
parser.add_argument('--mode', type=str, choices=['old', 'new', 'both'], default='both',
                    help="What to display: 'old', 'new', 'both'. Default is 'both'.")
args = parser.parse_args()

show_old = args.mode in ['old', 'both']
show_new = args.mode in ['new', 'both']

# read data from CSV file
try:
    data = pd.read_csv('trajectory_data.csv')
except FileNotFoundError:
    print("Error: File 'trajectory_data.csv' not found. Please run the C++ program first.")
    exit()


data['error_old'] = np.sqrt((data['gt_x'] - data['old_x'])**2 + (data['gt_y'] - data['old_y'])**2)
data['error_new'] = np.sqrt((data['gt_x'] - data['new_x'])**2 + (data['gt_y'] - data['new_y'])**2)
time_steps = np.arange(len(data))

# ANIMATION WINDOW SETUP
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7), gridspec_kw={'width_ratios': [1.5, 1]})
fig.canvas.manager.set_window_title(f"Odometry Calibration Analysis (Mode: {args.mode.upper()})")

# Setup Left Plot (Trajectory)
padding = 0.5
ax1.set_xlim(data[['gt_x', 'old_x', 'new_x']].min().min() - padding, 
             data[['gt_x', 'old_x', 'new_x']].max().max() + padding)
ax1.set_ylim(data[['gt_y', 'old_y', 'new_y']].min().min() - padding, 
             data[['gt_y', 'old_y', 'new_y']].max().max() + padding)
ax1.set_aspect('equal')
ax1.grid(True, linestyle='--', color='lightgray')
ax1.set_title('Movement Trajectory', fontsize=14)
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')

line_gt, = ax1.plot([], [], label='OptiTrack (Ground Truth)', color='dimgray', linewidth=3)
start_dot, = ax1.plot([], [], 'go', markersize=8, label='Start')

if show_old:
    line_old, = ax1.plot([], [], label='Old Odometry', color='crimson', linestyle='--', linewidth=2)
if show_new:
    line_new, = ax1.plot([], [], label='New Odometry (Calibrated)', color='royalblue', linestyle='-', linewidth=2)

ax1.legend(loc='upper left')

if not data.empty:
    start_dot.set_data([0], [0])

# Setup Right Plot (Error)
ax2.set_xlim(0, len(data))
max_err = 0
if show_old: max_err = max(max_err, data['error_old'].max())
if show_new: max_err = max(max_err, data['error_new'].max())
ax2.set_ylim(0, max_err * 1.1 if max_err > 0 else 1.0)

ax2.grid(True, linestyle='--', color='lightgray')
ax2.set_title('Error Accumulation (Deviation from OptiTrack)', fontsize=14)
ax2.set_xlabel('Steps (Time)')
ax2.set_ylabel('Error (Meters)')

if show_old:
    err_line_old, = ax2.plot([], [], label='Old Odom. Error', color='crimson', linewidth=2)
    text_err_old = ax2.text(0.05, 0.85, '', transform=ax2.transAxes, fontsize=12, color='crimson', fontweight='bold')
if show_new:
    err_line_new, = ax2.plot([], [], label='New Odom. Error', color='royalblue', linewidth=2)
    text_err_new = ax2.text(0.05, 0.78, '', transform=ax2.transAxes, fontsize=12, color='royalblue', fontweight='bold')

ax2.legend(loc='upper left')

# ANIMATION 
step = max(1, len(data) // 1)
print(f"Starting animation. Total points: {len(data)}, draw step: {step}...")

plt.ion() 
plt.show()

for i in range(1, len(data), step):
    current_chunk = data.iloc[:i]
    current_time = time_steps[:i]
    
    line_gt.set_data(current_chunk['gt_x'], current_chunk['gt_y'])
    
    if show_old:
        line_old.set_data(current_chunk['old_x'], current_chunk['old_y'])
        err_line_old.set_data(current_time, current_chunk['error_old'])
        cur_e_old = current_chunk['error_old'].iloc[-1]
        text_err_old.set_text(f"Old error: {cur_e_old:.3f} m")
        
    if show_new:
        line_new.set_data(current_chunk['new_x'], current_chunk['new_y'])
        err_line_new.set_data(current_time, current_chunk['error_new'])
        cur_e_new = current_chunk['error_new'].iloc[-1]
        text_err_new.set_text(f"New error: {cur_e_new:.3f} m")
        
    plt.pause(0.01)

# Draw up to 100%
line_gt.set_data(data['gt_x'], data['gt_y'])


dx = data['gt_x'].diff()
dy = data['gt_y'].diff()

total_path_length = np.sqrt(dx**2 + dy**2).sum()

print(f"Total Traveled Distance (Ground Truth): {total_path_length:.2f} meters")


print("\nFINAL RESULTS")
if show_old:
    line_old.set_data(data['old_x'], data['old_y'])
    err_line_old.set_data(time_steps, data['error_old'])
    fin_e_old = data['error_old'].iloc[-1]
    
    pct_e_old = (fin_e_old / total_path_length) * 100
    
    text_err_old.set_text(f"Final old: {fin_e_old:.3f} m ({pct_e_old:.2f}%)")
    print(f"Final error of OLD odometry: {fin_e_old:.3f} m ({pct_e_old:.2f}% of path)")

if show_new:
    line_new.set_data(data['new_x'], data['new_y'])
    err_line_new.set_data(time_steps, data['error_new'])
    fin_e_new = data['error_new'].iloc[-1]
    
    pct_e_new = (fin_e_new / total_path_length) * 100
    
    text_err_new.set_text(f"Final new: {fin_e_new:.3f} m ({pct_e_new:.2f}%)")
    print(f"Final error of NEW odometry: {fin_e_new:.3f} m ({pct_e_new:.2f}% of path)")

plt.ioff()
plt.show()
