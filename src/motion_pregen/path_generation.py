import math
import numpy as np
from scipy.spatial import cKDTree
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

# Parameters for path generation
# Strategy: Generate base paths with steering variations, then ROTATE them to cover -90° to +90°
# This ensures uniform path shapes across all directions (no distortion at extreme angles)
# Optimized: Only 49 paths per rotation group (7x7 variations) instead of 343
dis = 1.0
steering_angle = 27       # Steering angle range for path variations
delta_steering = 9        # Steering step (gives 7 values: -27, -18, -9, 0, 9, 18, 27)
scale = 0.65              # Scale factor for secondary/tertiary adjustments

# Rotation parameters - these define the output direction coverage
rotation_min = -90        # Minimum rotation angle
rotation_max = 90         # Maximum rotation angle
rotation_step = 10        # Step between rotation groups (19 groups total)

def rotate_point(x, y, angle_deg):
    """Rotate a point (x, y) by angle_deg degrees around origin."""
    angle_rad = np.radians(angle_deg)
    x_rot = x * np.cos(angle_rad) - y * np.sin(angle_rad)
    y_rot = x * np.sin(angle_rad) + y * np.cos(angle_rad)
    return x_rot, y_rot

# 1. Generate BASE paths (forward-facing, centered at shift1=0)
# Only generate 49 paths (7x7 from shift2/shift3 variations) - rotation handles direction
print("Generating base paths (optimized: 49 paths from middle steering group)...")
base_paths = []

# Use shift1=0 (forward-facing) as base - rotation will handle other directions
shift1 = 0
waypts_start = np.array([[0, 0], [dis, shift1]])

path_start_r = np.arange(0, dis + 0.01, 0.01)
cs_start = CubicSpline(waypts_start[:, 0], waypts_start[:, 1])
path_start_shift = cs_start(path_start_r)

path_start_x = path_start_r * np.cos(np.radians(path_start_shift))
path_start_y = path_start_r * np.sin(np.radians(path_start_shift))

# Store base path start (straight forward)
base_path_start = np.column_stack((path_start_x, path_start_y))

# Generate 7x7 = 49 path variations from shift2 and shift3
for shift2 in np.arange(-steering_angle * scale + shift1, (steering_angle * scale + shift1) + 0.1, delta_steering * scale):
    for shift3 in np.arange(-steering_angle * scale**2 + shift2, (steering_angle * scale**2 + shift2) + 0.1, delta_steering * scale**2):
        initial_waypts = np.column_stack((path_start_r, path_start_shift))
        additional_waypts = np.array([
            [2 * dis, shift2],
            [3 * dis - 0.001, shift3],
            [3 * dis, shift3]
        ])
        waypts = np.vstack((initial_waypts, additional_waypts))

        path_r = np.arange(0, waypts[-1, 0] + 0.01, 0.01)
        cs = CubicSpline(waypts[:, 0], waypts[:, 1])
        path_shift = cs(path_r)

        path_x = path_r * np.cos(np.radians(path_shift))
        path_y = path_r * np.sin(np.radians(path_shift))

        base_paths.append(np.column_stack((path_x, path_y)))

num_base_paths = len(base_paths)
print(f"Generated {num_base_paths} base paths (7x7 steering variations)")

# 2. Now rotate base paths to cover full -90° to +90° range
print("Rotating paths to cover full angle range...")
path_start_all = []
path_all = []
path_list = []

path_id = 0
group_id = 0

for rotation_angle in np.arange(rotation_min, rotation_max + 0.1, rotation_step):
    # Rotate the base path start (straight line) to this direction
    rot_start_x, rot_start_y = rotate_point(base_path_start[:, 0], base_path_start[:, 1], rotation_angle)
    path_start_z = np.zeros_like(rot_start_x)
    path_start = np.column_stack((rot_start_x, rot_start_y, path_start_z, np.ones_like(rot_start_x) * group_id))
    path_start_all.append(path_start)

    # Rotate all 49 base paths to this direction
    for base_path in base_paths:
        rot_x, rot_y = rotate_point(base_path[:, 0], base_path[:, 1], rotation_angle)
        path_z = np.zeros_like(rot_x)
        path = np.column_stack((rot_x, rot_y, path_z, np.ones_like(rot_x) * path_id, np.ones_like(rot_x) * group_id))

        path_all.append(path)
        path_list.append([rot_x[-1], rot_y[-1], 0, path_id, group_id])
        path_id += 1

    group_id += 1

# Calculate statistics
num_rotation_groups = len(np.arange(rotation_min, rotation_max + 0.1, rotation_step))
num_paths = path_id
paths_per_group = num_base_paths  # Each rotation group has all base paths
print(f"\nFinal output: {num_paths} paths in {num_rotation_groups} rotation groups ({paths_per_group} paths/group)")
print(f"Rotation range: {rotation_min}° to {rotation_max}° with {rotation_step}° steps")
print(f"Base steering: ±{steering_angle}° with {delta_steering}° steps")

# Use num_groups for compatibility with rest of script
num_groups = num_rotation_groups

# plot generated paths
group_label = [0] * num_groups
colors = plt.cm.tab20(np.linspace(0, 1, num_groups))  # Use tab20 for more colors
color_list = [tuple(color[:3]) for color in colors]

start_x = path_start_all[0]

fig = plt.figure(figsize=(10, 12))
ax = fig.add_subplot(111)

for path, path_l  in zip(path_all, path_list):
    gid = int(path_l[4])
    if group_label[gid] == 0:
        ax.plot(path[:, 0], path[:, 1], color=color_list[gid], label=f"Group {gid}")
        group_label[gid] = 1
    else:
        ax.plot(path[:, 0], path[:, 1], color=color_list[gid])
ax.plot(path_start_all[0][:, 0], path_start_all[0][:,1], marker='*', color='red', label='Path Start')
ax.plot(path_list[0][0], path_list[0][1], marker='x', color='red', markersize=15, markeredgewidth=2, label='Path List')

# Expand plot limits for full angle range (-90° to +90°)
# Paths can extend up to 3m in any direction when rotated
ax.set_xlim([-3.5, 3.5])
ax.set_ylim([-3.5, 3.5])
ax.set_aspect('equal')

ax.set_title(f'Pre-generated Paths ({num_paths} paths in {num_groups} groups)\nRotation: {rotation_min}° to {rotation_max}° (uniform shape, pre-rotated)')
ax.legend()
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.show()

# 2. Efficiently generate a sparse voxel grid containing only voxels near a path
# Parameters for patch matching and blocking
# These parameters are also used in the local_planner node
# NOTE: Updated bounds for full -90° to +90° angle coverage (paths can extend 3m in any direction)
# TODO: make it read from a yaml file
x_min, x_max = -3.5, 3.5   # Extended for paths rotated to all directions
y_min, y_max = -3.5, 3.5   # Extended for full left/right coverage
voxel_size = 0.05
search_radius = 0.1

print("Generating sparse voxel grid...")
voxel_to_paths = {}
path_data = np.vstack(path_all)
path_points = path_data[:, :2]
path_ids = path_data[:, 3].astype(int)

# For each path point, find all voxels within search_radius
search_radius_sq = search_radius**2
search_box_half_width = math.ceil(search_radius / voxel_size)

for i, point in enumerate(path_points):
    px, py = point
    path_id = path_ids[i]
    
    center_ix = int(math.floor((px - x_min) / voxel_size))
    center_iy = int(math.floor((py - y_min) / voxel_size))
    
    for ix_offset in range(-search_box_half_width, search_box_half_width + 1):
        for iy_offset in range(-search_box_half_width, search_box_half_width + 1):
            ix = center_ix + ix_offset
            iy = center_iy + iy_offset
            
            vx = x_min + (ix + 0.5) * voxel_size
            vy = y_min + (iy + 0.5) * voxel_size
            
            dist_sq = (px - vx)**2 + (py - vy)**2
            
            if dist_sq <= search_radius_sq:
                if (ix, iy) not in voxel_to_paths:
                    voxel_to_paths[(ix, iy)] = set()
                voxel_to_paths[(ix, iy)].add(path_id)

print(f"Generated {len(voxel_to_paths)} occupied voxels.")

# Create voxel_points array for plotting
voxel_indices = np.array(list(voxel_to_paths.keys()))
voxel_points = np.zeros((len(voxel_indices), 2))
voxel_points[:, 0] = x_min + (voxel_indices[:, 0] + 0.5) * voxel_size
voxel_points[:, 1] = y_min + (voxel_indices[:, 1] + 0.5) * voxel_size

# Plot voxels
fig = plt.figure(figsize=(10, 12))
ax = fig.add_subplot(111)
ax.plot(voxel_points[:, 0], voxel_points[:, 1], 'bx', label='Occupied Voxels')
ax.set_title(f'Sparse Voxel Points Visualization\n{len(voxel_points)} voxels')
ax.set_xlim([-3.5, 3.5])
ax.set_ylim([-3.5, 3.5])
ax.set_aspect('equal')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
ax.legend()
plt.show()

# Save the data into txt file
# Data structure: x y z group_id
array = np.vstack(path_start_all)
np.savetxt('pregen_path_start.txt', array, fmt="%f %f %f %d", delimiter=' ')

# Data structure: x y z path_id group_id
array = np.vstack(path_all)
np.savetxt('pregen_path_all.txt', array, fmt="%f %f %f %d %d", delimiter=' ')

# Data Structure: ix iy path_id1 path_id2 ... -1
# This format is more suitable for a sparse grid.
print("Saving voxel-path correspondence file...")
with open('pregen_voxel_path_corr.txt', 'w') as f:
    for (ix, iy), path_ids in sorted(voxel_to_paths.items()):
        f.write(f"{ix} {iy} ")
        for path_id in sorted(list(path_ids)):
            f.write(f"{path_id} ")
        f.write("-1\n")
print("Done.")
