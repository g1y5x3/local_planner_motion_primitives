import math
import numpy as np
from scipy.spatial import cKDTree
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

# Parameters for path generation
dis = 1.0
angle = 27
delta_angle = angle / 3
scale = 0.65

# 1. Generating paths
path_start_all = []
path_all = []
path_list = []

path_id = 0
group_id = 0

for shift1 in np.arange(-angle, angle+0.1, delta_angle):
    waypts_start = np.array([[0, 0], [dis, shift1]])
    
    path_start_r = np.arange(0, dis+0.01, 0.01)
    cs_start = CubicSpline(waypts_start[:, 0], waypts_start[:, 1])
    path_start_shift = cs_start(path_start_r)
    
    path_start_x = path_start_r * np.cos(np.radians(path_start_shift))
    path_start_y = path_start_r * np.sin(np.radians(path_start_shift))
    path_start_z = np.zeros_like(path_start_x)
    
    path_start = np.column_stack((path_start_x, path_start_y, path_start_z, np.ones_like(path_start_x) * group_id))
    path_start_all.append(path_start)
    
    for shift2 in np.arange(-angle * scale + shift1, (angle * scale + shift1)+0.1, delta_angle*scale):
        for shift3 in np.arange(-angle * scale**2 + shift2, (angle * scale**2 + shift2)+0.1, delta_angle*scale**2):
            initial_waypts = np.column_stack((path_start_r, path_start_shift))
            additional_waypts = np.array([
                [2 * dis, shift2],
                [3 * dis - 0.001, shift3],
                [3 * dis, shift3]
            ])
            waypts = np.vstack((initial_waypts, additional_waypts))
            
            path_r = np.arange(0, waypts[-1, 0]+0.01, 0.01)
            cs = CubicSpline(waypts[:, 0], waypts[:, 1])
            path_shift = cs(path_r)
            
            path_x = path_r * np.cos(np.radians(path_shift))
            path_y = path_r * np.sin(np.radians(path_shift))
            path_z = np.zeros_like(path_x)
            path = np.column_stack((path_x, path_y, path_z, np.ones_like(path_x) * path_id, np.ones_like(path_x) * group_id))

            path_all.append(path)
            path_list.append([path_x[-1], path_y[-1], path_z[-1], path_id, group_id])
            # plt.plot(path_x, path_y)
            path_id += 1
    
    group_id += 1

# plot generated paths
group_label = [0, 0, 0, 0, 0, 0, 0]
colors = plt.cm.tab10(np.linspace(0, 1, 7))
color_list = [tuple(color[:3]) for color in colors]

start_x = path_start_all[0]

fig = plt.figure(figsize=(10, 12))
ax = fig.add_subplot(111)

for path, path_l  in zip(path_all, path_list):
    if group_label[path_l[4]] == 0:
        ax.plot(path[:, 0], path[:, 1], color=color_list[path_l[4]], label=f"Path All Group {path_l[4]}")
        group_label[path_l[4]] = 1
    ax.plot(path[:, 0], path[:, 1], color=color_list[path_l[4]])
ax.plot(path_start_all[0][:, 0], path_start_all[0][:,1], marker='*', color='red', label='Path Start')
ax.plot(path_list[0][0], path_list[0][1], marker='x', color='red', markersize=15, markeredgewidth=2, label='Path List')

ax.set_xlim([-0.2, 3.2])
ax.set_ylim([-3, 3])

ax.set_title(f'Paths Visualization colored by group id \n{len(path_all)} paths with {len(path)} points each path')
ax.legend()
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.show()

# 2. Efficiently generate a sparse voxel grid containing only voxels near a path
# Parameters for patch matching and blocking
# These parameters are also used in the local_planner node
# TODO: make it read from a yaml file
x_min, x_max = 0, 3
y_min, y_max = -3, 3
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
ax.set_xlim([-0.2, 3.5])
ax.set_ylim([-3, 3])
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
