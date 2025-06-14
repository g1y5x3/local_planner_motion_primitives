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

# Generating paths
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

# Plot generated paths
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

# Parameters for patch matching and blocking
# These parameters are also used in the local_planner node
# TODO: make it read from a yaml file

voxel_size = 0.05
x_min = 0
x_max = 3.2
y_min = -3
y_max = 3
num_voxels_x = int(math.ceil((x_max - x_min) / voxel_size))
num_voxels_y = int(math.ceil((y_max - y_min) / voxel_size))

voxel_points = []
for ix in range(num_voxels_x):
    for iy in range(num_voxels_y):
        # without the + 0.5, the calculation would give the coordinates of the 
        # bottom-left corner of each voxel instead of the center of the voxel.
        x_center = x_min + (ix + 0.5) * voxel_size
        y_center = y_min + (iy + 0.5) * voxel_size
        voxel_points.append([x_center, y_center])

voxel_points = np.array(voxel_points)

# Plot voxels
fig = plt.figure(figsize=(10, 12))
ax = fig.add_subplot(111)
ax.plot(voxel_points[:, 0], voxel_points[:, 1], marker='x', color='blue', linestyle='none')

highlight_idx = 1494
ax.plot(voxel_points[highlight_idx, 0], voxel_points[highlight_idx, 1],
        marker='o', color='red', markersize=12, markeredgewidth=2, label='Highlighted Voxel')

ax.set_title(f'Voxel Points Visualization\n{len(voxel_points)} voxels\n'
             f'voxel x num {num_voxels_x}\nvoxel y num {num_voxels_y}\n'
             f'highlighted voxel index {highlight_idx} (x: {voxel_points[highlight_idx, 0]}, y: {voxel_points[highlight_idx, 1]})')
ax.set_xlim([-0.2, 3.5])
ax.set_ylim([-4.5, 4.5])
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
ax.legend()
plt.show()

#Collision checking (Memory Intensitve)
# For every points in the path, check whether there is any point next to it within the radius in the voxel points
search_radius = 0.085
path_points = np.vstack([path[:, :2] for path in path_all])
print(path_points.shape)
kdtree = cKDTree(path_points)
indices = kdtree.query_ball_point(voxel_points, search_radius)
print(indices.shape)
print(indices[highlight_idx])

# Plot voxels with correspondence
fig = plt.figure(figsize=(10, 12))
ax = fig.add_subplot(111)

for i, index in enumerate(indices):
    if index:
        ax.plot(voxel_points[i, 0], voxel_points[i, 1], marker='o', color='red', linestyle='none')
    else:
        ax.plot(voxel_points[i, 0], voxel_points[i, 1], marker='x', color='blue', linestyle='none')

ax.set_title(f'Voxel Points Visualization with Correspondence\n{len(voxel_points)} voxels')
ax.set_xlim([-0.2, 3.5])
ax.set_ylim([-4.5, 4.5])
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.show()

# Save the data into txt file
array = np.vstack(path_start_all)
np.savetxt('pregen_path_start.txt', array, fmt="%f %f %f %d", delimiter='')

array = np.vstack(path_all)
np.savetxt('pregen_path_all.txt', array, fmt="%f %f %f %d %d", delimiter='')

# Each row starting with an index represents a voxel, the following indices are the generated paths
# that pass through the voxel
with open('pregen_voxel_path_corr.txt', 'w') as f:
    array = np.vstack(path_all)
    for idx, sublist in enumerate(indices):
        f.write(f"{idx} ")
        
        path_id_rec = -1
        for list_id in sublist:
            path_id = int(array[list_id, 3])

            if path_id_rec == path_id:
                continue

            f.write(f"{path_id} ")
            path_id_rec = path_id

        f.write("-1\n")