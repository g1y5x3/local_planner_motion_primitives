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

search_radius = 0.45
voxel_size = 0.05
offset_x = 3.2
offset_y = 4.5
voxel_num_x = int((offset_x / voxel_size) + 1)
voxel_num_y = int(2 * (offset_y / voxel_size) + 1)

# Divide the space within the defined search area into voxels
# TODO: currently its assigning the same amout of points regardless of the scale_y
voxel_points = []
num_iterations = []
for ind_x in range(voxel_num_x):
    x = offset_x - voxel_size * ind_x
    scale_y = x / offset_x + search_radius / offset_y * (offset_x - x) / offset_x
    num_iterations.append((voxel_num_y + int(1/scale_y) - 1) // int(1/scale_y))
    print(f"scale_y {scale_y}")
    print(f"{(voxel_num_y + int(1/scale_y) - 1) // int(1/scale_y)}")
    for ind_y in range(0, voxel_num_y, int(1/scale_y)):
        print(f"ind_y {ind_y}")
        y = scale_y * (offset_y - voxel_size * ind_y)
        voxel_points.append([x, y])
print(f"totoal num {sum(num_iterations)}")

voxel_points = np.array(voxel_points)

# Plot voxels
fig = plt.figure(figsize=(10, 12))
ax = fig.add_subplot(111)
ax.plot(voxel_points[:, 0], voxel_points[:, 1], marker='x', color='blue', linestyle='none')
ax.set_title(f'Voxel Points Visualization\n{len(voxel_points)} voxels\nvoxel x num {voxel_num_x}\nvoxel y num {voxel_num_y}')
ax.set_xlim([-0.2, 3.5])
ax.set_ylim([-4.5, 4.5])
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.show()

#Collision checking (Memory Intensitve)
# For every points in the path, check whether there is any point next to it within the radius in the voxel points
path_points = np.vstack([path[:, :2] for path in path_all])
kdtree = cKDTree(path_points)
indices = kdtree.query_ball_point(voxel_points, search_radius)

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

array = np.vstack(path_list)
np.savetxt('pregen_path_list.txt', array, fmt="%f %f %f %d %d", delimiter='')

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