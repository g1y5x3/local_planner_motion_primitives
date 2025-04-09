import numpy as np
from scipy.spatial import cKDTree
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

# Parameters for path generation
dis = 1.0
angle = 27
delta_angle = angle / 3
scale = 0.65

path_start_all = []
path_all = []
path_list = []
path_id = 0
group_id = 0

# Generate paths
print("\nGenerating paths\n")

for shift1 in np.arange(-angle, angle + delta_angle, delta_angle):
    waypts_start = np.array([[0, 0], [dis, shift1]])
    path_start_r = np.arange(0, dis, 0.01)
    cs_start = CubicSpline(waypts_start[:, 0], waypts_start[:, 1])
    path_start_shift = cs_start(path_start_r)

    path_start_x = path_start_r * np.cos(np.radians(path_start_shift))
    path_start_y = path_start_r * np.sin(np.radians(path_start_shift))
    path_start_z = np.zeros_like(path_start_x)

    path_start = np.vstack((path_start_x, path_start_y, path_start_z, np.ones_like(path_start_x) * group_id)).T
    path_start_all.append(path_start)

    for shift2 in np.arange(-angle * scale + shift1, angle * scale + shift1 + delta_angle * scale, delta_angle * scale):
        for shift3 in np.arange(-angle * scale**2 + shift2, angle * scale**2 + shift2 + delta_angle * scale**2, delta_angle * scale**2):
            waypts = np.array([
                [path_start_r[-1], path_start_shift[-1]],
                [2 * dis, shift2],
                [3 * dis - 0.001, shift3],
                [3 * dis, shift3]
            ])

            path_r = np.arange(0, waypts[-1, 0], 0.01)
            cs = CubicSpline(waypts[:, 0], waypts[:, 1])
            path_shift = cs(path_r)

            path_x = path_r * np.cos(np.radians(path_shift))
            path_y = path_r * np.sin(np.radians(path_shift))
            path_z = np.zeros_like(path_x)

            path = np.vstack((path_x, path_y, path_z, np.ones_like(path_x) * path_id, np.ones_like(path_x) * group_id)).T
            path_all.append(path)
            path_list.append([path_x[-1], path_y[-1], path_z[-1], path_id, group_id])

            path_id += 1

    group_id += 1

# Plot paths
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for path in path_all:
    ax.plot(path[:, 0], path[:, 1], path[:, 2])
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.show()

print("Path Lists")
print(*path_list, sep='\n')
print("Path generation complete.")

# Parameters for patch matching and blocking
# these parameters should be the same as the local_planner node 
voxel_size = 0.05
search_radius = 0.45
offset_x = 3.2
offset_y = 4.5
voxel_num_x = int((offset_x / voxel_size) + 1)
voxel_num_y = int(2 * (offset_y / voxel_size) + 1)

# Prepare voxels
print("\nPreparing voxels\n")
voxel_points = []
for ind_x in range(voxel_num_x):
    x = offset_x - voxel_size * ind_x
    scale_y = x / offset_x + search_radius / offset_y * (offset_x - x) / offset_x
    for ind_y in range(voxel_num_y):
        y = scale_y * (offset_y - voxel_size * ind_y)
        voxel_points.append([x, y, 0])

voxel_points = np.array(voxel_points)

# Plot paths
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# for path in path_all:
ax.plot(voxel_points[:, 0], voxel_points[:, 1], voxel_points[:, 2])
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.show()

# # Collision checking
# print("\nCollision checking\n")
# path_points = np.vstack([path[:, :2] for path in path_all])

# print(f"path_points {path_points}")
# print(f"voxel_points {voxel_points}")

# kdtree = cKDTree(path_points)
# indices = kdtree.query_ball_point(voxel_points, search_radius)

# # Save correspondences
# print("\nSaving correspondences\n")
# correspondences = []
# for i, ind in enumerate(indices):
#     unique_paths = np.unique([path_all[int(idx)][0, 3] for idx in ind])
#     correspondences.append((i, unique_paths.tolist()))

# # Plot voxel points
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(voxel_points[:, 0], voxel_points[:, 1], np.zeros(len(voxel_points)), c='k', marker='.')
# plt.show()

# print("Processing complete.")