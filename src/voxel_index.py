import numpy as np
import math
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional, Union

class UniformGridIndexer:
    """
    A uniform grid spatial indexer providing O(1) coordinate-to-index mapping.
    Designed for real-time robotics applications with fixed grid dimensions.
    """
    
    def __init__(self, 
                 x_min: float, x_max: float, 
                 y_min: float, y_max: float,
                 voxel_size: float,
                 tolerance: float = 1e-6):
        """
        Initialize grid with specified boundaries and resolution.
        
        Parameters:
        x_min, x_max: X-axis boundaries (meters)
        y_min, y_max: Y-axis boundaries (meters)
        voxel_size: Grid cell size (meters)
        tolerance: Floating point comparison tolerance
        """
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.voxel_size = voxel_size
        self.tolerance = tolerance
        
        # Calculate grid dimensions and adjust boundaries
        self.num_cells_x = int(math.ceil((x_max - x_min) / voxel_size))
        self.num_cells_y = int(math.ceil((y_max - y_min) / voxel_size))
        self.total_cells = self.num_cells_x * self.num_cells_y
        
        # Actual grid boundaries aligned with voxel edges
        self.actual_x_max = x_min + self.num_cells_x * voxel_size
        self.actual_y_max = y_min + self.num_cells_y * voxel_size
        
        # Precompute all voxel centers for O(1) reverse lookup
        self.voxel_centers = self._precompute_voxel_centers()

    def _precompute_voxel_centers(self) -> np.ndarray:
        """Generate array of all voxel center coordinates."""
        centers = np.zeros((self.total_cells, 2))
        idx = 0
        for ix in range(self.num_cells_x):
            for iy in range(self.num_cells_y):
                x_center = self.x_min + (ix + 0.5) * self.voxel_size
                y_center = self.y_min + (iy + 0.5) * self.voxel_size
                centers[idx] = [x_center, y_center]
                idx += 1

        fig = plt.figure(figsize=(10, 12))
        ax = fig.add_subplot(111)
        ax.plot(centers[:, 0], centers[:, 1], marker='x', color='blue', linestyle='none')
        # ax.set_title(f'Voxel Points Visualization\n{len(voxel_points)} voxels\nvoxel x num {voxel_num_x}\nvoxel y num {voxel_num_y}')
        # ax.set_xlim([-0.2, 3.5])
        # ax.set_ylim([-4.5, 4.5])
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.show()

        return centers

    def world_to_grid_coords(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid indices."""
        if not (self.x_min <= x <= self.actual_x_max and 
                self.y_min <= y <= self.actual_y_max):
            raise ValueError(f"Coordinates ({x:.3f}, {y:.3f}) out of bounds")
        
        ix = int((x - self.x_min) // self.voxel_size)
        iy = int((y - self.y_min) // self.voxel_size)
        
        # Clamp to valid indices
        ix = min(max(ix, 0), self.num_cells_x - 1)
        iy = min(max(iy, 0), self.num_cells_y - 1)
        
        return ix, iy

    def world_to_flat_index(self, x: float, y: float) -> int:
        """Direct O(1) conversion from world coordinates to 1D index."""
        ix, iy = self.world_to_grid_coords(x, y)
        return ix * self.num_cells_y + iy

    def flat_index_to_world(self, flat_index: int) -> Tuple[float, float]:
        """Convert 1D index back to world coordinates."""
        if not (0 <= flat_index < self.total_cells):
            raise ValueError(f"Index {flat_index} out of range")
        return tuple(self.voxel_centers[flat_index])

    def get_neighboring_indices(self, flat_index: int, radius: int = 1) -> List[int]:
        """Get indices of neighboring cells within specified radius."""
        ix, iy = self.flat_index_to_grid_coords(flat_index)
        neighbors = []
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                nx, ny = ix + dx, iy + dy
                if 0 <= nx < self.num_cells_x and 0 <= ny < self.num_cells_y:
                    neighbors.append(nx * self.num_cells_y + ny)
        return neighbors

# Example usage for robotic path planning
if __name__ == "__main__":
    # Initialize grid matching your original parameters
    grid = UniformGridIndexer(
        x_min=0.0,
        x_max=3.2,
        y_min=-4.5,
        y_max=4.5,
        voxel_size=0.05
    )


    
    # Test coordinate conversion
    test_points = [
        (1.5, 2.0),
        (3.1, -4.4),
        (0.05, 0.05),
        (2.8, 3.0)
    ]
    
    for x, y in test_points:
        try:
            idx = grid.world_to_flat_index(x, y)
            x_rec, y_rec = grid.flat_index_to_world(idx)
            error = math.hypot(x - x_rec, y - y_rec)
            print(f"World: ({x:.2f}, {y:.2f}) → Index: {idx} → "
                  f"Reconstructed: ({x_rec:.2f}, {y_rec:.2f}), Error: {error:.4f}m")
        except ValueError as e:
            print(f"Error processing ({x:.2f}, {y:.2f}): {e}")