import numpy as np
import trimesh
import os

# Bucket dimensions in meters
BUCKET_LENGTH = 2.0  # Depth (Z direction)
BUCKET_WIDTH = 5.0   # Width (X direction)
BUCKET_HEIGHT = 3.0  # Height (Y direction)

def load_and_clean_pc(file_path):
    print(f"Loading {file_path}...")
    pc = trimesh.load(file_path)
    mask = np.isfinite(pc.vertices).all(axis=1)
    pc.vertices = pc.vertices[mask]
    if hasattr(pc, 'colors') and pc.colors is not None:
        pc.colors = pc.colors[mask]
    return pc

def identify_bucket_edge(pc, black_threshold=60):
    """
    Identify points that are 'black' (representing the metal bucket edge).
    """
    if not hasattr(pc, 'colors') or pc.colors is None:
        return None
        
    colors = pc.colors[:, :3]
    if np.max(colors) <= 1.0:
        black_threshold /= 255.0
        
    # Find black points
    mask = (colors < black_threshold).all(axis=1)
    edge_points = pc.vertices[mask]
    print(f"Detected {len(edge_points)} potential bucket edge points.")
    return edge_points, mask

def align_and_scale_by_edge(pc, edge_points):
    if edge_points is None or len(edge_points) < 10:
        print("Warning: Insufficient edge points. Using bounding box fallback.")
        pc.apply_translation(-np.mean(pc.vertices, axis=0))
        extents = pc.bounds[1] - pc.bounds[0]
        scale_factor = BUCKET_WIDTH / extents[0]
        pc.apply_scale(scale_factor)
        return pc

    # Center cloud around edge points
    edge_center = np.mean(edge_points, axis=0)
    pc.apply_translation(-edge_center)
    edge_points_centered = edge_points - edge_center
    
    # Span of edge points
    min_e, max_e = np.min(edge_points_centered, axis=0), np.max(edge_points_centered, axis=0)
    edge_width = max_e[0] - min_e[0]
    
    scale_factor = BUCKET_WIDTH / edge_width
    print(f"Scaling based on edge width ({edge_width:.2f}): {scale_factor:.6f}")
    pc.apply_scale(scale_factor)
    
    # Grounding: assume the edge points represent the top rim? 
    # Or if they are the floor? Usually the user said "black metal bucket edge".
    # Let's assume the edge points are roughly at the top or sides.
    # We'll ground the whole cloud so the lowest points are at Y=0.
    min_b, _ = pc.bounds
    pc.apply_translation([0, -min_b[1], 0])
    
    return pc

def calculate_material_volume(pc, edge_mask, grid_res=0.05):
    """
    Calculate volume of NON-edge points within the bucket boundaries.
    """
    # Filter out edge points for volume calculation
    material_points = pc.vertices[~edge_mask]
    
    # Further filter points to be within the 5x2x3 bucket box
    # Assuming centered at X=0, and Z goes from 0 to 2, Y from 0 to 3.
    x, y, z = material_points[:, 0], material_points[:, 1], material_points[:, 2]
    
    # Recenter X to be around 0
    x_mean = (np.min(x) + np.max(x)) / 2
    x -= x_mean
    
    # Mask for points inside the bucket volume
    mask = (np.abs(x) <= BUCKET_WIDTH/2) & (z >= 0) & (z <= BUCKET_LENGTH) & (y >= 0) & (y <= BUCKET_HEIGHT)
    
    x_in = x[mask]
    y_in = y[mask]
    z_in = z[mask]
    
    if len(y_in) == 0:
        print("No material points found inside the bucket boundaries.")
        return 0.0

    # Grid integration
    x_min, x_max = -BUCKET_WIDTH/2, BUCKET_WIDTH/2
    z_min, z_max = 0, BUCKET_LENGTH
    
    cols = int((x_max - x_min) / grid_res) + 1
    rows = int((z_max - z_min) / grid_res) + 1
    grid = np.zeros((rows, cols))
    
    ix = ((x_in - x_min) / grid_res).astype(int)
    iz = ((z_in - z_min) / grid_res).astype(int)
    
    for i in range(len(y_in)):
        if 0 <= iz[i] < rows and 0 <= ix[i] < cols:
            grid[iz[i], ix[i]] = max(grid[iz[i], ix[i]], y_in[i])
            
    # Volume = integral of height over the base area
    volume = np.sum(grid) * (grid_res**2)
    return volume

def main():
    pc_file = 'pointCloud.ply'
    if not os.path.exists(pc_file):
        print(f"Error: {pc_file} not found.")
        return
        
    pc = load_and_clean_pc(pc_file)
    edge_points, edge_mask = identify_bucket_edge(pc)
    pc = align_and_scale_by_edge(pc, edge_points)
    
    pc.export('pointCloudAligned.ply')
    print("Exported aligned point cloud to 'pointCloudAligned.ply'")
    
    volume = calculate_material_volume(pc, edge_mask)
    print(f"\nEstimated material volume: {volume:.4f} cubic meters")
    
    capacity = BUCKET_LENGTH * BUCKET_WIDTH * BUCKET_HEIGHT
    print(f"Bucket capacity: {capacity:.4f} cubic meters")
    print(f"Fill level: {(volume / capacity) * 100:.2f}%")

if __name__ == "__main__":
    main()
