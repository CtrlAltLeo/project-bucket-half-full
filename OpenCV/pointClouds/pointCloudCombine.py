import numpy as np
import trimesh
import os

def load_and_clean_pc(file_path):
    pc = trimesh.load(file_path)
    mask = np.isfinite(pc.vertices).all(axis=1)
    pc.vertices = pc.vertices[mask]
    if hasattr(pc, 'colors') and pc.colors is not None:
        pc.colors = pc.colors[mask]
    return pc

def identify_bucket_edge(pc, black_threshold=30):
    if not hasattr(pc, 'colors') or pc.colors is None:
        return None, None
    colors = pc.colors[:, :3]
    actual_thresh = black_threshold if np.max(colors) > 1.0 else black_threshold / 255.0
    mask = (colors < actual_thresh).all(axis=1)
    edge_points = pc.vertices[mask]
    
    # Filter outliers
    if len(edge_points) > 10:
        x = edge_points[:, 0]
        q1, q3 = np.percentile(x, [10, 90])
        iqr = q3 - q1
        edge_points = edge_points[(x >= q1 - 1.5*iqr) & (x <= q3 + 1.5*iqr)]
    return edge_points, mask

def align_and_scale(pc, edge_points, bucket_mesh):
    # Rotate: OpenCV X=Width, Y=Vertical, Z=Depth -> Mesh X=Depth, Y=Height, Z=Width
    v = pc.vertices
    pc.vertices = np.stack([v[:, 2], v[:, 1], v[:, 0]], axis=1)
    e = edge_points
    edge_pts_rot = np.stack([e[:, 2], e[:, 1], e[:, 0]], axis=1)
    
    target_width = bucket_mesh.extents[2]
    edge_width = np.max(edge_pts_rot[:, 2]) - np.min(edge_pts_rot[:, 2])
    scale_factor = target_width / max(edge_width, 1.0)
    pc.apply_scale(scale_factor)
    
    # Center edge points to mesh center
    edge_pts_scaled = edge_pts_rot * scale_factor
    e_center = np.mean(edge_pts_scaled, axis=0)
    m_center = bucket_mesh.centroid
    pc.apply_translation([m_center[0] - e_center[0], 0, m_center[2] - e_center[2]])
    
    # Grounding: Use Mesh Top (bounds[1][1]) as the reference for Edge Points
    e_top = np.max(pc.vertices[(pc.vertices[:, 2] > m_center[2]-0.5) & (pc.vertices[:, 2] < m_center[2]+0.5), 1])
    pc.apply_translation([0, bucket_mesh.bounds[1][1] - e_top, 0])
    
    return pc

def calculate_material_volume(pc, edge_mask, bucket_mesh, grid_res=0.01):
    material_points = pc.vertices[~edge_mask]
    b_min, b_max = bucket_mesh.bounds
    
    # Strict interior mask
    mask = (material_points[:, 0] > b_min[0]+0.1) & (material_points[:, 0] < b_max[0]-0.1) & \
           (material_points[:, 2] > b_min[2]+0.1) & (material_points[:, 2] < b_max[2]-0.1) & \
           (material_points[:, 1] > b_min[1])
    
    pts = material_points[mask]
    if len(pts) == 0: return 0.0

    x, z, y = pts[:, 0], pts[:, 2], pts[:, 1]
    x_min, x_max = np.min(x), np.max(x)
    z_min, z_max = np.min(z), np.max(z)
    
    cols = int((x_max - x_min) / grid_res) + 1
    rows = int((z_max - z_min) / grid_res) + 1
    grid = np.full((rows, cols), -np.inf)
    
    for i in range(len(y)):
        r, c = int((z[i] - z_min) / grid_res), int((x[i] - x_min) / grid_res)
        if 0 <= r < rows and 0 <= c < cols:
            grid[r, c] = max(grid[r, c], y[i])
            
    valid = np.argwhere(grid != -np.inf)
    print(f"Integrating over {len(valid)} cells at {grid_res}m resolution.")
    
    volume = 0.0
    for r, c in valid:
        tx, tz = x_min + c * grid_res, z_min + r * grid_res
        # Floor is roughly b_min[1] but we use nearest for precision
        query = np.array([[tx, grid[r, c], tz]])
        try:
            _, dist, _ = bucket_mesh.nearest.on_surface(query)
            floor_y = grid[r, c] - dist[0]
        except:
            floor_y = b_min[1]
        volume += max(0, grid[r, c] - floor_y) * (grid_res**2)
    return volume

def main():
    pc = load_and_clean_pc('pointCloud.ply')
    bucket = trimesh.load('bucket.obj')
    edge_pts, edge_mask = identify_bucket_edge(pc, 30)
    pc = align_and_scale(pc, edge_pts, bucket)
    pc.export('pointCloudAligned.ply')
    vol = calculate_material_volume(pc, edge_mask, bucket)
    print(f"Volume: {vol:.4f} m^3, Capacity: {np.prod(bucket.extents):.4f} m^3")
    print(f"Fill level: {(vol/np.prod(bucket.extents))*100:.2f}%")

if __name__ == "__main__":
    main()
