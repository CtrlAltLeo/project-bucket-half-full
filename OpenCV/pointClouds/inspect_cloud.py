import trimesh
import numpy as np

def clean_point_cloud(filename):
    p = trimesh.load(filename)
    # Filter out inf/nan
    mask = np.isfinite(p.vertices).all(axis=1)
    p.vertices = p.vertices[mask]
    if hasattr(p, 'colors'):
        p.colors = p.colors[mask]
    
    # Filter out points too far away (e.g. > 10000 mm)
    mask = (np.abs(p.vertices) < 10000).all(axis=1)
    p.vertices = p.vertices[mask]
    if hasattr(p, 'colors'):
        p.colors = p.colors[mask]
        
    print(f"Loaded {len(p.vertices)} valid points.")
    print(f"Bounds: {p.bounds}")
    return p

if __name__ == "__main__":
    clean_point_cloud('pointCloud.ply')
