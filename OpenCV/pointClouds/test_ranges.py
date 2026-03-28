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

def main():
    pc = load_and_clean_pc('pointCloud.ply')
    bucket = trimesh.load('bucket.obj')
    
    colors = pc.colors[:, :3]
    # Dirt: brown/gray, around 50-60
    dirt_mask = (colors >= 30) & (colors < 80)
    dirt_mask = dirt_mask.all(axis=1)
    
    # Bucket edge: black, < 30
    edge_mask = (colors < 30).all(axis=1)
    
    print(f"Total: {len(pc.vertices)}")
    print(f"Edge points: {np.sum(edge_mask)}")
    print(f"Dirt points: {np.sum(dirt_mask)}")
    
    # Let's see the unscaled ranges
    if np.sum(edge_mask) > 0:
        print(f"Edge PC-X range: {np.max(pc.vertices[edge_mask, 0]) - np.min(pc.vertices[edge_mask, 0])}")
        print(f"Edge PC-Y range: {np.max(pc.vertices[edge_mask, 1]) - np.min(pc.vertices[edge_mask, 1])}")
        print(f"Edge PC-Z range: {np.max(pc.vertices[edge_mask, 2]) - np.min(pc.vertices[edge_mask, 2])}")
    
    if np.sum(dirt_mask) > 0:
        print(f"Dirt PC-X range: {np.max(pc.vertices[dirt_mask, 0]) - np.min(pc.vertices[dirt_mask, 0])}")
        print(f"Dirt PC-Y range: {np.max(pc.vertices[dirt_mask, 1]) - np.min(pc.vertices[dirt_mask, 1])}")
        print(f"Dirt PC-Z range: {np.max(pc.vertices[dirt_mask, 2]) - np.min(pc.vertices[dirt_mask, 2])}")

if __name__ == "__main__":
    main()
