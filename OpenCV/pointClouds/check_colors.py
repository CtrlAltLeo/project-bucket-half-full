import trimesh
import numpy as np

pc = trimesh.load('pointCloud.ply')
print(f"Total points: {len(pc.vertices)}")
if hasattr(pc, 'colors') and pc.colors is not None:
    colors = pc.colors[:, :3]
    print(f"Color range: {np.min(colors, axis=0)} to {np.max(colors, axis=0)}")
    print(f"Average color: {np.mean(colors, axis=0)}")
    # Sample some points
    print("First 5 colors:\n", colors[:5])
else:
    print("No colors found in point cloud.")
