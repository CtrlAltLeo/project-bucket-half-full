import trimesh
import numpy as np

pc = trimesh.load('pointCloud.ply')
colors = pc.colors[:, :3]
print(f"Total: {len(colors)}")
for thresh in [10, 20, 30, 40, 50, 60, 70]:
    mask = (colors < thresh).all(axis=1)
    print(f"Thresh {thresh}: {np.sum(mask)} points ({(np.sum(mask)/len(colors))*100:.1f}%)")
