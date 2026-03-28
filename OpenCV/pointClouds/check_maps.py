import cv2
import numpy as np

cv_file = cv2.FileStorage('stereoMap.xml', cv2.FileStorage_READ)
map_x = cv_file.getNode('stereoMapL_x').mat()
map_y = cv_file.getNode('stereoMapL_y').mat()

print(f"Map X range: {np.nanmin(map_x)} to {np.nanmax(map_x)}")
print(f"Map Y range: {np.nanmin(map_y)} to {np.nanmax(map_y)}")
print(f"Map X mean: {np.nanmean(map_x)}")
print(f"Sample values (top-left 5x5):\n{map_x[:5, :5, 0]}")
