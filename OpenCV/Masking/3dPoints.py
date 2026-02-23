import cv2 as cv
import numpy as np

# Configurables

F = 750
Cx = 320
Cy = 240

K = np.array([
    [F, 0, Cx],
    [0, F, Cy],
    [0, 0, 1]
], dtype=np.float64)

cam1Pts = np.array([
        [213,129], [525,120], [216,407], [520,370],
        [102,112], [371,102], [381,251], [302,118] 
], dtype=np.float64)

cam2Pts = np.array([
        [138,106], [485,116], [143,369], [483,391], 
        [251,74], [539,81], [291,240], [353,94],
], dtype=np.float64)

# Calculate Essential MAtrix

E, mask = cv.findEssentialMat(cam1Pts, cam2Pts, K, 
                              method=cv.RANSAC, prob=0.99, threshold=1.0)

inliers, R, t, mask = cv.recoverPose(E, cam1Pts, cam2Pts, K)

# Remove Outlier Points
mask = mask.ravel().astype(bool)

cam1Pts_in = cam1Pts[mask]
cam2Pts_in = cam2Pts[mask]

# Build Projection MAtrixes

cam1 = np.hstack((np.eye(3), np.zeros((3,1))))
cam2 = np.hstack((R, t))

cam1 = K @ cam1
cam2 = K @ cam2

# Triangulate Points

reshapedCam1 = cam1Pts_in.T
reshapedCam2 = cam2Pts_in.T

points4d = cv.triangulatePoints(cam1, cam2, reshapedCam1, reshapedCam2)

triangulatedPts = points4d[:3] / points4d[3]
triangulatedPts = triangulatedPts.T

# Print 3D Points

for i, pt in enumerate(triangulatedPts):
    print(f"Point {i}: {pt}")