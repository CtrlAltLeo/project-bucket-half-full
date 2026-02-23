import cv2 as cv
import numpy as np
import os

# Configurables

F = 750
Cx = 320
Cy = 240

K = np.array([
    [F, 0, Cx],
    [0, F, Cy],
    [0, 0, 1]
], dtype=np.float64)

#Import Photos

script_dir = os.path.dirname(os.path.abspath(__file__))

img_path = os.path.join(script_dir, "boxCam1.png")
img1 = cv.imread(img_path)

img_path = os.path.join(script_dir, "boxCam2.png")
img2 = cv.imread(img_path)

# Feature Detection

orb = cv.ORB_create(nfeatures = 500)

keyPoints1, descriptor1 = orb.detectAndCompute(img1, None)
keyPoints2, descriptor2 = orb.detectAndCompute(img2, None)

bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
matches = bf.match(descriptor1, descriptor2)
matches = sorted(matches, key=lambda x: x.distance)

cam1Pts = np.float64([keyPoints1[m.queryIdx].pt for m in matches])
cam2Pts = np.float64([keyPoints2[m.trainIdx].pt for m in matches])

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

# Create Depth Threshold

Z = triangulatedPts[:,2]

box = Z < np.median(Z)

projected, _ = cv.projectPoints(triangulatedPts, np.zeros(3), np.zeros(3), K, None)
projected = projected.reshape(-1, 2).astype(int)

# Draw MAsk

for i, pt in enumerate(projected):
    if box[i]:
        cv.circle(img1, tuple(pt), 3, (0,255,0), -1)

cv.imshow("img1", img1)