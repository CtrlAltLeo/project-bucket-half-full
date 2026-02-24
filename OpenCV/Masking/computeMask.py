import cv2 as cv
import numpy as np
import os

#Import Photos

script_dir = os.path.dirname(os.path.abspath(__file__))

img_path = os.path.join(script_dir, "angle1.jpg")
img1 = cv.imread(img_path)
img1 = cv.resize(img1, (960,720), cv.INTER_AREA)

img_path = os.path.join(script_dir, "angle2.jpg")
img2 = cv.imread(img_path)
img2 = cv.resize(img2, (960,720), cv.INTER_AREA)

# Configurables

height, width = img1.shape[:2]

F = 0.8 * width
Cx = width / 2
Cy = height / 2

K = np.array([
    [F, 0, Cx],
    [0, F, Cy],
    [0, 0, 1]
], dtype=np.float64)

# Feature Detection

orb = cv.ORB_create(nfeatures=5000)

keyPoints1, descriptor1 = orb.detectAndCompute(img1, None)
keyPoints2, descriptor2 = orb.detectAndCompute(img2, None)

bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
matches = bf.match(descriptor1, descriptor2)

# Sort matches by quality (optional but recommended)
matches = sorted(matches, key=lambda x: x.distance)

pts1 = np.float64([keyPoints1[m.queryIdx].pt for m in matches])
pts2 = np.float64([keyPoints2[m.trainIdx].pt for m in matches])

cam1Pts = np.float64([keyPoints1[m.queryIdx].pt for m in matches])
cam2Pts = np.float64([keyPoints2[m.trainIdx].pt for m in matches])


print(len(matches))
# Calculate Essential MAtrix

E, mask = cv.findEssentialMat(cam1Pts, cam2Pts, K, 
                              method=cv.RANSAC, prob=0.99, threshold=1.0)

inliers, R, t, mask = cv.recoverPose(E, cam1Pts, cam2Pts, K)

print(len(mask))

# Remove Outlier Points
mask = mask.ravel().astype(bool)

cam1Pts_in = cam1Pts[mask]
cam2Pts_in = cam2Pts[mask]

# Build Projection Matrixes

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

book = Z < np.mean(Z)

# Draw Mask
for pt in cam1Pts_in:
    cv.circle(img1, tuple(np.int32(pt)), 3, (0,255,0), -1)

cv.imshow("Masked Image", img1)

cv.waitKey(0)