import cv2
import numpy as np

imgR = cv2.imread('imgr.png')
# Downsample
def downsample_image(image, reduce_factor):
    for i in range(reduce_factor):
        row, col = image.shape[:2]
        image = cv2.pyrDown(image, dstsize=(col//2, row//2))
    return image

imgR = downsample_image(imgR, 3)
print(f"Downsampled imgR mean: {imgR.mean(axis=(0,1))}")
print(f"Downsampled imgR max: {imgR.max()}")

imgL = cv2.imread('imgl.png')
imgL = downsample_image(imgL, 3)

imgLgray = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
imgRgray = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

block_size = 5
min_disp = -1
max_disp = 31
num_disp = max_disp - min_disp

stereo = cv2.StereoSGBM_create(minDisparity=min_disp,
    numDisparities=num_disp,
    blockSize=block_size)

disparity = stereo.compute(imgLgray, imgRgray)
print(f"Disparity min: {disparity.min()}, max: {disparity.max()}")

mask = disparity > disparity.min()
print(f"Masked points count: {np.sum(mask)}")

colors = cv2.cvtColor(imgR, cv2.COLOR_BGR2RGB)
masked_colors = colors[mask]
print(f"Masked colors mean: {masked_colors.mean(axis=0)}")
print(f"Masked colors max: {masked_colors.max()}")
