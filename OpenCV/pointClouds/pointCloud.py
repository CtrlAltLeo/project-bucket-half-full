# This converts a stereo map into a point cloud (.ply)
import cv2
import numpy as np 
import glob
from tqdm import tqdm
import PIL.ExifTags
import PIL.Image
# from matplotlib import pyplot as plt  # Disabled for headless

# Downsamples image x number (reduce_factor) of times. 
def downsample_image(image, reduce_factor):
	for i in range(0,reduce_factor):
		#Check if image is color or grayscale
		if len(image.shape) > 2:
			row,col = image.shape[:2]
		else:
			row,col = image.shape

		image = cv2.pyrDown(image, dstsize= (col//2, row // 2))
	return image


#=========================================================
# Stereo Calibration and rectification
#=========================================================
# Camera parameters to undistort and rectify images
cv_file = cv2.FileStorage()
cv_file.open('stereoMap.xml', cv2.FileStorage_READ)

stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
stereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
stereoMapR_y = cv_file.getNode('stereoMapR_y').mat()

Q = cv_file.getNode('q').mat()

# Using NEW image data
#imgL = cv2.imread('images/stereoLeft/imageL1.png')
#imgR = cv2.imread('images/stereoRight/imageR1.png')
imgL = cv2.imread('imgl.png')
imgR = cv2.imread('imgr.png')

if imgL is None or imgR is None:
    print("Error: Could not load images.")
    exit(1)

# Undistort and rectify images
# imgR = cv2.remap(imgR, stereoMapR_x, stereoMapR_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
# imgL = cv2.remap(imgL, stereoMapL_x, stereoMapL_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

print(f"imgL mean (no remap): {imgL.mean()}")
                
# Downsample each image 3 times (because they're too big)
imgL = downsample_image(imgL,3)
imgR = downsample_image(imgR,3)

imgLgray = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
imgRgray = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)


#=========================================================
# Create Disparity map from Stereo Vision
#=========================================================

# Set disparity parameters
block_size = 5
min_disp = -1
max_disp = 31
num_disp = max_disp - min_disp # Needs to be divisible by 16

# Create Block matching object. 
stereo = cv2.StereoSGBM_create(minDisparity= min_disp,
	numDisparities = num_disp,
	blockSize = block_size,
	uniquenessRatio = 5,
	speckleWindowSize = 5,
	speckleRange = 2,
	disp12MaxDiff = 2,
	P1 = 8 * 3 * block_size**2,
	P2 = 32 * 3 * block_size**2)

# Compute disparity map
disparity_map = stereo.compute(imgLgray, imgRgray)

#=========================================================
# Generate Point Cloud from Disparity Map
#=========================================================

# Get new downsampled width and height 
h,w = imgR.shape[:2]

# Convert disparity map to float32 and divide by 16 as show in the documentation
disparity_map = np.float32(np.divide(disparity_map, 16.0))

# Reproject points into 3D
points_3D = cv2.reprojectImageTo3D(disparity_map, Q, handleMissingValues=False)
# Get color of the reprojected points from the LEFT image (matching the disparity map)
colors = cv2.cvtColor(imgL, cv2.COLOR_BGR2RGB)

# Get rid of points with invalid depth (disparity <= 0)
mask_map = disparity_map > 0

# Mask colors and points. 
output_points = points_3D[mask_map]
output_colors = colors[mask_map]

print(f"Output colors min: {output_colors.min()}, max: {output_colors.max()}, mean: {output_colors.mean()}")


# Function to create point cloud file
def create_point_cloud_file(vertices, colors, filename):
	colors = colors.reshape(-1,3)
	vertices = np.hstack([vertices.reshape(-1,3),colors])

	ply_header = '''ply
		format ascii 1.0
		element vertex %(vert_num)d
		property float x
		property float y
		property float z
		property uchar red
		property uchar green
		property uchar blue
		end_header
		'''
	with open(filename, 'w') as f:
		f.write(ply_header %dict(vert_num=len(vertices)))
		np.savetxt(f,vertices,'%f %f %f %d %d %d')


output_file = 'pointCloud.ply'

# Generate point cloud file
print(f"Generating {output_file}...")
create_point_cloud_file(output_points, output_colors, output_file)
print("Done.")
