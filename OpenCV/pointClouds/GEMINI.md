# Project Bucket Half Full
This folder is a collection of files for a perception project that uses Stereo Cameras and the bucket of a large loader (for scooping dirt or gravel) to determine how much volume
is in the bucket.

## What we have
captureStereoImages.py - This is a run once program that captures images from a stereo camera and stores them for the next program.
cameraCalibrate.py - This is run once, and generates a stereoMap.xml which includes the imperfections measured in the camera, and a Q value used for generating point clouds
pointCloud.py - This takes an image (by path, not from cli) and combines it with the stereoMap.xml to generate a .ply file with the point cloud data.

## What we want
pointCloudCombine.py - This program will take the point cloud .ply file, and combine it with a 3D model library (trimesh). This program should have 2 main goals: Mapping the detected 
edge of the bucket from the Point Cloud onto the already known dimensions of the bucket, which
will scale and align the point cloud (which seems to grow bigger than renderable in blender)
to the scale of the bucket. It will also take an Integral (which isn't neccessary if there 
are other ways) to calculate the area between the measured material mesh layer that is inside
of the bucket. 

# Tech Stack
Python 3.10.12
OpenCV
numpy
trimesh

We also use Vim, VSCode, IsaacSim and Blender, but you aren't responsible for any of these.
