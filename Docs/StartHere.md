# Start Here
Hello Future Students. Here is an outline of this project so far, and instructions for using everything we've worked on. These
Docs were written in the Spring of 2026.
Leo Devick, Aidan Contant-Guy, Noah Deabel, Daniel Kane and Paul Pellegrin worked on the first iteration of this project. So if you
don't like these docs, you know who to complain to!

# High Level Overview + What we acomplished
We undertook this project with Bobcat, it was called the Bobcat Perception Project. The goal was to use a Stereo Camera attached
to the arm of loader to figure out the volume of dirt in the bucket. We also aimed to track load and dump events. We split our
project into two teams: the Vision team, tasked with calculating dirt volume from stereo images, and the Simulation Team, which
used IsaacSim to create training data that could be consumed by the image software. 
The vision team had two main ideas for solving the problem: Segmentation and Point Clouds.

## Segmentation

Paul P can add writeup here.

## Point Clouds
A Stereo Camera (used with OpenCV) can pull a Point Cloud from two stereo images. A Point Cloud is a collection of points in 3 space
that have X,Y,Z coordinates as well as RGB colors. This is covered more in `Theory.md` but in essence one can find the distance of 
a point from the Stereo Camera by finding the distaince between a certain pixel $P$ in both images. You can try this for yourself by
looking at a point, and closing the left eye, then the right. The distiance $d$ (how much the pixel moves between your two eyes) can
be used along with the known distance betwen the eyes (or Stereo camera lenses) to solve for a distaince $D$ between the camera/eyes
and the point $P$.

To find the volume, we came up with an idea to combine the known parameters of the Bucket (Width, Length, Depth) with the measured
surface from the point cloud. If we can virtually match up the measured bucket from the point cloud with the accurate 3D model of the
bucket, we can find the area between the measured dirt point cloud and the known floor of the bucket to estimate the volume. Look
at our Final Report under `Docs/ProjectReports/` for a diagram showing this idea.

## An overview of the work flow
1. Image data is created in IsaacSim, and exported as PNGs.
    - If you're using machine learning, you're going to want thousands of images and their labels (most like the volume of dirt).
    - Point Clouds need a lot of data for accuracy, but since it's more of extraction than training the model you'll need less
2. Images are consumed by software using either method (ML, Point Clouds)
3. Results (Volume, status) are output

# Repo Map
Here is a map of everthing we've included:
- CalibrationPOC - C++ Program to try out camera calibration with openCV
- OpenCV/
    - Examples/
    - Masking/
    - pointClouds/
    - requirements.txt
- Segmenation/
- pyLauncher
- Docs (you are here)
    - StartHere.md - this document
    - OpenCV.md
    - Segmentation.md
    - IsaacSim.md
    - Theory.md
    - USD.md
    - ProjectReports/ (all of our quad charts, slides, final report pdf, etc)


# Versions and Tooling
We used C++
We used Python 3.10.5 (point clouds) and Python 3.8 (Segmentation)
We used Blender 5.0.1 before IsaacSim.
We used IsaacSim 5.1.0.

MeshLab is a good tool for viewing the point clouds in color, but Blender can render .ply files too.

## Libraries
### Python Libraries
#### OpenCV
  OpenCV is the gold standard for open source computer vision. It comes with tons of features and classes which really simplify
  computer vision.
  [OpenCV home](https://docs.opencv.org/4.x/index.html)
  [OpenCV Camera Calibration](https://docs.opencv.org/4.x/d4/d94/tutorial_camera_calibration.html)
#### Numpy
#### glob
#### tqdm

## Python Enviornments
Python is a tricky animal to work with sometimes, especialy when verions get out of whack. We used two methods: pyLauncher, which was
more Windows friendly, and a cli application called pyenv.

### PyLauncher

### PyEnv
PyEnv is very simple to setup and use.
[PyEnv](https://github.com/pyenv/pyenv)

## Simulation: IsaacSim
IsaacSim is an extremely accurate and detailed physics and 3D rendering software. Think of it as Blender Extreme. During this project,
we ran IsaacSim in a Docker Container on Bobcat's Oren machine that had a NVidia 4090 GPU. 
**Please, Do Not try and run IsaacSim on your Personal Computer. 
You will have only tears unless you have 64G of Ram and a 4090 or better.**
IsaacSim allows you to stream it through a WebRTC (Real Time Communication) Client, so you can access IsaacSim from your puny Linux
laptop and not be sad.

### Streaming Client
[IsaacSim Installs](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html#isaac-sim-latest-release)
From this link, only grab the IsaacSim WebRTC Client for your machine. Don't try to install all of Isaac Sim, that needs to run
on heavy hardware. 
After installing, you should be able to run it with `./isaac_sim_rtc.AppImage`, and connect to the remote server by inputting the
IP Address.


## Resources
### An Invitation to 3-D Vision by Yi Ma 
ISBN: 978-0-387-00893-6

We read bits of this book to get some of the theory of Computer Vision. Most of it is linear algebra.
