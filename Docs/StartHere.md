# Start Here
Hello Future Students. Here is an outline of this project so far, and instructions for using everything we've worked on. These
Docs were written in the Spring of 2026.

# High Level Overview + What we acomplished
We undertook this project with Bobcat, it was called the Bobcat Perception Project. The goal was to use a Stereo Camera attached
to the arm of loader to figure out the volume of dirt in the bucket. We also aimed to track load and dump events. We split our
project into two teams: the Vision team, tasked with calculating dirt volume from stereo images, and the Simulation Team, which
used IsaacSim to create training data that could be consumed by the image software. 
The vision team had two main ideas for solving the problem: Segmentation and Point Clouds.

## Segmentation

## Point Clouds


## Repo Map
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
    - ProjectReports/ (all of our quad charts, slides, final report pdf, etc)


# Versions and Tooling
We used C++
We used Python 3.10.5 and 3.8.

## Python Enviornments
Python is a tricky animal to work with sometimes, especialy when verions get out of whack. We used two methods: pyLauncher, which was
more Windows friendly, and a cli application called pyenv.

### PyLauncher

### PyEnv
PyEnv is very simple to setup and use.
[PyEnv](https://github.com/pyenv/pyenv)

## Simulation: IsaacSim


