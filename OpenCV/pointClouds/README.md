# Technical Documentation: Stereo Perception for Volumetric Analysis

Images live in an images.tar archive. These are old images created in Blender for calibratoin
and testing.

## 1. Executive Summary
This project implements a computer vision pipeline designed for volumetric estimation of material within a heavy equipment loader bucket. Utilizing stereo photogrammetry, the system reconstructs a three-dimensional point cloud of the bucket's contents, aligns this data with a known geometric model of the bucket, and performs numerical integration to determine the volume of the contained material.

---

## 2. Component Analysis

### 2.1 `captureStereoImages.py`
**Purpose:** Synchronized data acquisition from a binocular camera system.
**Functionality:** It initializes two hardware video captures simultaneously. The script provides a real-time preview and allows the operator to trigger a synchronized frame capture (mapped to the 'S' key). These image pairs are essential for both initial calibration and subsequent depth inference.

### 2.2 `cameraCalibrate.py`
**Purpose:** Computational determination of intrinsic and extrinsic camera parameters.
**Functionality:** This script utilizes a chessboard-based calibration routine (9x6 intersections). It processes a series of stereo image pairs to compute:
- **Intrinsic Matrices:** Focal lengths and optical centers.
- **Distortion Coefficients:** Correction factors for radial and tangential lens distortion.
- **Extrinsic Parameters:** Rotation and translation vectors defining the spatial relationship between the two sensors.
The output is serialized into `stereoMap.xml`, containing the undistortion maps and the Perspective Transformation Matrix ($Q$).

### 2.3 `pointCloud.py`
**Purpose:** Three-dimensional reconstruction from rectified stereo pairs.
**Functionality:** 
1. **Rectification:** Applies the maps from `stereoMap.xml` to align the epipolar lines of the stereo pair.
2. **Disparity Estimation:** Employs the Semi-Global Block Matching (`StereoSGBM`) algorithm to compute a disparity map.
3. **Reprojection:** Utilizes the $Q$ matrix to transform disparity data into Euclidean coordinates ($X, Y, Z$).
The resulting data is exported as an ASCII-formatted Polygon File Format (`.ply`).

### 2.4 `pointCloudCombine.py`
**Purpose:** Spatial alignment, scaling, and volumetric computation.
**Functionality:** This module represents the analytical core of the system. It identifies the bucket's physical boundaries by filtering for low-reflectance "black" points characteristic of the metal rim. It then scales the relative point cloud to the known physical dimensions ($5.0\text{m} \times 2.0\text{m} \times 3.0\text{m}$). Volume is calculated via a discrete Riemann sum (grid integration) of the height values over the bucket's base area.

### 2.5 `inspect_cloud.py`
**Purpose:** Data sanitation and validation.
**Functionality:** A utility script that leverages the `trimesh` library to filter non-finite values (NaN/Inf) and prune spatial outliers (points exceeding 10 meters). It serves as a diagnostic tool to verify the structural integrity of generated point clouds.

---

## 3. Operational Workflow

1.  **Calibration Phase:** 
    - Execute `captureStereoImages.py` to collect 20-30 pairs of a chessboard pattern at varying orientations.
    - Run `cameraCalibrate.py` to generate the `stereoMap.xml`.
2.  **Acquisition Phase:** 
    - Position the loader bucket within the stereo field of view.
    - Capture the scene using `captureStereoImages.py`, ensuring the files are named appropriately for `pointCloud.py`.
3.  **Processing Phase:**
    - Execute `pointCloud.py` to generate the raw `pointCloud.ply`.
    - Run `pointCloudCombine.py` to perform the final alignment and obtain the volumetric fill percentage.

---

## 4. Future Research & Development

### 4.1 Enhanced Semantic Segmentation
The current reliance on "black thresholding" for bucket edge detection is sensitive to lighting conditions and material color. Future iterations should implement a **Mask R-CNN** or **U-Net** architecture to semantically segment "bucket" vs. "material" within the 2D image plane prior to 3D projection.

### 4.2 Dynamic Scale Calibration
While current scaling is hard-coded to bucket dimensions, implementing a **PnP (Perspective-n-Point)** algorithm using known markers on the bucket would allow for dynamic scaling and orientation adjustment, accommodating camera vibration or movement during operation.

### 4.3 Temporal Filtering
To mitigate noise in the volume estimation, a **Kalman Filter** or a simple temporal moving average should be applied across multiple frames to produce a stable volume reading in a production environment.

### 4.4 GPU Optimization
The `StereoSGBM` and volumetric integration are currently CPU-bound. Offloading these computations to CUDA via OpenCV's `cuda::StereoBM` or `cupy` would enable real-time processing at higher frame rates.
