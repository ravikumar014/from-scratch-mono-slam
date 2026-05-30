# From Scratch Monocular Visual SLAM

A complete implementation of a **Monocular Visual SLAM (Simultaneous Localization and Mapping)** pipeline in Python, built from first principles using classical computer vision and geometric methods.

The system estimates camera motion from a monocular video stream, reconstructs a sparse 3D map of the environment, and visualizes both the estimated trajectory and reconstructed scene in real time.

<p align="center">
  <img src="assets/slam_demo2.gif" width = "1000">
</p>

---

## Overview

Visual SLAM is a fundamental problem in robotics, autonomous navigation, AR/VR, and 3D reconstruction.

This project demonstrates the complete SLAM pipeline:

1. Feature Detection
2. Feature Description
3. Feature Matching
4. Epipolar Geometry Estimation
5. Relative Pose Recovery
6. 3D Point Triangulation
7. Sparse Map Construction
8. Camera Trajectory Estimation
9. Real-Time Visualization

Unlike many SLAM repositories that rely on large external frameworks, this implementation exposes the mathematical and algorithmic foundations of SLAM in a compact and educational codebase.

---

## Features

### Feature Extraction

- Shi-Tomasi Corner Detection
- ORB Feature Descriptors
- Dense feature sampling
- Feature filtering for improved stability

### Feature Matching

- Brute Force Matcher
- Hamming Distance Metric
- Lowe Ratio Test
- Outlier Rejection

### Motion Estimation

- Essential Matrix Estimation
- RANSAC-based Robust Fitting
- Relative Pose Recovery
- Camera Motion Tracking

### 3D Reconstruction

- Linear Triangulation
- Depth Validation
- Parallax Filtering
- Reprojection Error Filtering

### Mapping

- Sparse 3D Point Cloud Generation
- Incremental Map Growth
- Landmark Management

### Visualization

- Live Feature Tracking
- Camera Trajectory Visualization
- Sparse Point Cloud Rendering
- Open3D Real-Time Viewer
- OpenCV-Based Visualization Dashboard

---

## Pipeline

```text
Input Video
     │
     ▼
Feature Detection
     │
     ▼
ORB Descriptor Extraction
     │
     ▼
Feature Matching
     │
     ▼
Essential Matrix Estimation
     │
     ▼
Pose Recovery (R,t)
     │
     ▼
Triangulation
     │
     ▼
Sparse 3D Map
     │
     ▼
Trajectory Estimation
     │
     ▼
Real-Time Visualization
```

---

## Project Structure

```text
from-scratch-mono-slam/

├── src/
│   ├── main.py
│   ├── extractor.py
│   ├── pointmap.py
│   ├── visualizer.py
│   ├── live_3d.py
│   ├── display.py
│   └── utils.py
│
├── assets/
│   ├── slam_demo.gif
│   └── images/
│
├── docs/
│   ├── THEORY.md
│   ├── IMPLEMENTATION.md
│   └── MATH.md
│
├── requirements.txt
├── LICENSE
└── README.md
```

---

## Installation

### Clone Repository

```bash
git clone https://github.com/yourusername/from-scratch-mono-slam.git

cd from-scratch-mono-slam
```

### Create Environment

```bash
python -m venv venv

source venv/bin/activate
```

### Install Dependencies

```bash
pip install -r requirements.txt
```

---

## Dependencies

- Python 3.10+
- OpenCV
- NumPy
- SciPy
- Scikit-Image
- Open3D
- PyOpenGL
- PySDL2
- tqdm

Install all requirements using:

```bash
pip install -r requirements.txt
```

---

## Running the Project

### Run SLAM on a Video

Place a video inside:

```text
videos/
```

Update the video path inside `main.py`:

```python
cap = cv2.VideoCapture("videos/test_nyc.mp4")
```

Run:

```bash
python src/main.py
```

---

## Visualization Outputs

The system provides:

### Feature Matching View

Displays tracked correspondences between consecutive frames.

### Trajectory View

Top-down visualization of estimated camera motion.

### Sparse Point Cloud

Real-time reconstruction of scene geometry.

### Open3D Visualization

Interactive 3D point cloud rendering.

---

## Example Results

### Camera Trajectory

| Input Video | Output |
|------------|---------|
| Urban Scene | Estimated Camera Path |
| Road Sequence | Sparse Map Reconstruction |

<p align="center">
  <img src="assets/slam_demo2.gif">
</p>

---

## Mathematical Foundations

The implementation is based on classical multiple-view geometry.

### Camera Projection Model

A 3D point

$$
X = [X,Y,Z,1]^T
$$

is projected into image coordinates by

$$
x = K[R|t]X
$$

where:

- $K$ = Camera Intrinsic Matrix
- $R$ = Rotation Matrix
- $t$ = Translation Vector

---

### Essential Matrix

Given corresponding image points:

$$
x_2^T E x_1 = 0
$$

where:

$$
E = [t]_\times R
$$

The essential matrix encodes relative camera motion.

---

### Pose Recovery

Pose is recovered from:

$$
E = U \Sigma V^T
$$

using Singular Value Decomposition (SVD).

---

### Triangulation

A 3D point is reconstructed by intersecting projection rays from multiple views.

This implementation uses linear triangulation via SVD.

---

### Reprojection Error

For a reconstructed point:

$$
e = ||x - \hat{x}||
$$

where:

- $x$ = observed image point
- $\hat{x}$ = projected reconstructed point

Points with high reprojection error are rejected.

---

## Current Limitations

This project is intended for educational and research purposes.

Current limitations include:

- Monocular scale ambiguity
- No loop closure
- No global optimization
- No bundle adjustment
- Sparse reconstruction only
- Sensitive to motion blur and dynamic objects

---

## Future Work

Planned improvements:

- Bundle Adjustment
- Loop Closure Detection
- ORB-SLAM Style Keyframe Management
- Pose Graph Optimization
- Dense Reconstruction
- Stereo SLAM Support
- RGB-D Integration
- ROS2 Integration
- CUDA Acceleration
- Neural Feature Matching (SuperPoint / LightGlue)

---

## Educational Goals

This repository is designed for:

- Computer Vision Students
- Robotics Researchers
- SLAM Beginners
- Autonomous Systems Engineers
- Researchers studying Visual Odometry and Mapping

The code emphasizes readability and understanding of the underlying mathematics over production-level optimization.

---

## References

### Books

- Multiple View Geometry in Computer Vision — Hartley & Zisserman
- Computer Vision: Algorithms and Applications — Richard Szeliski

### Papers

- ORB: An Efficient Alternative to SIFT or SURF
- MonoSLAM: Real-Time Single Camera SLAM
- ORB-SLAM
- ORB-SLAM2
- ORB-SLAM3

---

## Citation

If you find this project useful in your research, please cite:

```bibtex
@software{fromscratchmonoslam,
  author = {Ravi Kumar},
  title = {From Scratch Monocular Visual SLAM},
  year = {2026},
  url = {https://github.com/yourusername/from-scratch-mono-slam}
}
```

---

## Author

**Ravi Kumar U**

M.Tech in Machine Learning and Computing  
Indian Institute of Space Science and Technology (IIST)

Research Interests:

- Artificial Intelligence
- Computer Vision
- Robotics
- Reinforcement Learning
- Visual SLAM
- 3D Reconstruction

---

## License

This project is released under the MIT License.

See the LICENSE file for details.
