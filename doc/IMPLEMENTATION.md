# IMPLEMENTATION.md

# Implementation Details

This document explains the software architecture and implementation details of the Monocular Visual SLAM system.

---

# System Architecture

The SLAM pipeline is implemented as a sequence of modules:

```text
Video Stream
    │
    ▼
Frame Acquisition
    │
    ▼
Feature Extraction
    │
    ▼
Feature Matching
    │
    ▼
Motion Estimation
    │
    ▼
Pose Recovery
    │
    ▼
Triangulation
    │
    ▼
3D Point Generation
    │
    ▼
Map Update
    │
    ▼
Visualization
```

Each module is intentionally separated to make the implementation easier to understand and extend.

---

# Module Overview

## main.py

Main entry point of the SLAM pipeline.

Responsibilities:

* Load video frames
* Create Frame objects
* Match consecutive frames
* Estimate camera motion
* Triangulate new map points
* Update map
* Visualize results

Main execution loop:

```python
while cap.isOpened():
    ret, frame = cap.read()

    if ret:
        process_frame(frame)
```

The entire SLAM pipeline is executed inside `process_frame()`.

---

## extractor.py

Responsible for feature extraction and feature matching.

Contains:

### Feature Detection

Uses Shi-Tomasi corner detection:

```python
cv2.goodFeaturesToTrack(...)
```

Reasons:

* Fast
* Stable
* Produces high-quality corners

Detected corners become candidate feature locations.

---

### Descriptor Computation

ORB descriptors are computed at detected feature locations.

```python
orb.compute(...)
```

Advantages:

* Rotation invariant
* Fast binary descriptor
* Suitable for real-time systems

---

### Feature Matching

Feature correspondences are established using:

```python
cv2.BFMatcher()
```

Distance metric:

```text
Hamming Distance
```

because ORB produces binary descriptors.

---

### Lowe Ratio Test

Ambiguous matches are removed using:

```python
m.distance < 0.85 * n.distance
```

This rejects weak correspondences.

---

### Essential Matrix Estimation

Matched points are used to estimate:

```python
cv2.findEssentialMat(...)
```

using RANSAC.

Purpose:

* Reject outliers
* Recover camera geometry

---

### Pose Recovery

Camera motion is recovered using:

```python
cv2.recoverPose(...)
```

Output:

```text
Rotation R
Translation t
```

These form the relative camera transformation.

---

## Frame Class

Each image frame is represented by:

```python
class Frame
```

Stores:

```text
Frame ID
Feature Points
Descriptors
Camera Pose
Intrinsic Matrix
```

The pose is represented as a 4×4 homogeneous transformation matrix.

---

# Camera Pose Representation

Each frame maintains:

```text
T = [R | t]
```

stored as:

```python
4 × 4 matrix
```

Format:

```text
| R11 R12 R13 tx |
| R21 R22 R23 ty |
| R31 R32 R33 tz |
|  0   0   0  1  |
```

---

# Motion Estimation

For two consecutive frames:

```text
Frame(k-1)
Frame(k)
```

The relative transformation is:

```text
T_rel
```

The current pose is computed as:

```python
current_pose = T_rel @ previous_pose
```

This accumulates motion over time.

---

# Triangulation

Implemented inside:

```python
triangulate(...)
```

Purpose:

Recover 3D world points from matched image observations.

Inputs:

```text
Pose 1
Pose 2
Point in Image 1
Point in Image 2
```

Method:

Direct Linear Transform (DLT)

For each correspondence:

```text
AX = 0
```

A Singular Value Decomposition is used:

```python
np.linalg.svd(A)
```

The final 3D point is the smallest singular vector.

---

# Parallax Filtering

A minimum parallax requirement is enforced:

```python
parallax > threshold
```

Reason:

Small parallax produces unstable triangulation.

Benefits:

* Better depth estimation
* Reduced reconstruction noise

---

# Depth Validation

After triangulation:

```python
pts4d[:,2] > 0
```

is enforced.

Purpose:

Reject points behind the camera.

---

# Reprojection Error Filtering

Each triangulated point is projected back into both images.

Error:

```text
Observed Point
-
Projected Point
```

Points with excessive reprojection error are removed.

Benefits:

* Removes unstable landmarks
* Improves map quality

---

# pointmap.py

Implements the global map.

Contains:

## Map

Stores:

```text
Frames
3D Points
Viewer State
```

Acts as the central repository for the SLAM state.

---

## Point

Represents a reconstructed landmark.

Stores:

```text
3D Position
Observed Frames
Feature Indices
```

Each landmark can be observed in multiple images.

---

# Sparse Mapping

New landmarks are continuously added.

Workflow:

```text
Feature Match
      ↓
Triangulate
      ↓
Validate
      ↓
Create Point
      ↓
Add To Map
```

This gradually builds a sparse 3D representation of the environment.

---

# Visualization

Two visualization systems are implemented.

---

## visualizer.py

Produces:

### Image View

Displays:

* Feature correspondences
* Motion tracks

### Trajectory View

Displays:

* Estimated camera path

### Point Cloud View

Displays:

* Top-down sparse map

The visualization canvas combines all views into a single dashboard.

---

## live_3d.py

Uses Open3D.

Provides:

* Interactive point cloud rendering
* Camera trajectory visualization
* Real-time map updates

Advantages:

* Zoom
* Rotate
* Pan
* Inspect reconstruction

---

# Robustness Improvements

Several practical improvements are implemented.

---

## Sky Removal

Upper image regions are ignored.

Reason:

Sky contains poor visual features.

Benefits:

* Better tracking
* More stable motion estimation

---

## Right-Side Feature Enforcement

Additional features are collected on the image right side.

Reason:

Prevent spatial imbalance.

Benefits:

* Improved Essential Matrix estimation
* Better triangulation geometry

---

## Pose Stabilization

Translation magnitude is monitored.

Large jumps are constrained.

Benefits:

* Reduced drift
* Smoother trajectory

---

## Map Size Limiting

The number of landmarks is bounded.

Purpose:

Prevent memory growth during long sequences.

---

# Computational Complexity

## Feature Detection

```text
O(N)
```

where N is the number of image pixels.

---

## Feature Matching

```text
O(M²)
```

where M is the number of descriptors.

Brute-force matching compares every descriptor pair.

---

## Essential Matrix Estimation

```text
O(K)
```

for K RANSAC iterations.

---

## Triangulation

```text
O(P)
```

where P is the number of matched correspondences.

---

# Current Limitations

The current implementation focuses on educational clarity.

Limitations include:

* Monocular scale ambiguity
* No loop closure
* No bundle adjustment
* No keyframe selection
* Sparse reconstruction only
* Drift accumulation over long trajectories

---

# Future Improvements

Potential extensions:

## Geometry

* Bundle Adjustment
* Pose Graph Optimization

## Mapping

* Dense Reconstruction
* Multi-view Triangulation

## Tracking

* Keyframe Management
* Loop Closure Detection

## Learning-Based Methods

* SuperPoint Features
* LightGlue Matching
* Neural Bundle Adjustment

## Robotics

* ROS2 Integration
* Real-Time Deployment
* Multi-Camera SLAM

---

# Design Philosophy

This project prioritizes:

1. Readability
2. Mathematical transparency
3. Educational value
4. Minimal external dependencies

The goal is to expose the core concepts behind Visual SLAM while maintaining a working end-to-end implementation suitable for experimentation and research.
