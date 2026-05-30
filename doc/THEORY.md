# THEORY.md

# Theory Behind Monocular Visual SLAM

## Introduction

Simultaneous Localization and Mapping (SLAM) is the process of estimating a camera's position while simultaneously constructing a map of the surrounding environment.

In monocular SLAM, only a single camera is available.

The system must solve two problems simultaneously:

1. Localization

   * Where is the camera?

2. Mapping

   * What does the environment look like?

This repository implements a classical feature-based monocular Visual SLAM pipeline.

---

# Problem Formulation

Given a sequence of images:

[
I_1, I_2, I_3, \ldots , I_n
]

estimate:

1. Camera poses

[
T_1, T_2, T_3, \ldots , T_n
]

2. A set of 3D landmarks

[
X_1, X_2, X_3, \ldots , X_m
]

where:

[
T_i \in SE(3)
]

represents camera position and orientation.

---

# Camera Model

Visual SLAM begins with the pinhole camera model.

A 3D world point:

[
X=
\begin{bmatrix}
X\
Y\
Z\
1
\end{bmatrix}
]

is projected into image coordinates as:

[
x = K[R|t]X
]

where:

* (K) = Intrinsic Matrix
* (R) = Rotation Matrix
* (t) = Translation Vector

---

# Camera Intrinsic Matrix

The intrinsic matrix is:

[
K=
\begin{bmatrix}
f_x & 0 & c_x\
0 & f_y & c_y\
0 & 0 & 1
\end{bmatrix}
]

where:

* (f_x,f_y) = focal lengths
* (c_x,c_y) = principal point

The implementation uses:

[
K=
\begin{bmatrix}
F & 0 & W/2\
0 & F & H/2\
0 & 0 & 1
\end{bmatrix}
]

---

# Homogeneous Coordinates

Image points are represented as:

[
x=
\begin{bmatrix}
u\
v\
1
\end{bmatrix}
]

rather than

[
(u,v)
]

because homogeneous coordinates simplify projective geometry operations.

---

# Feature Detection

The first step in Visual SLAM is identifying repeatable image locations.

This implementation uses:

### Shi-Tomasi Corners

A pixel is considered a corner if image intensity changes significantly in multiple directions.

The structure tensor is:

[
M=
\begin{bmatrix}
I_x^2 & I_xI_y\
I_xI_y & I_y^2
\end{bmatrix}
]

where:

* (I_x) = horizontal gradient
* (I_y) = vertical gradient

The corner score is:

[
R = \min(\lambda_1,\lambda_2)
]

where:

[
\lambda_1,\lambda_2
]

are eigenvalues of (M).

Large eigenvalues indicate strong corners.

---

# Feature Description

Corners alone are insufficient.

A descriptor is required.

This implementation uses ORB descriptors.

ORB combines:

1. FAST Corner Detector
2. BRIEF Binary Descriptor

Advantages:

* Rotation invariant
* Fast computation
* Compact representation

Each feature becomes a binary vector.

---

# Feature Matching

Feature descriptors are matched between frames.

The similarity metric is:

[
d(a,b)
]

using Hamming Distance.

For binary descriptors:

[
d_H(a,b)
========

\sum_{i=1}^{n}
(a_i \oplus b_i)
]

where:

[
\oplus
]

denotes XOR.

Smaller distance implies better match.

---

# Lowe Ratio Test

Many matches are ambiguous.

For descriptor (d):

Let:

[
m_1
]

be the best match and

[
m_2
]

be the second-best match.

Accept match only if:

[
\frac{m_1}{m_2}
<
0.85
]

This rejects unstable correspondences.

---

# Epipolar Geometry

Two camera views observe the same scene.

The geometric relationship between views is called Epipolar Geometry.

---

# Epipolar Constraint

Given matching points:

[
x_1
]

and

[
x_2
]

the following relationship holds:

[
x_2^T F x_1 = 0
]

where:

[
F
]

is the Fundamental Matrix.

---

# Fundamental Matrix

The Fundamental Matrix encodes:

* Camera motion
* Relative geometry

Properties:

[
rank(F)=2
]

and

[
det(F)=0
]

It maps points in one image to epipolar lines in another.

---

# Essential Matrix

If camera intrinsics are known:

[
E = K^T F K
]

The Essential Matrix directly encodes camera motion.

Its constraint is:

[
x_2^T E x_1 = 0
]

---

# Essential Matrix Decomposition

The Essential Matrix can be decomposed using SVD:

[
E = U \Sigma V^T
]

From this decomposition:

[
R
]

and

[
t
]

can be recovered.

---

# Relative Camera Motion

Camera motion is represented as:

[
T=
\begin{bmatrix}
R&t\
0&1
\end{bmatrix}
]

where:

[
R \in SO(3)
]

and

[
t \in \mathbb{R}^3
]

---

# Pose Composition

Suppose:

[
T_{k-1}
]

is previous camera pose and

[
T_{rel}
]

is motion between frames.

Current pose:

[
T_k=T_{rel}T_{k-1}
]

This accumulates motion over time.

---

# Triangulation

Once corresponding points are known in two images, their 3D position can be reconstructed.

---

# Geometric Idea

Each image observation generates a ray:

[
r_1
]

and

[
r_2
]

The 3D point lies at the intersection of these rays.

Due to noise, rays rarely intersect perfectly.

Therefore, least-squares estimation is used.

---

# Linear Triangulation

For each correspondence:

[
AX=0
]

where:

[
A=
\begin{bmatrix}
x_1P_1^{(3)}-P_1^{(1)}\
y_1P_1^{(3)}-P_1^{(2)}\
x_2P_2^{(3)}-P_2^{(1)}\
y_2P_2^{(3)}-P_2^{(2)}
\end{bmatrix}
]

The solution is obtained using SVD:

[
A = U\Sigma V^T
]

The reconstructed point is:

[
X = V(:,4)
]

the singular vector associated with the smallest singular value.

---

# Parallax

Parallax is the apparent displacement of a feature between views.

[
p=
|x_1-x_2|
]

Large parallax:

* Better depth estimation

Small parallax:

* Poor triangulation accuracy

The implementation rejects low-parallax matches.

---

# Depth Validation

A reconstructed point must lie in front of both cameras.

Condition:

[
Z > 0
]

Points violating this are removed.

---

# Reprojection

A reconstructed point is projected back into the image.

[
\hat{x}=PX
]

where:

[
P=K[R|t]
]

---

# Reprojection Error

The reconstruction quality is measured by:

[
e=
|x-\hat{x}|
]

where:

* (x) = observed point
* (\hat{x}) = projected point

Small error indicates accurate reconstruction.

Large error indicates incorrect triangulation.

---

# Sparse Mapping

Each triangulated point becomes a landmark.

The map is:

[
M=
{X_1,X_2,\ldots,X_n}
]

where:

[
X_i
]

are reconstructed 3D points.

The map grows as additional frames are processed.

---

# Visual Odometry vs SLAM

Visual Odometry:

* Estimates trajectory only

[
T_1,T_2,\ldots,T_n
]

SLAM:

* Estimates trajectory
* Estimates map

[
(T,M)
]

This project performs both.

---

# Scale Ambiguity

Monocular cameras cannot determine absolute scale.

For example:

[
t
]

and

[
5t
]

produce identical image observations.

Therefore:

Monocular SLAM recovers geometry only up to scale.

---

# Drift

Pose estimation errors accumulate over time.

If:

[
T_1,T_2,\ldots,T_n
]

contain small errors, total drift increases.

Without correction:

* trajectory bends
* map deforms

---

# Bundle Adjustment

Bundle Adjustment jointly optimizes:

* Camera Poses
* Landmark Positions

Optimization objective:

[
\min
\sum_{i,j}
|
x_{ij}
------

\pi(T_i,X_j)
|^2
]

where:

* (x_{ij}) = observed image point
* (X_j) = landmark
* (T_i) = camera pose
* (\pi) = projection function

Bundle Adjustment is considered the gold standard of SLAM optimization.

This repository currently does not implement Bundle Adjustment.

---

# Loop Closure

Loop Closure detects when the camera revisits a previously seen location.

Benefits:

* Reduces drift
* Improves consistency
* Enables large-scale mapping

Modern systems such as ORB-SLAM use loop closure extensively.

---

# Relationship to ORB-SLAM

This implementation contains the core ideas behind ORB-SLAM:

✓ Feature Extraction

✓ Feature Matching

✓ Essential Matrix Estimation

✓ Pose Recovery

✓ Triangulation

✓ Sparse Mapping

✓ Trajectory Estimation

Missing components:

✗ Bundle Adjustment

✗ Keyframe Management

✗ Loop Closure

✗ Pose Graph Optimization

✗ Place Recognition

---

# Summary

This project implements a complete educational monocular Visual SLAM pipeline using classical multiple-view geometry.

The mathematical foundation relies on:

1. Projective Geometry
2. Epipolar Geometry
3. Linear Algebra
4. Optimization
5. Multiple View Reconstruction

Together these techniques allow a moving camera to estimate its trajectory while simultaneously reconstructing a sparse 3D representation of the environment.
