# MATH.md

# Mathematical Foundations of Monocular Visual SLAM

## Introduction

This document derives the mathematical foundations behind the Visual SLAM pipeline implemented in this repository.

The complete system relies on:

1. Linear Algebra
2. Projective Geometry
3. Multiple View Geometry
4. Optimization
5. Numerical Methods

The goal is to estimate:

* Camera trajectory
* Sparse 3D scene structure

from a sequence of monocular images.

---

# Coordinate Systems

Visual SLAM operates across multiple coordinate systems.

## World Coordinates

[
X_w=
\begin{bmatrix}
X_w\
Y_w\
Z_w\
1
\end{bmatrix}
]

Represents a point in the global map.

---

## Camera Coordinates

[
X_c=
\begin{bmatrix}
X_c\
Y_c\
Z_c
\end{bmatrix}
]

Relationship:

[
X_c = RX_w+t
]

where

[
R \in SO(3)
]

and

[
t\in\mathbb{R}^3
]

---

## Image Coordinates

Image points:

[
x=
\begin{bmatrix}
u\
v
\end{bmatrix}
]

represent pixel locations.

---

# Homogeneous Coordinates

Euclidean point:

[
(x,y)
]

becomes

[
\tilde{x}
=========

\begin{bmatrix}
x\
y\
1
\end{bmatrix}
]

Advantages:

* Enables matrix-based projection
* Simplifies geometry operations
* Handles points at infinity

---

# Camera Projection Model

A camera maps:

[
\mathbb{R}^3
\rightarrow
\mathbb{R}^2
]

using:

[
x = K[R|t]X
]

where:

[
P=K[R|t]
]

is called the projection matrix.

---

# Intrinsic Matrix

The intrinsic matrix is:

K=\begin{bmatrix}f_x&0&c_x\0&f_y&c_y\0&0&1\end{bmatrix}

where:

* (f_x,f_y) = focal lengths
* (c_x,c_y) = principal point

---

# Camera Pose

The pose matrix is:

[
T=
\begin{bmatrix}
R&t\
0&1
\end{bmatrix}
]

where:

[
R^TR=I
]

and

[
det(R)=1
]

This defines the Special Euclidean Group:

[
SE(3)
]

---

# Rotation Matrices

A valid rotation matrix satisfies:

[
R^TR=I
]

and

[
det(R)=1
]

Examples:

Rotation around x-axis:

[
R_x(\theta)=
\begin{bmatrix}
1&0&0\
0&\cos\theta&-\sin\theta\
0&\sin\theta&\cos\theta
\end{bmatrix}
]

Rotation around y-axis:

[
R_y(\theta)=
\begin{bmatrix}
\cos\theta&0&\sin\theta\
0&1&0\
-\sin\theta&0&\cos\theta
\end{bmatrix}
]

Rotation around z-axis:

[
R_z(\theta)=
\begin{bmatrix}
\cos\theta&-\sin\theta&0\
\sin\theta&\cos\theta&0\
0&0&1
\end{bmatrix}
]

---

# Feature Matching Mathematics

Let

[
d_i
]

and

[
d_j
]

be ORB descriptors.

The Hamming distance is:

[
H(d_i,d_j)
==========

\sum_{k=1}^{n}
(d_{ik}\oplus d_{jk})
]

where

[
\oplus
]

denotes XOR.

Lower Hamming distance indicates greater similarity.

---

# Epipolar Geometry

Consider two camera views.

A point:

[
X
]

projects to:

[
x_1
]

and

[
x_2
]

The relationship between the two observations is:

[
x_2^T F x_1 = 0
]

where

[
F
]

is the Fundamental Matrix.

---

# Derivation of Fundamental Matrix

For two cameras:

[
P_1=[I|0]
]

[
P_2=[R|t]
]

The epipolar constraint becomes:

[
x_2^T[t]_\times R x_1=0
]

where

[
[t]_\times
]

is the skew-symmetric matrix:

[
[t]_\times=
\begin{bmatrix}
0&-t_z&t_y\
t_z&0&-t_x\
-t_y&t_x&0
\end{bmatrix}
]

---

# Essential Matrix

The Essential Matrix is:

E=[t]_{\times}R

Properties:

[
rank(E)=2
]

and

[
det(E)=0
]

The constraint becomes:

[
x_2^T E x_1 = 0
]

for normalized coordinates.

---

# Eight Point Algorithm

Given at least eight correspondences:

[
(x_i,y_i)
\leftrightarrow
(x_i',y_i')
]

construct:

[
A=
\begin{bmatrix}
x_1x_1' & x_1y_1' & x_1 & y_1x_1' & y_1y_1' & y_1 & x_1' & y_1' & 1 \
\vdots
\end{bmatrix}
]

The solution satisfies:

[
Af=0
]

Solve using SVD:

[
A=U\Sigma V^T
]

The Fundamental Matrix is reshaped from:

[
f
]

corresponding to the smallest singular value.

---

# Singular Value Decomposition

Many stages of SLAM rely on SVD.

General form:

A=U\Sigma V^T

where:

* (U) orthogonal
* (V) orthogonal
* (\Sigma) diagonal

Applications:

* Essential Matrix decomposition
* Triangulation
* Bundle Adjustment
* Pose optimization

---

# Recovering Camera Pose

Given:

[
E=U\Sigma V^T
]

Define:

[
W=
\begin{bmatrix}
0&-1&0\
1&0&0\
0&0&1
\end{bmatrix}
]

Then:

[
R=UWV^T
]

and

[
t=U(:,3)
]

These provide relative camera motion.

---

# Pose Composition

Camera trajectory is formed by repeated multiplication:

[
T_k
===

T_{rel}
T_{k-1}
]

Expanding:

[
T_k
===

T_kT_{k-1}\cdots T_1
]

This accumulates motion over time.

---

# Triangulation

Goal:

Recover 3D point:

[
X
]

from image observations.

---

# Projection Equations

For camera:

[
P_1
]

[
x_1=P_1X
]

For camera:

[
P_2
]

[
x_2=P_2X
]

---

# DLT Triangulation

Cross-product constraints:

[
x_1\times(P_1X)=0
]

[
x_2\times(P_2X)=0
]

These generate:

[
AX=0
]

with:

[
A=
\begin{bmatrix}
x_1P_1^{(3)}-P_1^{(1)}\
y_1P_1^{(3)}-P_1^{(2)}\
x_2P_2^{(3)}-P_2^{(1)}\
y_2P_2^{(3)}-P_2^{(2)}
\end{bmatrix}
]

---

# Solving Triangulation

Compute:

[
A=U\Sigma V^T
]

The reconstructed point is:

[
X=V(:,4)
]

corresponding to the smallest singular value.

---

# Homogeneous Normalization

Triangulation returns:

[
X=
\begin{bmatrix}
X\
Y\
Z\
W
\end{bmatrix}
]

Convert to Euclidean coordinates:

[
X=
\left(
\frac{X}{W},
\frac{Y}{W},
\frac{Z}{W}
\right)
]

---

# Parallax

Parallax:

[
p=
|x_1-x_2|
]

Depth uncertainty:

[
\sigma_Z
\propto
\frac{1}{p}
]

Large parallax:

* Better depth estimates

Small parallax:

* Poor triangulation

---

# Reprojection Error

Projected estimate:

[
\hat{x}=PX
]

Error:

e=|x-\hat{x}|_2

where:

* (x) = observed point
* (\hat{x}) = projected point

Smaller error implies higher reconstruction quality.

---

# Least Squares Optimization

Many SLAM problems are written as:

[
\min_x
\sum_i r_i^2
]

where:

[
r_i
]

is a residual.

Objective:

Minimize geometric error.

---

# Bundle Adjustment

Bundle Adjustment jointly optimizes:

* Camera poses
* Landmark positions

Objective:

[
\min
\sum_{i,j}
|
x_{ij}
-\pi(T_i,X_j)
|^2
]

This is a nonlinear least-squares problem.

---

# Gauss-Newton Method

Update rule:

[
\Delta x
========

-(J^TJ)^{-1}J^Tr
]

where:

* (J) = Jacobian
* (r) = residual vector

New estimate:

[
x_{new}
=======

x+\Delta x
]

---

# Levenberg-Marquardt

Improves stability:

[
(J^TJ+\lambda I)\Delta x
========================

-J^Tr
]

where:

[
\lambda
]

controls damping.

Used extensively in modern SLAM systems.

---

# Lie Groups

SLAM optimization operates on:

[
SO(3)
]

for rotations and

[
SE(3)
]

for poses.

---

# SO(3)

Special Orthogonal Group:

[
SO(3)
=====

{R:R^TR=I,\det(R)=1}
]

Represents valid rotations.

---

# SE(3)

Special Euclidean Group:

[
SE(3)
=====

\begin{bmatrix}
R&t\
0&1
\end{bmatrix}
]

Represents rigid-body motion.

---

# Scale Ambiguity

Monocular SLAM cannot determine absolute scale.

If:

[
t
]

is valid then

[
\alpha t
]

is equally valid.

Thus:

[
X
\sim
\alpha X
]

for any positive scalar:

[
\alpha
]

This is a fundamental limitation of monocular systems.

---

# Drift Accumulation

Each pose estimate contains error:

[
T_i+\epsilon_i
]

Accumulated trajectory becomes:

[
\hat{T}
=======

(T_n+\epsilon_n)\cdots(T_1+\epsilon_1)
]

causing drift.

Loop closure and optimization are used to correct this.

---

# Connection to Modern SLAM

The mathematical foundation of modern systems such as:

* ORB-SLAM
* ORB-SLAM2
* ORB-SLAM3
* DSO
* LSD-SLAM
* VINS-Mono

is built upon the concepts derived in this document:

1. Projective Geometry
2. Epipolar Geometry
3. Triangulation
4. Optimization
5. Lie Theory

These ideas remain the backbone of state-of-the-art visual SLAM systems.
