# EpsAvlc_toys

This repo stores the codes I wrote for fun.

## Overview
### [image_stitching](./image_stitching/)

Stitch two images.

### [fisheye_to_pinhole](./fisheye_to_pinhole/)

Turn fisheye camera's image into pinhole camera's image. 

### [stereo reconstruction](./stereo_reconstruction/)

Use two images to obtain dense point cloud. (Not finished)

### [cuda_raycaster](./cuda_raycaster/)

A GPU-accelerated raycaster for octomap.

### [b_spline](./b_spline/)

Functional-programming-based B spline implementation.
### [autodiff](./autodiff/)

A ceres-like automatic derivatives computor.

### [convex polygon library](https://github.com/EpsAvlc/CPL)
A convex polygon clip library.

### [float to float](./float_to_float/)

Convert float into bits, then convert bits into float following chapter 2.4 of CSAPP.

### [hungary_optimizer](./hungary_optimizer/)

hungray algorithm to solve the assignment problem.

### [sliding_grid](./sliding_grid/)
Sliding gridmap/octomap. Moving gird origin without remallocing memory. 

## Installation

```bash
git clone https://github.com/EpsAvlc/EpsAvlc_toys.git
git submodule update --recursive --init
