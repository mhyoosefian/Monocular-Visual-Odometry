# Monocular-Visual-Odometry
This repo contains an implementation of a conventional Monocular Visual Odomtery (MVO) and an enhanced MVO referred to as Fast MVO (FMVO). MVO refers to the process of the incrementally estimating the position and orientation of a single camera moving in the 3D space. 

# Conventional MVO
There are severla approaches to MVO, among which I have focused on "2D-2D motion estimation". In this approach, The following steps are implemented for each two consecutive images:
1.  A number of features are extracted in the first image.
2.  These features are tracked in the second image.
3.  The motion is estimated using Essential or Fundamental matrix.
4.  A local optimization is performed to minimize the reprojection error.

