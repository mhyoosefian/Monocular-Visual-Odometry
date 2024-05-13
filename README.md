# Monocular-Visual-Odometry
This repo contains an implementation of a conventional Monocular Visual Odomtery (MVO) and an enhanced MVO referred to as Fast MVO (FMVO). MVO refers to the process of the incrementally estimating the position and orientation of a single camera moving in the 3D space. 

# Conventional MVO
There are several approaches to MVO, among which I have focused on "2D-2D motion estimation". In this approach, The following steps are implemented for each two consecutive images:
1.  A number of features are extracted in the first image.
2.  These features are tracked in the second image.
3.  The motion is estimated using Essential or Fundamental matrix.
4.  A local optimization is performed to minimize the reprojection error.

# Fast MVO
In the fast MVO, instead of extracting features in every two images, features are only extracted in some images, referred to as keyframes. The criteria based on which these keyframes are chosen is different from pervailing methods in the literature. An image is a keyframe if its feature tracks is below a threshold. In essence, when the first image is received, a number of features are extracted in it. Steps 2 an d 3 of the conventional MVO are performed to find the motion of the camera. As the camera moves, some features will no longer be in the field of view of the camera. As a result, as the camera moves, the number of features being tracked decreases. When the number of features is below a certain threshols, all remaining features are used to perform a local optimization, and new features are extracted in the last image (i.e., the keyframe). Hence, the FMVO approach can be summarized in the following steps.
1. A number of features are extracted from the keyframe
2. These features are tracked in future images.
3. The motion is estimated using Essential or Fundamental matrix.
4. If the number of tracked features is less than a threshold, a local optimization is performed to minimize the reprojection error.

# Results
The conventional MVO and the Fast MVO are tested on the [EuRoC mav dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets), the sequence MH-01.

## Position and orientation 
The MVO and FMVO results of estimating x-y and x-z trajectories and orientation are shown below.
<img src="/images/position.png" width="50%" height="50%">
<img src="/images/orientation.png" width="50%" height="50%">

## RMSE of position and orientation
The RMSE of the estimations are plotted below.

<img src="/images/positionRMSE.png" width="50%" height="50%">
<img src="/images/orientationRMSE.png" width="50%" height="50%">

## Run time
the time each iteration takes for both MVO and FMVO are shown below. Also, the accumulative run-time is plotted.
<img src="/images/runtime.png" width="50%" height="50%">
<img src="/images/accRuntime.png" width="50%" height="50%">

## Comparison
A comparison of the performance of the two algorithms is provided in the Table below. The first six rows in this table are median of the RMSE results. The last row
is the median run-time of each iteration of the algorithms.
<img src="/images/table.png" width="50%" height="50%">

# Citation
If you use the code in your research work, please cite the following paper.
