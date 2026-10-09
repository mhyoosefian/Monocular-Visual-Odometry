# Fast Monocular Visual Odometry

MATLAB implementation of a conventional Monocular Visual Odometry (MVO) pipeline and a Fast MVO (FMVO) that extracts features only at keyframes. MVO incrementally estimates the position (up to scale) and orientation of a single camera moving in 3D space. On sequence MH_01 of the EuRoC MAV dataset, the FMVO runs about 20 times faster than the conventional MVO with comparable position accuracy.

**For an explanation of the method and the results, see the project page:**
**https://mhyoosefian.github.io/projects/monocular-visual-odometry/**

## Repository structure

```
MVO/                 conventional MVO (features re-extracted in every image)
  runMe.m            runs the algorithm and stores the results
  utils/             feature handling, eight-point algorithm, triangulation, optimization
FMVO/                Fast MVO (features extracted only at keyframes)
  runMe.m
  utils/
plotBothResults.m    plots the stored results of both algorithms
images/              result figures
```

## How to use the code

**Reproduce the results:** run `plotBothResults.m`. It uses the stored results of both algorithms to plot the trajectories, the RMSE of position and orientation, and the run-times, and to compute the comparison table.

**Run the algorithms yourself:**

1. Download sequence **MH_01** of the [EuRoC MAV dataset](https://projects.asl.ethz.ch/datasets/euroc-mav/).
2. Place the downloaded `mav0` folder next to the `MVO` and `FMVO` folders.
3. Run `runMe.m` in the `MVO` folder (conventional MVO) and/or in the `FMVO` folder (Fast MVO). The results are stored in each folder.
4. Run `plotBothResults.m` to plot the new results.

## Citation

If you use the code in your research work, please cite the following paper, as the idea behind the FMVO was developed in this paper.

```
@article{abdollahi2024improved,
  title={An improved multi-state constraint Kalman filter for visual-inertial odometry},
  author={Abdollahi, MR and Pourtakdoust, Seid H and Nooshabadi, MH Yoosefian and Pishkenari, Hossein Nejat},
  journal={Journal of the Franklin Institute},
  volume={361},
  number={15},
  pages={107130},
  year={2024},
  publisher={Elsevier}
}
```
