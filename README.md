
# lidar_background_removal

Background removal in (2D or 3D) lidar scan data that coincides with an occupancy grid map.

[![Build Status](https://travis-ci.org/yzrobot/lidar_background_removal.svg?branch=main)](https://travis-ci.org/yzrobot/lidar_background_removal) [![License](https://img.shields.io/badge/License-BSD%203--Clause-green.svg)](https://opensource.org/licenses/BSD-3-Clause)

## How to build 
```sh
cd ~/catkin_ws/src/
git clone https://github.com/yzrobot/lidar_background_removal.git
cd ~/catkin_ws
catkin_make
```

## Run
```sh
rosrun lidar_background_removal lidar_background_removal
```

## Test environment 
```
Ubuntu 20.04 LTS
ROS Noetic
```

## Citation ##
If you are considering using this code, please reference the following:
```
@article{io22software,
   author  = {Iaroslav Okunevich and Vincent Hilaire and Stephane Galland and Olivier Lamotte and Liubov Shilova and Yassine Ruichek and Zhi Yan},
   title   = {Software-hardware Integration and Human-centered Benchmarking for Socially-compliant Robot Navigation},
   journal = {CoRR},
   volume = {abs/2210.15628},
   year = {2022},
   url = {http://arxiv.org/abs/2210.15628},
   archivePrefix = {arXiv},
   eprint = {2210.15628}
}
```
