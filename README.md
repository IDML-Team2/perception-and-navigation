# Perception and Navigation -- Wu



## RealSense D455f Setup

### Overview

This repository documents the setup, configuration, and testing of the Intel RealSense D455f depth camera on the NVIDIA Jetson Orin Nano.

The goal is to establish a reliable RGB-D sensing system for future robotics applications, including 3D environment perception, point cloud processing, and SLAM.

### Hardware and Software

| Component          | Specification                      |
|--------------------|------------------------------------|
| Computing Platform | NVIDIA Jetson Orin Nano            |
| Operating System   | Ubuntu 22.04                       |
| ROS Distribution   | ROS 2 Humble                       |
| Depth Camera       | Intel RealSense D455f              |
| Camera SDK         | Intel RealSense SDK (librealsense) |
| Visualization      | RealSense Viewer / RViz2           |

### Process

- Jetson environment setup
- RealSense SDK compilation
- Camera detection and connection
- RGB and depth stream visualization
- 3D point cloud visualization
- TBD
- TBD


### Roadmap
1. implement 2D LiDAR SLAM (using `slam_toolbox`) and 3D obstacle perception with the D455f camera.
2. utilize RTAB-Map to perform joint mapping and localization by fusing RGB-D and LiDAR data.
3. integrate the mobile base, odometry, and the Nav2 stack to achieve autonomous navigation.

Throughout the development process, it is also necessary to address coordinate transformations (TF), sensor calibration, data fusion, perception, and performance optimization for the Jetson platform.
