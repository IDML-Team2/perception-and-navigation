# Perception and Navigation -- Wu



## RealSense D455f Setup

### Overview

This repository documents the setup, configuration, and testing of the Intel RealSense D455f depth camera on the NVIDIA Jetson Orin Nano, as well as early perception experiments for a mobile robotics platform.

The goal is to establish a reliable RGB-D sensing system for robotics applications, including 3D environment perception, color-based cube detection and 3D position estimation, point cloud processing, SLAM, and future navigation and manipulation interfaces.

### Hardware and Software

| Component          | Specification                      |
|--------------------|------------------------------------|
| Computing Platform | NVIDIA Jetson Orin Nano |
| Operating System | Ubuntu 22.04 |
| ROS Distribution | ROS 2 Humble |
| Depth Camera | Intel RealSense D455f |
| Camera SDK | Intel RealSense SDK (`librealsense`) |
| ROS Camera Integration | RealSense ROS 2 Wrapper (`realsense2_camera`) |
| Cube Perception | Python, OpenCV HSV segmentation, aligned depth |
| Visualization | RealSense Viewer / RViz2 |

### Process

1. Set up the Jetson environment and verify the D455f USB connection.
2. Compile and install RealSense SDK (`librealsense`).
3. Install and build the RealSense ROS 2 Wrapper.
4. Verify RGB, depth, and point cloud topics in ROS 2 and RViz2.
5. Implement HSV-based cube detection and estimate 3D positions using depth data.
6. Extend perception with local point clouds and integrate with the larger SLAM/navigation project.


### Roadmap
1. implement 2D LiDAR SLAM (using `slam_toolbox`) and 3D obstacle perception with the D455f camera.
2. utilize RTAB-Map to perform joint mapping and localization by fusing RGB-D and LiDAR data.
3. integrate the mobile base, odometry, and the Nav2 stack to achieve autonomous navigation.

Throughout the development process, it is also necessary to address coordinate transformations (TF), sensor calibration, data fusion, perception, and performance optimization for the Jetson platform.
