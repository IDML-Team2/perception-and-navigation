
# Intel RealSense D455f Installation

## 1. Overview

This document describes the installation and configuration
of the Intel RealSense D455f on the NVIDIA Jetson Orin Nano.

## 2. Prerequisites

- NVIDIA Jetson Orin Nano
- Ubuntu 22.04
- Intel RealSense D455f
- USB 3.0 connection
- Internet connection

## 3. Check Camera Connection

Connect the camera to the Jetson via USB 3.0.

```bash
lsusb
lsusb -t
```

Verify that the camera is detected and connected
using a USB 3.x interface.

## 4. Install Dependencies

Update the package lists:

```bash
sudo apt update
```

Install the required build tools:

```bash
sudo apt install \
  git cmake build-essential \
  libssl-dev libusb-1.0-0-dev \
  libudev-dev pkg-config \
  libgtk-3-dev libglfw3-dev \
  libgl1-mesa-dev libglu1-mesa-dev
```

Additional build dependencies should be documented
according to the actual installation process.

## 5. Configure RealSense SDK

Navigate to the RealSense SDK directory and create a build directory:

```bash
cd ~/librealsense
mkdir -p build
cd build
```

Configure the SDK using CMake:

```bash
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_EXAMPLES=ON \
  -DBUILD_GRAPHICAL_EXAMPLES=ON
```

The RSUSB backend is used to communicate with the camera via USB without requiring kernel modifications.

**Expected output:**

```text
-- Configuring done
-- Generating done
```

This indicates that CMake configuration was completed successfully.

## 6. Build and Install RealSense SDK

Compile the SDK:

```bash
make -j2
```

The `-j2` option uses two parallel jobs to reduce memory usage during compilation on the Jetson.

Once compilation completes without errors, install the SDK:

```bash
sudo make install
```


## 7. Verify Installation

Connect the Intel RealSense D455f to the Jetson via USB 3.0.

Launch the RealSense Viewer:

```bash
realsense-viewer
```

If you have not yet executed `sudo make install`, it is recommended to first run the compiled program directly:

```bash
~/librealsense/build/Release/realsense-viewer
```


**Expected result:**

The RealSense Viewer launches successfully and detects the connected D455f camera.

The following features can be tested:

- RGB image streaming
- Depth image streaming
- 3D point cloud visualization


## 8. Install ROS2 Wrapper

The RealSense ROS2 Wrapper allows the D455f to publish camera data as ROS2 topics, including RGB images, depth images, and 3D point clouds.

### 8.1 Source ROS2 Environment

Load the ROS2 Humble environment:

```bash
source /opt/ros/humble/setup.bash
```

### 8.2 Create ROS2 Workspace

Create a workspace and navigate to the source directory:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Clone the RealSense ROS2 Wrapper repository:

```bash
git clone -b ros2-master https://github.com/realsenseai/realsense-ros.git
```

### 8.3 Install Build Tools and Dependencies

Install the required ROS2 build tools:

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep
```

Initialize and update rosdep:

```bash
sudo rosdep init
rosdep update
```

**Note:** `sudo rosdep init` only needs to be executed once per system. If rosdep has already been initialized, this step can be skipped.

### 8.4 Build ROS2 Wrapper

Navigate to the workspace:

```bash
cd ~/ros2_ws
```

Build the RealSense ROS2 Wrapper:

```bash
colcon build --symlink-install --parallel-workers 2
```

The `--parallel-workers 2` option limits the number of parallel package build workers to reduce resource usage on the Jetson.


### 8.5 Source Workspace

After a successful build, load both the ROS2 Humble environment and the compiled workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Verify that the RealSense ROS2 package is available:

```bash
ros2 pkg list | grep realsense
```

**Expected result:**
```text
realsense2_camera
realsense2_camera_msgs
realsense2_description
```

## 9. Launch RealSense D455f

Before launching the camera:

- Close RealSense Viewer.
- Connect the D455f to the Jetson via USB 3.0.
- Ensure that the ROS2 environment and workspace have been sourced.

Launch the camera with RGB, depth, and point cloud streaming enabled:

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  pointcloud__neon__enable:=true
```

**Expected result:**

The RealSense camera node starts and publishes camera data through ROS2 topics.

Keep this terminal running while checking the published topics.

## 10. Verify ROS2 Topics

Open a second terminal.

Load the ROS2 environment and workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

List all available ROS2 topics:

```bash
ros2 topic list
```

**Expected topics:**

```text
/camera/camera/color/image_raw
/camera/camera/depth/image_rect_raw
/camera/camera/depth/color/points
```

These topics provide the following data:

| Topic | Data |
|---|---|
| `/camera/camera/color/image_raw` | RGB image |
| `/camera/camera/depth/image_rect_raw` | Depth image |
| `/camera/camera/depth/color/points` | 3D point cloud |

The exact topic names may vary depending on the RealSense ROS Wrapper configuration.

To verify that the camera is publishing image data, check the topic frequency:

```bash
ros2 topic hz /camera/camera/color/image_raw
```

Check the depth image frequency:

```bash
ros2 topic hz /camera/camera/depth/image_rect_raw
```

To verify the point cloud message type:

```bash
ros2 topic type /camera/camera/depth/color/points
```

**Expected message type:**

```text
sensor_msgs/msg/PointCloud2
```

## 11. Visualize Point Cloud in RViz2

Once the ROS2 topics are successfully published, launch RViz2 in a new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2
```

### 11.1 Configure RViz2

1. Click **Add** in the Displays panel.
2. Select **PointCloud2**.
3. Choose the corresponding point cloud topic:

   `/camera/camera/depth/color/points`

4. Set **Fixed Frame** to a valid camera coordinate frame published by the camera.
5. Adjust the PointCloud2 QoS settings if necessary.

### 11.2 Expected Result

RViz2 should display the 3D point cloud generated by the RealSense D455f.


