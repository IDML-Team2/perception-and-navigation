## HSV Cube Detection

This section integrates the original HSV cube detection README. The example detector uses RGB images and aligned depth to estimate a 3D point for each color-segmented cube. Run these commands in a ROS 2 Humble environment with the required `cube_detector.py` script available.

The camera launch below is the *detection-specific configuration*: it enables aligned depth and disables full point cloud publishing to reduce overhead. This differs intentionally from the point-cloud verification launch used during installation above.

### Install dependencies
```bash
sudo apt update
sudo apt install -y python3-opencv python3-numpy ros-humble-cv-bridge ros-humble-message-filters
source /opt/ros/humble/setup.bash
```

### Terminal 1: camera
```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=true \
  align_depth.enable:=true enable_sync:=true pointcloud.enable:=false
```

### Terminal 2: detector
```bash
source /opt/ros/humble/setup.bash
python3 cube_detector.py
```

### Terminal 3: inspect outputs
```bash
ros2 topic echo /cube_detections --once
ros2 topic hz /cube_detection/image
rviz2
```
In RViz2, add `Image` display on `/cube_detection/image`.

Outputs:
- `/cube_detection/image` overlay picture
- `/cube_detections` JSON string with color, center pixel, XYZ in meters
- `/cube_positions` PoseArray of unlabeled 3D points for future interfaces

XYZ is an approximate point on the visible cube surface, not the exact 3D cube center or grasp pose. Coordinate frame is RealSense color optical frame: right/down/forward.

If your camera publishes under a different namespace, modify the three topic names at the top of `CubeDetector.__init__`.

### Detection Output Reference

| ROS 2 topic | Output | Purpose |
|---|---|---|
| `/cube_detection/image` | Annotated image | View color detections and overlays |
| `/cube_detections` | JSON string | Color label, center pixel and XYZ (meters) |
| `/cube_positions` | `PoseArray` | Unlabeled 3D points for downstream interfaces |

**Coordinate interpretation:** XYZ describes an approximate point on the visible cube surface, **not** the exact center of the cube or a complete grasp pose. Coordinates are expressed in the RealSense color optical frame (right, down, forward). Downstream navigation/manipulation modules will need appropriate TF transforms.

**Implementation note:** `cube_detector.py` is a separate script required by these commands; the Markdown documentation does not include the script itself. If the camera publishes under a different namespace, update the three subscribed camera topic names in `CubeDetector.__init__`.

## Roadmap

1. Implement 2D LiDAR SLAM (using `slam_toolbox`) and 3D obstacle perception with the D455f camera.
2. Utilize RTAB-Map for joint mapping and localization by combining RGB-D and LiDAR data.
3. Integrate the mobile base, odometry, and Nav2 to enable autonomous navigation.
4. For cube perception, generate a **local point cloud only within each detected cube's region of interest (ROI)**, then estimate geometric properties such as size and visible surface orientation for future grasp planning.

Throughout development, coordinate transformations (TF), sensor calibration, data fusion, perception accuracy and Jetson performance optimization remain important.