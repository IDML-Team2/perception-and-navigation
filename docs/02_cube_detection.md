
### HSV Cube Detection

The detector uses RealSense color and aligned depth data to identify colored object regions and publish approximate 3D positions. This stage focuses on perception; navigation and grasp control are separate future integrations.

The current implementation uses OpenCV HSV color segmentation for object detection and RGB-D information for 3D localization. Detection results are published through ROS 2 topics for visualization and downstream processing.

#### 1. Install dependencies

```bash
sudo apt update
sudo apt install -y python3-opencv python3-numpy ros-humble-cv-bridge ros-humble-message-filters
source /opt/ros/humble/setup.bash
```

The `realsense2_camera` ROS 2 package must also be installed and available in the ROS 2 environment.

#### 2. Terminal 1: Start the camera

```bash
source /opt/ros/humble/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=true \
  align_depth.enable:=true enable_sync:=true pointcloud.enable:=false
```

Depth alignment is enabled for associating color pixels with depth measurements. The full point cloud is disabled for this detection workflow to reduce unnecessary computation.

#### 3. Terminal 2: Start the detector

From the directory containing `cube_detector.py`:

```bash
source /opt/ros/humble/setup.bash
python3 cube_detector.py
```

Alternatively, if running from the root of the `perception-and-navigation` repository with the script stored in `cube_vision/`:

```bash
source /opt/ros/humble/setup.bash
python3 cube_vision/cube_detector.py
```

> **Note:** `cube_detector.py` is required for this step. The README documents the workflow but does not replace the implementation.

#### 4. Terminal 3: Inspect outputs

Check the available detection topics:

```bash
ros2 topic list | grep cube
```

Inspect color detection results:

```bash
ros2 topic echo /cube_detections --once
```

Monitor the published 3D positions in real time:

```bash
ros2 topic echo /cube_positions/cube_point
```

Verify the message type and publisher:

```bash
ros2 topic info /cube_positions/cube_point
```

Check the annotated image publishing frequency:

```bash
ros2 topic hz /cube_detection/image
```

Start RViz2:

```bash
rviz2
```

In RViz2, add an **Image** display and select `/cube_detection/image`.

#### 5. Published outputs

The Cube Detector publishes the following ROS 2 topics:

| Topic | Message Type | Description |
|---|---|---|
| `/cube_detection/image` | `sensor_msgs/msg/Image` | Camera image with detection overlays |
| `/cube_detections` | `std_msgs/msg/String` | JSON string containing detected color, center pixel, and XYZ coordinates |
| `/cube_positions` | `geometry_msgs/msg/PoseArray` | Array of unlabeled 3D object positions |
| `/cube_positions/cube_point` | `geometry_msgs/msg/PointStamped` | Timestamped 3D position in the camera coordinate frame |

**Coordinate interpretation:** XYZ is an approximate point on the visible object surface, **not** the exact 3D cube center or a grasp pose.

Coordinates are reported in the RealSense **color optical frame**:

- **X:** Right
- **Y:** Down
- **Z:** Forward

If the camera uses a different ROS 2 namespace, modify the subscribed camera topic names in `CubeDetector.__init__` in `cube_detector.py`.

#### 6. PointStamped Position Interface

A `PointStamped` publisher has been added to `cube_detector.py` to provide a standard ROS 2 interface for downstream navigation and manipulation modules.

**Topic:** `/cube_positions/cube_point`

**Message Type:** `geometry_msgs/msg/PointStamped`

The message includes:

| Field | Description |
|---|---|
| `header.stamp` | Timestamp from the corresponding RGB image |
| `header.frame_id` | Camera optical coordinate frame |
| `point.x` | Horizontal 3D coordinate in meters |
| `point.y` | Vertical 3D coordinate in meters |
| `point.z` | Forward depth coordinate in meters |

**Example output:**

```yaml
header:
  stamp:
    sec: 1794914000
    nanosec: 123456789
  frame_id: camera_color_optical_frame
point:
  x: -0.10
  y: 0.19
  z: 0.37
```

The values above are illustrative.

**Implementation**

The publisher is initialized in the detector node:

```python
from geometry_msgs.msg import PointStamped

self.point_pub = self.create_publisher(
    PointStamped,
    '/cube_positions/cube_point',
    10
)
```

After a valid 3D position has been calculated, the detection callback publishes:

```python
point_msg = PointStamped()

point_msg.header = rgb_msg.header

point_msg.point.x = float(x)
point_msg.point.y = float(y)
point_msg.point.z = float(z)

self.point_pub.publish(point_msg)
```

The published position uses the RGB image header to preserve the timestamp and coordinate frame.

The position message is published after a valid depth measurement has been obtained.

**Verification**

The publisher has been successfully tested. ROS 2 messages containing timestamps, coordinate frame information, and XYZ positions are available through:

```bash
ros2 topic echo /cube_positions/cube_point
```

The output updates when valid color-based detections are available.

**Future integration**

Other ROS 2 nodes can subscribe to the position topic to receive detected object coordinates.

For navigation, positions must be transformed from the camera frame into the robot or map frame using TF2. A separate navigation module must calculate a suitable approach pose before sending navigation goals to Nav2.

For robotic manipulation, additional geometric information is needed to estimate the object's dimensions, surface orientation, and suitable grasp poses.

**Current limitations**

- `PointStamped` contains XYZ coordinates but does not include the detected color or object ID.
- Color information is available separately through `/cube_detections`.
- A single `PointStamped` message cannot distinguish multiple detected objects by itself.
- The published coordinates represent an approximate visible-surface position rather than a reconstructed object center.
- The current HSV method identifies colored regions and does not independently verify whether the object is a cube.
- Quantitative position accuracy and temporal stability have not yet been systematically evaluated.

A structured multi-object detection interface may be introduced in a later stage.

#### 7. Current Implementation Status

**Completed**

- [x] Intel RealSense D455f integration with ROS 2 Humble.
- [x] RGB and aligned depth acquisition.
- [x] RGB-D and full point cloud visualization in RViz2.
- [x] HSV-based color region detection.
- [x] Contour extraction and pixel center estimation.
- [x] Initial XYZ position estimation using aligned depth and camera intrinsics.
- [x] Detection result visualization in RViz2.
- [x] ROS 2 detection output publishing.
- [x] `PointStamped` position publisher implementation.
- [x] Successful verification of `/cube_positions/cube_point`.

**Planned**

- [ ] Quantitative position accuracy and stability evaluation.
- [ ] Cube-specific geometric filtering.
- [ ] ROI-based local point cloud generation.
- [ ] Point cloud filtering and surface normal estimation.
- [ ] Cube dimension and geometric center estimation.
- [ ] Extended multi-object detection interface.
- [ ] TF2-based integration with navigation and robotic manipulation modules.

### Roadmap

1. Implement 2D LiDAR SLAM (using `slam_toolbox`) and 3D obstacle perception with the D455f camera.
2. Utilize RTAB-Map to perform joint mapping and localization by fusing RGB-D and LiDAR data.
3. Integrate the mobile base, odometry, and the Nav2 stack to achieve autonomous navigation.
4. Extend cube detection with ROI-based local point cloud processing to estimate object dimensions and surface orientation for future grasping interfaces.

#### Next Step: ROI-Based Local Point Cloud Processing

The next development stage focuses on extracting 3D geometric information from detected objects.

Instead of processing the full camera point cloud, the system will use the detected object's HSV mask and aligned depth image to generate a local point cloud containing only the selected object region.

**Planned workflow:**

1. Extract the detected object's ROI or segmentation mask.
2. Select valid depth pixels within the detected region.
3. Convert depth pixels into 3D points using camera intrinsics.
4. Filter noisy points and estimate visible surface geometry.
5. Publish the local point cloud as a ROS 2 `PointCloud2` message.

**Proposed output topic:**

```text
/cube_positions/local_cloud
```

**Proposed message type:**

```text
sensor_msgs/msg/PointCloud2
```

The local point cloud will support further estimation of:

- Visible object surface geometry.
- Object dimensions.
- Surface normal vectors.
- Approximate geometric center.
- Candidate grasp geometry for future manipulation tasks.

Full object reconstruction and accurate grasp-pose estimation may require additional geometric assumptions or multiple viewpoints.

This extension will remain part of the perception module. Navigation, motion planning, and robotic control will be handled by separate components.

Throughout the development process, it is also necessary to address coordinate transformations (TF), sensor calibration, data fusion, perception, and performance optimization for the Jetson platform.
