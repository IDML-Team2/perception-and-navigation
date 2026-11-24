#!/usr/bin/env python3
"""ROS 2 Humble: HSV cube detection + aligned RealSense depth -> camera-frame XYZ.

Inputs (default realsense2_camera names):
  /camera/camera/color/image_raw
  /camera/camera/aligned_depth_to_color/image_raw
  /camera/camera/color/camera_info

Outputs:
  /cube_detection/image  sensor_msgs/Image (overlay)
  /cube_detections       std_msgs/String (JSON: color + approximate surface XYZ)
  /cube_positions        geometry_msgs/PoseArray (unlabeled XYZ, same order as JSON)
"""
import json

import cv2
import numpy as np
import message_filters
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


# HSV OpenCV hue range: 0...179, saturation/value: 0...255.
# Adapt these ranges to your physical cubes and lighting.
COLOR_RANGES = {
    'red': [((0, 90, 70), (10, 255, 255)),
            ((170, 90, 70), (179, 255, 255))],
    'green': [((35, 80, 60), (85, 255, 255))],
    'blue': [((90, 80, 60), (130, 255, 255))],
    'yellow': [((18, 80, 80), (35, 255, 255))],
}
MIN_AREA_PX = 500
MIN_DEPTH_M = 0.20
MAX_DEPTH_M = 6.00


class CubeDetector(Node):
    def __init__(self):
        super().__init__('cube_detector')
        self.bridge = CvBridge()
        self.cam_info = None
        self.frame_count = 0
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self.on_camera_info, qos_profile_sensor_data)

        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw',
            qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw',
            qos_profile=qos_profile_sensor_data)
        # Match RGB and aligned-depth frames with timestamps within 150 ms.
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=15, slop=0.15)
        self.sync.registerCallback(self.on_frames)

        self.image_pub = self.create_publisher(Image, '/cube_detection/image', 10)
        self.json_pub = self.create_publisher(String, '/cube_detections', 10)
        self.positions_pub = self.create_publisher(PoseArray, '/cube_positions', 10)
        self.get_logger().info('Cube detector started; waiting for RGB, aligned depth and CameraInfo.')

    def on_camera_info(self, msg):
        self.cam_info = msg

    def on_frames(self, rgb_msg, depth_msg):
        if self.cam_info is None:
            return
        try:
            bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge conversion failed: {exc}')
            return

        if depth.ndim != 2 or bgr.shape[:2] != depth.shape[:2]:
            self.get_logger().warn('RGB and aligned depth image resolutions do not match.')
            return

        if depth_msg.encoding == '16UC1':
            depth_m = depth.astype(np.float32) * 0.001  # millimeters -> meters
        elif depth_msg.encoding == '32FC1':
            depth_m = depth.astype(np.float32)          # already meters
        else:
            self.get_logger().warn(f'Unexpected depth encoding: {depth_msg.encoding}')
            return

        fx, fy = self.cam_info.k[0], self.cam_info.k[4]
        ppx, ppy = self.cam_info.k[2], self.cam_info.k[5]
        if fx <= 0 or fy <= 0:
            self.get_logger().warn('Invalid camera intrinsics')
            return

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        overlay = bgr.copy()
        results = []
        poses = PoseArray()
        poses.header = rgb_msg.header
        # Optical frame of color camera; same pixel geometry as aligned depth.
        poses.header.frame_id = self.cam_info.header.frame_id or rgb_msg.header.frame_id

        for color_name, intervals in COLOR_RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lo, hi in intervals:
                mask |= cv2.inRange(hsv, np.array(lo, dtype=np.uint8),
                                     np.array(hi, dtype=np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < MIN_AREA_PX:
                    continue
                moments = cv2.moments(contour)
                if moments['m00'] == 0:
                    continue
                u = int(round(moments['m10'] / moments['m00']))
                v = int(round(moments['m01'] / moments['m00']))
                h, w = depth_m.shape
                if not (0 <= u < w and 0 <= v < h):
                    continue

                # Median depth from a small, colored neighborhood around the center;
                # fall back to all colored pixels in the contour bounding box.
                r = 10
                x0, x1 = max(u-r, 0), min(u+r+1, w)
                y0, y1 = max(v-r, 0), min(v+r+1, h)
                patch = depth_m[y0:y1, x0:x1]
                patch_mask = mask[y0:y1, x0:x1] > 0
                valid = patch[patch_mask & np.isfinite(patch) &
                              (patch >= MIN_DEPTH_M) & (patch <= MAX_DEPTH_M)]
                if valid.size < 5:
                    bx, by, bw, bh = cv2.boundingRect(contour)
                    patch = depth_m[by:by+bh, bx:bx+bw]
                    local_mask = np.zeros((bh, bw), dtype=np.uint8)
                    cv2.drawContours(local_mask, [contour - (bx, by)], -1, 255, -1)
                    valid = patch[(local_mask > 0) & np.isfinite(patch) &
                                  (patch >= MIN_DEPTH_M) & (patch <= MAX_DEPTH_M)]
                if valid.size < 5:
                    continue

                z = float(np.median(valid))
                x = float((u - ppx) * z / fx)
                y = float((v - ppy) * z / fy)

                result = {
                    'color': color_name, 'pixel': [u, v],
                    'xyz_m': [round(x, 3), round(y, 3), round(z, 3)],
                    'area_px': round(float(area), 1)
                }
                results.append(result)
                poses.poses.append(Pose(position=Point(x=x, y=y, z=z),
                                        orientation=Quaternion(w=1.0)))

                cv2.drawContours(overlay, [contour], -1, (0, 255, 255), 2)
                cv2.circle(overlay, (u, v), 4, (255, 255, 255), -1)
                caption = f'{color_name}: X={x:.2f} Y={y:.2f} Z={z:.2f} m'
                cv2.putText(overlay, caption, (max(u-90, 4), max(v-12, 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 3)
                cv2.putText(overlay, caption, (max(u-90, 4), max(v-12, 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        image_out = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        image_out.header = rgb_msg.header
        self.image_pub.publish(image_out)
        self.positions_pub.publish(poses)
        json_out = String()
        json_out.data = json.dumps({'frame_id': poses.header.frame_id,
                                    'detections': results}, ensure_ascii=False)
        self.json_pub.publish(json_out)
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(f'Visible color objects: {len(results)}; {json_out.data}')


def main(args=None):
    rclpy.init(args=args)
    node = CubeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
