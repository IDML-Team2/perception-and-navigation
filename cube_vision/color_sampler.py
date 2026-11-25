import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import numpy as np


class ColorSampler(Node):

    def __init__(self):
        super().__init__('color_sampler')

        self.bridge = CvBridge()
        self.frame = None
        self.radius = 5

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        cv2.namedWindow('Color Sampler')
        cv2.setMouseCallback(
            'Color Sampler',
            self.mouse_callback
        )

    def image_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8'
        )

        cv2.imshow('Color Sampler', self.frame)
        cv2.waitKey(1)

    def mouse_callback(self, event, x, y, flags, param):

        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.frame is None:
            return

        r = self.radius

        roi = self.frame[
            max(0, y-r):y+r+1,
            max(0, x-r):x+r+1
        ]

        rgb = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        rgb = rgb.reshape(-1, 3)
        hsv = hsv.reshape(-1, 3)

        print('\n' + '=' * 40)
        print(f'Position: ({x}, {y})')

        print('RGB Mean:', rgb.mean(axis=0).astype(int))
        print('RGB Min: ', rgb.min(axis=0))
        print('RGB Max: ', rgb.max(axis=0))

        print('HSV Mean:', hsv.mean(axis=0).astype(int))
        print('HSV Min: ', hsv.min(axis=0))
        print('HSV Max: ', hsv.max(axis=0))


def main():
    rclpy.init()
    node = ColorSampler()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
