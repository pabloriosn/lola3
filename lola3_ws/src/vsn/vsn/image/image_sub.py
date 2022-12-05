import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageSubscriber(Node):
    def __init__(self, sub_name: str = "/camera/color/image_raw") -> None:
        super().__init__('Image_subscriber_node')
        self._rgb_image = None

        self.get_logger().info("Connecting camera...")
        # Create the subscriber
        self.subscription = self.create_subscription(Image, sub_name, self._rgb_callback, 1)
        #rclpy.spin(image_subscriber)

        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()

    def _rgb_callback(self, data: Image) -> np.array:
        try:
            # Convert Ros Image message to OpenCv image
            self._rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def get_image_rgb(self):
        return self._rgb_image
