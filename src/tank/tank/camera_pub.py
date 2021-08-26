#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv2 import VideoCapture
from cv_bridge import CvBridge


class CamPublisher(Node):

    def __init__(self, camera_device: int):
        super().__init__('sonar_publisher')
        self._publisher = self.create_publisher(Image, 'camera', 10)
        self._cap: VideoCapture = VideoCapture(camera_device)
        self._bridge = CvBridge()
        timer_period = 0.1  # seconds

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        result, frame = self._cap.read()
        if result is True:
            self._publisher.publish(self._bridge.cv2_to_imgmsg(frame))
        self.get_logger().info('camera read:{0}'.format(result))

    def clean_up(self):

        self._publisher.destroy()
        super().destroy_node()


def main(args=None):

    try:
        rclpy.init(args=args)
        cam_publisher = CamPublisher(0)
        rclpy.spin(cam_publisher)
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        cam_publisher.clean_up()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
