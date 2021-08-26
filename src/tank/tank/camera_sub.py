

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv2.data import haarcascades
from cv_bridge import CvBridge


class CameraSub(Node):

    def __init__(self):
        super().__init__('camera_sub')
        print(f'haarcascades:{haarcascades}')
        self._face_cascade = cv2.CascadeClassifier(
            haarcascades + 'haarcascade_frontalface_alt2.xml')
        self._eye_cascade = cv2.CascadeClassifier(
            haarcascades + 'haarcascade_eye_tree_eyeglasses.xml')
        self._bridge = CvBridge()
        self._subscriber = self.create_subscription(
            msg_type=Image,
            topic='camera',
            callback=self.listener_callback,
            qos_profile=10
        )

        self._subscriber

        self.get_logger().info('camera sub starting')

    def listener_callback(self, image: Image):
        frame = self._bridge.imgmsg_to_cv2(image)
        frame_grey = cv2.cvtColor(
            src=frame,
            code=cv2.COLOR_BGR2GRAY
        )
        frame_grey = cv2.equalizeHist(
            src=frame_grey
        )
        faces = self._face_cascade.detectMultiScale(frame_grey)
        self.get_logger().info('faces:{0}'.format(len(faces)))
        for (x, y, w, h) in faces:
            cv2.rectangle(img=frame, pt1=(x, y),
                          pt2=(x + h, y + w),
                          color=(0, 255, 0), thickness=3)
            faceROI = frame_grey[y:y+h, x:x+w]
            eyes = self._eye_cascade.detectMultiScale(faceROI)
            for (x2, y2, w2, h2) in eyes:
                eye_center = (x + x2 + w2//2, y + y2 + h2//2)
                radius = int(round((w2 + h2)*0.25))
                frame = cv2.circle(img=frame, center=eye_center, radius=radius,
                                   color=(255, 0, 0), thickness=4)

        cv2.imshow(
            winname='camera_sub',
            mat=frame
        )
        cv2.waitKey(1)

    def clean_up(self):
        self._subscriber.destroy()
        super().destroy_node()


def main(args=None):
    try:
        rclpy.init(args=args)
        camera_sub = CameraSub()
        rclpy.spin(camera_sub)
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
