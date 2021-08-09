
from adafruit_servokit import ServoKit
import rclpy
from rclpy.node import Node

JOINT_GRIP = {
    'max': 90,
    'min': 0,
    'angle': 0,
    'joint': 15
}
JOINT_WRIST = {
    'max': 180,
    'min': 0,
    'angle': 0,
    'joint': 14
}

JOINT_ARM = {
    'max': 140,
    'min': 0,
    'angle': 0,
    'joint': 13
}

JOINT_ELBOW = {
    'max': 160,
    'min': 10,
    'angle': 0,
    'joint': 12
}


class ServoArm():

    def __init__(self):
        self._kit = ServoKit(channels=16)
        self._kit.servo[JOINT_GRIP['joint']].angle = JOINT_GRIP['max']
        self._kit.servo[JOINT_WRIST['joint']].angle = JOINT_WRIST['min']
        self._kit.servo[JOINT_ARM['joint']].angle = JOINT_ARM['min']
        self._kit.servo[JOINT_ELBOW['joint']].angle = JOINT_ELBOW['min']

    def moveJoint(self, joint: int, angle: int, max: int, min: int):
        if(min < angle < max):
            self._kit.servo[joint].angle = angle


class SubscriberArm(Node):

    def __init__(self, servoArm: ServoArm):
        super().__init__('arm_subscriber')
        self.servo_arm_ = servoArm
        self.i = 0
        # test
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.i += 20
        if(self.i > 180):
            self.i = 0
        JOINT_GRIP['angle'] = self.i
        JOINT_WRIST['angle'] = self.i
        JOINT_ARM['angle'] = self.i
        JOINT_ELBOW['angle'] = self.i
        self.servo_arm_.moveJoint(**JOINT_GRIP)
        self.servo_arm_.moveJoint(**JOINT_WRIST)
        self.servo_arm_.moveJoint(**JOINT_ARM)
        self.servo_arm_.moveJoint(**JOINT_ELBOW)
        print(f'i:${self.i}')


def main(args=None):

    try:
        rclpy.init(args=args)
        servoArm = ServoArm()
        subscriber_arm = SubscriberArm(servoArm)
        rclpy.spin(subscriber_arm)
    except KeyboardInterrupt:
        print("KeyboardInterrupt")

    finally:
        subscriber_arm.destroy_node()
        rclpy.shutdown()
        print('shutdown')


if __name__ == '__main__':
    main()
