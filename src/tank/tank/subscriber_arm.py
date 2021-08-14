

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
import tf_transformations
from math import degrees as Degrees
from tank.servo_arm import ServoArm
from tank.servo_arm import JOINT_ELBOW,JOINT_GRIP,JOINT_SHOULDER,JOINT_WRIST

class SubscriberArm(Node):

    def __init__(self, servoArm: ServoArm):
        super().__init__('arm_subscriber')
        self.servo_arm_ = servoArm
        self.i = 0
        # test
        # timer_period = 0.7  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.dec_flag_ = False
        self.still_moving = False
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.listener_callback,
            10)
        self.subscription
        print('initialized')

    def listener_callback(self, msg: TFMessage):
        transforms = msg.transforms
        for transformStamped in transforms:

            deg = self.get_deg(transformStamped)
            if(transformStamped.child_frame_id == 'single_rrbot_link2'):
                JOINT_SHOULDER['angle'] = deg
                self.servo_arm_.moveJoint(**JOINT_SHOULDER)
            elif (transformStamped.child_frame_id == 'single_rrbot_link3'):
                JOINT_ELBOW['angle'] = deg
                self.servo_arm_.moveJoint(**JOINT_ELBOW)
        
        # self.servo_arm_.moveJoint(**JOINT_ELBOW)

    def get_deg(self, transformStamped: TransformStamped):
        quaternion: Quaternion = transformStamped.transform.rotation
        print(f'{transformStamped.child_frame_id}:{quaternion}')
        r,p,y = tf_transformations.euler_from_quaternion(
            [quaternion.w, quaternion.x, quaternion.y, quaternion.z])
        # print(f'euler:{r},{p},{y}')
        deg = Degrees(p)
        angle = 90 - deg
        print(f'deg:{deg}, angle:{angle}')
        return angle


    def timer_callback(self):
        if(self.still_moving == False):
            self.still_moving = True
            if(self.i > 180):
                self.dec_flag_ = True
            elif(self.i < 0):
                self.i = 0
                self.dec_flag_ = False

            if(self.dec_flag_):
                self.i -= 1
            else:
                self.i += 1
            self.still_moving = False

        JOINT_GRIP['angle'] = self.i
        JOINT_WRIST['angle'] = self.i
        JOINT_ELBOW['angle'] = self.i
        JOINT_SHOULDER['angle'] = self.i
        self.servo_arm_.moveJoint(**JOINT_GRIP)
        self.servo_arm_.moveJoint(**JOINT_WRIST)
        self.servo_arm_.moveJoint(**JOINT_ELBOW)
        self.servo_arm_.moveJoint(**JOINT_SHOULDER)
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
