

import sys


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Int8

try:
    import RPi.GPIO as GPIO
except Exception:
    import Mock.GPIO as GPIO


class LineInfraRed():
    INFRA_RIGHT = 20
    INFRA_MIDDLE = 16
    INFRA_LEFT = 19

    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.INFRA_RIGHT, GPIO.IN)
        GPIO.setup(self.INFRA_MIDDLE, GPIO.IN)
        GPIO.setup(self.INFRA_LEFT, GPIO.IN)

    def get_infra_status(self):
        right = GPIO.input(self.INFRA_RIGHT)
        middle = GPIO.input(self.INFRA_MIDDLE)
        left = GPIO.input(self.INFRA_LEFT)
        return right, middle, left


class LineInfraRedPublisher(Node):

    def __init__(self, lineInfra: LineInfraRed):
        super().__init__('line_infrared')
        # super().__init__('sonar_publisher')
        self.publisher_ = self.create_publisher(Int8, 'robot/infra/line', 10)
        self.lineInfra = lineInfra
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        right, middle, left = self.lineInfra.get_infra_status()
        print(f'right:{right}, middle:{middle}, left:{left}')
        
        msg = Int8()
        data = 0
        data = data or (right << 2)
        data = data or (middle << 1)
        data = data or (left)
        
        print(f'data:{data}')
        msg.data = data
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s,%s,%s"' % right, middle, left)


def main(args=None):

    # rclpy.init(args=args)
    # lineInfra = LineInfraRed()
    # minimal_publisher = LineInfraRedPublisher(lineInfra)
    # rclpy.spin(minimal_publisher)
    # minimal_publisher.destroy_node()
    # rclpy.shutdown()

    try:

        rclpy.init(args=args)
        lineInfra = LineInfraRed()
        minimal_publisher = LineInfraRedPublisher(lineInfra)
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        print('KeyboardInterrupt')
    except Exception:
        print('Exception error', sys.exc_info())

    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
