

import sys


import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from tank.line_infrared import LineInfraRed


class LineInfraRedPublisher(Node):

    def __init__(self, lineInfra: LineInfraRed):
        super().__init__('line_infrared')

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


def main(args=None):

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
