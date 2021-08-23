#!/usr/bin/python3
# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tank.motor import Motor


class TwistSubscriber(Node):

    def __init__(self, motor: Motor):
        super().__init__('twist_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.motor = motor
        self.get_logger().info('init')
        timer_period = 0.2  # seconds
        self.last_time_received = time.time()
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        current_time = time.time() - self.last_time_received

        if(current_time > 0.5):
            self.motor.stop()
            self.last_time_received = time.time()

    def listener_callback(self, msg: Twist):

        print(f'linear.x:{msg.linear.x},right:{msg.angular.z}')

        left = (msg.linear.x + msg.angular.z) * 100
        right = (msg.linear.x - msg.angular.z) * 100

        print(f'left:{left},right:{right}')

        self.motor.move(left, right)
        self.last_time_received = time.time()


def main(args=None):

    try:
        rclpy.init(args=args)
        motor = Motor()
        twist_subscriber = TwistSubscriber(motor)
        rclpy.spin(twist_subscriber)
    except KeyboardInterrupt:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        print('KeyboardInterrupt')
    except Exception:
        print('other error', sys.exc_info()[0])
    finally:
        twist_subscriber.destroy_node()
        motor.destroy()
        rclpy.shutdown()
        print('onDestroy')


if __name__ == '__main__':
    main()
