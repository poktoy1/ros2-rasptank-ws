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
import rclpy
from rclpy.node import Node
from tank.sonar import Sonar
from sensor_msgs.msg import Range


class SonarPublisher(Node):

    def __init__(self, sonar: Sonar):
        super().__init__('sonar_publisher')
        self.publisher_ = self.create_publisher(Range, 'robot/sonar/collision', 10)
        self.sonar = sonar
        timer_period = 0.5  # seconds

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg: Range = self.sonar.getRange()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.range)


def main(args=None):

    try:
        rclpy.init(args=args)
        sonar = Sonar()
        minimal_publisher = SonarPublisher(sonar)
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
        sonar.destroy()


if __name__ == '__main__':
    main()
