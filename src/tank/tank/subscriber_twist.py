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
try:
    import RPi.GPIO as GPIO
except Exception:
    import Mock.GPIO as GPIO


class Motor():

    # from motor schematic
    Motor_A_EN = 4
    Motor_B_EN = 17
    Motor_A_Pin1 = 14
    Motor_A_Pin2 = 15
    Motor_B_Pin1 = 27
    Motor_B_Pin2 = 18

    def __init__(self):
        # global pwm_A, pwm_B
        self.setup()
        self.stop()
        try:
            self.pwm_A = GPIO.PWM(self.Motor_A_EN, 2000)
            self.pwm_B = GPIO.PWM(self.Motor_B_EN, 2000)
        except Exception:
            pass

    def stop(self):  # Motor stops

        GPIO.output(self.Motor_A_Pin1, GPIO.LOW)
        GPIO.output(self.Motor_A_Pin2, GPIO.LOW)
        GPIO.output(self.Motor_B_Pin1, GPIO.LOW)
        GPIO.output(self.Motor_B_Pin2, GPIO.LOW)
        GPIO.output(self.Motor_A_EN, GPIO.LOW)
        GPIO.output(self.Motor_B_EN, GPIO.LOW)

    def setup(self):  # Motor initialization

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.Motor_A_EN, GPIO.OUT)
        GPIO.setup(self.Motor_B_EN, GPIO.OUT)
        GPIO.setup(self.Motor_A_Pin1, GPIO.OUT)
        GPIO.setup(self.Motor_A_Pin2, GPIO.OUT)
        GPIO.setup(self.Motor_B_Pin1, GPIO.OUT)
        GPIO.setup(self.Motor_B_Pin2, GPIO.OUT)

    def move(self, left, right):  # Motor move

        left_forward = left > 0
        right_forward = right > 0

        in_min = 0
        in_max = 100
        out_min = 0
        out_max = 100

        abs_left = abs(left)
        abs_right = abs(right)
        if(abs_left > in_max):
            abs_left = in_max
        if(abs_right > in_max):
            abs_right = in_max
        map_left = (abs_left - in_min) * (out_max - out_min) / \
            (in_max - in_min) + out_min
        map_right = (abs_right - in_min) * (out_max - out_min) / \
            (in_max - in_min) + out_min

        print(f'leftForward:{left_forward}, rightForward:{right_forward}')
        print(f'mapLeft:{map_left}, mapRight:{map_right}')
        if left_forward:
            GPIO.output(self.Motor_B_Pin1, GPIO.LOW)
            GPIO.output(self.Motor_B_Pin2, GPIO.HIGH)
        else:
            GPIO.output(self.Motor_B_Pin1, GPIO.HIGH)
            GPIO.output(self.Motor_B_Pin2, GPIO.LOW)
        if right_forward:
            GPIO.output(self.Motor_A_Pin1, GPIO.HIGH)
            GPIO.output(self.Motor_A_Pin2, GPIO.LOW)
        else:
            GPIO.output(self.Motor_A_Pin1, GPIO.LOW)
            GPIO.output(self.Motor_A_Pin2, GPIO.HIGH)

        self.pwm_B.start(0)
        self.pwm_B.ChangeDutyCycle(map_left)
        self.pwm_A.start(100)
        self.pwm_A.ChangeDutyCycle(map_right)
        # time.sleep(0.4)
        # self.motorStop()

    def destroy(self):
        self.stop()
        GPIO.cleanup()


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
        timer_period = 0.5  # seconds
        self.last_time_received = time.time()
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        current_time = time.time() - self.last_time_received

        if(current_time > 1):
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
