

try:
    from adafruit_servokit import ServoKit
except Exception:
    pass

import asyncio
import numpy as np

from dataclasses import dataclass


@dataclass
class JointClass:
    max_angle: float
    min_angle: float
    angle: float
    joint: int


JOINT_GRIP = JointClass(max_angle=90, min_angle=0, angle=0, joint=15)

JOINT_WRIST = JointClass(max_angle=180, min_angle=0, angle=0,  joint=14)


JOINT_ELBOW = JointClass(max_angle=140, min_angle=0, angle=0,  joint=13)


JOINT_SHOULDER = JointClass(max_angle=140, min_angle=20, angle=90,  joint=12)


SLEEP_DURATION = 1e-3


class ServoArm():

    def __init__(self):
        try:
            self.__kit = ServoKit(channels=16)
            self.__kit.servo[JOINT_GRIP.joint].angle = JOINT_GRIP.max_angle/2
            self.__kit.servo[JOINT_WRIST.joint].angle = JOINT_WRIST.max_angle/2
            self.__kit.servo[JOINT_ELBOW.joint].angle = JOINT_ELBOW.max_angle/2
            self.__kit.servo[JOINT_SHOULDER.joint].angle = JOINT_SHOULDER.max_angle/2
        except Exception:
            pass

    def moveJoint(self, jointClass: JointClass):
        try:
            if(jointClass.min_angle < jointClass.angle < jointClass.max_angle):
                self.__kit.servo[jointClass.joint].angle = jointClass.angle
        except Exception:
            pass

    async def run_servos(self, jointClasses: list()):
        try:
            tasks = await asyncio.gather(
                *[self.__servo_sweep(jointClass) for jointClass in jointClasses])
            return tasks
        except Exception:
            pass

    async def __servo_sweep(self, jointClass: JointClass):
        try:

            current_angle = self.__kit.servo[jointClass.joint].angle

            if(jointClass.angle > current_angle):
                start_index = current_angle
                end_index = jointClass.angle
                step_ = 0.5
            else:
                start_index = current_angle
                end_index = jointClass.angle
                step_ = -0.5

            for i in np.arange(start_index, end_index, step_):
                jointClass.angle = i
                self.moveJoint(jointClass)
                await asyncio.sleep(SLEEP_DURATION)
            current_angle = self.__kit.servo[jointClass.joint].angle
            return current_angle

        except Exception:
            pass


async def main(args=None):
    print('Test sweep')
    servoArm = ServoArm()

    await asyncio.sleep(2)
    while(True):
        jointClasses = [JOINT_GRIP, JOINT_WRIST, JOINT_ELBOW, JOINT_SHOULDER]
        JOINT_GRIP.angle = 0
        JOINT_WRIST.angle = 0
        JOINT_ELBOW.angle = 120
        JOINT_SHOULDER.angle = 70

        await servoArm.run_servos(jointClasses)

        JOINT_GRIP.angle = 85
        JOINT_WRIST.angle = 180
        JOINT_ELBOW.angle = 70
        JOINT_SHOULDER.angle = 90
        await servoArm.run_servos(jointClasses)


if __name__ == '__main__':
    asyncio.run(main())
    print('main')
