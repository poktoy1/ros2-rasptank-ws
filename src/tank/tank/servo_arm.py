

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
    current_angle: float
    joint: int


JOINT_GRIP = JointClass(max_angle=100, min_angle=30, angle=0, current_angle=0, joint=15)
JOINT_WRIST = JointClass(max_angle=170, min_angle=30, angle=0, current_angle=0, joint=14)
JOINT_ELBOW = JointClass(max_angle=90, min_angle=20, angle=0, current_angle=0, joint=13)
JOINT_SHOULDER = JointClass(max_angle=90, min_angle=10, angle=10, current_angle=0, joint=12)
JOINT_SHOULDER_BASE = JointClass(max_angle=90, min_angle=10, angle=0, current_angle=0, joint=10)
JOINT_WAIST = JointClass(max_angle=180, min_angle=0, angle=0, current_angle=0, joint=9)

SLEEP_DURATION = 1e-3


class ServoArm():

    def __init__(self):
        try:
            self.__kit = ServoKit(channels=16)
            # self.__kit.servo[JOINT_GRIP.joint].angle = JOINT_GRIP.min_angle
            # self.__kit.servo[JOINT_WRIST.joint].angle = JOINT_WRIST.max_angle/2
            # self.__kit.servo[JOINT_ELBOW.joint].angle = JOINT_ELBOW.max_angle/2
            # self.__kit.servo[JOINT_SHOULDER.joint].angle = JOINT_SHOULDER.max_angle/2
        except Exception:
            pass

    def moveJoint(self, jointClass: JointClass):
        try:
            if(jointClass.min_angle < jointClass.angle < jointClass.max_angle):
                self.__kit.servo[jointClass.joint].angle = jointClass.angle
                jointClass.current_angle = jointClass.angle
            
        except Exception:
            pass

        return jointClass

    async def run_servos(self, jointClasses: list()):
        try:
            tasks = await asyncio.gather(
                *[self.__servo_sweep(jointClass) for jointClass in jointClasses])
            # print(f'tasks:{tasks}')
            return tasks
        except Exception:
            pass

    async def __servo_sweep(self, jointClass: JointClass):
        try:

            current_angle = jointClass.current_angle

            if(jointClass.angle >= current_angle):
                start_index = current_angle
                end_index = jointClass.angle
                step_ = 0.1
            else:
                start_index = current_angle
                end_index = jointClass.angle
                step_ = -0.1

            for i in np.arange(start_index, end_index, step_):
                jointClass.angle = i
                results = self.moveJoint(jointClass)
                # print(f'current_angle:{results}')
                await asyncio.sleep(SLEEP_DURATION)
                
            
            return results

        except Exception:
            pass


async def main(args=None):
    print('Test sweep')
    servoArm = ServoArm()

    await asyncio.sleep(2)
    while(True):
        # jointClasses = [JOINT_GRIP, JOINT_WRIST, JOINT_ELBOW, JOINT_SHOULDER]
        jointClasses = [JOINT_GRIP, JOINT_WRIST]
        JOINT_GRIP.angle = JOINT_GRIP.min_angle
        JOINT_WRIST.angle = JOINT_WRIST.min_angle
        JOINT_ELBOW.angle = JOINT_ELBOW.min_angle
        JOINT_SHOULDER.angle = JOINT_SHOULDER.min_angle
        JOINT_SHOULDER_BASE.angle = JOINT_SHOULDER_BASE.min_angle
        JOINT_WAIST.angle = JOINT_WAIST.min_angle

        results = await servoArm.run_servos([JOINT_SHOULDER])
        print(f'results:{results}')
        results = await servoArm.run_servos([JOINT_SHOULDER_BASE])
        print(f'results:{results}') 

        await asyncio.sleep(5)

        JOINT_GRIP.angle = JOINT_GRIP.max_angle
        JOINT_WRIST.angle = JOINT_WRIST.max_angle
        JOINT_ELBOW.angle =JOINT_ELBOW.max_angle
        JOINT_SHOULDER.angle = JOINT_SHOULDER.max_angle
        JOINT_SHOULDER_BASE.angle = JOINT_SHOULDER_BASE.max_angle
        JOINT_WAIST.angle = JOINT_WAIST.max_angle

        results = await servoArm.run_servos([JOINT_SHOULDER_BASE])
        print(f'results:{results}')
        results = await servoArm.run_servos([JOINT_SHOULDER])
        print(f'results:{results}')   
        
        
        await asyncio.sleep(5)
        

if __name__ == '__main__':
    asyncio.run(main())
    print('main')
