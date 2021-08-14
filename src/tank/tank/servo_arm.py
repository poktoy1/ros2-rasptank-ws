

try:
    from adafruit_servokit import ServoKit
except Exception:
    pass

import asyncio
import time
import numpy as np

JOINT_GRIP = {
    'max_angle': 90,
    'min_angle': 0,
    'angle': 0,
    'joint': 15
}
JOINT_WRIST = {
    'max_angle': 180,
    'min_angle': 0,
    'angle': 0,
    'joint': 14
}

JOINT_ELBOW = {
    'max_angle': 140,
    'min_angle': 0,
    'angle': 0,
    'joint': 13
}

JOINT_SHOULDER = {
    'max_angle': 140,
    'min_angle': 20,
    'angle': 90,
    'joint': 12
}

SLEEP_DURATION = 1e-3


class ServoArm():

    def __init__(self):
        try:
            self._kit = ServoKit(channels=16)
            self._kit.servo[JOINT_GRIP['joint']].angle = JOINT_GRIP['max_angle']/2
            self._kit.servo[JOINT_WRIST['joint']].angle = JOINT_WRIST['max_angle']/2
            self._kit.servo[JOINT_ELBOW['joint']].angle = JOINT_ELBOW['max_angle']/2
            self._kit.servo[JOINT_SHOULDER['joint']].angle = JOINT_SHOULDER['max_angle']/2
        except Exception:
            pass

    def moveJoint(self, joint: int, angle: float, max_angle: float, min_angle: float):
        try:
            if(min_angle < angle < max_angle):
                self._kit.servo[joint].angle = angle
        except Exception:
            pass

    async def test_sweep(self, joint: int, angle: int, max_angle: int, min_angle: int):
        try:

            current_angle = self._kit.servo[joint].angle

            if(angle > current_angle):
                start_index = current_angle
                end_index = angle
                step_ = 0.5
            else:
                start_index = current_angle
                end_index = angle
                step_ = -0.5

            for i in np.arange(start_index, end_index, step_):

                self.moveJoint(joint, i, max_angle, min_angle)
                await asyncio.sleep(SLEEP_DURATION)
            current_angle = self._kit.servo[joint].angle

        except Exception:
            pass


async def main(args=None):
    print('Test sweep')
    servoArm = ServoArm()

    await asyncio.sleep(2)
    while(True):
        JOINT_GRIP['angle'] = 0
        JOINT_WRIST['angle'] = 0
        JOINT_ELBOW['angle'] = 120
        JOINT_SHOULDER['angle'] = 70
        await asyncio.gather(servoArm.test_sweep(**JOINT_ELBOW),
                             servoArm.test_sweep(**JOINT_WRIST),
                             servoArm.test_sweep(**JOINT_GRIP),
                             servoArm.test_sweep(**JOINT_SHOULDER),
                             )
        JOINT_GRIP['angle'] = 85
        JOINT_WRIST['angle'] = 180
        JOINT_ELBOW['angle'] = 70
        JOINT_SHOULDER['angle'] = 90
        await asyncio.gather(servoArm.test_sweep(**JOINT_ELBOW),
                             servoArm.test_sweep(**JOINT_WRIST),
                             servoArm.test_sweep(**JOINT_GRIP),
                             servoArm.test_sweep(**JOINT_SHOULDER),
                             )


if __name__ == '__main__':
    asyncio.run(main())
    # main()
    print('main')
