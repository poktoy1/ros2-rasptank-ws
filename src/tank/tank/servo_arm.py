


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

JOINT_ELBOW = {
    'max': 140,
    'min': 0,
    'angle': 0,
    'joint': 13
}

JOINT_SHOULDER = {
    'max': 140,
    'min': 20,
    'angle': 90,
    'joint': 12
}



class ServoArm():

    def __init__(self):
        try:
            self._kit = ServoKit(channels=16)
            self._kit.servo[JOINT_GRIP['joint']].angle = JOINT_GRIP['max']/2
            self._kit.servo[JOINT_WRIST['joint']].angle = JOINT_WRIST['max']/2
            self._kit.servo[JOINT_ELBOW['joint']].angle = JOINT_ELBOW['max']/2
            self._kit.servo[JOINT_SHOULDER['joint']].angle = JOINT_SHOULDER['max']/2
        except Exception:
            pass

    def moveJoint(self, joint: int, angle: int, max: int, min: int):
        try:
            if(min < angle < max):
                self._kit.servo[joint].angle = angle
        except Exception:
            pass