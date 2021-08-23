
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


def main(args=None):
    pass


if __name__ == '__main__':
    main()
