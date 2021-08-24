import time
from sensor_msgs.msg import Range
try:
    import RPi.GPIO as GPIO
    DEBUG = False
except Exception:
    import Mock.GPIO as GPIO
    import random
    DEBUG = True


class Sonar():

    TRIGGER = 11
    ECHO = 8
    SONAR_TIMEOUT = 0.02

    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.TRIGGER, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.ECHO, GPIO.IN)
        self.msg = Range()
        self.msg.radiation_type = 0
        self.msg.min_range = 0.5 if (not DEBUG) else 1.7
        self.msg.max_range = 4.0
        self.last_saved_time = time.time()

    def getRange(self):

        GPIO.output(self.TRIGGER, GPIO.HIGH)
        time.sleep(0.000010)
        GPIO.output(self.TRIGGER, GPIO.LOW)
        self.last_saved_time = time.time()
        current_time = 0.0

        while (not GPIO.input(self.ECHO) and (current_time < self.SONAR_TIMEOUT)):
            current_time = time.time() - self.last_saved_time
            pass
        echo_off_time = time.time() if (not DEBUG) else (time.time() + 0.005)

        self.last_saved_time = time.time()
        current_time = 0.0

        while (GPIO.input(self.ECHO) and (current_time < self.SONAR_TIMEOUT)):
            current_time = time.time() - self.last_saved_time
            pass
        echo_on_time = time.time() if (not DEBUG) else (time.time() + random.uniform(0.005, 0.015))

        time_traveled = echo_on_time - echo_off_time
        range = time_traveled * (343/2)
        self.msg.range = range
        return self.msg

    def destroy(self):
        GPIO.cleanup()
