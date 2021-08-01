#!/usr/bin/python3
try:
    import RPi.GPIO as GPIO
except:
    import Mock.GPIO as GPIO

class Motor():

    #from motor schematic
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
        except:
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

    def move(self, left, right):    #Motor move

        left_forward = left > 0
        right_forward = right > 0

        in_min = 0
        in_max = 200
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