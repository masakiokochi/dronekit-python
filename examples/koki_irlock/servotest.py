import pigpio
from time import sleep

class ServoController:
    def __init__(self, gpio_pin):
        self.pi = pigpio.pi()
        self.gpio_pin = gpio_pin

    def set_servo_position(self, pwm):
        """サーボモータのPWMパルス幅を設定します（1000usから2000usの範囲で）"""
        if pwm > 1700:
            pwm = 1700
        elif pwm < 1300:
            pwm = 1300
        self.pi.set_servo_pulsewidth(self.gpio_pin, pwm)

    def stop(self):
        """サーボモータの制御を停止します"""
        self.pi.set_servo_pulsewidth(self.gpio_pin, 0)

def pwm_callback(gpio, level, tick):
    global rise_tick
    if level == 1:
        rise_tick = tick
    elif level == 0:
        pwm_width = tick - rise_tick
        if 500 < pwm_width < 2500:
            servo1.set_servo_position(pwm_width)
            servo2.set_servo_position(pwm_width)
            servo3.set_servo_position(pwm_width)
            servo4.set_servo_position(pwm_width)

rise_tick = 0
PWM_INPUT_PIN = 5
PIN_S1 = 19
PIN_S2 = 26
PIN_S3 = 13
PIN_S4 = 6
SERVO_PINS = [PIN_S1, PIN_S2, PIN_S3, PIN_S4]

pi = pigpio.pi()
pi.set_mode(PIN_S1, pigpio.OUTPUT)
pi.set_mode(PIN_S2, pigpio.OUTPUT)
pi.set_mode(PIN_S3, pigpio.OUTPUT)
pi.set_mode(PIN_S4, pigpio.OUTPUT)
pi.set_mode(PWM_INPUT_PIN, pigpio.INPUT)

servo1 = ServoController(PIN_S1)
servo2 = ServoController(PIN_S2)
servo3 = ServoController(PIN_S3)
servo4 = ServoController(PIN_S4)

pi.callback(PWM_INPUT_PIN, pigpio.EITHER_EDGE, pwm_callback)

try:
    while True:
        sleep(1)
except KeyboardInterrupt:
    servo.stop()
    pi.stop()
