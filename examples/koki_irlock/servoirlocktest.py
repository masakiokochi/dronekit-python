#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import math
import subprocess
import select
import os
import sys
import argparse
import pigpio

# パラメータ設定
# サーボ関係
PWM_MAX = 1700
PWM_MIN = 1300
PWM_INPUT_PIN = 5
PIN_S1 = 19
PIN_S2 = 26
PIN_S3 = 13
PIN_S4 = 6
SERVO_PINS = [PIN_S1, PIN_S2, PIN_S3, PIN_S4]

# pixycam
PIXY_MAX_X = 315
PIXY_MAX_Y = 207
PIXY_CENTER_X = 158
PIXY_CENTER_Y = 104
PIXY_FOV_X = 60    # [degree] field of view
PIXY_FOV_Y = PIXY_FOV_X/(PIXY_MAX_X+1)*(PIXY_MAX_Y+1)

# PID関係
X_P = 0.5
X_I = 0.0
X_D = 0.2
Y_P = X_P
Y_I = X_I
Y_D = X_D

def process_frame_data(line, current_frame, max_area, center_x, center_y):
    parts = line.split()

    # 'frame'というキーワードが含まれているかどうかを確認
    if "frame" not in parts[0] or len(parts) < 10:
        print(line.strip())
        return current_frame, max_area, center_x, center_y

    frame_number = int(parts[1][:-1])  # 'frame 1234:' から '1234' を取得

    x, y, width, height, area = extract_object_data(parts)
    max_area, center_x, center_y = update_max_area_object(x, y, width, height, area, max_area, center_x, center_y)

    return frame_number, max_area, center_x, center_y


def extract_object_data(parts):
    x = int(parts[5])
    y = int(parts[7])
    width = int(parts[9])
    height = int(parts[11])
    area = width * height
    return x, y, width, height, area


def update_max_area_object(x, y, width, height, area, max_area, center_x, center_y):
    if area > max_area:
        max_area = area
        center_x = x + width / 2
        center_y = y + height / 2
    return max_area, center_x, center_y


def calc_target_dist(center_x, center_y):
    dist = 1    # [m] for test
    x_from_center = center_x - PIXY_CENTER_X
    y_from_center = center_y - PIXY_CENTER_Y
    x_dist_max = dist * math.tan(math.radians(PIXY_FOV_X / 2))
    y_dist_max = dist * math.tan(math.radians(PIXY_FOV_Y / 2))
    x_dist = x_dist_max / PIXY_CENTER_X * x_from_center
    y_dist = y_dist_max / PIXY_CENTER_Y * y_from_center
    # print(f"x dist {x_dist} y dist {y_dist}")
    return x_dist, y_dist

# calculate x y distance from pixy coordinate to vehicle coordinate
# pixy coordinate : x right, y back
# vehicle coordinate : x front, y right
def dist_pixy2vehicle(x_pixy, y_pixy):
    x_vehicle = - y_pixy
    y_vehicle = x_pixy
    return x_vehicle, y_vehicle


class PIDController:
    def __init__(self, P, I, D):
        self.P = P
        self.I = I
        self.D = D
        self.reset()

    def reset(self):
        self._prev = 0
        self._integral = 0
        self._prev_time = time.time()

    def compute(self, target, current):
        current_time = time.time()
        dt = current_time - self._prev_time

        # PID制御の各項を計算
        error = target - current
        self._integral += error * dt
        #derivative = (current - self._prev) / dt
        derivative = (self._prev - current) / dt

        # 各項を重み付けして合計
        output = self.P * error + self.I * self._integral + self.D * derivative

        # 現在のヨー角と時間を保存
        self._prev = current
        self._prev_time = current_time

        return output


def pwm_constrain(pwm):
    if pwm > PWM_MAX:
            pwm = PWM_MAX
    elif pwm < PWM_MIN:
        pwm = PWM_MIN
    return pwm

class ServoController:
    def __init__(self, gpio_pin):
        self.pi = pigpio.pi()
        self.gpio_pin = gpio_pin

    def set_servo_position(self, pwm):
        """サーボモータのPWMパルス幅を設定します（1000usから2000usの範囲で）"""
        pwm = pwm_constrain(pwm)
        self.pi.set_servo_pulsewidth(self.gpio_pin, pwm)

    def stop(self):
        """サーボモータの制御を停止します"""
        self.pi.set_servo_pulsewidth(self.gpio_pin, 0)


def throttle2pwmdiff(thr):
    pulsewidth = int(thr * 500)
    return pulsewidth


def pwm_diff_rev(diff, pwm):
    if pwm >= 1500:
        diff_max = PWM_MAX - pwm
        diff_rev = min(diff_max, diff)
    else:
        diff_max = pwm - PWM_MIN
        diff_rev = min(diff_max, diff)
    return diff_rev



def pwm_callback(gpio, level, tick):
    global rise_tick
    global throttle_x, throttle_y
    if level == 1:
        rise_tick = tick
    elif level == 0:
        pwm_width = tick - rise_tick
        if 500 < pwm_width < 2500:
            pwm_width = pwm_constrain(pwm_width)
            pwm_diff_x = throttle2pwmdiff(throttle_x)
            pwm_diff_y = throttle2pwmdiff(throttle_y)
            pwm_diff_x_rev = pwm_diff_rev(pwm_diff_x, pwm_width)
            pwm_diff_y_rev = pwm_diff_rev(pwm_diff_y, pwm_width)

            pwm1 = int(pwm_width + pwm_diff_y_rev)
            pwm2 = int(pwm_width - pwm_diff_x_rev)
            pwm3 = int(pwm_width - pwm_diff_y_rev)
            pwm4 = int(pwm_width + pwm_diff_x_rev)
            print(f"pwm1 {pwm1} pwm2 {pwm2} pwm3 {pwm3} pwm4 {pwm4}")

            servo1.set_servo_position(pwm1)
            servo2.set_servo_position(pwm2)
            servo3.set_servo_position(pwm3)
            servo4.set_servo_position(pwm4)


def cleanup_process(process):
    print("Clean up")
    # サブプロセスの標準出力を閉じる
    process.stdout.close()
    # サブプロセスを終了させる
    process.terminate()
    # サブプロセスが終了するのを待つ
    process.wait()
    print("Subprocess terminated")


# irlockの起動
pixy_path = os.path.expanduser("~/pixy/build/hello_pixy/hello_pixy")
process = subprocess.Popen(['sudo', pixy_path], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

current_frame = None
next_frame = None
max_area = 0  # 最大面積を初期化
center_x, center_y = None, None  # 中心座標を初期化

# フラコンからのsteeringの読み込み
rise_tick = 0

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

# xy方向のコントローラ起動
x_controller = PIDController(X_P, X_I, X_D)
y_controller = PIDController(Y_P, Y_I, Y_D)
throttle_x = 0
throttle_y = 0

try:
    while True:
        # selectを使用して、出力が利用可能かどうかを確認（待機時間0）
        rlist, _, _ = select.select([process.stdout], [], [], 0.1)
        #time.sleep(1)
        if rlist:
            # 出力がある場合の処理
            line = process.stdout.readline()
            if not line:
                #set_mode_loiter()
                #stop_vehicle()
                print("Received empty line")
                break
            next_frame, max_area, center_x, center_y = process_frame_data(line, current_frame, max_area, center_x, center_y)

            if next_frame != current_frame and current_frame is not None and center_x is not None:
                #print(f"Frame {current_frame} target center: ({center_x}, {center_y})")
                # run with irlock
                # calculate distance with pixycam coordinate
                dist_x_pixy, dist_y_pixy = calc_target_dist(center_x, center_y)
                # calculate distance with vehicle coordinate from pixycam corrdinate
                dist_x_vehicle, dist_y_vehicle = dist_pixy2vehicle(dist_x_pixy, dist_y_pixy)
                #print(f"dist x {dist_x_vehicle} dist y {dist_y_vehicle}")
                # compute PID
                throttle_x = x_controller.compute(0, dist_x_vehicle)
                throttle_y = y_controller.compute(0, dist_y_vehicle)
                #print(f"throttle x {dist_x_vehicle} throttle y {dist_y_vehicle}")
                #control_vehicle(dist_north, dist_east)
                #control_vehicle(0.1, 0)

                max_area = 0
                center_x, center_y = None, None
            current_frame = next_frame
            # ここに出力がある場合の追加の動作を記述
        else:
            # 出力がない場合の処理
            # ここに出力がない場合の追加の動作を記述

            print("No output detected")
            throttle_x = 0
            throttle_y = 0
            x_controller.reset()
            y_controller.reset()
            #stop_vehicle()
            pass
except KeyboardInterrupt:
    print('\Exiting due to keyboard interrupt...')
finally:
    cleanup_process(process)
    pi.stop()
