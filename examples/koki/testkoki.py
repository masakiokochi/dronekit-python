#!/usr/bin/env python
# -*- coding: utf-8 -*-
import threading
import time
from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import math
import subprocess
import select
import os
import sys
import argparse
import pigpio


process = None

_mode = None
_pitch = 0.0
_roll = 0.0
_yaw = 0.0
_dist = 0.0
_north_speed = 0.0
_east_speed = 0.0
_down_speed = 0.0
_current_location = LocationGlobalRelative(0.0, 0.0, 0.0)

global _target_yaw
global _prev_yaw
global _integral

_target_yaw = 0.0
_prev_yaw = 0.0
_integral = 0.0

YAW_P = 1.0
YAW_I = 0.1
YAW_D = 0.01

# Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    print("Please set connection_string")
    sys.exit(1) # finish this program


def main():
    global process
    global vehicle  # グローバル変数を使用することを宣言
    # Connect to the Vehicle
    vehicle = connect_vehicle()  # vehicle変数に代入
    # prearm check
    prearm_check()
    # mode change to LATGUIDED
    set_mode_LATGUIDED()
    YawPIDController.init()
    # set target yaw
    set_target_yaw()
    # arm
    arm_vehicle()
    # control vehicle
    control_vehicle()


def connect_vehicle():
    print('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)
    return vehicle


def prearm_check():
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    return


def set_mode_LATGUIDED():
    vehicle.mode = VehicleMode("MANUAL")
    time.sleep(1)
    # Vehicle should arm in GUIDED mode
    vehicle.mode = VehicleMode("LATGUIDED")
    time.sleep(1)
    return


def set_target_yaw():
    _target_yaw = vehicle.attitude.yaw


def arm_vehicle():
    print("Arming motors")
    vehicle.armed = True
    time.sleep(1)
    # Confirm vehicle armed before attempting to take off
    for _ in range(5):  # 5回の試行
        if vehicle.armed:
            print("Vehicle is armed!")
            break
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(1)
    else:
        print("Failed to arm the vehicle.")
        return

def disarm_vehicle():
    print("Disarming motors")
    vehicle.armed = False
    time.sleep(1)
    # Confirm vehicle disarmed
    for _ in range(5):  # 5回の試行
        if not vehicle.armed:
            print("Vehicle is disarmed!")
            break
        print(" Waiting for disarming...")
        vehicle.armed = False
        time.sleep(1)
    else:
        print("Failed to disarm the vehicle.")
        return


def stop_vehicle():
    # 位置を設定
    target_location = LocationGlobalRelative(_current_location.lat, _current_location.lon, _current_location.alt)
    # 新しい位置に移動
    vehicle.simple_goto(target_location)

def control_vehicle():
    global _mode, _pitch, _roll, _yaw, _dist, _north_speed, _east_speed, _down_speed
    global _current_location

    current_time = time.time()
    print(current_time)

    update_vehicle_state()
    yaw_controller = YawPIDController()
    throttle_yaw = yaw_controller.compute(_target_yaw, _yaw)
    print("throttle_yaw:%f", throttle_yaw)
    #set_servo(throttle_yaw)
    # タイマーをセット
    timer = threading.Timer(1.0, control_vehicle)
    timer.control_vehicle()


def update_vehicle_state():
    current_time = time.time()
    print(current_time)
    print(_yaw)

    _mode = vehicle.mode.name
    _pitch = vehicle.attitude.pitch
    _roll = vehicle.attitude.roll
    _yaw = vehicle.attitude.yaw
    _dist = vehicle.rangefinder.distance
    _north_speed, _east_speed, _down_speed = vehicle.velocity
    _current_location = vehicle.location.global_relative_frame
    # 5秒ごとにprint_message関数を実行するタイマーをセット
    timer = threading.Timer(1.0, update_vehicle_state)
    timer.update_vehicle_state()


class YawPIDController:
    def __init__(self):
        self.YAW_P = YAW_P
        self.YAW_I = YAW_I
        self.YAW_D = YAW_D

        self._prev_yaw = 0
        self._integral = 0
        self._prev_time = time.time()

    def compute(self, setpoint, _yaw):
        current_time = time.time()
        dt = current_time - self._prev_time

        # PID制御の各項を計算
        error = setpoint - _yaw
        self._integral += error * dt
        derivative = (error - self._prev_yaw) / dt

        # 各項を重み付けして合計
        output = self.YAW_P * error + self.YAW_I * self._integral + self.YAW_D * derivative

        # 現在のヨー角と時間を保存
        self._prev_yaw = _yaw
        self._prev_time = current_time

        return output


class ServoController:
    def __init__(self, gpio_pin):
        self.pi = pigpio.pi()
        self.gpio_pin = gpio_pin

    def set_servo_pulsewidth(self, throttle):
        """サーボモータのPWMパルス幅を設定します（1000usから2000usの範囲で）"""
        if throttle > 1:
            throttle = 1
        elif throttle < -1:
            throttle = -1
        pulsewidth = int((throttle + 1) * 500 + 1000)
        self.pi.set_servo_pulsewidth(self.gpio_pin, pulsewidth)

    def stop(self):
        """サーボモータの制御を停止します"""
        self.pi.set_servo_pulsewidth(self.gpio_pin, 0)
        self.pi.stop()

if __name__ == "__main__":
    main()
