#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Libraries for koki
'''

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import math
import subprocess
import select
import os
import sys
import argparse
import param as pm
import threading


_mode = None
_pitch = 0.0
_roll = 0.0
_yaw = 0.0
_dist = 0.0
_north_speed = 0.0
_east_speed = 0.0
_down_speed = 0.0
_current_location = LocationGlobalRelative(0.0, 0.0, 0.0)

_target_yaw = 0.0


def connect_vehicle(connection_string):
    global vehicle
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


def init_vehicle():
    set_mode_LATGUIDED()
    _target_yaw = vehicle.attitude.yaw
    return


def set_mode_LATGUIDED():
    global vehicle
    vehicle.mode = VehicleMode("MANUAL")
    time.sleep(1)
    # Vehicle should arm in GUIDED mode
    vehicle.mode = VehicleMode("LATGUIDED")
    time.sleep(1)
    return


def set_mode_LOITER():
    global vehicle
    if _mode == "LOITER":
        return
    vehicle.mode = VehicleMode("LOITER")
    time.sleep(1)
    return


def arm_vehicle():
    global vehicle
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
    global vehicle
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


def update_vehicle_state():
    global vehicle
    global _mode, _pitch, _roll, _yaw, _dist, _north_speed, _east_speed, _down_speed
    global _current_location

    _mode = vehicle.mode.name
    _pitch = vehicle.attitude.pitch
    _roll = vehicle.attitude.roll
    _yaw = vehicle.attitude.yaw
    _dist = vehicle.rangefinder.distance
    _north_speed, _east_speed, _down_speed = vehicle.velocity
    _current_location = vehicle.location.global_relative_frame
    #time.sleep(0.1)
    timer = threading.Timer(UPDATE_TIME, update_vehicle_state)
    timer.update_vehicle_state()


def control_vehicle(yaw_controller, servo_group):
    current_time = time.time()
    throttle_yaw = yaw_controller.compute(_target_yaw, _yaw)
    print("throttle_yaw:%f", throttle_yaw)
    servo_group.set_positions([throttle_yaw, throttle_yaw, throttle_yaw, throttle_yaw])
    # タイマーをセット
    timer = threading.Timer(CONTROL_TIME, control_vehicle, args=(yaw_controller,))
    timer.start()
    print("timer set")


class PIDController:
    def __init__(self, P, I, D):
        self.P = P
        self.I = I
        self.D = D

        self._prev = 0
        self._integral = 0
        self._prev_time = time.time()

    def compute(self, target, current):
        current_time = time.time()
        dt = current_time - self._prev_time

        # PID制御の各項を計算
        error = target - current
        self._integral += error * dt
        derivative = (error - self._prev) / dt

        # 各項を重み付けして合計
        output = self.P * error + self.I * self._integral + self.D * derivative

        # 現在のヨー角と時間を保存
        self._prev = current
        self._prev_time = current_time

        return output

class ServoController:
    def __init__(self, gpio_pin):
        self.pi = pigpio.pi()
        self.gpio_pin = gpio_pin

    def set_servo_position(self, throttle):
        """サーボモータのPWMパルス幅を設定します（1000usから2000usの範囲で）"""
        if throttle > pm.SERVO_MAX:
            throttle = pm.SERVO_MAX
        elif throttle < pm.SERVO_MIN:
            throttle = pm.SERVO_MIN
        pulsewidth = int((throttle + 1) * 500 + 1000)
        self.pi.set_servo_pulsewidth(self.gpio_pin, pulsewidth)

    def stop(self):
        """サーボモータの制御を停止します"""
        self.pi.set_servo_pulsewidth(self.gpio_pin, 0)
        self.pi.stop()


class ServoControllerGroup:
    def __init__(self, pins):
        self.servos = [ServoController(pin) for pin in pins]

    def set_positions(self, throttles):
        for servo, throttle in zip(self.servos, throttles):
            servo.set_servo_position(throttle)

    def stop_all(self):
        for servo in self.servos:
            servo.stop()


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


def stop_vehicle():
    global vehicle
    # 位置を設定
    target_location = LocationGlobalRelative(_current_location.lat, _current_location.lon, _current_location.alt)
    # 新しい位置に移動
    vehicle.simple_goto(target_location)


def control_vehicle(north, east):
    global vehicle
    # 現在の緯度・経度をラジアンに変換
    lat_rad = math.radians(_current_location.lat)
    lon_rad = math.radians(_current_location.lon)
    print(_current_location.lat)
    # 移動距離を緯度・経度に変換
    new_lat = _current_location.lat + (north / 6378137.0) * (180 / math.pi)
    new_lon = _current_location.lon + (east / 6378137.0) * (180 / math.pi) / math.cos(lat_rad)
    print(new_lat)

    # 新しい位置を設定
    target_location = LocationGlobalRelative(new_lat, new_lon, _current_location.alt)

    # 新しい位置に移動
    vehicle.simple_goto(target_location)


def calc_target_dist(center_x, center_y):
    dist = 1    # [m] for test
    x_from_center = center_x - _pixy_center_x
    y_from_center = center_y - _pixy_center_y
    x_dist_max = dist * math.tan(math.radians(_pixy_fov_x / 2))
    y_dist_max = dist * math.tan(math.radians(_pixy_fov_y / 2))
    x_dist = x_dist_max / _pixy_center_x * x_from_center
    y_dist = y_dist_max / _pixy_center_y * y_from_center
    # print(f"x dist {x_dist} y dist {y_dist}")
    return x_dist, y_dist

# calculate x y distance from pixy coordinate to vehicle coordinate
# pixy coordinate : x right, y back
# vehicle coordinate : x front, y right
def dist_pixy2vehicle(x_pixy, y_pixy):
    x_vehicle = - y_pixy
    y_vehicle = x_pixy
    return x_vehicle, y_vehicle


def dist_vehicle2NE(x_vehicle, y_vehicle):
    dist_north = x_vehicle*math.cos(math.radians(_yaw)) - y_vehicle*math.sin(math.radians(_yaw))
    dist_east = x_vehicle*math.sin(math.radians(_yaw)) + y_vehicle*math.cos(math.radians(_yaw))
    return dist_north, dist_east


def cleanup_process(process):
    print("Clean up")
    # サブプロセスの標準出力を閉じる
    process.stdout.close()
    # サブプロセスを終了させる
    process.terminate()
    # サブプロセスが終了するのを待つ
    process.wait()
    print("Subprocess terminated")


def close_vehicle():
    global vehicle
    # disarm
    #disarm_vehicle()
    # Close vehicle object before exiting script
    print("Close vehicle object")
    vehicle.close()
