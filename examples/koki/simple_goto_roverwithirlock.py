#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
simple_goto.py: GUIDED mode "simple goto" example (Copter Only)

Demonstrates how to arm and takeoff in Copter and how to navigate to points using Vehicle.simple_goto.

Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html
"""

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import math
import subprocess
import select
import os
import sys
import argparse

"""
global _mode
global _pitch
global _roll
global _yaw
global _dist
global _north_speed
global _east_speed
global _down_speed
global _current_location
"""
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

# pixycam parameter
_pixy_max_x = 315
_pixy_max_y = 207
_pixy_center_x = 158
_pixy_center_y = 104
_pixy_fov_x = 60    # [degree] field of view
_pixy_fov_y = _pixy_fov_x/(_pixy_max_x+1)*(_pixy_max_y+1)

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
    # arm
    arm_vehicle()
    # C++プログラムをサブプロセスとして実行
    pixy_path = os.path.expanduser("~/pixy/build/hello_pixy/hello_pixy")
    process = subprocess.Popen(['sudo', pixy_path], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    current_frame = None
    next_frame = None
    max_area = 0  # 最大面積を初期化
    center_x, center_y = None, None  # 中心座標を初期化

    while True:
        # update vehicle state
        update_vehicle_state()
        #print("yaw", math.degrees(_yaw), "deg")
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
                print(f"Frame {current_frame} target center: ({center_x}, {center_y})")
                # run with irlock
                # calculate distance with pixycam coordinate
                dist_x_pixy, dist_y_pixy = calc_target_dist(center_x, center_y)
                # calculate distance with vehicle coordinate from pixycam corrdinate
                dist_x_vehicle, dist_y_vehicle = dist_pixy2vehicle(dist_x_pixy, dist_y_pixy)
                # calculate distance with NE corrdinate from vehicle coordinate
                dist_north, dist_east = dist_vehicle2NE(dist_x_vehicle, dist_y_vehicle)
                print(f"dist north {dist_north} dist east {dist_east}")
                #control_vehicle(dist_north, dist_east)
                control_vehicle(0.1, 0)
                break

                max_area = 0
                center_x, center_y = None, None
            current_frame = next_frame
            # ここに出力がある場合の追加の動作を記述
        else:
            # 出力がない場合の処理
            # ここに出力がない場合の追加の動作を記述
            #print("No output detected")
            #stop_vehicle()
            pass
    print("roop break")
    cleanup()


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


def set_mode_LOITER():
    if _mode == "LOITER":
        return
    vehicle.mode = VehicleMode("LOITER")
    time.sleep(1)
    return


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

def update_vehicle_state():
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
    # 位置を設定
    target_location = LocationGlobalRelative(_current_location.lat, _current_location.lon, _current_location.alt)
    # 新しい位置に移動
    vehicle.simple_goto(target_location)


def control_vehicle(north, east):
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


def cleanup():
    print("Clean up")
    global process
    # サブプロセスの標準出力を閉じる
    process.stdout.close()
    # サブプロセスを終了させる
    process.terminate()
    # サブプロセスが終了するのを待つ
    process.wait()
    print("Subprocess terminated")
    # disarm
    #disarm_vehicle()
    # Close vehicle object before exiting script
    print("Close vehicle object")
    vehicle.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print('Exiting due to keyboard interrupt...')
    finally:
        cleanup()
