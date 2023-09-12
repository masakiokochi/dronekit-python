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
import libraries as lib
import param as pm


process = None


# Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect

if not connection_string:
    print("Please set connection_string")
    sys.exit(1) # finish this program

def main():
    global process

    # Connect to the Vehicle
    vehicle = lib.connect_vehicle(connection_string)  # vehicle変数に代入
    # prearm check
    lib.prearm_check()
    # mode change to LATGUIDED
    lib.init_vehicle()

    yaw_controller = lib.PIDController(pm.YAW_P, pm.YAW_I, pm.YAW_D)
    servo_group = lib.ServoControllerGroup(SERVO_PINS)

    # arm
    lib.arm_vehicle()
    # C++プログラムをサブプロセスとして実行
    #pixy_path = os.path.expanduser("~/pixy/build/hello_pixy/hello_pixy")
    #process = subprocess.Popen(['sudo', pixy_path], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    # update vehicle state
    lib.update_vehicle_state()
    lib.control_vehicle(yaw_controller, servo_group)



    current_frame = None
    next_frame = None
    max_area = 0  # 最大面積を初期化
    center_x, center_y = None, None  # 中心座標を初期化

    while True:

        '''
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
        '''
    print("roop break")
    #lib.cleanup(process)
    lib.close_vehicle()




if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print('Exiting due to keyboard interrupt...')
    finally:
        lib.cleanup(process)
        lib.vehicle_close()
