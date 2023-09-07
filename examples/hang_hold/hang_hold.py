#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
dronekit HangHold demo
"""

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative


# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    required=True,  # この引数を必須にする
                    help="Vehicle connection target string. e.g. --connect udp:127.0.0.1:14550, --connect /dev/ttyACM0")
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)



print("Set default/target groundspeed to 3")
vehicle.groundspeed = 3

"""
Arms vehicle
"""

print("Basic pre-arm checks")
# Don't try to arm until autopilot is ready
while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)

print("Arming motors")
# Vehicle should arm in GUIDED mode
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

# Confirm vehicle armed before attempting to take off
while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)

print("Going towards first point for 30 seconds ...")
point1 = LocationGlobalRelative(-35.361354, 149.165218, 20)
vehicle.simple_goto(point1)

# sleep so we can see the change in map
time.sleep(30)

print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
point2 = LocationGlobalRelative(-35.363244, 149.168801, 20)
vehicle.simple_goto(point2, groundspeed=10)

# sleep so we can see the change in map
time.sleep(30)

print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()
