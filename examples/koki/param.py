#!/usr/bin/env python
# -*- coding: utf-8 -*-

#timer
UPDATE_TIME = 0.1
CONTROL_TIME = 0.1

# pixycam parameter
_pixy_max_x = 315
_pixy_max_y = 207
_pixy_center_x = 158
_pixy_center_y = 104
_pixy_fov_x = 60    # [degree] field of view
_pixy_fov_y = _pixy_fov_x/(_pixy_max_x+1)*(_pixy_max_y+1)

# Servo parameters
PIN_S1 = 19
PIN_S2 = 26
PIN_S3 = 13
PIN_S4 = 6
SERVO_PINS = [PIN_S1, PIN_S2, PIN_S3, PIN_S4]
SERVO_MAX = 0.2
SERVO_MIN = -0.2

# PID parameters
YAW_P = 1.0
YAW_I = 0.0
YAW_D = 0.0
