#!/usr/bin/env pybricks-micropython
import sys
import __init__

from pybricks import robotics
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.robotics import DriveBase

# Robot def
ev3 = EV3Brick()

# Motor def
left_motor = Motor(Port.C)
right_motor = Motor(Port.B)
lift_motor = Motor(Port.A)
robot = DriveBase(left_motor, right_motor, wheel_diameter = 47, axle_track = 128)

# Sensor def
light_sensor = ColorSensor(Port.S3)
ultra_sensor = UltrasonicSensor(Port.S4)
touch_sensor = TouchSensor(Port.S1)

# Constant Values
DRIVE_SPEED = 60
TURN_RADIUS = 10
robot.settings(DRIVE_SPEED, 100, 50)



def main(): # Main method
    return 0

if __name__ == '__main__':
    sys.exit(main())