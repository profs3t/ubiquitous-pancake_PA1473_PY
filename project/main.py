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
color_sensor = ColorSensor(Port.S3)
ultra_sensor = UltrasonicSensor(Port.S4)
touch_sensor = TouchSensor(Port.S1)

# Constant Values
DRIVE_SPEED = 60
TURN_RADIUS = 10
robot.settings(DRIVE_SPEED, 100, 50)
PROPORTIONAL_GAIN = 1.2

# Color Values
WHITE = 85
BLACK = 9
#RED = 
#BLUE = 
#YELLOW = 
#BROWN = 
#PINK = 
#GREEN = 

find_color_list = [BLACK, WHITE, GREEN]

def path_find(find_color_list): #Follow a predetermened path
    use_color = color_sensor.reflection()
    threshold = (use_color + WHITE) / 2
    while len(find_color_list) > 1:
        if color_sensor.reflection() == find_color_list[0]:
            find_color_list.pop(0)
            path_find(find_color_list)
        # Calculate the deviation from the threshold.
        deviation = color_sensor.reflection() - threshold

        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation

        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)


def find_item():
    distance = ultra_sensor.distance(silent=False)
    


def pick_up_item(): #Picks up an item
    


def main(): # Main method
    return 0


if __name__ == '__main__':
    sys.exit(main())

