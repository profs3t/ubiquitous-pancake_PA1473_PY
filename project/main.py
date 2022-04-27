#!/usr/bin/env pybricks-micropython
import sys
#import __init__
import math
import time

from pybricks import robotics
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import ImageFile

# Robot def
ev3 = EV3Brick()

# Motor def
left_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE, gears = [12, 20])
right_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE, gears = [12, 20])
lift_motor = Motor(Port.A, Direction.CLOCKWISE, gears = [12, 36])

robot = DriveBase(left_motor, right_motor, wheel_diameter = 47, axle_track = 128)

# Sensor def
color_sensor = ColorSensor(Port.S3)
ultra_sensor = UltrasonicSensor(Port.S4)
touch_sensor = TouchSensor(Port.S1)

# Constant Values
DRIVE_SPEED = 20
TURN_RADIUS = 10
#robot.settings(DRIVE_SPEED, 100, 50)
PROPORTIONAL_GAIN = 0.8

robotLen = 150 # MÄT!

# Color Values
WHITE = (95, 79, 100)
BLACK = (8, 7, 14)
BLUE = (14, 26, 70)
MAGENTA = (17, 12, 60)
YELLOW = (80, 51, 19)
BROWN = (18, 8, 17)
PINK = (73, 31, 54)
GREEN = (22, 17, 18)
LIGHTGREEN = (14, 35, 31)
GREY = (9, 7, 18)

#find_color_list = [BLACK, WHITE]

def path_find(find_color_list): #Follow a predetermened path
    use_color = colorMix(color_sensor.rgb())
    threshold = (use_color + colorMix(WHITE)) / 2
    while len(find_color_list) > 1:
        if color_sensor.rgb() == find_color_list[1]:
            find_color_list.pop(0)
            path_find(find_color_list)
        # Calculate the deviation from the threshold.
        deviation = threshold - colorMix(color_sensor.rgb())

        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation

        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)

def colorMix(rgbColor):
    color = 0
    #for i in rgbColor:
        #color += rgbColor[i]
    color = rgbColor[0] + rgbColor[1] + rgbColor[2]
    return color

def find_item():
    robot.straight(100)
    if color_sensor.rgb() == BROWN:
        turn = True
        background = BROWN
    elif color_sensor.rgb() == GREY:
        turn = False
        background = GREY

    use_color = YELLOW
    threshold = (colorMix(use_color) + colorMix(background)) / 2
    while touch_sensor.pressed() == False:
        # Calculate the deviation from the threshold.
        if turn == True:
            deviation = threshold - colorMix(color_sensor.rgb())
        elif turn == False:
            deviation = colorMix(color_sensor.rgb()) - threshold

        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation

        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)

    check_if_elevated()

def check_if_elevated():
    lift_motor.run_time(50, 1000, Stop.HOLD, True)
    print('fist check', lift_motor.angle())
    if lift_motor.angle() < 40:
        lift_motor.run_target(200, -10)
        robot.straight(-150)
        pick_up()

def check_pick_up():
    if touch_sensor.pressed() == False:
        #ev3.screen.load_image('knocked_out.png')
        print('droped pallet')
        time.sleep(1)

def pick_up_item(): #Picks up an item
    lift_motor.run_time(50, 1000, Stop.HOLD, True)
    #lift_motor.run_target(500, 40, Stop.HOLD, True)

    #check_pick_up()

def pick_up():
    print('pick_up')
    lift_motor.run_target(200, -10)
    lift_motor.run_target(200, 35, Stop.HOLD, True)
    
    robot.straight(150)

    #while touch_sensor.pressed() == False:
        #robot.straight(5)
    
    lift_motor.run_target(200, 55, Stop.HOLD, True)
    robot.straight(-200)
    check_pick_up()

def main(): # Main method
    find_item()
    time.sleep(5)
    lift_motor.run_target(200, -10)
    return 0

if __name__ == '__main__':
    sys.exit(main())
