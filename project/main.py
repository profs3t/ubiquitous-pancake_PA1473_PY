#!/usr/bin/env pybricks-micropython
import sys
#import __init__
import math
import time
has_pallet = False
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
WHITE = (41, 56, 81)
BLACK = (3, 5, 4)
BLUE = (7, 30, 54)
DARKBLUE = (6, 20, 32)
MAGENTA = (17, 12, 60)
YELLOW = (33, 33, 4)
BROWN = (8, 6, 6)
PINK = (36, 14, 20)
GREEN = (10, 12, 6)
LIGHTGREEN = (5, 24, 8)
GREY = (3, 5, 6)


find_way_home = []


def path_find(find_color_list): #Follow a predetermened path
    use_color = colorMix(color_sensor.rgb())
    threshold = (use_color + colorMix(WHITE)) / 2
    while len(find_color_list) > 1:
        print("has_pallet", has_pallet)
        if has_pallet is False:
            misplaced()
        if has_pallet is True:
            if check_pick_up() is False:
                """find_way_home.reversed()
                path_find(find_way_home)"""
                ev3.screen.clear()
                ev3.screen.draw_text(40, 50, 'DROPED')

                time.sleep(3)
        if colorMix(find_color_list[1]) - 5 <= colorMix(color_sensor.rgb()) <= colorMix(find_color_list[1]) + 5:
            find_way_home.append(find_color_list[0])
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
    color = rgbColor[0] + rgbColor[1] + rgbColor[2]
    return color

def find_item(): #  follows line to pallet
    robot.straight(100)
    
    if color_sensor.rgb() == GREY:
        turn = False
        background = GREY
    elif color_sensor.rgb() == BROWN:
        turn = True
        background = BROWN

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

def check_if_elevated(): # checkes whether or not pallet is elevated or not
    has_pallet = True
    lift_motor.run_time(50, 1000, Stop.HOLD, True)
    print('fist check', lift_motor.angle())
    if lift_motor.angle() < 40:
        lift_motor.run_target(200, -10)
        robot.straight(-150)
        pick_up()
    else:
        lift_motor.run_target(200, -10)
        lift_motor.run_target(200, 35, Stop.HOLD, True)

def check_pick_up(): # checks whether or not the pallet is dropped
    if touch_sensor.pressed() == False:
        print('droped pallet')
        time.sleep(1)
        return False
    else:
        return True

def pick_up_item(): #Test so picking up ite works
    lift_motor.run_time(50, 1000, Stop.HOLD, True)
    #lift_motor.run_target(500, 40, Stop.HOLD, True)

    #check_pick_up()

def pick_up(): # pickes up pallet
    lift_motor.run_target(200, -10)
    lift_motor.run_target(200, 35, Stop.HOLD, True)
    
    robot.straight(150)

    #while touch_sensor.pressed() == False:
        #robot.straight(5)
    
    lift_motor.run_target(200, 55, Stop.HOLD, True)
    robot.straight(-200)
    lift_motor.run_target(200 , 10, Stop.HOLD, True)
    #check_pick_up()

def misplaced(): # cehcks for misplaced items and drives around
    if ultra_sensor.distance(silent=False) == 326:
        print("sees pallet")
    elif ultra_sensor.distance(silent=False) < 350:
        robot.stop()
        ev3.screen.clear()
        ev3.screen.draw_text(40, 50, 'misplaced item')
        time.sleep(10)
        ev3.screen.clear()
        
        
        """
        while ultra_sensor.distance(silent=False) < 350:
            robot.turn(-10)
        robot.turn(-30)
        while True:
            robot.drive(DRIVE_SPEED, 5)

        print("exit sväng")
        
        robot.turn(120)
        robot.straight(200) # Same
        robot.turn(-120)
        robot.straight(420) # lenght of pallet plus clearence
        robot.turn(-120)
        robot.straight(200) # Same
        robot.turn(120)"""

def checkDist():
    while True:
        distance = ultra_sensor.distance(silent=False)
        ev3.screen.draw_text(40, 50, distance)
        time.sleep(2)
        ev3.screen.clear()

def checkColor():
    while True:
        color = color_sensor.rgb()
        ev3.screen.draw_text(40, 50, color)
        time.sleep(3)
        ev3.screen.clear()

def main(): # Main method
    #path_find([LIGHTGREEN, GREEN, PINK, BLACK])
    #find_item()
    #ev3.screen.clear()
    #ev3.screen.draw_text(40, 50, 'pick up')
    #time.sleep(15)
    check_if_elevated()
    path_find([BLACK, PINK, GREEN])
    #lift_motor.run_target(200, -10)
    #checkColor()

def main2():
    checkDist()

if __name__ == '__main__':
    sys.exit(main())
