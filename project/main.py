#!/usr/bin/env pybricks-micropython
import sys
#import __init__
import time
from pybricks import robotics
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.robotics import DriveBase
#from ev3dev2.sensor.lego import ColorSensot

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
PROPORTIONAL_GAIN = 0.8

# RGB Color Values
WHITE = (31, 39, 68)
BLACK = (3, 5, 4)
BLUE = (8, 35, 66)
DARKBLUE = (6, 20, 32)
MAGENTA = (17, 12, 60)
YELLOW = (35, 37, 9)
BROWN = (8, 7, 7)
PINK = (38, 17, 24)
GREEN = (11, 15, 9)
LIGHTGREEN = (5, 24, 8)
GREY = (4, 6, 7)


pigVariation = 2
find_way_home = [GREEN, BLUE, BLACK]

hasPallet = False

#color_sensor.calibrate_white()

def path_find(find_color_list): #Follow a predetermened path
    use_color = colorMix(color_sensor.rgb())
    threshold = (use_color + colorMix(WHITE)) / 2
    while len(find_color_list) > 1:
        print("hasPallet", hasPallet)
        if hasPallet is False: # checks for missplaced items
            misplaced()
        if hasPallet is True:
            if check_pick_up() is False: # checks if item has been dropped
                ev3.screen.clear()
                ev3.screen.draw_text(40, 50, 'DROPED PALLET')
                time.sleep(3)
                ev3.screen.clear()
                robot.turn(180)
                find_way_home.reversed()
                path_find(find_way_home)
 
        #if colorMix(find_color_list[1]) - 5 <= colorMix(color_sensor.rgb()) <= colorMix(find_color_list[1]) + 5: # old, use ICE
            
        if (find_color_list[1][0] - pigVariation <= color_sensor.rgb()[0] <= find_color_list[1][0] + pigVariation and
            find_color_list[1][1] - pigVariation <= color_sensor.rgb()[1] <= find_color_list[1][1] + pigVariation and
            find_color_list[1][2] - pigVariation <= color_sensor.rgb()[2] <= find_color_list[1][2] + pigVariation):
            # checks if the color seen by sensor is next color to follow

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

def colorMix2(rgbColor):
    return rgbColor[0] + rgbColor[1] + rgbColor[2]

def find_item(): #  follows line to pallet
    robot.straight(100) # drives forward to see background color of warehouse
    
    if (GREY[0] - pigVariation <= color_sensor.rgb()[0] <= GREY[0] + pigVariation and
        GREY[1] - pigVariation <= color_sensor.rgb()[1] <= GREY[1] + pigVariation and
        GREY[2] - pigVariation <= color_sensor.rgb()[2] <= GREY[2] + pigVariation):
        # checks if background is grey or brown in warehouse
        turn = False
        background = GREY
    elif (BROWN[0] - pigVariation <= color_sensor.rgb()[0] <= BROWN[0] + pigVariation and
        BROWN[1] - pigVariation <= color_sensor.rgb()[1] <= BROWN[1] + pigVariation and
        BROWN[2] - pigVariation <= color_sensor.rgb()[2] <= BROWN[2] + pigVariation):
        # checks if background is grey or brown in warehouse
        turn = True
        background = BROWN


    # Old function used as backup
    """
    if color_sensor.rgb() == GREY:
        turn = False
        background = GREY
    elif color_sensor.rgb() == BROWN:
        turn = True
        background = BROWN
    """    

    use_color = YELLOW
    threshold = (colorMix(use_color) + colorMix(background)) / 2
    while not touch_sensor.pressed():
        # Calculate the deviation from the threshold.
        if turn:
            deviation = threshold - colorMix(color_sensor.rgb())
        elif not turn:
            deviation = colorMix(color_sensor.rgb()) - threshold

        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation

        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)

    check_if_elevated()

def check_if_elevated(): # checkes whether or not pallet is elevated or not
    hasPallet = False
    hasPallet = not hasPallet
    lift_motor.run_time(50, 1000, Stop.HOLD, True)
    print('fist check', lift_motor.angle())
    if lift_motor.angle() < 40:
        lift_motor.run_target(200, -10)
        robot.straight(-150)
        lift_motor.run_target(200, 35, Stop.HOLD, True)
        robot.straight(150)
        lift_motor.run_target(200, 55, Stop.HOLD, True)
        robot.straight(-200)
        lift_motor.run_target(200 , 10, Stop.HOLD, True)
    else:
        lift_motor.run_target(200, -10)
        lift_motor.run_target(200, 35, Stop.HOLD, True)

def check_pick_up(): # checks whether or not the pallet is dropped
    if touch_sensor.pressed() == False:
        hasPallet = True
        hasPallet = not hasPallet
        print('DROPED PALLET')
        time.sleep(1)
        return False
    else:
        return True

def pick_up_item(): #Test so picking up ite works
    lift_motor.run_time(50, 1000, Stop.HOLD, True)
    #lift_motor.run_target(500, 40, Stop.HOLD, True)

def misplaced(): # checks for misplaced items
    if ultra_sensor.distance(silent=False) == 326:
        print("has pallet")
    elif ultra_sensor.distance(silent=False) < 10:
        robot.stop()
        ev3.screen.clear()
        ev3.screen.draw_text(40, 50, 'misplaced item')
        time.sleep(5)
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

def checkDist(): # test functions
    while True:
        distance = ultra_sensor.distance(silent=False)
        ev3.screen.draw_text(40, 50, distance)
        time.sleep(2)
        ev3.screen.clear()

def checkColor(): # test functions
    while True:
        color = color_sensor.rgb()
        ev3.screen.draw_text(40, 50, color)
        time.sleep(2)
        ev3.screen.clear()

def main(): # Main method
    #path_find([GREEN, BLUE, BLACK])
    find_item()
    time.sleep(10)
    path_find([BLACK, BLUE, GREEN])


def main2():
    checkColor()

if __name__ == '__main__':
    sys.exit(main())
