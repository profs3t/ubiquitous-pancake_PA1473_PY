#!/usr/bin/env pybricks-micropython
import sys
#import __init__
import time
from pybricks import robotics
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.robotics import DriveBase

from os import system


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

# Buttons
rightButton = Button.RIGHT
leftButton = Button.LEFT

# Constant Values
DRIVE_SPEED = 20
TURN_RADIUS = 10
PROPORTIONAL_GAIN = 0.8

# RGB Color Values
WHITE = (93, 80, 100)
BLACK = (7, 7, 15)
BLUE = (8, 35, 66)
DARKBLUE = (6, 20, 32)
MAGENTA = (17, 12, 60)
YELLOW = (71, 47, 21)
BROWN = (19, 9, 22)
PINK = (96, 22, 54)
GREEN = (22, 16, 21)
LIGHTGREEN = (5, 24, 8)
GREY = (4, 6, 7)

pigVariation = 2
find_way_home = [GREEN, BLUE, BLACK]
dropped_way = [GREEN, MAGENTA, BLACK]
BlueWare = [GREEN, BLUE, BLACK]
RedWare = [GREEN, PINK, BLACK]

hasPallet = False
background = None

def path_find(find_color_list): #Follow a predetermened path
    global hasPallet
    #find_color_list[0] = colorMix(color_sensor.rgb())
    threshold = (colorMix(find_color_list[0]) + colorMix(WHITE)) / 2
    while len(find_color_list) > 1:
        #print("hasPallet", hasPallet)
        if (find_color_list[1][0] - pigVariation <= color_sensor.rgb()[0] <= find_color_list[1][0] + pigVariation and
            find_color_list[1][1] - pigVariation <= color_sensor.rgb()[1] <= find_color_list[1][1] + pigVariation and
            find_color_list[1][2] - pigVariation <= color_sensor.rgb()[2] <= find_color_list[1][2] + pigVariation):
            # checks if the color seen by sensor is next color to follow
            
            #print(f'Found {find_color_list[1]}')
            print("Found", find_color_list[1])
            time.sleep(0.1)

            find_way_home.append(find_color_list[0])
            find_color_list.pop(0)
            path_find(find_color_list)

        if hasPallet is False: # checks for missplaced items
            misplaced()

        if hasPallet is True:
            if check_pick_up() is False: # checks if item has been dropped
                robot.turn(180)
                find_way_home.reverse()
                path_find(find_way_home)

        
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
    global background
    robot.straight(150) # drives forward to see background color of warehouse
    robot.turn(-10)
    print('Checks the background color')
    time.sleep(0.1)
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

    print("Background is", background)
    time.sleep(0.1)

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
    global hasPallet
    hasPallet = not hasPallet
    print('Checks if item is elevated')
    lift_motor.run_time(50, 1500, Stop.HOLD, True)
    print('Angle:', lift_motor.angle())
    if lift_motor.angle() < 40:
        print('Item is elevated')
        time.sleep(0.1)
        lift_motor.run_target(200, -10)
        robot.straight(-150)
        lift_motor.run_target(200, 35, Stop.HOLD, True)
        robot.straight(150)
        lift_motor.run_target(200, 55, Stop.HOLD, True)
        robot.straight(-200)
        lift_motor.run_target(200 , 55, Stop.HOLD, True)
        leave_area()
    else:
        print('Picks up item')
        time.sleep(0.1)
        lift_motor.run_target(200, -10)
        lift_motor.run_target(200, 55, Stop.HOLD, True)
        leave_area()

def check_pick_up(): # checks whether or not the pallet is dropped
    global hasPallet
    if touch_sensor.pressed() == False:
        #hasPallet = True
        hasPallet = not hasPallet
        print('Robot dropped the pallet')
        time.sleep(1)
        return False
    else:
        return True

def leave_area2():
    global background
    robot.turn(360)
    """while not (WHITE[0] - pigVariation <= color_sensor.rgb()[0] <= WHITE[0] + pigVariation and
            WHITE[1] - pigVariation <= color_sensor.rgb()[1] <= WHITE[1] + pigVariation and
            WHITE[2] - pigVariation <= color_sensor.rgb()[2] <= WHITE[2] + pigVariation):"""
    while not color_sensor.rgb() == colorMix(WHITE):
        robot.straight(2)
    if background == GREY:
        while not (BLUE[0] - pigVariation <= color_sensor.rgb()[0] <= BLUE[0] + pigVariation and
                BLUE[1] - pigVariation <= color_sensor.rgb()[1] <= BLUE[1] + pigVariation and
                BLUE[2] - pigVariation <= color_sensor.rgb()[2] <= BLUE[2] + pigVariation):
            robot.drive(DRIVE_SPEED, -10)
    elif background == BROWN:
        """while not (PINK[0] - pigVariation <= color_sensor.rgb()[0] <= PINK[0] + pigVariation and
                PINK[1] - pigVariation <= color_sensor.rgb()[1] <= PINK[1] + pigVariation and
                PINK[2] - pigVariation <= color_sensor.rgb()[2] <= PINK[2] + pigVariation):"""
        while not color_sensor == colorMix(PINK):
            robot.drive(DRIVE_SPEED, -10)

    print("The robot has left the ", background," area")
    time.sleep(0.1)

def leave_area():
    global background
    print("leave area")
    time.sleep(0.1)
    robot.turn(-250)
    if background == GREY:
        path_find([GREY, BLACK, BLUE])
    elif background == BROWN:
        path_find([BROWN, BLACK, PINK])

    print("The robot has left the ", background," area")
    time.sleep(0.1)
    

def misplaced(): # checks for misplaced items
    #if ultra_sensor.distance(silent=False) == 326:
        #print('Has pallet')
    if ultra_sensor.distance(silent=False) < 100:
        robot.stop()
        print('Misplaced item')
        time.sleep(0.1)


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
        print(color)
        time.sleep(2)
        ev3.screen.clear()

def change_pickup_color():
    while True:
        robot.stop()
        pickColor = None
        commit = False
        print("Where do you wanna go?")
        print("Left Red | Right Blue")
        while commit is False:
            time.sleep(0.1)
            if rightButton in ev3.buttons.pressed():
                commit = True
                print("Blue warehouse")
                pickColor = "blue"
                time.sleep(0.1)
                path_find(BlueWare)
            elif leftButton in ev3.buttons.pressed():
                commit = True
                print("RED warehouse")
                pickColor = "red"
                time.sleep(0.1)
                path_find(RedWare)

        commit = False
        robot.stop()
        print("Do you wanna pick up", pickColor)
        print("Left No | Right Yes")
        while commit is False:
            time.sleep(0.1)
            if rightButton in ev3.buttons.pressed():
                commit = True
                print("Pick up")
                time.sleep(0.1)
                find_item()
            elif leftButton in ev3.buttons.pressed():
                commit = True
                print("Return")
                time.sleep(0.1)
                robot.turn(150)
                if pickColor == "blue":
                    path_find([BLUE, GREEN])
                elif pickColor == "red":
                    path_find([PINK, GREEN])


def main(): # Main method
    #path_find([GREEN, BLUE, BLACK])
    #find_item()
    #time.sleep(10)
    #path_find([BLACK, BLUE, GREEN])
    change_pickup_color()

def main2():
    checkColor()
    return 0

if __name__ == '__main__':
    sys.exit(main())
