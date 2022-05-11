#!/usr/bin/env pybricks-micropython
import sys
#import __init__
import time
from pybricks import robotics
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.robotics import DriveBase

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
dropped_way = [GREEN, MAGENTA, BLACK]
BlueWare = [GREEN, BLUE, BLACK]
RedWare = [GREEN, PINK, BLACK]

hasPallet = False
background = None

def path_find(find_color_list): #Follow a predetermened path
    global hasPallet
    use_color = colorMix(color_sensor.rgb())
    threshold = (use_color + colorMix(WHITE)) / 2
    while len(find_color_list) > 1:
        print("hasPallet", hasPallet)
        if hasPallet is False: # checks for missplaced items
            misplaced()
        if hasPallet is True:
            if check_pick_up() is False: # checks if item has been dropped
                robot.turn(180)
                #find_way_home.reversed()
                path_find(dropped_way)

        #if colorMix(find_color_list[1]) - 5 <= colorMix(color_sensor.rgb()) <= colorMix(find_color_list[1]) + 5: # old, use ICE

        if (find_color_list[1][0] - pigVariation <= color_sensor.rgb()[0] <= find_color_list[1][0] + pigVariation and
            find_color_list[1][1] - pigVariation <= color_sensor.rgb()[1] <= find_color_list[1][1] + pigVariation and
            find_color_list[1][2] - pigVariation <= color_sensor.rgb()[2] <= find_color_list[1][2] + pigVariation):
            # checks if the color seen by sensor is next color to follow
            
            print(f'Found {find_color_list[1]}')

            #find_way_home.append(find_color_list[0])
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
    global background
    robot.straight(100) # drives forward to see background color of warehouse
    print('Checks the background color')
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

    print(f'Background is {background}')

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
    global hasPallet
    #hasPallet = False
    hasPallet = not hasPallet
    print('Checks if item is elevated')
    lift_motor.run_time(50, 1000, Stop.HOLD, True)
    print('Angle:', lift_motor.angle())
    if lift_motor.angle() < 40:
        print('Item is elevated')
        lift_motor.run_target(200, -10)
        robot.straight(-150)
        lift_motor.run_target(200, 35, Stop.HOLD, True)
        robot.straight(150)
        lift_motor.run_target(200, 55, Stop.HOLD, True)
        robot.straight(-200)
        lift_motor.run_target(200 , 10, Stop.HOLD, True)
    else:
        print('Picks up item')
        lift_motor.run_target(200, -10)
        lift_motor.run_target(200, 35, Stop.HOLD, True)

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

def leave_area():
    global background
    robot.turn(180)
    while not (WHITE[0] - pigVariation <= color_sensor.rgb()[0] <= WHITE[0] + pigVariation and
            WHITE[1] - pigVariation <= color_sensor.rgb()[1] <= WHITE[1] + pigVariation and
            WHITE[2] - pigVariation <= color_sensor.rgb()[2] <= WHITE[2] + pigVariation):
        robot.straight(2)
    if background == GREY:
        while not (BLUE[0] - pigVariation <= color_sensor.rgb()[0] <= BLUE[0] + pigVariation and
                BLUE[1] - pigVariation <= color_sensor.rgb()[1] <= BLUE[1] + pigVariation and
                BLUE[2] - pigVariation <= color_sensor.rgb()[2] <= BLUE[2] + pigVariation):
            robot.drive(DRIVE_SPEED, -10)
    elif background == BROWN:
        while not (PINK[0] - pigVariation <= color_sensor.rgb()[0] <= PINK[0] + pigVariation and
                PINK[1] - pigVariation <= color_sensor.rgb()[1] <= PINK[1] + pigVariation and
                PINK[2] - pigVariation <= color_sensor.rgb()[2] <= PINK[2] + pigVariation):
            robot.drive(DRIVE_SPEED, 10)

    print(f'The robot has left the {background} area')

def misplaced(): # checks for misplaced items
    if ultra_sensor.distance(silent=False) == 326:
        print('Has pallet')
    elif ultra_sensor.distance(silent=False) < 10:
        robot.stop()
        print('Misplaced item')

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

def change_pickup_color():
    while True:
        pickColor = None
        commit = False
        print("Where do you wanna go?")
        while commit is False:
            answer = input("Pick up [Blue], [Red]: ")
            if answer.capitalize() == "BLUE":
                commit = True
                pickColor = "blue"
                sys("cls")
                path_find(BlueWare)
            elif answer.capitalize() == "RED":
                commit = True
                pickColor = "red"
                sys("cls")
                path_find(RedWare)
            else:
                print("Invalid answer, try again")

        sys("cls")
        commit = False
        print("Do you wanna pick up", pickColor)
        while commit is False:
            answer = input("YES or NO: ")
            if answer.capitalize() == "YES":
                sys("cls")
                commit = True
                find_item()
            elif answer.capitalize() == "NO":
                sys("cls")
                commit = True
                robot.turn(180)
                if pickColor == "blue":
                    path_find([BLUE, GREEN])
                elif pickColor == "red":
                    path_find([PINK, GREEN])
            else:
                print("Invalid answer, try again")

def main(): # Main method
    #path_find([GREEN, BLUE, BLACK])
    find_item()
    time.sleep(10)
    path_find([BLACK, BLUE, GREEN])

def main2():
    checkColor()

if __name__ == '__main__':
    sys.exit(main())
