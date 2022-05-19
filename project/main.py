#!/usr/bin/env pybricks-micropython

import sys
import time
from pybricks import robotics
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port, Stop, Direction, Button
from pybricks.robotics import DriveBase

# ev3-brick def
ev3 = EV3Brick()

# Motor def
left_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE, gears = [12, 20])
right_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE, gears = [12, 20])
lift_motor = Motor(Port.A, Direction.CLOCKWISE, gears = [12, 36])

# Drivebase def
robot = DriveBase(left_motor, right_motor, wheel_diameter = 47, axle_track = 128)

# Sensor def
color_sensor = ColorSensor(Port.S3)
ultra_sensor = UltrasonicSensor(Port.S4)
touch_sensor = TouchSensor(Port.S1)

# Button def
rightButton = Button.RIGHT
leftButton = Button.LEFT
upButton = Button.UP
downButton = Button.DOWN

# Constant Values
DRIVE_SPEED = 30
TURN_RADIUS = 10
PROPORTIONAL_GAIN = 0.8
pigVariation = 4 # Changes how much variation in the RGB values the robot will accept

# Color RGB Values
WHITE = (69, 88, 100)
BLACK = (5, 7, 9)
BLUE = (13, 48, 100)
MAGENTA = (12, 12, 44)
YELLOW = (52, 51, 12)
BROWN = (13, 9, 13)
PINK = (58, 24, 43)
GREEN = (17, 20, 15)
LIGHTGREEN = (11, 55, 24)
GREY = (6, 8, 15)

# Possible ways
BlueWare = [GREEN, BLUE, BLACK]
RedWare = [GREEN, PINK, BLACK]
WareHouse = [GREEN, MAGENTA, BLACK]
Deliver = [GREEN, LIGHTGREEN, BLACK]

# Global variables
hasPallet = False # Determines if the robot currently has a pallet picked up
background = None # The background of the current warehouse
pickColor = None # Determines which general area the robot is in or going to

def path_find(find_color_list, turn_bool):
    '''Follows a predetermened path.
    Input: Takes a list of colors RGB values as a tuple with three ints, to follow and a bool to determine
    which side of the line to follow'''
    global hasPallet
    threshold = (colorMix(find_color_list[0]) + colorMix(WHITE)) / 2
    while len(find_color_list) > 1:
        if (find_color_list[1][0] - pigVariation <= color_sensor.rgb()[0] <= find_color_list[1][0] + pigVariation and
            find_color_list[1][1] - pigVariation <= color_sensor.rgb()[1] <= find_color_list[1][1] + pigVariation and
            find_color_list[1][2] - pigVariation <= color_sensor.rgb()[2] <= find_color_list[1][2] + pigVariation):
            # ^ Checks if the color seen by sensor is next color to follow ^
            
            print("Found", find_color_list[1])
            time.sleep(0.1)

            find_color_list.pop(0)
            path_find(find_color_list, turn_bool)

        misplaced() # Checks for missplaced items or robots

        if hasPallet is True:
            if check_pick_up() is False: # Checks if item has been dropped
                returnHome()

        # Calculate the deviation from the threshold
        if turn_bool:
            deviation = threshold - colorMix(color_sensor.rgb())
        elif not turn_bool:
            deviation = colorMix(color_sensor.rgb()) - threshold
        # Calculate the turn rate
        turn_rate = PROPORTIONAL_GAIN * deviation

        # Set the drive base speed and turn rate
        robot.drive(DRIVE_SPEED, turn_rate)

def colorMix(rgbColor):
    '''Input: Takes a tuple of three ints and adds them together
    Returns: Int'''
    color = 0
    color = rgbColor[0] + rgbColor[1] + rgbColor[2]
    return color

def returnHome():
    '''When called: Send robot back to warhouse from where it is'''
    global pickColor
    if pickColor == "red":
        path_find([PINK, GREEN, MAGENTA, BLACK], True)
    elif pickColor == "blue":
        path_find([BLUE, GREEN, MAGENTA, BLACK], True)
    elif pickColor == "deliver":
        path_find([LIGHTGREEN, GREEN, MAGENTA, BLACK], True)
    else:
        return 0

def find_item():
    '''When called: Robot will check what warehous it's at and find an item too pick up'''
    global background
    lift_motor.run_target(200, 0)
    robot.straight(150) # Drives forward to see background color of warehouse
    robot.turn(-10)
    print("Checks the background color")
    time.sleep(0.1)
    if (GREY[0] - pigVariation <= color_sensor.rgb()[0] <= GREY[0] + pigVariation and
        GREY[1] - pigVariation <= color_sensor.rgb()[1] <= GREY[1] + pigVariation and
        GREY[2] - pigVariation <= color_sensor.rgb()[2] <= GREY[2] + pigVariation):
        # ^ Checks if background is grey or brown in warehouse
        turn = False
        background = GREY
    elif (BROWN[0] - pigVariation <= color_sensor.rgb()[0] <= BROWN[0] + pigVariation and
        BROWN[1] - pigVariation <= color_sensor.rgb()[1] <= BROWN[1] + pigVariation and
        BROWN[2] - pigVariation <= color_sensor.rgb()[2] <= BROWN[2] + pigVariation):
        # ^ Checks if background is grey or brown in warehouse
        turn = True
        background = BROWN

    print("Background is", background)
    time.sleep(0.1)

    # Pathfinding on yellow line with grey or brown background
    use_color = YELLOW
    threshold = (colorMix(use_color) + colorMix(background)) / 2
    while not touch_sensor.pressed():
        # Calculate the deviation from the threshold
        if turn:
            deviation = threshold - colorMix(color_sensor.rgb())
        elif not turn:
            deviation = colorMix(color_sensor.rgb()) - threshold

        # Calculate the turn rate
        turn_rate = PROPORTIONAL_GAIN * deviation

        # Set the drive base speed and turn rate
        robot.drive(DRIVE_SPEED, turn_rate)
    check_if_elevated()

def check_if_elevated():
    '''When called: Robot will try to pick up an item. If two items are stacked the robot will pick up the elevated one'''
    global hasPallet
    hasPallet = not hasPallet
    print("Checks if item is elevated")
    lift_motor.run_time(50, 1500, Stop.HOLD, True)
    print("Angle:", lift_motor.angle())
    if lift_motor.angle() < 40:
        print("Item is elevated")
        time.sleep(0.1)
        lift_motor.run_target(200, -10)
        robot.straight(-150)
        lift_motor.run_target(200, 35, Stop.HOLD, True)
        robot.straight(150)
        lift_motor.run_target(200, 55, Stop.HOLD, True)
        robot.straight(-200)
        lift_motor.run_target(200 , 25, Stop.HOLD, True)
        leave_area()
    else:
        print("Picks up item")
        time.sleep(0.1)
        lift_motor.run_target(200, 25, Stop.HOLD, True)
        leave_area()

def check_pick_up():
    '''When called: The robot will check if the item is still picked up.
    Return: Bool. True if pallet still there, False if pallet is dropped'''
    global hasPallet
    if touch_sensor.pressed() == False:
        hasPallet = not hasPallet
        print("Robot dropped the pallet")
        time.sleep(1)
        return False
    else:
        return True

def leave_area():
    '''When called: The robot leaves the warehouse'''
    global background
    print("Leaving area")
    time.sleep(0.1)
    if background == GREY:
        robot.turn(180)
        robot.straight(200)
        path_find([GREY, BLACK, BLUE], False)
    elif background == BROWN:
        robot.turn(-250)
        path_find([BROWN, BLACK, PINK], True)
    print("The robot has left the", background, "area")
    time.sleep(0.1)

def misplaced():
    '''When called: The robot will check if there is an item or robot in it's way.'''
    global hasPallet
    if 100 < ultra_sensor.distance(silent=False) < 350:
        robot.stop()
        print("Robot or item in path")
        time.sleep(1)

def change_pickup_color():
    '''When called: Will give option where the robot should go to and if it should commit to enter a warehouse'''
    while True:
        robot.stop()
        global pickColor
        commit = False
        print("Where do you wanna go?")
        print("Left Red | Right Blue") # Checks where to go to
        print("Up Delivery | Down Warehouse")
        while commit is False:
            time.sleep(0.1)
            if rightButton in ev3.buttons.pressed():
                commit = True
                print("Blue warehouse")
                pickColor = "blue"
                time.sleep(0.1)
                path_find(BlueWare, True)
            elif leftButton in ev3.buttons.pressed():
                commit = True
                print("RED warehouse")
                pickColor = "red"
                time.sleep(0.1)
                path_find(RedWare, True)
            elif upButton in ev3.buttons.pressed():
                commit = True
                print("Deliver and Pickup")
                pickColor = "deliver"
                time.sleep(0.1)
                path_find(Deliver, True)
            elif downButton in ev3.buttons.pressed():
                commit = True
                print("Warehouse")
                pickColor = "warehouse"
                time.sleep(0.1)
                path_find(WareHouse, True)

        commit = False
        robot.stop()
        print("Do you wanna pick up", pickColor)
        print("Left No | Right Yes") # Checks if to commit with the pick up or return
        while commit is False:
            time.sleep(0.1)
            if rightButton in ev3.buttons.pressed():
                commit = True
                print("Pick up")
                time.sleep(0.1)
                find_item()
                time.sleep(0.1)
                if pickColor == "blue": # Returns to roundabout
                    path_find([BLUE, GREEN], True)
                elif pickColor == "red":
                    path_find([PINK, GREEN], True)
                elif pickColor == "deliver":
                    path_find([LIGHTGREEN, GREEN], True)
                elif pickColor == "warehouse":
                    path_find([MAGENTA, GREEN], True)
            elif leftButton in ev3.buttons.pressed():
                commit = True
                print("Return") # Returns to roundabout
                time.sleep(0.1)
                robot.turn(150)
                if pickColor == "blue":
                    path_find([BLUE, GREEN, GREEN], True)
                elif pickColor == "red":
                    path_find([PINK, GREEN, GREEN], True)
                elif pickColor == "deliver":
                    path_find([LIGHTGREEN, GREEN], True)
                elif pickColor == "warehouse":
                    path_find([MAGENTA, GREEN], True)
                pickColor = None

def checkColor():
    '''Test function.
    When called: Will loop until stopped manually. Will write out the three RGB values the robot sees in the terminal.
    RGB values range from 0 to 100'''
    while True:
        color = color_sensor.rgb()
        print(color)
        time.sleep(2)

def main():
    '''Main method.'''
    lift_motor.reset_angle(0)
    lift_motor.run_target(200, 15)
    # ^ Elevates the lift so that the robot doesn't get stuck
    change_pickup_color()

if __name__ == '__main__':
    sys.exit(main())
